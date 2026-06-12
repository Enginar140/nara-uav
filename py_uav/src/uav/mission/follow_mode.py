"""FollowMode -- rakip takibi. C++ FollowMode portu (Proportional Navigation).

Fazlar:
  APPROACH  : uzak (>pursuit_range) -> tahmini hedefe HSS-guvenli rota planla
  PURSUIT   : yakin -> PN intercept, dogrudan DO_REPOSITION
  REACQUIRE : hedef kayboldu -> max_reacquire_s bekle, sonra PATROL'a dus

Not: lokal hiz kontrolu olmadigindan intercept her zaman pozisyon hedefi
(DO_REPOSITION) ile gonderilir.
"""
from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional

from ..core import geo
from ..core.time_utils import elapsed_s, now_monotonic
from ..core.types import GoalType, MissionGoal, Position, UAVState
from ..infra.log import get_logger
from ..nav.navigation_engine import NavigationEngine
from ..world_model.world_model import WorldModel
from .mode_base import IMissionMode

_log = get_logger("follow_mode")


class FollowPhase(Enum):
    APPROACH = auto()
    PURSUIT = auto()
    REACQUIRE = auto()


@dataclass
class FollowConfig:
    pursuit_range_m: float = 200.0
    reacquire_timeout_s: float = 5.0
    max_reacquire_s: float = 30.0
    prediction_horizon_s: float = 3.0
    target_stale_s: float = 3.0
    plan_update_interval_s: float = 3.0
    route_lock_s: float = 8.0
    route_goal_update_m: float = 120.0
    route_force_replan_m: float = 350.0
    commit_route_until_complete: bool = True
    route_prediction_min_s: float = 3.0
    route_prediction_max_s: float = 10.0
    approach_standoff_m: float = 150.0
    route_chain_min_spacing_m: float = 120.0
    target_position_alpha: float = 0.30
    target_velocity_alpha: float = 0.35
    target_filter_reset_m: float = 500.0
    direct_command_interval_s: float = 1.5
    direct_goal_update_m: float = 70.0
    approach_speed_ms: float = 20.0
    pursuit_speed_ms: float = 30.0
    pn_gain: float = 3.0
    pn_max_lead_deg: float = 45.0
    # Vision fallback: world-model hedefi bayatlayinca kameradan surdur
    vision_fallback: bool = False
    vision_module_id: str = "vision_tracker"
    vision_lock_age_s: float = 0.5


class FollowMode(IMissionMode):
    def __init__(self, nav: NavigationEngine, world_model: WorldModel,
                 config: FollowConfig | None = None):
        self._nav = nav
        self._wm = world_model
        self.cfg = config or FollowConfig()
        self._target_id = -1
        self._phase = FollowPhase.APPROACH
        self._permanently_lost = False
        self._lost_cb: Optional[Callable[[], None]] = None
        self._target_lost_t = 0.0
        self._last_plan_t = 0.0
        self._last_plan_goal: Optional[Position] = None
        self._route_lock_until_t = 0.0
        self._filtered_target: Optional[UAVState] = None
        self._last_direct_goal: Optional[Position] = None
        self._last_direct_t = 0.0
        # perception (vision fallback)
        self._perception = None
        self._bus = None
        self._perc_sub_id = None
        self._perc_active = False
        self._using_vision = False
        self._last_perc = None
        self._last_perc_t = 0.0
        # PN LOS izleme
        self._los_init = False
        self._prev_los_bearing = 0.0
        self._prev_los_t = 0.0

    def set_target_id(self, tid: int) -> None:
        self._target_id = tid

    def get_target_id(self) -> int:
        return self._target_id

    def get_phase(self) -> FollowPhase:
        return self._phase

    def set_target_lost_callback(self, cb: Callable[[], None]) -> None:
        self._lost_cb = cb

    def set_perception_engine(self, engine, bus) -> None:
        self._perception = engine
        self._bus = bus

    def _enable_perception(self) -> None:
        if not self.cfg.vision_fallback or not self._perception or not self._bus:
            return
        if self._perc_active:
            return
        from ..infra.event_bus import EventType
        self._perc_sub_id = self._bus.subscribe(EventType.PERCEPTION_UPDATED, self._on_perception)
        self._perc_active = True
        _log.info("FOLLOW: vision fallback hazir (modul='%s')", self.cfg.vision_module_id)

    def _disable_perception(self) -> None:
        if not self._perc_active:
            return
        if self._bus and self._perc_sub_id is not None:
            self._bus.unsubscribe(self._perc_sub_id)
            self._perc_sub_id = None
        if self._perception and self._perception.active_id() == self.cfg.vision_module_id:
            self._perception.set_active("")
        self._last_perc = None
        self._perc_active = False
        self._using_vision = False

    def _on_perception(self, ev) -> None:
        pe = ev.payload
        if pe is None:
            return
        if pe.has_lock:
            self._last_perc = pe.target
            self._last_perc_t = now_monotonic()
        else:
            self._last_perc = None

    def _tick_vision_fallback(self) -> bool:
        if not self._perc_active or not self._nav:
            return False
        if self._perception and self._perception.active_id() != self.cfg.vision_module_id:
            self._perception.set_active(self.cfg.vision_module_id)
        if self._last_perc is None:
            return False
        if elapsed_s(self._last_perc_t) > self.cfg.vision_lock_age_s:
            return False
        if not self._nav.dispatch_perception_target(self._last_perc):
            return False
        if not self._using_vision:
            _log.info("FOLLOW: vision fallback'e gecildi (conf=%.2f)", self._last_perc.confidence)
            self._using_vision = True
        return True

    def on_enter(self) -> None:
        _log.info("FOLLOW: giriliyor, target_id=%d", self._target_id)
        self._phase = FollowPhase.APPROACH
        self._permanently_lost = False
        self._los_init = False
        self._using_vision = False
        self._reset_route_lock()
        self._enable_perception()

    def on_exit(self) -> None:
        self._disable_perception()
        self._nav.clear_route()
        self._reset_route_lock()

    def tick(self, own_state: UAVState) -> None:
        if self._target_id < 0:
            return
        target = self._wm.get_opponent(self._target_id)
        fresh = target is not None and target.age_seconds() <= self.cfg.target_stale_s
        if not fresh:
            # vazgecmeden once vision fallback dene (kamera/algi hedefi goruyorsa)
            if self.cfg.vision_fallback and self._tick_vision_fallback():
                return
            if self._phase != FollowPhase.REACQUIRE:
                self._transition(FollowPhase.REACQUIRE)
            self._tick_reacquire(own_state)
            return
        # world-model hedefi geri geldi: vision fallback'i birak, stale yaw'i temizle
        if self._using_vision:
            self._using_vision = False
            self._nav.release_guided_hold()
        if self._phase == FollowPhase.APPROACH:
            target = self._filter_target(target)
            self._tick_approach(own_state, target)
        elif self._phase == FollowPhase.PURSUIT:
            target = self._filter_target(target)
            self._tick_pursuit(own_state, target)
        elif self._phase == FollowPhase.REACQUIRE:
            self._transition(FollowPhase.APPROACH)
            target = self._filter_target(target)
            self._tick_approach(own_state, target)

    def get_mission_goal(self) -> MissionGoal:
        return MissionGoal(type=GoalType.TRACK_TARGET, target_id=self._target_id)

    def name(self) -> str:
        return "FOLLOW"

    # -- fazlar --
    def _tick_approach(self, own: UAVState, target: UAVState) -> None:
        if self.cfg.commit_route_until_complete and self._nav.has_active_route():
            self._nav.update(own.position)
            return

        dist = geo.haversine_distance(own.position, target.position)
        route_goals = self._build_approach_route(own, target)
        candidate = route_goals[-1]
        if dist <= self.cfg.pursuit_range_m:
            if self._nav.can_fly_direct(candidate):
                self._transition(FollowPhase.PURSUIT)
                self._tick_pursuit(own, target)
            else:
                self._tick_routed_target(own, candidate, self.cfg.approach_speed_ms,
                                         route_goals)
            return
        self._tick_routed_target(own, candidate, self.cfg.approach_speed_ms, route_goals)

    def _tick_pursuit(self, own: UAVState, target: UAVState) -> None:
        if self.cfg.commit_route_until_complete and self._nav.has_active_route():
            self._nav.update(own.position)
            return

        dist = geo.haversine_distance(own.position, target.position)
        if dist > self.cfg.pursuit_range_m * 1.5:
            self._transition(FollowPhase.APPROACH)
            self._los_init = False
            return
        intercept = self._predict_with_pn(own, target)
        if self._nav.can_fly_direct(intercept):
            if self._nav.has_active_route():
                self._nav.clear_route()
                self._reset_route_lock()
            self._fly_direct_stable(intercept, self.cfg.pursuit_speed_ms)
        else:
            self._tick_routed_target(own, intercept, self.cfg.pursuit_speed_ms)

    def _tick_reacquire(self, own: UAVState) -> None:
        since_lost = elapsed_s(self._target_lost_t)
        if since_lost >= self.cfg.max_reacquire_s:
            _log.warning("FOLLOW: %.1fs hedef yok, vazgeciliyor", since_lost)
            self._permanently_lost = True
            if self._lost_cb:
                self._lost_cb()
            return
        self._nav.update(own.position)

    # -- tahmin --
    def _predict_target(self, target: UAVState, horizon_s: Optional[float] = None) -> Position:
        dt = self.cfg.prediction_horizon_s if horizon_s is None else horizon_s
        brg = math.degrees(math.atan2(target.velocity.east, target.velocity.north))
        dist = target.velocity.horizontal_speed() * dt
        pred = geo.destination_point(target.position, brg, dist)
        pred.altitude = target.position.altitude
        return pred

    def _predict_route_target(self, own: UAVState, target: UAVState, speed_ms: float) -> Position:
        dist = geo.haversine_distance(own.position, target.position)
        time_to_go = dist / max(5.0, speed_ms)
        horizon = max(self.cfg.prediction_horizon_s, time_to_go * 0.25)
        horizon = min(max(horizon, self.cfg.route_prediction_min_s),
                      self.cfg.route_prediction_max_s)
        return self._predict_target(target, horizon)

    def _build_approach_route(self, own: UAVState, target: UAVState) -> list[Position]:
        predicted = self._predict_route_target(own, target, self.cfg.approach_speed_ms)
        dist_to_predicted = geo.haversine_distance(own.position, predicted)
        standoff = max(0.0, min(self.cfg.approach_standoff_m, dist_to_predicted * 0.35))
        if standoff > 1.0:
            approach_bearing = geo.bearing(predicted, own.position)
            approach_goal = geo.destination_point(predicted, approach_bearing, standoff)
            approach_goal.altitude = predicted.altitude
        else:
            approach_goal = predicted

        goals: list[Position] = [approach_goal]
        if geo.haversine_distance(approach_goal, predicted) >= self.cfg.route_chain_min_spacing_m:
            goals.append(predicted)
        return goals

    def _predict_with_pn(self, own: UAVState, target: UAVState) -> Position:
        los_bearing = geo.bearing(own.position, target.position)
        now = now_monotonic()
        if not self._los_init:
            self._prev_los_bearing = los_bearing
            self._prev_los_t = now
            self._los_init = True
            return self._predict_target(target)
        dt = now - self._prev_los_t
        if dt < 0.05:
            return self._predict_target(target)
        los_delta = geo.angle_difference(los_bearing, self._prev_los_bearing)
        los_rate = math.radians(los_delta) / dt
        self._prev_los_bearing = los_bearing
        self._prev_los_t = now
        dist = geo.haversine_distance(own.position, target.position)
        closing = max(5.0, target.velocity.horizontal_speed() + own.airspeed)
        t_go = dist / closing
        pn_off = self.cfg.pn_gain * los_rate * t_go
        max_lead = math.radians(self.cfg.pn_max_lead_deg)
        pn_off = max(-max_lead, min(max_lead, pn_off))
        intercept_bearing = los_bearing + math.degrees(pn_off)
        intercept_dist = min(self.cfg.pursuit_speed_ms * t_go, dist * 1.5)
        intercept = geo.destination_point(own.position, intercept_bearing, intercept_dist)
        intercept.altitude = target.position.altitude
        return intercept

    def _filter_target(self, raw: UAVState) -> UAVState:
        if self._filtered_target is None or self._filtered_target.id != raw.id:
            self._filtered_target = copy.deepcopy(raw)
            return copy.deepcopy(raw)

        prev = self._filtered_target
        jump = geo.haversine_distance(prev.position, raw.position)
        if jump >= self.cfg.target_filter_reset_m:
            self._filtered_target = copy.deepcopy(raw)
            return copy.deepcopy(raw)

        pa = min(max(self.cfg.target_position_alpha, 0.0), 1.0)
        va = min(max(self.cfg.target_velocity_alpha, 0.0), 1.0)
        offset = geo.position_to_ned(raw.position, prev.position)
        filtered = copy.deepcopy(prev)
        filtered.position = geo.ned_to_position(
            geo.NED(offset.north * pa, offset.east * pa, offset.down * pa),
            prev.position,
        )
        filtered.velocity.north = prev.velocity.north * (1.0 - va) + raw.velocity.north * va
        filtered.velocity.east = prev.velocity.east * (1.0 - va) + raw.velocity.east * va
        filtered.velocity.down = prev.velocity.down * (1.0 - va) + raw.velocity.down * va
        filtered.groundspeed = filtered.velocity.horizontal_speed()
        filtered.airspeed = raw.airspeed
        if filtered.groundspeed > 0.5:
            filtered.heading = geo.normalize_angle(
                math.degrees(math.atan2(filtered.velocity.east, filtered.velocity.north)))
        else:
            filtered.heading = raw.heading
        filtered.timestamp_us = raw.timestamp_us
        self._filtered_target = filtered
        return copy.deepcopy(filtered)

    def _tick_routed_target(self, own: UAVState, goal: Position, speed_ms: float,
                            route_goals: Optional[list[Position]] = None) -> None:
        if self._should_replan_route(goal):
            goals = route_goals or [goal]
            if self._plan_committed_route(goals, speed_ms):
                now = now_monotonic()
                self._last_plan_t = now
                self._last_plan_goal = goal
                self._route_lock_until_t = now + max(0.0, self.cfg.route_lock_s)
                self._reset_direct_hold()
                return
        self._nav.update(own.position)

    def _plan_committed_route(self, goals: list[Position], speed_ms: float) -> bool:
        if hasattr(self._nav, "make_safe_route"):
            return self._nav.make_safe_route(goals, speed_ms)
        return self._nav.plan_route(goals[-1], speed_ms)

    def _fly_direct_stable(self, goal: Position, speed_ms: float) -> bool:
        now = now_monotonic()
        if self._last_direct_goal is not None:
            age = now - self._last_direct_t
            shift = geo.haversine_distance(self._last_direct_goal, goal)
            if age < self.cfg.direct_command_interval_s:
                return True
            if shift < self.cfg.direct_goal_update_m and age < self.cfg.direct_command_interval_s * 4.0:
                return True
        sent = self._nav.fly_direct(goal, speed_ms)
        if sent:
            self._last_direct_goal = goal
            self._last_direct_t = now
        return sent

    def _should_replan_route(self, goal: Position) -> bool:
        if self.cfg.commit_route_until_complete and self._nav.has_active_route():
            return False
        if not self._nav.has_active_route() or self._last_plan_goal is None:
            return True
        shift = geo.haversine_distance(self._last_plan_goal, goal)
        if shift >= self.cfg.route_force_replan_m:
            return True
        now = now_monotonic()
        if now < self._route_lock_until_t:
            return False
        if elapsed_s(self._last_plan_t) < self.cfg.plan_update_interval_s:
            return False
        return shift >= self.cfg.route_goal_update_m

    def _reset_route_lock(self) -> None:
        self._last_plan_t = 0.0
        self._last_plan_goal = None
        self._route_lock_until_t = 0.0

    def _reset_direct_hold(self) -> None:
        self._last_direct_goal = None
        self._last_direct_t = 0.0

    def _transition(self, phase: FollowPhase) -> None:
        _log.info("FOLLOW: %s -> %s", self._phase.name, phase.name)
        if self._phase == FollowPhase.REACQUIRE and phase != FollowPhase.REACQUIRE:
            self._nav.clear_route()
            self._reset_route_lock()
            self._reset_direct_hold()
        if phase == FollowPhase.PURSUIT:
            self._nav.clear_route()
            self._reset_route_lock()
            self._reset_direct_hold()
        if phase == FollowPhase.REACQUIRE:
            self._filtered_target = None
            self._reset_direct_hold()
        self._phase = phase
        if phase == FollowPhase.REACQUIRE:
            self._target_lost_t = now_monotonic()

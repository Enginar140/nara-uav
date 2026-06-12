"""QRDiveMode -- QR/kamikaze hedefe dalis. C++ QRDiveMode portu.

Fazlar: APPROACH -> ALIGN -> DIVE -> CLIMB -> COMPLETED

Dalis pozisyon-adimi (flyDirect) ile yapilir; velocity komutu YOK (ArduPlane
sabit-kanatta hiz setpoint'ini yok sayar). Opsiyonel perception kilidi varsa
body-offset ile terminal yonlendirme yapilabilir (NavigationEngine.fly_body_offset).
DIVE/ALIGN sirasinda HSS yakinsa dalis iptal -> CLIMB.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional

from ..comm.flight_commander import FlightCommander
from ..core import geo
from ..core.time_utils import elapsed_s, now_monotonic
from ..core.types import GoalType, MissionGoal, Position, UAVState
from ..infra.log import get_logger
from ..nav.navigation_engine import NavigationEngine
from ..nav.waypoint_chain import ChainStatus
from ..world_model.world_model import WorldModel
from .mode_base import IMissionMode

_log = get_logger("qr_dive_mode")


class DivePhase(Enum):
    APPROACH = auto()
    ALIGN = auto()
    DIVE = auto()
    CLIMB = auto()
    COMPLETED = auto()


@dataclass
class QRDiveConfig:
    approach_altitude_m: float = 120.0
    min_altitude_m: float = 20.0
    climb_altitude_m: float = 100.0
    entry_corridor_m: float = 500.0
    align_heading_tolerance_deg: float = 15.0
    align_altitude_tolerance_m: float = 10.0
    dive_pitch_deg: float = -45.0
    dive_speed_ms: float = 25.0
    target_pass_distance_m: float = 15.0
    climb_timeout_s: float = 30.0
    dive_step_interval_s: float = 0.5
    hss_abort_buffer_m: float = 20.0
    # Terminal faz: taze algi kilidi varsa kor pozisyon-adimi yerine onu kullan
    use_perception_terminal: bool = True
    perception_lock_age_s: float = 0.5
    perception_module_id: str = "qr_simple"


class QRDiveMode(IMissionMode):
    def __init__(self, nav: NavigationEngine, world_model: WorldModel,
                 commander: FlightCommander, config: QRDiveConfig | None = None):
        self._nav = nav
        self._wm = world_model
        self._cmd = commander
        self.cfg = config or QRDiveConfig()
        self._dive_target: Optional[Position] = None
        self._entry_point = Position()
        self._entry_valid = False
        self._phase = DivePhase.APPROACH
        self._phase_start = 0.0
        self._completion_cb: Optional[Callable[[], None]] = None
        # perception (terminal yonlendirme)
        self._perception = None
        self._bus = None
        self._perc_sub_id = None
        self._perc_active = False
        self._last_perc = None
        self._last_perc_t = 0.0

    def set_dive_target(self, target: Position) -> None:
        self._dive_target = target
        self._entry_valid = False

    def set_completion_callback(self, cb: Callable[[], None]) -> None:
        self._completion_cb = cb

    def set_perception_engine(self, engine, bus) -> None:
        self._perception = engine
        self._bus = bus

    def _enable_perception(self) -> None:
        if not self.cfg.use_perception_terminal or not self._perception or not self._bus:
            return
        if self._perc_active:
            return
        from ..infra.event_bus import EventType
        self._perc_sub_id = self._bus.subscribe(EventType.PERCEPTION_UPDATED, self._on_perception)
        self._perception.set_active(self.cfg.perception_module_id)
        self._perc_active = True
        _log.info("QR_DIVE: perception '%s' aktif", self.cfg.perception_module_id)

    def _disable_perception(self) -> None:
        if not self._perc_active:
            return
        if self._bus and self._perc_sub_id is not None:
            self._bus.unsubscribe(self._perc_sub_id)
            self._perc_sub_id = None
        if self._perception and self._perception.active_id() == self.cfg.perception_module_id:
            self._perception.set_active("")
        self._last_perc = None
        self._perc_active = False

    def _on_perception(self, ev) -> None:
        pe = ev.payload
        if pe is None:
            return
        self._last_perc = pe.target if pe.has_lock else None
        if pe.has_lock:
            self._last_perc_t = now_monotonic()

    def _try_perception_dispatch(self) -> bool:
        if not self._perc_active or not self._nav or self._last_perc is None:
            return False
        if elapsed_s(self._last_perc_t) > self.cfg.perception_lock_age_s:
            return False
        return self._nav.dispatch_perception_target(self._last_perc)

    def get_phase(self) -> DivePhase:
        return self._phase

    def is_completed(self) -> bool:
        return self._phase == DivePhase.COMPLETED

    def on_enter(self) -> None:
        _log.info("QR_DIVE: giriliyor")
        self._phase = DivePhase.APPROACH
        self._phase_start = now_monotonic()
        self._enable_perception()  # APPROACH sirasinda isin -> DIVE'da kilit hazir
        if self._dive_target is None:
            _log.error("QR_DIVE: dalis hedefi yok")
            return
        self._calculate_entry_point()
        if self._entry_valid:
            self._nav.plan_route(self._entry_point)

    def on_exit(self) -> None:
        self._disable_perception()
        self._nav.clear_route()

    def tick(self, own_state: UAVState) -> None:
        if self._dive_target is None:
            return
        # DIVE/ALIGN sirasinda HSS abort
        if self._phase in (DivePhase.DIVE, DivePhase.ALIGN):
            for z in self._wm.get_active_hss_zones():
                if geo.haversine_distance(own_state.position, z.center) < z.radius + self.cfg.hss_abort_buffer_m:
                    _log.warning("QR_DIVE: HSS %d cok yakin, dalis iptal -> CLIMB", z.id)
                    self._enter_phase(DivePhase.CLIMB)
                    return
        if self._phase == DivePhase.APPROACH:
            self._tick_approach(own_state)
        elif self._phase == DivePhase.ALIGN:
            self._tick_align(own_state)
        elif self._phase == DivePhase.DIVE:
            self._tick_dive(own_state)
        elif self._phase == DivePhase.CLIMB:
            self._tick_climb(own_state)

    def get_mission_goal(self) -> MissionGoal:
        return MissionGoal(type=GoalType.DIVE_TARGET, target_position=self._dive_target)

    def name(self) -> str:
        return "QR_DIVE"

    # -- fazlar --
    def _tick_approach(self, own: UAVState) -> None:
        status = self._nav.update(own.position)
        if status in (ChainStatus.CHAIN_COMPLETE, ChainStatus.EMPTY):
            if geo.haversine_distance(own.position, self._entry_point) < 150.0:
                _log.info("QR_DIVE: entry point'e varildi -> ALIGN")
                self._enter_phase(DivePhase.ALIGN)
            else:
                self._nav.plan_route(self._entry_point)

    def _tick_align(self, own: UAVState) -> None:
        heading_to_target = geo.bearing(own.position, self._dive_target)
        heading_diff = abs(geo.angle_difference(own.heading, heading_to_target))
        altitude_diff = abs(own.position.altitude - self.cfg.approach_altitude_m)
        if elapsed_s(self._phase_start) > 30.0:
            _log.warning("QR_DIVE: align timeout, dalisa zorlaniyor")
            self._enter_phase(DivePhase.DIVE)
            return
        if (heading_diff < self.cfg.align_heading_tolerance_deg
                and altitude_diff < self.cfg.align_altitude_tolerance_m):
            _log.info("QR_DIVE: hizalandi -> DIVE")
            self._enter_phase(DivePhase.DIVE)
            return
        align_target = Position(self._entry_point.latitude, self._entry_point.longitude,
                                self.cfg.approach_altitude_m)
        self._nav.fly_direct(align_target)

    def _tick_dive(self, own: UAVState) -> None:
        dist = geo.haversine_distance(own.position, self._dive_target)
        if (dist < self.cfg.target_pass_distance_m
                or own.position.altitude <= self.cfg.min_altitude_m
                or elapsed_s(self._phase_start) > 30.0):
            _log.info("QR_DIVE: dalis tamam (dist=%.1f alt=%.1f) -> CLIMB",
                      dist, own.position.altitude)
            self._enter_phase(DivePhase.CLIMB)
            return
        # Once taze algi kilidi (frame-aware dispatch); yoksa kor pozisyon-adimi
        if self._try_perception_dispatch():
            return
        # pozisyon-adimi dalis (velocity yok)
        pitch_rad = math.radians(self.cfg.dive_pitch_deg)
        descent_rate = self.cfg.dive_speed_ms * math.sin(abs(pitch_rad))
        forward_speed = self.cfg.dive_speed_ms * math.cos(abs(pitch_rad))
        step = self.cfg.dive_step_interval_s
        bearing_to_target = geo.bearing(own.position, self._dive_target)
        next_pos = geo.destination_point(own.position, bearing_to_target, forward_speed * step)
        next_pos.altitude = max(own.position.altitude - descent_rate * step, self.cfg.min_altitude_m)
        self._nav.fly_direct(next_pos, self.cfg.dive_speed_ms)

    def _tick_climb(self, own: UAVState) -> None:
        if (own.position.altitude >= self.cfg.climb_altitude_m
                or elapsed_s(self._phase_start) > self.cfg.climb_timeout_s):
            _log.info("QR_DIVE: tirmanma tamam (alt=%.1f) -> COMPLETED", own.position.altitude)
            self._phase = DivePhase.COMPLETED
            if self._completion_cb:
                self._completion_cb()
            return
        climb_pos = Position(own.position.latitude, own.position.longitude,
                             self.cfg.climb_altitude_m)
        self._nav.fly_direct(climb_pos)

    def _calculate_entry_point(self) -> None:
        own = self._wm.get_own_state()
        approach_bearing = geo.bearing(own.position, self._dive_target)
        opposite = geo.normalize_angle(approach_bearing + 180.0)
        self._entry_point = geo.destination_point(self._dive_target, opposite, self.cfg.entry_corridor_m)
        self._entry_point.altitude = self.cfg.approach_altitude_m
        self._entry_valid = True
        _log.info("QR_DIVE: entry point bearing=%.1f, hedeften %.0fm", opposite, self.cfg.entry_corridor_m)

    def _enter_phase(self, phase: DivePhase) -> None:
        self._phase = phase
        self._phase_start = now_monotonic()

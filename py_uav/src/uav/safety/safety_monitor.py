"""SafetyMonitor -- surekli (~20Hz) guvenlik gozlemcisi (asyncio gorevi).

C++ SafetyMonitor portu: boundary / HSS / collision(TTC) / GPS / battery / comm
kontrolu, durum degisiminde EventBus'a olay yayinlar.
"""
from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass

from ..core import geo
from ..core.types import (
    BoundaryStatus,
    RiskLevel,
    SafetyState,
    Threat,
    UAVState,
)
from ..infra.event_bus import EventBus, EventType
from ..infra.log import get_logger
from ..world_model.world_model import WorldModel
from .zone_checker import ZoneChecker

_log = get_logger("safety_monitor")


@dataclass
class SafetyMonitorConfig:
    rate_hz: float = 20.0
    hss_buffer_m: float = 40.0
    boundary_warn_m: float = 30.0
    collision_critical_m: float = 30.0
    collision_high_m: float = 80.0
    collision_medium_m: float = 200.0
    collision_low_m: float = 500.0
    ttc_critical_s: float = 5.0
    ttc_high_s: float = 10.0
    event_throttle_s: float = 1.0


class SafetyMonitor:
    def __init__(self, world_model: WorldModel, event_bus: EventBus,
                 config: SafetyMonitorConfig | None = None):
        self._wm = world_model
        self._bus = event_bus
        self.cfg = config or SafetyMonitorConfig()
        self._checker = ZoneChecker()
        self._state = SafetyState()
        self._heartbeat_ok = True
        self._running = False
        self._task: asyncio.Task | None = None
        self._last_event = 0.0
        # tek-seferlik uyari bayraklari
        self._gps_low_warned = False
        self._gps_crit_warned = False
        self._batt_low_warned = False
        self._batt_crit_warned = False

    async def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._task = asyncio.create_task(self._loop(), name="safety_monitor")

    async def stop(self) -> None:
        self._running = False
        if self._task:
            self._task.cancel()

    def get_safety_state(self) -> SafetyState:
        return self._state

    def notify_heartbeat(self) -> None:
        self._heartbeat_ok = True

    def notify_heartbeat_lost(self) -> None:
        self._heartbeat_ok = False

    async def _loop(self) -> None:
        period = 1.0 / max(1.0, self.cfg.rate_hz)
        while self._running:
            self.evaluate()
            await asyncio.sleep(period)

    # Test edilebilir tek adim
    def evaluate(self) -> SafetyState:
        own = self._wm.get_own_state()
        if own.is_valid():
            self._state.collision_risk = RiskLevel.NONE
            self._state.active_threats = []
            self._state.hss_violations = []
            boundary = self._wm.get_boundary()
            zones = self._wm.get_active_hss_zones()
            opponents = self._wm.get_all_opponents()
            if boundary is not None:
                self._check_boundary(own, boundary)
            self._check_hss(own, zones)
            self._check_collisions(own, opponents)
            self._check_gps(own)
            self._check_battery(own)
            self._check_comm()
        self._publish_if_changed()
        return self._state

    def _check_boundary(self, own: UAVState, boundary) -> None:
        if not self._checker.is_inside_boundary(own.position, boundary):
            self._state.boundary_status = BoundaryStatus.OUTSIDE
            self._bus.publish(EventType.BOUNDARY_VIOLATION)
            return
        d = self._checker.distance_to_boundary(own.position, boundary)
        self._state.boundary_status = (
            BoundaryStatus.NEAR_BOUNDARY if d < self.cfg.boundary_warn_m else BoundaryStatus.INSIDE
        )

    def _check_hss(self, own: UAVState, zones) -> None:
        for z in zones:
            if geo.haversine_distance(own.position, z.center) < z.radius + self.cfg.hss_buffer_m:
                self._state.hss_violations.append(z.id)
                self._bus.publish(EventType.HSS_VIOLATION, z.id)

    def _check_collisions(self, own: UAVState, opponents) -> None:
        for opp in opponents:
            dist = geo.haversine_distance(own.position, opp.position)
            rel_vn = opp.velocity.north - own.velocity.north
            rel_ve = opp.velocity.east - own.velocity.east
            brg = math.radians(geo.bearing(own.position, opp.position))
            closing = -(rel_vn * math.cos(brg) + rel_ve * math.sin(brg))
            ttc = dist / closing if closing > 0.5 else math.inf

            risk = RiskLevel.NONE
            if dist < self.cfg.collision_critical_m or ttc < self.cfg.ttc_critical_s:
                risk = RiskLevel.CRITICAL
            elif dist < self.cfg.collision_high_m or ttc < self.cfg.ttc_high_s:
                risk = RiskLevel.HIGH
            elif dist < self.cfg.collision_medium_m:
                risk = RiskLevel.MEDIUM
            elif dist < self.cfg.collision_low_m:
                risk = RiskLevel.LOW

            if risk > self._state.collision_risk:
                self._state.collision_risk = risk
            if risk >= RiskLevel.MEDIUM:
                self._state.active_threats.append(Threat(
                    aircraft_id=opp.id, distance=dist,
                    bearing=geo.bearing(own.position, opp.position),
                    closing_speed=closing, time_to_collision=ttc, risk_level=risk,
                ))

        if self._state.collision_risk >= RiskLevel.HIGH and self._state.active_threats:
            self._bus.publish(EventType.COLLISION_WARNING, self._state.active_threats[0])

    def _check_gps(self, own: UAVState) -> None:
        if not (own.gps_fix_type > 0 or own.gps_hdop < 9000):
            return
        if own.gps_fix_type <= 1 and own.gps_hdop > 500:
            if not self._gps_crit_warned:
                self._bus.publish(EventType.GPS_QUALITY_CRITICAL)
                self._gps_crit_warned = True
        elif own.gps_fix_type <= 2 and own.gps_hdop > 300:
            if not self._gps_low_warned:
                self._bus.publish(EventType.GPS_QUALITY_LOW)
                self._gps_low_warned = True
            self._gps_crit_warned = False
        else:
            self._gps_low_warned = False
            self._gps_crit_warned = False

    def _check_battery(self, own: UAVState) -> None:
        if own.battery_remaining < 0:
            return
        if own.battery_remaining < 10:
            if not self._batt_crit_warned:
                self._bus.publish(EventType.BATTERY_CRITICAL)
                self._batt_crit_warned = True
        elif own.battery_remaining < 25:
            if not self._batt_low_warned:
                self._bus.publish(EventType.BATTERY_LOW)
                self._batt_low_warned = True
            self._batt_crit_warned = False
        else:
            self._batt_low_warned = False
            self._batt_crit_warned = False

    def _check_comm(self) -> None:
        # comm_loss yalniz SafetyState'e yansitilir; HEARTBEAT_TIMEOUT/RECOVERED
        # olaylarini MavlinkBridge yayinlar (tek yetkili kaynak -> cift RTL yok).
        self._state.comm_loss = not self._heartbeat_ok

    def _publish_if_changed(self) -> None:
        import time
        now = time.monotonic()
        if now - self._last_event < self.cfg.event_throttle_s:
            return
        if not self._state.is_safe():
            self._bus.publish(EventType.SAFETY_VIOLATION, self._state)
            self._last_event = now

"""safety katmani: zone_checker, safety_gate, safety_monitor, escape_manager."""
import pytest

from uav.core import geo
from uav.core.types import (
    Boundary,
    BoundaryStatus,
    HSSZone,
    Position,
    RiskLevel,
    UAVState,
    Velocity,
)
from uav.infra.event_bus import EventBus, EventType
from uav.safety.escape_manager import EscapeManager
from uav.safety.safety_gate import SafetyGate, SafetyGateConfig
from uav.safety.safety_monitor import SafetyMonitor, SafetyMonitorConfig
from uav.safety.zone_checker import ZoneChecker
from uav.world_model.world_model import WorldModel

# ~5km kenarli kare boundary, merkez (37.82, -122.35)
BND = Boundary(vertices=[
    Position(37.80, -122.37), Position(37.84, -122.37),
    Position(37.84, -122.33), Position(37.80, -122.33),
])
CENTER = Position(37.82, -122.35, 100.0)


def test_zone_checker_boundary_and_hss():
    zc = ZoneChecker()
    assert zc.is_inside_boundary(CENTER, BND)
    assert not zc.is_inside_boundary(Position(37.90, -122.35), BND)
    zones = [HSSZone(id=1, center=CENTER, radius=100, active=True)]
    assert zc.is_inside_any_hss(CENTER, zones, 40.0)
    far = geo.destination_point(CENTER, 0.0, 300.0)
    assert not zc.is_inside_any_hss(far, zones, 40.0)


def test_zone_checker_segment_intersects_hss():
    zc = ZoneChecker()
    zones = [HSSZone(id=1, center=CENTER, radius=100, active=True)]
    a = geo.destination_point(CENTER, 270.0, 300.0)
    b = geo.destination_point(CENTER, 90.0, 300.0)  # merkezden gecer
    assert zc.segment_intersects_any_hss(a, b, zones, 40.0)
    c = geo.destination_point(CENTER, 0.0, 500.0)
    d = geo.destination_point(c, 90.0, 100.0)
    assert not zc.segment_intersects_any_hss(c, d, zones, 40.0)


def _wm_with(boundary=None, zones=None, own=None):
    wm = WorldModel()
    if boundary:
        wm.set_boundary(boundary)
    for z in zones or []:
        wm.update_hss_zone(z)
    if own:
        wm.set_own_state(own)
    return wm


def test_safety_gate_validate_and_clamp_hss():
    wm = _wm_with(boundary=BND, zones=[HSSZone(id=1, center=CENTER, radius=100, active=True)])
    gate = SafetyGate(ZoneChecker(), wm, SafetyGateConfig(hss_buffer_m=40, boundary_buffer_m=20))
    # HSS icindeki hedef gecersiz
    assert not gate.validate_position(CENTER)
    # clamp HSS disina iter
    clamped = gate.clamp_to_safe(CENTER)
    assert geo.haversine_distance(clamped, CENTER) >= 100 + 40
    assert gate.validate_position(clamped)


def test_safety_gate_altitude_limits():
    wm = _wm_with(boundary=BND)
    gate = SafetyGate(ZoneChecker(), wm, SafetyGateConfig(min_altitude_m=30, max_altitude_m=400))
    assert not gate.validate_position(Position(37.82, -122.35, 10.0))
    assert not gate.validate_position(Position(37.82, -122.35, 500.0))
    assert gate.validate_position(Position(37.82, -122.35, 100.0))


def test_safety_monitor_hss_violation_event():
    bus = EventBus()
    fired = []
    bus.subscribe(EventType.HSS_VIOLATION, lambda ev: fired.append(ev.payload))
    own = UAVState(id=0, position=CENTER, timestamp_us=1, gps_fix_type=3, battery_remaining=80)
    wm = _wm_with(boundary=BND, zones=[HSSZone(id=42, center=CENTER, radius=100, active=True)], own=own)
    mon = SafetyMonitor(wm, bus, SafetyMonitorConfig())
    state = mon.evaluate()
    assert 42 in state.hss_violations
    assert fired == [42]


def test_safety_monitor_collision_risk():
    bus = EventBus()
    own = UAVState(id=0, position=CENTER, velocity=Velocity(20, 0, 0),
                   timestamp_us=1, gps_fix_type=3, battery_remaining=80)
    # 25m kuzeyde, bize dogru gelen rakip -> CRITICAL
    opp_pos = geo.destination_point(CENTER, 0.0, 25.0)
    opp = UAVState(id=9, position=opp_pos, velocity=Velocity(-20, 0, 0), timestamp_us=1)
    wm = _wm_with(boundary=BND, own=own, zones=[])
    wm.update_opponent(opp)
    mon = SafetyMonitor(wm, bus, SafetyMonitorConfig())
    state = mon.evaluate()
    assert state.collision_risk == RiskLevel.CRITICAL


def test_safety_monitor_boundary_outside():
    bus = EventBus()
    own = UAVState(id=0, position=Position(37.90, -122.35, 100), timestamp_us=1,
                   gps_fix_type=3, battery_remaining=80)
    wm = _wm_with(boundary=BND, own=own)
    mon = SafetyMonitor(wm, bus, SafetyMonitorConfig())
    state = mon.evaluate()
    assert state.boundary_status == BoundaryStatus.OUTSIDE


def test_escape_manager_hss_escape():
    wm = _wm_with(boundary=BND, zones=[HSSZone(id=1, center=CENTER, radius=100, active=True)])
    gate = SafetyGate(ZoneChecker(), wm, SafetyGateConfig(hss_buffer_m=40))
    em = EscapeManager(ZoneChecker(), wm, gate)
    own = UAVState(id=0, position=CENTER, timestamp_us=1)
    escape = em.generate_escape_target(own)
    assert escape is not None
    assert gate.validate_position(escape)
    assert geo.haversine_distance(escape, CENTER) >= 100 + 40

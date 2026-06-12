"""core.types temel davranis testleri."""
from uav.core.types import (
    Boundary,
    BoundaryStatus,
    HSSZone,
    Position,
    RiskLevel,
    SafetyState,
    Threat,
    UAVState,
    Velocity,
)


def test_position_equality_tolerance():
    assert Position(1.0, 2.0, 3.0) == Position(1.0 + 1e-10, 2.0, 3.0)
    assert Position(1.0, 2.0, 3.0) != Position(1.1, 2.0, 3.0)


def test_velocity_speeds():
    v = Velocity(3.0, 4.0, 12.0)
    assert v.horizontal_speed() == 5.0
    assert v.speed_3d() == 13.0


def test_uavstate_validity_and_age():
    s = UAVState()
    assert not s.is_valid()
    s.timestamp_us = 1
    assert s.is_valid()
    assert s.age_seconds() >= 0.0


def test_hss_effective_radius():
    z = HSSZone(id=1, center=Position(), radius=100.0, active=True, buffer_distance=40.0)
    assert z.effective_radius() == 140.0


def test_boundary_center_and_valid():
    b = Boundary(vertices=[Position(0, 0), Position(0, 2), Position(2, 2), Position(2, 0)])
    assert b.is_valid()
    c = b.get_center()
    assert c.latitude == 1.0 and c.longitude == 1.0
    assert not Boundary(vertices=[Position(0, 0), Position(1, 1)]).is_valid()


def test_safety_state_is_safe():
    s = SafetyState()
    assert s.is_safe()
    s.collision_risk = RiskLevel.HIGH
    assert not s.is_safe()
    s2 = SafetyState(boundary_status=BoundaryStatus.OUTSIDE)
    assert not s2.is_safe()
    s3 = SafetyState(hss_violations=[3])
    assert not s3.is_safe()


def test_threat_defaults():
    t = Threat(aircraft_id=7)
    assert t.risk_level == RiskLevel.NONE
    assert t.time_to_collision == float("inf")

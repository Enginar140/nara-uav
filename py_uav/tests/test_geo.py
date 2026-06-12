"""core.geo testleri: C++ matematigine sadakat + LocalFrame tutarliligi."""
import math

import pytest

from uav.core import geo
from uav.core.types import Boundary, Position

SF = Position(37.8200, -122.3500, 100.0)


def test_haversine_zero():
    assert geo.haversine_distance(SF, SF) == pytest.approx(0.0, abs=1e-6)


def test_haversine_known_distance():
    # 1 derece enlem ~ 111.2 km (haversine, R=6371km)
    a = Position(0.0, 0.0, 0.0)
    b = Position(1.0, 0.0, 0.0)
    d = geo.haversine_distance(a, b)
    assert d == pytest.approx(111195.0, rel=1e-3)


def test_bearing_cardinal():
    a = Position(0.0, 0.0, 0.0)
    assert geo.bearing(a, Position(1.0, 0.0, 0.0)) == pytest.approx(0.0, abs=0.5)   # kuzey
    assert geo.bearing(a, Position(0.0, 1.0, 0.0)) == pytest.approx(90.0, abs=0.5)  # dogu


def test_destination_roundtrip():
    dst = geo.destination_point(SF, 45.0, 1000.0)
    back = geo.haversine_distance(SF, dst)
    assert back == pytest.approx(1000.0, rel=1e-3)
    assert geo.bearing(SF, dst) == pytest.approx(45.0, abs=0.5)


def test_ned_roundtrip():
    p = geo.destination_point(SF, 120.0, 500.0)
    ned = geo.position_to_ned(p, SF)
    back = geo.ned_to_position(ned, SF)
    assert geo.haversine_distance(p, back) == pytest.approx(0.0, abs=0.5)


def test_normalize_and_diff():
    assert geo.normalize_angle(-90.0) == pytest.approx(270.0)
    assert geo.normalize_angle(450.0) == pytest.approx(90.0)
    assert geo.angle_difference(10.0, 350.0) == pytest.approx(20.0)
    assert geo.angle_difference(350.0, 10.0) == pytest.approx(-20.0)


def test_point_in_polygon():
    poly = [Position(0, 0), Position(0, 1), Position(1, 1), Position(1, 0)]
    assert geo.point_in_polygon(Position(0.5, 0.5), poly)
    assert not geo.point_in_polygon(Position(2.0, 2.0), poly)


def test_segments_intersect():
    a1, a2 = Position(0, 0), Position(0.01, 0.01)
    b1, b2 = Position(0, 0.01), Position(0.01, 0)
    assert geo.segments_intersect(a1, a2, b1, b2)
    # paralel ayrik
    assert not geo.segments_intersect(
        Position(0, 0), Position(0, 0.01), Position(0.01, 0), Position(0.01, 0.01)
    )


def test_segment_intersects_circle():
    center = Position(0.0, 0.0, 0.0)
    # segment merkezden ~0 gecer
    assert geo.segment_intersects_circle(
        Position(-0.001, 0.0), Position(0.001, 0.0), center, 50.0
    )
    far = geo.destination_point(center, 90.0, 1000.0)
    assert not geo.segment_intersects_circle(
        far, geo.destination_point(far, 0.0, 100.0), center, 50.0
    )


def test_local_frame_roundtrip():
    bnd = Boundary(vertices=[
        Position(37.85, -122.39), Position(37.80, -122.38),
        Position(37.82, -122.31), Position(37.86, -122.33),
    ])
    frame = geo.LocalFrame.from_boundary_or_point(bnd, SF)
    p = geo.destination_point(SF, 33.0, 2500.0)
    x, y = frame.to_xy(p)
    back = frame.to_position(x, y, p.altitude)
    assert geo.haversine_distance(p, back) == pytest.approx(0.0, abs=0.5)


def test_local_frame_distance_matches_haversine():
    frame = geo.LocalFrame(SF)
    a = geo.destination_point(SF, 10.0, 300.0)
    b = geo.destination_point(SF, 200.0, 800.0)
    ax, ay = frame.to_xy(a)
    bx, by = frame.to_xy(b)
    planar = math.hypot(bx - ax, by - ay)
    hav = geo.haversine_distance(a, b)
    assert planar == pytest.approx(hav, rel=2e-3)

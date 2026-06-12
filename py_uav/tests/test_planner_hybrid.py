"""Hybrid planner katmani: dubins, cost_field, hybrid A*, planner kademeleri.

Senaryolar sartname kaynakli:
  - Dar/muhurlu HSS koridoru UCULABILIR olmali (sayfa 17: koridorlarda
    kilitlenme beklenir) -> kademe 2 (hybrid A*).
  - QR dalisi giris noktasina DALIS EKSENI heading'iyle varis -> goal heading.
  - Pop-up HSS sonrasi replan'de yan (homotopi) korunur -> flip-flop yok.
"""
import math
import time

import pytest

from uav.core import geo
from uav.core.types import Boundary, HSSZone, Position
from uav.nav import dubins
from uav.nav.cost_field import CostMap, CostMapConfig, ZoneCircle
from uav.nav.hybrid_astar import HybridAStarConfig, search
from uav.nav.planner import PathPlanner, PlannerConfig
from uav.nav.waypoint_chain import ChainStatus, Waypoint, WaypointChain

BND = Boundary(vertices=[
    Position(37.79, -122.39), Position(37.85, -122.39),
    Position(37.85, -122.31), Position(37.79, -122.31),
], min_altitude=30, max_altitude=400)
CENTER = Position(37.82, -122.35, 100.0)


def _pos(bearing, dist, alt=100.0):
    p = geo.destination_point(CENTER, bearing, dist)
    p.altitude = alt
    return p


# ---------------- dubins ----------------

def test_dubins_endpoint_reconstruction_random():
    import random
    random.seed(7)
    for _ in range(300):
        q0 = (random.uniform(-500, 500), random.uniform(-500, 500),
              random.uniform(0, 2 * math.pi))
        q1 = (random.uniform(-500, 500), random.uniform(-500, 500),
              random.uniform(0, 2 * math.pi))
        r = random.uniform(20, 150)
        p = dubins.shortest_path(q0, q1, r)
        assert p is not None
        ex, ey, et = dubins.path_endpoint(p)
        assert math.hypot(ex - q1[0], ey - q1[1]) < 0.5
        assert abs(dubins.mod2pi(et - q1[2] + math.pi) - math.pi) < 1e-3


def test_dubins_free_end_reaches_point():
    import random
    random.seed(11)
    for _ in range(300):
        q0 = (random.uniform(-500, 500), random.uniform(-500, 500),
              random.uniform(0, 2 * math.pi))
        tgt = (random.uniform(-500, 500), random.uniform(-500, 500))
        p = dubins.shortest_path_to_point(q0, tgt, random.uniform(20, 150))
        if p is None:
            continue  # hedef donus cemberi icinde: gecerli sonuc
        ex, ey, _ = dubins.path_endpoint(p)
        assert math.hypot(ex - tgt[0], ey - tgt[1]) < 0.5


def test_dubins_sampling_step_bound():
    p = dubins.shortest_path((0, 0, 0), (300, 200, math.pi / 2), 80)
    pts = dubins.sample_path(p, 15.0)
    for (x1, y1, _), (x2, y2, _) in zip(pts, pts[1:]):
        assert math.hypot(x2 - x1, y2 - y1) < 16.0


# ---------------- cost_field ----------------

def test_costmap_hard_and_soft():
    cm = CostMap([ZoneCircle(1, 0.0, 0.0, 100.0)],
                 [(-1000, -1000), (1000, -1000), (1000, 1000), (-1000, 1000)],
                 20.0, CostMapConfig(soft_band_m=100.0, soft_cost_max=4.0))
    assert not cm.hard_free(0.0, 50.0)          # duvar ici
    assert cm.hard_free(0.0, 150.0)             # band ici ama serbest
    assert cm.point_cost(0.0, 110.0) > cm.point_cost(0.0, 190.0) > 0.0
    assert cm.point_cost(0.0, 500.0) == pytest.approx(0.0)
    assert not cm.segment_hard_free(-200.0, 0.0, 200.0, 0.0)  # merkezden gecer
    assert cm.segment_hard_free(-200.0, 150.0, 200.0, 150.0)


def test_costmap_signature_sides_differ():
    cm = CostMap([ZoneCircle(1, 0.0, 0.0, 100.0)], None, 0.0, CostMapConfig())
    above = [(-400.0, 0.0), (-200.0, 150.0), (0.0, 180.0), (200.0, 150.0), (400.0, 0.0)]
    below = [(x, -y) for x, y in above]
    sa = cm.signature_of(above)
    sb = cm.signature_of(below)
    assert sa and sb and sa != sb


# ---------------- hybrid A* ----------------

def _xy_corridor_map():
    bnd = [(-2000, -2000), (2000, -2000), (2000, 2000), (-2000, 2000)]
    zones = [ZoneCircle(1, 0.0, 230.0, 155.0), ZoneCircle(2, 0.0, -230.0, 155.0)]
    return CostMap(zones, bnd, 20.0, CostMapConfig())


def test_hybrid_corridor_and_kinematics():
    cmap = _xy_corridor_map()
    path = search((-1200.0, 0.0, 0.0), (1200.0, 0.0), None, 90.0, cmap,
                  HybridAStarConfig(time_budget_s=3.0))
    assert path is not None
    assert all(cmap.hard_free(x, y) for x, y, _ in path)
    # ardisik heading degisimi ornek araligina gore donus yaricapiyla uyumlu
    for (x1, y1, t1), (x2, y2, t2) in zip(path, path[1:]):
        d = math.hypot(x2 - x1, y2 - y1)
        dth = abs(dubins.mod2pi(t2 - t1 + math.pi) - math.pi)
        if d > 1.0:
            assert dth <= d / 90.0 + 0.05   # R=90 alt siniri (+tolerans)


def test_hybrid_goal_heading():
    cmap = _xy_corridor_map()
    path = search((-1200.0, 0.0, 0.0), (1200.0, 0.0), math.pi / 2, 90.0, cmap,
                  HybridAStarConfig(time_budget_s=3.0))
    assert path is not None
    err = abs(dubins.mod2pi(path[-1][2] - math.pi / 2 + math.pi) - math.pi)
    assert err < math.radians(25.0)


def test_hybrid_reverse_start_heading():
    cmap = _xy_corridor_map()
    path = search((-1200.0, 0.0, math.pi), (1200.0, 0.0), None, 90.0, cmap,
                  HybridAStarConfig(time_budget_s=3.0))
    assert path is not None
    assert all(cmap.hard_free(x, y) for x, y, _ in path)


# ---------------- planner kademeleri ----------------

def _sealed_corridor_zones():
    """Konfor marjiyla TUM rotalar muhurlu; sert duvarla yalniz orta koridor acik."""
    return [
        HSSZone(id=1, center=_pos(0.0, 305.0), radius=150, active=True),
        HSSZone(id=2, center=_pos(180.0, 305.0), radius=150, active=True),
        HSSZone(id=3, center=_pos(0.0, 1900.0), radius=1500, active=True),
        HSSZone(id=4, center=_pos(180.0, 1900.0), radius=1500, active=True),
    ]


def test_planner_sealed_corridor_uses_hybrid():
    p = PathPlanner(PlannerConfig())
    start = _pos(270.0, 1500.0)
    goal = _pos(90.0, 1500.0)
    hss = _sealed_corridor_zones()
    t0 = time.perf_counter()
    res = p.plan(start, goal, BND, hss, start_heading_deg=90.0, speed_ms=20.0)
    elapsed = time.perf_counter() - t0
    assert res.success, res.error
    assert res.stage == "hybrid"
    assert p.is_path_safe(res.waypoints, BND, hss)
    # koridorun icinden gecmis olmali (merkez hattina yakin)
    assert min(geo.haversine_distance(w, CENTER) for w in res.waypoints) < 120.0
    # performans butcesi (CI payli)
    assert elapsed < 2.0


def test_planner_sealed_corridor_reduced_graph_rescue():
    p = PathPlanner(PlannerConfig(dubins_smoothing_enabled=False))
    res = p.plan(_pos(270.0, 1500.0), _pos(90.0, 1500.0), BND,
                 _sealed_corridor_zones(), speed_ms=20.0)
    assert res.success, res.error
    assert res.stage == "graph_reduced"


def test_planner_goal_heading_arrival():
    p = PathPlanner(PlannerConfig())
    start = _pos(270.0, 1500.0)
    goal = _pos(90.0, 1500.0)
    res = p.plan(start, goal, BND, [], start_heading_deg=90.0,
                 goal_heading_deg=180.0, speed_ms=22.0)
    assert res.success, res.error
    assert res.stage == "hybrid"
    arrival = geo.bearing(res.waypoints[-2], res.waypoints[-1])
    assert abs(geo.angle_difference(arrival, 180.0)) < 30.0


def test_planner_speed_dependent_turn_radius():
    p = PathPlanner(PlannerConfig(min_turn_radius_m=50.0, max_bank_deg=35.0))
    assert p.turn_radius_for(None) == 50.0
    assert p.turn_radius_for(10.0) == 50.0          # min alti
    r30 = p.turn_radius_for(30.0)                   # v^2/(g*tan35) ~ 131m
    assert 120.0 < r30 < 145.0


def test_planner_signature_preference_keeps_side():
    p = PathPlanner(PlannerConfig())
    hss = [HSSZone(id=1, center=CENTER, radius=200, active=True)]
    start = _pos(270.0, 1200.0)
    goal = _pos(90.0, 1200.0)
    first = p.plan(start, goal, BND, hss, start_heading_deg=90.0, speed_ms=20.0)
    assert first.success and first.signature
    # ayni problemi onceki imza tercihiyle tekrar coz -> ayni yan
    again = p.plan(start, goal, BND, hss, start_heading_deg=90.0, speed_ms=20.0,
                   prefer_signature=first.signature)
    assert again.success
    assert again.signature == first.signature


def test_planner_direct_margin_allows_corridor_pursuit():
    p = PathPlanner(PlannerConfig())
    hss = [
        HSSZone(id=1, center=_pos(0.0, 305.0), radius=150, active=True),
        HSSZone(id=2, center=_pos(180.0, 305.0), radius=150, active=True),
    ]
    a = _pos(270.0, 600.0)
    b = _pos(90.0, 600.0)
    # konfor marjiyla koridor kapali, direct (pursuit) marjiyla acik
    assert not p.is_direct_clear(a, b, BND, hss)
    assert p.is_direct_clear(a, b, BND, hss, margin_m=p.direct_margin())


# ---------------- waypoint chain: along-track gecis ----------------

def test_chain_advances_when_passed_along_track():
    chain = WaypointChain()
    wp1 = _pos(0.0, 500.0)
    wp2 = _pos(0.0, 1000.0)
    chain.load([Waypoint(wp1, acceptance_radius_m=50),
                Waypoint(wp2, acceptance_radius_m=50)])
    # bacak baslangici CENTER olur
    assert chain.update(CENTER) == ChainStatus.IN_PROGRESS
    # WP1'in 80m YANINDAN gecip dikmesinin OTESINE dustuk (mesafe > acceptance)
    passed = geo.destination_point(wp1, 0.0, 30.0)      # 30m ileri
    passed = geo.destination_point(passed, 90.0, 80.0)  # 80m yana
    passed.altitude = 100.0
    assert geo.haversine_distance(passed, wp1) > 50.0
    assert chain.update(passed) == ChainStatus.WAYPOINT_REACHED
    assert chain.current_target().position == wp2


def test_chain_no_advance_when_far_cross_track():
    chain = WaypointChain()
    wp1 = _pos(0.0, 500.0)
    chain.load([Waypoint(wp1, acceptance_radius_m=50)])
    chain.update(CENTER)
    # dikmenin otesinde ama 300m yanal sapma -> gecis sayilmaz
    wide = geo.destination_point(wp1, 0.0, 50.0)
    wide = geo.destination_point(wide, 90.0, 300.0)
    wide.altitude = 100.0
    assert chain.update(wide) == ChainStatus.IN_PROGRESS

"""nav katmani: planner (HSS detour), waypoint_chain, navigation_engine."""
import math

import pytest

from uav.core import geo
from uav.core.types import Boundary, HSSZone, Position
from uav.nav.planner import PathPlanner, PlannerConfig
from uav.nav.speed_manager import SpeedManager
from uav.nav.waypoint_chain import ChainStatus, Waypoint, WaypointChain

# Buyuk bos alan boundary (~6km kare), merkez ~ (37.82, -122.35)
BND = Boundary(vertices=[
    Position(37.79, -122.39), Position(37.85, -122.39),
    Position(37.85, -122.31), Position(37.79, -122.31),
], min_altitude=30, max_altitude=400)
CENTER = Position(37.82, -122.35, 100.0)


def _cfg():
    return PlannerConfig(hss_buffer_m=40, boundary_buffer_m=20,
                         min_turn_radius_m=50, turn_safety_margin_m=50,
                         max_segment_length_m=120)


def test_planner_direct_when_clear():
    p = PathPlanner(_cfg())
    start = geo.destination_point(CENTER, 270.0, 1500.0)
    goal = geo.destination_point(CENTER, 90.0, 1500.0)
    # ortada HSS yok -> direkt
    res = p.plan(start, goal, BND, [])
    assert res.success
    assert len(res.waypoints) == 2


def test_planner_detours_around_hss():
    p = PathPlanner(_cfg())
    start = geo.destination_point(CENTER, 270.0, 1500.0)
    goal = geo.destination_point(CENTER, 90.0, 1500.0)
    # tam ortada, duz hatti kesen HSS
    hss = [HSSZone(id=1, center=CENTER, radius=200, active=True)]
    res = p.plan(start, goal, BND, hss)
    assert res.success, res.error
    assert len(res.waypoints) > 2  # detour eklendi
    # HARD guvenli: hicbir nokta HSS+buffer icinde degil
    for wp in res.waypoints:
        assert geo.haversine_distance(wp, CENTER) >= 200 + 40 - 1.0
    # hicbir segment HSS+buffer dairesini kesmiyor
    assert p.is_path_safe(res.waypoints, BND, hss)


def test_planner_keeps_escape_prevention_clearance():
    p = PathPlanner(PlannerConfig(
        hss_buffer_m=40,
        boundary_buffer_m=20,
        min_turn_radius_m=50,
        turn_safety_margin_m=50,
        escape_prevention_margin_m=80,
        max_segment_length_m=120,
    ))
    start = geo.destination_point(CENTER, 270.0, 1500.0)
    goal = geo.destination_point(CENTER, 90.0, 1500.0)
    hss = [HSSZone(id=1, center=CENTER, radius=200, active=True)]

    res = p.plan(start, goal, BND, hss)

    assert res.success, res.error
    min_clearance = 200 + p.planning_margin()
    for wp in res.waypoints:
        assert geo.haversine_distance(wp, CENTER) >= min_clearance - 3.0


def test_planner_can_exit_soft_margin_when_hard_safe():
    p = PathPlanner(PlannerConfig(
        hss_buffer_m=40,
        boundary_buffer_m=20,
        min_turn_radius_m=50,
        turn_safety_margin_m=50,
        escape_prevention_margin_m=80,
        max_segment_length_m=120,
    ))
    # Hard guvenli mesafe 240m, soft planlama koridoru 370m civari.
    start = geo.destination_point(CENTER, 270.0, 300.0)
    goal = geo.destination_point(CENTER, 90.0, 1500.0)
    hss = [HSSZone(id=1, center=CENTER, radius=200, active=True)]

    res = p.plan(start, goal, BND, hss)

    assert res.success, res.error
    assert p.is_path_safe(res.waypoints, BND, hss)


def test_planner_segments_clear_hss_hard():
    p = PathPlanner(_cfg())
    start = geo.destination_point(CENTER, 225.0, 1800.0)
    goal = geo.destination_point(CENTER, 45.0, 1800.0)
    hss = [HSSZone(id=1, center=CENTER, radius=300, active=True)]
    res = p.plan(start, goal, BND, hss)
    assert res.success, res.error
    # her segmenti orta noktalardan ornekle: hepsi guvenli mesafede
    for i in range(1, len(res.waypoints)):
        a, b = res.waypoints[i - 1], res.waypoints[i]
        for t in (0.25, 0.5, 0.75):
            ned = geo.position_to_ned(b, a)
            sample = geo.ned_to_position(geo.NED(ned.north * t, ned.east * t, 0), a)
            assert geo.haversine_distance(sample, CENTER) >= 300 + 40 - 5.0


def test_planner_make_safe_waypoint_list():
    p = PathPlanner(_cfg())
    wp_list = [
        geo.destination_point(CENTER, 270.0, 1500.0),
        geo.destination_point(CENTER, 90.0, 1500.0),   # bu bacak HSS'i keser
        geo.destination_point(CENTER, 0.0, 1500.0),
    ]
    hss = [HSSZone(id=1, center=CENTER, radius=250, active=True)]
    res = p.make_safe(wp_list, BND, hss)
    assert res.success, res.error
    assert p.is_path_safe(res.waypoints, BND, hss)
    # verilen waypoint'ler korunuyor (ilk ve son dahil)
    assert geo.haversine_distance(res.waypoints[0], wp_list[0]) < 1.0
    assert geo.haversine_distance(res.waypoints[-1], wp_list[-1]) < 1.0


def test_planner_two_hss_corridor():
    p = PathPlanner(_cfg())
    start = geo.destination_point(CENTER, 270.0, 2000.0)
    goal = geo.destination_point(CENTER, 90.0, 2000.0)
    north = geo.destination_point(CENTER, 0.0, 350.0)
    south = geo.destination_point(CENTER, 180.0, 350.0)
    hss = [
        HSSZone(id=1, center=north, radius=200, active=True),
        HSSZone(id=2, center=south, radius=200, active=True),
    ]
    res = p.plan(start, goal, BND, hss)
    assert res.success, res.error
    assert p.is_path_safe(res.waypoints, BND, hss)


def test_is_direct_clear():
    p = PathPlanner(_cfg())
    start = geo.destination_point(CENTER, 270.0, 1500.0)
    goal = geo.destination_point(CENTER, 90.0, 1500.0)
    assert p.is_direct_clear(start, goal, BND, [])
    hss = [HSSZone(id=1, center=CENTER, radius=200, active=True)]
    assert not p.is_direct_clear(start, goal, BND, hss)


# ---------------- WaypointChain ----------------

def test_waypoint_chain_progress():
    chain = WaypointChain()
    a = geo.destination_point(CENTER, 0.0, 500.0)
    b = geo.destination_point(CENTER, 0.0, 1000.0)
    chain.load([Waypoint(a, acceptance_radius_m=80), Waypoint(b, acceptance_radius_m=80)])
    assert chain.update(CENTER) == ChainStatus.IN_PROGRESS
    assert chain.update(a) == ChainStatus.WAYPOINT_REACHED
    assert chain.current_target().position == b
    assert chain.update(b) == ChainStatus.CHAIN_COMPLETE
    assert chain.is_complete()


def test_waypoint_chain_lookahead_can_latch_next_target():
    chain = WaypointChain()
    a = geo.destination_point(CENTER, 0.0, 500.0)
    b = geo.destination_point(CENTER, 0.0, 1000.0)
    chain.load([Waypoint(a, acceptance_radius_m=80), Waypoint(b, acceptance_radius_m=80)])

    assert chain.peek_next().position == b
    assert chain.advance_to_next()
    assert chain.current_target().position == b


def test_speed_manager_clamp():
    sm = SpeedManager()
    assert sm.cruise_speed() == 20.0
    assert sm.clamp(100.0) == 35.0
    assert sm.clamp(1.0) == 12.0


# ---------------- NavigationEngine (GUIDED sequencing) ----------------

def _make_engine():
    from uav.comm.flight_commander import ArduPilotMode, FlightCommander
    from uav.nav.navigation_engine import NavigationEngine
    from uav.safety.safety_gate import SafetyGate, SafetyGateConfig
    from uav.safety.zone_checker import ZoneChecker
    from uav.world_model.world_model import WorldModel
    from uav.core.types import UAVState
    from tests.test_comm import FakeIO

    wm = WorldModel()
    wm.set_boundary(BND)
    own = UAVState(id=0, position=geo.destination_point(CENTER, 270.0, 1500.0),
                   heading=90.0, timestamp_us=1, gps_fix_type=3, battery_remaining=80)
    wm.set_own_state(own)
    io = FakeIO()
    fc = FlightCommander(io)
    fc.update_mode_from_heartbeat(int(ArduPilotMode.GUIDED))
    gate = SafetyGate(ZoneChecker(), wm, SafetyGateConfig())
    eng = NavigationEngine(PathPlanner(_cfg()), fc, gate, SpeedManager(), wm,
                           min_cmd_interval_s=0.0)
    return eng, io, wm, own


def test_engine_plan_route_sends_reposition():
    from pymavlink import mavutil
    eng, io, wm, own = _make_engine()
    goal = geo.destination_point(CENTER, 90.0, 1500.0)
    assert eng.plan_route(goal, airspeed=22.0)
    assert len(io.command_int) >= 1
    assert io.command_int[-1][3] == mavutil.mavlink.MAV_CMD_DO_REPOSITION
    assert eng.has_active_route()


def test_engine_make_safe_route_detours():
    eng, io, wm, own = _make_engine()
    wm.update_hss_zone(HSSZone(id=1, center=CENTER, radius=250, active=True))
    goal = geo.destination_point(CENTER, 90.0, 1500.0)
    assert eng.make_safe_route([goal])
    # cok bacakli zincir olusmali (detour)
    assert eng.has_active_route()


def test_engine_clamps_unsafe_goal():
    eng, io, wm, own = _make_engine()
    wm.update_hss_zone(HSSZone(id=1, center=CENTER, radius=200, active=True))
    # hedef HSS icinde -> clamp + (gerekirse) plan; en azindan cokmeden donmeli
    res = eng.plan_route(CENTER, airspeed=20.0)
    assert isinstance(res, bool)


def test_engine_fly_velocity_ned_sends_when_safe():
    from pymavlink import mavutil
    eng, io, wm, own = _make_engine()
    # kuzeye 15 m/s; ufuk*hiz kadar ileri segment bos alanda -> guvenli
    assert eng.fly_velocity_ned(15.0, 0.0, 0.0)
    args = io.local_ned[-1]
    mm = mavutil.mavlink
    assert args[2] == mm.MAV_FRAME_LOCAL_NED
    assert args[7] == 15.0                       # vx
    assert args[3] & mm.POSITION_TARGET_TYPEMASK_X_IGNORE  # pozisyon ignore


def test_engine_fly_velocity_rejected_when_unsafe():
    eng, io, wm, own = _make_engine()
    # uca demand'in yansiyacagi bolgeyi kaplayan buyuk HSS -> segment guvensiz
    wm.update_hss_zone(HSSZone(id=2, center=own.position, radius=500, active=True))
    assert not eng.fly_velocity_ned(15.0, 0.0, 0.0)
    assert len(io.local_ned) == 0


def test_engine_dispatch_velocity_frame():
    from uav.perception.perception_types import PerceptionFrame, PerceptionTarget
    eng, io, wm, own = _make_engine()
    tgt = PerceptionTarget(frame=PerceptionFrame.VELOCITY_NED, a=0.0, b=12.0, c=0.0)
    assert eng.dispatch_perception_target(tgt)
    assert io.local_ned[-1][8] == 12.0           # vy (doguya)

"""mission katmani: director gecis kurallari, follow PN, qr_dive faz makinesi."""
import math

import pytest

from uav.comm.flight_commander import ArduPilotMode, FlightCommander
from uav.core import geo
from uav.core.types import (
    Boundary,
    HSSZone,
    MissionMode,
    Position,
    UAVState,
    Velocity,
)
from uav.infra.event_bus import EventBus
from uav.mission.follow_mode import FollowConfig, FollowMode, FollowPhase
from uav.mission.mission_director import MissionConfig, MissionDirector, PostDiveAction
from uav.mission.qr_dive_mode import DivePhase, QRDiveConfig, QRDiveMode
from uav.nav.navigation_engine import NavigationEngine
from uav.nav.planner import PathPlanner, PlannerConfig
from uav.nav.speed_manager import SpeedManager
from uav.safety.safety_gate import SafetyGate, SafetyGateConfig
from uav.safety.zone_checker import ZoneChecker
from uav.world_model.world_model import WorldModel
from tests.test_comm import FakeIO

BND = Boundary(vertices=[
    Position(37.79, -122.39), Position(37.85, -122.39),
    Position(37.85, -122.31), Position(37.79, -122.31),
], min_altitude=30, max_altitude=400)
CENTER = Position(37.82, -122.35, 100.0)


def _stack(own_pos=None, guided=True):
    wm = WorldModel()
    wm.set_boundary(BND)
    own = UAVState(id=0, position=own_pos or geo.destination_point(CENTER, 270.0, 1500.0),
                   heading=90.0, airspeed=20.0, timestamp_us=1,
                   gps_fix_type=3, battery_remaining=80)
    wm.set_own_state(own)
    io = FakeIO()
    fc = FlightCommander(io)
    if guided:
        fc.update_mode_from_heartbeat(int(ArduPilotMode.GUIDED))
    gate = SafetyGate(ZoneChecker(), wm, SafetyGateConfig())
    nav = NavigationEngine(PathPlanner(PlannerConfig()), fc, gate, SpeedManager(), wm,
                           min_cmd_interval_s=0.0)
    return wm, fc, io, nav


# ---------------- MissionDirector ----------------

def _director(guided=True):
    wm, fc, io, nav = _stack(guided=guided)
    bus = EventBus()
    md = MissionDirector(nav, fc, wm, bus, MissionConfig())
    md.initialize()
    return md, wm, fc, io


def test_director_starts_in_ardupilot():
    md, *_ = _director()
    assert md.get_current_mode() == MissionMode.ARDUPILOT


def test_director_transition_rules():
    md, wm, fc, io = _director()
    # ARDUPILOT -> PATROL izinli
    assert md.request_mode(MissionMode.PATROL)
    assert md.get_current_mode() == MissionMode.PATROL
    # PATROL -> FOLLOW izinli
    md.set_follow_target(5)
    assert md.request_mode(MissionMode.FOLLOW)
    # FOLLOW -> QR_DIVE izinli
    md.set_dive_target(CENTER)
    assert md.request_mode(MissionMode.QR_DIVE)
    # QR_DIVE -> FOLLOW YASAK
    assert not md.request_mode(MissionMode.FOLLOW)
    # QR_DIVE -> PATROL izinli
    assert md.request_mode(MissionMode.PATROL)


def test_director_guided_sent_on_transition():
    from pymavlink import mavutil
    md, wm, fc, io = _director()
    md.request_mode(MissionMode.PATROL)
    set_modes = [a for a in io.command_long if a[2] == mavutil.mavlink.MAV_CMD_DO_SET_MODE]
    # GUIDED'e zaten gectik (guided=True) ama PATROL girişinde tekrar gerekmiyor;
    # guided degilken test:
    md2, wm2, fc2, io2 = _director(guided=False)
    md2.request_mode(MissionMode.PATROL)
    set_modes2 = [a for a in io2.command_long if a[2] == mavutil.mavlink.MAV_CMD_DO_SET_MODE]
    assert len(set_modes2) >= 1  # GUIDED komutu gonderildi


def test_director_rtl():
    md, wm, fc, io = _director()
    from pymavlink import mavutil
    md.request_mode(MissionMode.PATROL)
    md.trigger_rtl("test")
    assert md.get_current_mode() == MissionMode.ARDUPILOT
    set_modes = [a for a in io.command_long if a[2] == mavutil.mavlink.MAV_CMD_DO_SET_MODE]
    # son set_mode RTL olmali (p2 = custom mode, index 4)
    assert set_modes[-1][4] == float(int(ArduPilotMode.RTL))


# ---------------- FollowMode PN ----------------

class FakeFollowNav:
    def __init__(self, direct_safe=True):
        self.direct_safe = direct_safe
        self.active_route = False
        self.planned = []
        self.direct = []
        self.updates = 0
        self.clears = 0

    def can_fly_direct(self, target, escape=False):
        return self.direct_safe

    def plan_route(self, goal, airspeed=-1.0):
        self.planned.append((goal, airspeed))
        self.active_route = True
        return True

    def make_safe_route(self, waypoints, airspeed=-1.0):
        self.planned.append((list(waypoints), airspeed))
        self.active_route = True
        return True

    def has_active_route(self):
        return self.active_route

    def update(self, own_pos):
        self.updates += 1

    def fly_direct(self, target, airspeed=-1.0):
        self.direct.append((target, airspeed))
        return True

    def clear_route(self):
        self.active_route = False
        self.clears += 1

    def release_guided_hold(self):
        return True


def test_follow_approach_to_pursuit():
    wm, fc, io, nav = _stack()
    fm = FollowMode(nav, wm)
    fm.set_target_id(9)
    fm.on_enter()
    own = wm.get_own_state()
    # hedef 100m ileride (pursuit menzilinde) -> PURSUIT
    target_pos = geo.destination_point(own.position, 90.0, 100.0)
    wm.update_opponent(UAVState(id=9, position=target_pos, velocity=Velocity(5, 0, 0),
                                timestamp_us=__import__("time").time_ns() // 1000))
    fm.tick(own)
    assert fm.get_phase() == FollowPhase.PURSUIT


def test_follow_does_not_enter_direct_pursuit_when_hss_blocks_segment():
    wm, fc, io, nav = _stack()
    fake_nav = FakeFollowNav(direct_safe=False)
    fm = FollowMode(fake_nav, wm)
    fm.set_target_id(9)
    fm.on_enter()
    own = wm.get_own_state()
    target_pos = geo.destination_point(own.position, 90.0, 100.0)
    wm.update_opponent(UAVState(id=9, position=target_pos, velocity=Velocity(5, 0, 0),
                                timestamp_us=__import__("time").time_ns() // 1000))

    fm.tick(own)

    assert fm.get_phase() == FollowPhase.APPROACH
    assert len(fake_nav.planned) == 1
    assert fake_nav.direct == []


def test_follow_route_lock_suppresses_small_target_replans():
    wm, fc, io, nav = _stack()
    fake_nav = FakeFollowNav(direct_safe=False)
    fm = FollowMode(fake_nav, wm, FollowConfig(
        route_lock_s=60.0,
        route_goal_update_m=20.0,
        route_force_replan_m=300.0,
    ))
    fm.set_target_id(9)
    fm.on_enter()
    own = wm.get_own_state()
    target_pos = geo.destination_point(own.position, 90.0, 1000.0)
    wm.update_opponent(UAVState(id=9, position=target_pos, velocity=Velocity(5, 0, 0),
                                timestamp_us=__import__("time").time_ns() // 1000))
    fm.tick(own)

    moved_target = geo.destination_point(target_pos, 0.0, 80.0)
    wm.update_opponent(UAVState(id=9, position=moved_target, velocity=Velocity(5, 0, 0),
                                timestamp_us=__import__("time").time_ns() // 1000))
    fm.tick(own)

    assert len(fake_nav.planned) == 1
    assert fake_nav.updates == 1


def test_follow_approach_plans_committed_waypoint_chain():
    wm, fc, io, nav = _stack()
    fake_nav = FakeFollowNav(direct_safe=False)
    fm = FollowMode(fake_nav, wm, FollowConfig(
        approach_standoff_m=150.0,
        route_chain_min_spacing_m=120.0,
    ))
    fm.set_target_id(9)
    fm.on_enter()
    own = wm.get_own_state()
    target_pos = geo.destination_point(own.position, 90.0, 1000.0)
    wm.update_opponent(UAVState(id=9, position=target_pos, velocity=Velocity(5, 0, 0),
                                timestamp_us=__import__("time").time_ns() // 1000))

    fm.tick(own)

    planned_goals, speed = fake_nav.planned[0]
    assert isinstance(planned_goals, list)
    assert len(planned_goals) == 2
    assert speed == pytest.approx(fm.cfg.approach_speed_ms)


def test_follow_commits_active_route_before_entering_pursuit():
    wm, fc, io, nav = _stack()
    fake_nav = FakeFollowNav(direct_safe=True)
    fake_nav.active_route = True
    fm = FollowMode(fake_nav, wm)
    fm.set_target_id(9)
    fm.on_enter()
    own = wm.get_own_state()
    target_pos = geo.destination_point(own.position, 90.0, 100.0)
    wm.update_opponent(UAVState(id=9, position=target_pos, velocity=Velocity(5, 0, 0),
                                timestamp_us=__import__("time").time_ns() // 1000))

    fm.tick(own)

    assert fm.get_phase() == FollowPhase.APPROACH
    assert fake_nav.updates == 1
    assert fake_nav.direct == []
    assert fake_nav.planned == []


def test_follow_commit_suppresses_force_replan_while_route_active():
    wm, fc, io, nav = _stack()
    fake_nav = FakeFollowNav(direct_safe=False)
    fake_nav.active_route = True
    fm = FollowMode(fake_nav, wm, FollowConfig(route_force_replan_m=1.0))
    fm.set_target_id(9)
    fm.on_enter()
    own = wm.get_own_state()
    target_pos = geo.destination_point(own.position, 90.0, 1500.0)
    wm.update_opponent(UAVState(id=9, position=target_pos, velocity=Velocity(5, 0, 0),
                                timestamp_us=__import__("time").time_ns() // 1000))

    fm.tick(own)

    assert fake_nav.updates == 1
    assert fake_nav.planned == []


def test_follow_direct_pursuit_deadband_suppresses_small_command_changes():
    wm, fc, io, nav = _stack()
    fake_nav = FakeFollowNav(direct_safe=True)
    fm = FollowMode(fake_nav, wm, FollowConfig(
        direct_command_interval_s=10.0,
        direct_goal_update_m=50.0,
    ))
    fm.set_target_id(9)
    fm.on_enter()
    own = wm.get_own_state()
    target_pos = geo.destination_point(own.position, 90.0, 100.0)
    wm.update_opponent(UAVState(id=9, position=target_pos, velocity=Velocity(5, 0, 0),
                                timestamp_us=__import__("time").time_ns() // 1000))
    fm.tick(own)

    moved_target = geo.destination_point(target_pos, 0.0, 20.0)
    wm.update_opponent(UAVState(id=9, position=moved_target, velocity=Velocity(5, 0, 0),
                                timestamp_us=__import__("time").time_ns() // 1000))
    fm.tick(own)

    assert fm.get_phase() == FollowPhase.PURSUIT
    assert len(fake_nav.direct) == 1


def test_follow_reacquire_when_target_stale():
    wm, fc, io, nav = _stack()
    fm = FollowMode(nav, wm)
    fm.set_target_id(9)
    fm.on_enter()
    own = wm.get_own_state()
    # rakip yok -> REACQUIRE
    fm.tick(own)
    assert fm.get_phase() == FollowPhase.REACQUIRE


# ---------------- QRDiveMode faz makinesi ----------------

def test_qr_dive_phases():
    wm, fc, io, nav = _stack()
    qd = QRDiveMode(nav, wm, fc, QRDiveConfig(entry_corridor_m=500, approach_altitude_m=120))
    qd.set_dive_target(CENTER)
    qd.on_enter()
    assert qd.get_phase() == DivePhase.APPROACH
    assert qd._entry_valid
    # entry point hedefin yaklasim-tersi yonunde ~500m
    assert geo.haversine_distance(qd._entry_point, CENTER) == pytest.approx(500, rel=0.05)


def test_qr_dive_hss_abort():
    wm, fc, io, nav = _stack()
    # DIVE fazinda HSS yakininda -> CLIMB
    qd = QRDiveMode(nav, wm, fc, QRDiveConfig())
    qd.set_dive_target(CENTER)
    qd.on_enter()
    qd._phase = DivePhase.DIVE
    wm.update_hss_zone(HSSZone(id=1, center=CENTER, radius=100, active=True))
    own = UAVState(id=0, position=geo.destination_point(CENTER, 0.0, 110.0),
                   heading=180.0, timestamp_us=1)
    wm.set_own_state(own)
    qd.tick(own)
    assert qd.get_phase() == DivePhase.CLIMB


def test_qr_dive_completion_callback():
    wm, fc, io, nav = _stack()
    fired = []
    qd = QRDiveMode(nav, wm, fc, QRDiveConfig(climb_altitude_m=100))
    qd.set_dive_target(CENTER)
    qd.set_completion_callback(lambda: fired.append(True))
    qd.on_enter()
    qd._phase = DivePhase.CLIMB
    # yeterli irtifada -> COMPLETED + callback
    own = UAVState(id=0, position=Position(CENTER.latitude, CENTER.longitude, 105.0),
                   timestamp_us=1)
    qd.tick(own)
    assert qd.get_phase() == DivePhase.COMPLETED
    assert fired == [True]

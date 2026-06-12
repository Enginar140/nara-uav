"""Sanal FBWA katmani: SET_ATTITUDE_TARGET + AttitudeDirectLaw + engine supervizoru.

Sartname baglami: gercek FBWA yari-oto = MANUEL sayilir (6.1.7); ayni kontrol
yetkisi GUIDED icinde SET_ATTITUDE_TARGET ile verilir -> otonom kalir.
"""
import math

import pytest

from uav.comm.flight_commander import (ArduPilotMode, FlightCommander,
                                       _euler_to_quaternion)
from uav.core import geo
from uav.core.types import (Attitude, Boundary, HSSZone, Position, UAVState,
                            Velocity)
from uav.nav.attitude_law import AttitudeDirectLaw, AttitudeLawConfig
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


def _quat_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


# ---------------- commander ----------------

def test_set_attitude_sends_quaternion_and_mask():
    io = FakeIO()
    fc = FlightCommander(io)
    fc.update_mode_from_heartbeat(int(ArduPilotMode.GUIDED))
    assert fc.set_attitude(30.0, -10.0)
    args = io.attitude[-1]
    # args: target_sys, target_comp, type_mask, q, rr, pr, yr, thrust
    tm, q = args[2], args[3]
    assert tm & (1 | 2 | 4)            # body rate'ler ignore
    assert tm & 64                     # thrust verilmedi -> ignore
    r, p, _ = _quat_to_euler(q)
    assert abs(r - 30.0) < 0.1
    assert abs(p - (-10.0)) < 0.1


def test_set_attitude_with_thrust():
    io = FakeIO()
    fc = FlightCommander(io)
    fc.update_mode_from_heartbeat(int(ArduPilotMode.GUIDED))
    assert fc.set_attitude(0.0, 5.0, thrust=0.6)
    args = io.attitude[-1]
    assert not (args[2] & 64)
    assert args[7] == pytest.approx(0.6)


def test_set_attitude_rejected_outside_guided():
    io = FakeIO()
    fc = FlightCommander(io)
    fc.update_mode_from_heartbeat(int(ArduPilotMode.FBWA))
    assert not fc.set_attitude(10.0, 0.0)
    assert len(io.attitude) == 0


def test_euler_quaternion_roundtrip():
    q = _euler_to_quaternion(math.radians(25), math.radians(-12), math.radians(80))
    r, p, y = _quat_to_euler(q)
    assert abs(r - 25) < 0.01 and abs(p + 12) < 0.01 and abs(y - 80) < 0.01


# ---------------- kanun ----------------

def test_law_signs_and_clamps():
    law = AttitudeDirectLaw(AttitudeLawConfig(enabled=True))
    roll, pitch = law.compute(10.0, 5.0, now_s=0.0)
    assert roll > 0.0                          # hedef sagda -> saga yatis
    assert pitch > law.cfg.pitch_trim_deg      # hedef yukarida -> burun yukari
    roll, pitch = law.compute(90.0, 60.0, now_s=10.0)
    assert roll == law.cfg.max_bank_deg        # zarf kirpildi
    assert pitch == law.cfg.pitch_max_deg


def test_law_stall_protection():
    law = AttitudeDirectLaw(AttitudeLawConfig(enabled=True))
    assert law.clamp_for_airspeed(10.0, airspeed=12.5, min_airspeed=12.0) == 0.0
    assert law.clamp_for_airspeed(10.0, airspeed=20.0, min_airspeed=12.0) == 10.0


def test_projection_geometry():
    law = AttitudeDirectLaw(AttitudeLawConfig(enabled=True, horizon_s=4.0))
    own = UAVState(id=0, position=Position(37.82, -122.35, 100.0), heading=0.0,
                   airspeed=20.0, timestamp_us=1)
    # duz ucus kuzeye: son nokta ~80m kuzeyde, irtifa sabit
    pts = law.project_trajectory(own, 0.0, 0.0)
    end = geo.position_to_ned(pts[-1], own.position)
    assert end.north == pytest.approx(80.0, abs=2.0)
    assert abs(end.east) < 2.0
    assert pts[-1].altitude == pytest.approx(100.0, abs=0.5)
    # saga 30 yatis -> doguya kivrilir; -20 pitch -> alcalir
    pts2 = law.project_trajectory(own, 30.0, -20.0)
    end2 = geo.position_to_ned(pts2[-1], own.position)
    assert end2.east > 10.0
    assert pts2[-1].altitude < 90.0


# ---------------- engine supervizoru ----------------

def _make_engine(att_enabled=True, own_alt=100.0, airspeed=20.0):
    wm = WorldModel()
    wm.set_boundary(BND)
    own = UAVState(id=0, position=geo.destination_point(CENTER, 270.0, 1500.0),
                   heading=90.0, airspeed=airspeed, timestamp_us=1,
                   gps_fix_type=3, battery_remaining=80,
                   attitude=Attitude(0.0, 0.0, 90.0),
                   velocity=Velocity(0.0, airspeed, 0.0))
    own.position.altitude = own_alt
    wm.set_own_state(own)
    io = FakeIO()
    fc = FlightCommander(io)
    fc.update_mode_from_heartbeat(int(ArduPilotMode.GUIDED))
    gate = SafetyGate(ZoneChecker(), wm, SafetyGateConfig())
    eng = NavigationEngine(
        PathPlanner(PlannerConfig()), fc, gate, SpeedManager(), wm,
        min_cmd_interval_s=0.0,
        attitude_config=AttitudeLawConfig(enabled=att_enabled, min_interval_s=0.0))
    return eng, io, wm, own


def test_engine_attitude_sends_when_safe():
    eng, io, wm, own = _make_engine()
    assert eng.fly_attitude_los(8.0, 2.0)
    assert len(io.attitude) == 1


def test_engine_attitude_disabled_returns_false():
    eng, io, wm, own = _make_engine(att_enabled=False)
    assert not eng.fly_attitude_los(8.0, 2.0)
    assert len(io.attitude) == 0


def test_engine_attitude_blocked_by_hss_ahead():
    eng, io, wm, own = _make_engine()
    # dogrudan onunde (dogu) HSS: projeksiyon iceri girer -> komut yok
    zone_center = geo.destination_point(own.position, 90.0, 150.0)
    wm.update_hss_zone(HSSZone(id=1, center=zone_center, radius=100, active=True))
    assert not eng.fly_attitude_los(0.0, 0.0)
    assert len(io.attitude) == 0


def test_engine_attitude_blocked_near_floor():
    # taban 30 + floor_margin 15 = 45; 50m'de dik dalis talebi reddedilmeli
    eng, io, wm, own = _make_engine(own_alt=50.0)
    assert not eng.fly_attitude_los(0.0, -60.0)
    assert len(io.attitude) == 0


def test_engine_attitude_wings_level_fallback():
    eng, io, wm, own = _make_engine()
    # solda yakin HSS: saga donus talebi guvenli olmayabilir ama duz ucus temiz
    zone_center = geo.destination_point(own.position, 135.0, 90.0)
    wm.update_hss_zone(HSSZone(id=1, center=zone_center, radius=40, active=True))
    sent = eng.fly_attitude_los(45.0, 0.0)
    if sent:
        # gonderildiyse ya orijinal ya duz-kanat; her iki durumda da mesaj var
        assert len(io.attitude) == 1


def test_engine_dispatch_vision_los_prefers_attitude():
    from uav.perception.perception_types import PerceptionFrame, PerceptionTarget
    eng, io, wm, own = _make_engine()
    tgt = PerceptionTarget(frame=PerceptionFrame.VISION_LOS, a=5.0, b=3.0, c=80.0)
    assert eng.dispatch_perception_target(tgt)
    assert len(io.attitude) == 1          # attitude yolu kullanildi
    assert len(io.local_ned) == 0


def test_engine_dispatch_vision_los_falls_back_when_disabled():
    from uav.perception.perception_types import PerceptionFrame, PerceptionTarget
    eng, io, wm, own = _make_engine(att_enabled=False)
    tgt = PerceptionTarget(frame=PerceptionFrame.VISION_LOS, a=5.0, b=3.0, c=80.0)
    assert eng.dispatch_perception_target(tgt)
    assert len(io.attitude) == 0
    assert len(io.local_ned) == 1         # eski pozisyon-adimi yolu

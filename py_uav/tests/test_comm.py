"""comm katmani: FlightCommander (sahte IO) + StatusTextParser testleri."""
import math

from pymavlink import mavutil

from uav.comm.flight_commander import ArduPilotMode, FlightCommander
from uav.comm.mavlink_bridge import MavlinkBridge
from uav.comm.mavlink_io import MavlinkIO
from uav.comm.statustext_parser import StatusTextParser, StatusTextType
from uav.core.types import HSSZone, MissionMode, Position
from uav.infra.event_bus import EventBus
from uav.world_model.world_model import WorldModel

m = mavutil.mavlink


class FakeIO:
    """MavlinkIO'nun gonderme arayuzunu taklit eder; cagrilari kaydeder."""

    def __init__(self):
        self.command_int = []
        self.command_long = []
        self.local_ned = []

    def send_command_int(self, *args):
        self.command_int.append(args)
        return True

    def send_command_long(self, *args):
        self.command_long.append(args)
        return True

    def send_set_position_target_local_ned(self, *args):
        self.local_ned.append(args)
        return True


def make_commander(guided=True):
    io = FakeIO()
    fc = FlightCommander(io)
    if guided:
        fc.update_mode_from_heartbeat(int(ArduPilotMode.GUIDED))
    return fc, io


def test_fly_to_position_do_reposition():
    fc, io = make_commander()
    assert fc.fly_to_position(Position(37.8, -122.3, 100.0), ground_speed_ms=20.0)
    assert len(io.command_int) == 1
    args = io.command_int[0]
    # args: target_sys, target_comp, frame, command, p1, p2, p3, p4, x, y, z
    assert args[3] == m.MAV_CMD_DO_REPOSITION
    assert args[2] == m.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    assert args[4] == 20.0                       # ground speed
    assert args[8] == int(37.8 * 1e7)            # lat
    assert args[9] == int(-122.3 * 1e7)          # lon
    assert args[10] == 100.0                     # alt


def test_fly_to_position_duplicate_suppressed():
    fc, io = make_commander()
    fc.fly_to_position(Position(37.8, -122.3, 100.0))
    fc.fly_to_position(Position(37.8, -122.3, 100.0))  # ayni -> bastir
    assert len(io.command_int) == 1
    # yeterince uzak -> yeni komut
    fc.fly_to_position(Position(37.9, -122.4, 100.0))
    assert len(io.command_int) == 2


def test_body_offset_requires_guided():
    fc, io = make_commander(guided=False)
    assert not fc.fly_to_body_offset(100, 0, -20)
    assert len(io.local_ned) == 0
    fc.update_mode_from_heartbeat(int(ArduPilotMode.GUIDED))
    assert fc.fly_to_body_offset(100, 20, -10, yaw_deg=45.0)
    assert len(io.local_ned) == 1
    args = io.local_ned[0]
    # args: tsys, tcomp, frame, type_mask, x, y, z, vx..., yaw, yaw_rate
    assert args[2] == m.MAV_FRAME_BODY_OFFSET_NED
    assert args[4] == 100 and args[5] == 20 and args[6] == -10
    # yaw verildi -> YAW_IGNORE bayragi KAPALI
    type_mask = args[3]
    assert not (type_mask & m.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
    # velocity her zaman ignore
    assert type_mask & m.POSITION_TARGET_TYPEMASK_VX_IGNORE


def test_body_offset_yaw_ignore_when_nan():
    fc, io = make_commander()
    fc.fly_to_body_offset(50, 0, 0)  # yaw=nan
    type_mask = io.local_ned[0][3]
    assert type_mask & m.POSITION_TARGET_TYPEMASK_YAW_IGNORE


def test_fly_to_local_ned_absolute_frame():
    fc, io = make_commander()
    assert fc.fly_to_local_ned(500.0, 200.0, -120.0)
    args = io.local_ned[-1]
    # MUTLAK lokal: frame LOCAL_NED, pozisyon dolu, velocity ignore
    assert args[2] == m.MAV_FRAME_LOCAL_NED
    assert args[4] == 500.0 and args[5] == 200.0 and args[6] == -120.0
    type_mask = args[3]
    assert not (type_mask & m.POSITION_TARGET_TYPEMASK_X_IGNORE)
    assert type_mask & m.POSITION_TARGET_TYPEMASK_VX_IGNORE


def test_fly_velocity_ned_masks_position():
    fc, io = make_commander()
    assert fc.fly_velocity_ned(15.0, 5.0, -2.0)
    args = io.local_ned[-1]
    assert args[2] == m.MAV_FRAME_LOCAL_NED
    type_mask = args[3]
    # pozisyon + ivme IGNORE, velocity AKTIF
    assert type_mask & m.POSITION_TARGET_TYPEMASK_X_IGNORE
    assert type_mask & m.POSITION_TARGET_TYPEMASK_Y_IGNORE
    assert type_mask & m.POSITION_TARGET_TYPEMASK_Z_IGNORE
    assert type_mask & m.POSITION_TARGET_TYPEMASK_AX_IGNORE
    assert not (type_mask & m.POSITION_TARGET_TYPEMASK_VX_IGNORE)
    # vx,vy,vz = args[7..9]
    assert args[7] == 15.0 and args[8] == 5.0 and args[9] == -2.0


def test_fly_velocity_body_frame():
    fc, io = make_commander()
    assert fc.fly_velocity_body(20.0, 0.0, 0.0)
    args = io.local_ned[-1]
    assert args[2] == m.MAV_FRAME_BODY_OFFSET_NED
    assert args[7] == 20.0


def test_velocity_requires_guided():
    fc, io = make_commander(guided=False)
    assert not fc.fly_velocity_ned(10.0, 0.0, 0.0)
    assert len(io.local_ned) == 0


def test_set_mode_and_ack():
    fc, io = make_commander()
    fc.set_mode(ArduPilotMode.RTL)
    assert io.command_long[-1][2] == m.MAV_CMD_DO_SET_MODE
    # ACK akisini dogrula
    fc.fly_to_position(Position(1, 2, 3))
    fc.handle_command_ack(m.MAV_CMD_DO_REPOSITION, m.MAV_RESULT_ACCEPTED)
    from uav.comm.flight_commander import AckStatus
    assert fc.get_ack_status() == AckStatus.ACCEPTED


def test_change_airspeed_validation():
    fc, io = make_commander()
    assert not fc.change_airspeed(2.0)   # cok dusuk
    assert fc.change_airspeed(25.0)
    assert io.command_long[-1][2] == 43000


def test_inbound_connection_string_matches_cpp_bind_semantics():
    assert MavlinkIO._cpp_compatible_connection_string(
        "udpin:127.0.0.1:14550") == "udpin:0.0.0.0:14550"
    assert MavlinkIO._connection_candidates(
        "udpin:127.0.0.1:14550") == [
            "udpin:0.0.0.0:14550",
            "udpin:127.0.0.1:14550",
        ]
    assert MavlinkIO._cpp_compatible_connection_string(
        "tcpin:localhost:14552") == "tcpin:0.0.0.0:14552"
    assert MavlinkIO._cpp_compatible_connection_string(
        "udpout:127.0.0.1:14550") == "udpout:127.0.0.1:14550"
    assert MavlinkIO._cpp_compatible_connection_string(
        "udpin:192.168.1.20:14550") == "udpin:192.168.1.20:14550"


class FakeMavlinkLink:
    def __init__(self):
        self.sent_messages = []
        self.connected = True

    def is_connected(self):
        return self.connected

    def send_message(self, msg):
        self.sent_messages.append(msg)
        return True

    def send_heartbeat(self):
        return True


class FakeMavlinkMessage:
    def __init__(self, msg_type):
        self._msg_type = msg_type

    def get_type(self):
        return self._msg_type

    def get_msgbuf(self):
        return b"msg"


def make_bridge():
    return MavlinkBridge("", WorldModel(), EventBus())


def test_bridge_relays_pixhawk_and_gcs_messages():
    bridge = make_bridge()
    pix = FakeMavlinkLink()
    gcs = FakeMavlinkLink()
    bridge._pix = pix
    bridge._gcs = gcs

    from_pixhawk = FakeMavlinkMessage("NAMED_VALUE_FLOAT")
    from_gcs = FakeMavlinkMessage("MISSION_COUNT")
    bridge._on_pixhawk_msg(from_pixhawk)
    bridge._on_gcs_msg(from_gcs)

    assert gcs.sent_messages == [from_pixhawk]
    assert pix.sent_messages == [from_gcs]
    stats = bridge.get_stats()
    assert stats.messages_received == 2
    assert stats.messages_sent == 2


def test_bridge_hss_statustext_replaces_old_zones():
    bridge = make_bridge()
    bridge._wm.update_hss_zone(HSSZone(id=99, center=Position(1, 2, 0), radius=50, active=True))

    bridge._handle_statustext("HSS:1:7:37.80:-122.30:150")

    zones = bridge._wm.get_all_hss_zones()
    assert [z.id for z in zones] == [7]
    assert zones[0].radius == 150


def test_bridge_heartbeat_filters_by_vehicle_type_not_autopilot_field():
    bridge = make_bridge()

    gcs_msg = type("Heartbeat", (), {
        "type": m.MAV_TYPE_GCS,
        "custom_mode": int(ArduPilotMode.GUIDED),
        "base_mode": m.MAV_MODE_FLAG_SAFETY_ARMED,
    })()
    bridge._handle_heartbeat(gcs_msg)
    assert bridge.commander.get_current_mode() == ArduPilotMode.INITIALISING

    aircraft_msg = type("Heartbeat", (), {
        "type": m.MAV_TYPE_FIXED_WING,
        "custom_mode": int(ArduPilotMode.GUIDED),
        "base_mode": m.MAV_MODE_FLAG_SAFETY_ARMED,
    })()
    bridge._handle_heartbeat(aircraft_msg)
    assert bridge.commander.get_current_mode() == ArduPilotMode.GUIDED
    assert bridge.commander.is_armed()
    assert bridge.get_stats().flight_mode == "GUIDED"


# ---------------- StatusTextParser ----------------

def test_parse_riv():
    p = StatusTextParser()
    res = p.parse("RIV:7:37.8:-122.3:120:90:25:0")
    assert res and res.type == StatusTextType.RIV
    opp = res.opponent
    assert opp.id == 7
    assert opp.position.latitude == 37.8
    assert opp.heading == 90.0
    # heading 90 (dogu) -> dogu hizi pozitif, kuzey ~0
    assert opp.velocity.east == 25.0
    assert abs(opp.velocity.north) < 1e-6


def test_parse_hss():
    p = StatusTextParser()
    res = p.parse("HSS:2:1:37.80:-122.30:150:2:37.85:-122.35:200")
    assert res and len(res.hss_zones) == 2
    assert res.hss_zones[0].id == 1 and res.hss_zones[0].radius == 150
    assert res.hss_zones[1].center.longitude == -122.35


def test_parse_bnd():
    p = StatusTextParser()
    res = p.parse("BND:3:37.80:-122.30:37.85:-122.30:37.85:-122.35")
    assert res and res.boundary and len(res.boundary.vertices) == 3


def test_parse_mode_commands():
    p = StatusTextParser()
    assert p.parse("FREE").mode_command.target_mode == MissionMode.PATROL
    assert p.parse("ARDUPILOT").mode_command.target_mode == MissionMode.ARDUPILOT
    lock = p.parse("LOCK:9").mode_command
    assert lock.target_mode == MissionMode.FOLLOW and lock.target_id == 9
    kam = p.parse("KAMIKAZE:37.8:-122.3").mode_command
    assert kam.target_mode == MissionMode.QR_DIVE
    assert kam.target_pos.latitude == 37.8


def test_parse_multipart():
    # Gonderici TUM mesaji ("HSS:..." dahil) parcalara boler; icerikler
    # birlestirilince orijinal onekli mesaj yeniden olusur.
    p = StatusTextParser()
    assert p.parse("HSS[1/2]:HSS:2:1:37.80:-122.30:") is None  # henuz tamamlanmadi
    res = p.parse("HSS[2/2]:150:2:37.85:-122.35:200")
    assert res and len(res.hss_zones) == 2

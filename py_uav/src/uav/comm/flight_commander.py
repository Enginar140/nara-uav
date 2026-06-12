"""Ucus komutlayici -- ArduPlane GUIDED komut yuzeyi.

ArduPlane fork'unun (nara/local-control, Katman 0+1+2) destekledigi komutlar:
  - DO_REPOSITION (command_int): lat/lon/alt'a git + ground speed.  [is gucu]
  - SET_POSITION_TARGET_LOCAL_NED:
      * POZISYON: LOCAL_NED (EKF origin'e MUTLAK), LOCAL_OFFSET_NED / BODY_OFFSET_NED
        (mevcut konuma goreli). Opsiyonel yaw. -> L1+TECS ile ucar.
      * HIZ (Katman 2): velocity (vN,vE,vD) firmware'de heading(COG)+airspeed+climb
        slew'lerine AYRISTIRILIR. Sabit-kanat icin gercek bir NED hiz servosu yok;
        bu yuzden "hiz" = yon + hiz buyuklugu + tirmanis demand'i olarak ucar.
        Stream kesilirse firmware guided_timeout sonrasi loiter'a doner (flyaway yok).
  - GUIDED_CHANGE_SPEED/ALTITUDE/HEADING (ArduPilot ext 43000/43001/43002).
  - DO_SET_MODE, COMPONENT_ARM_DISARM.

NOT: Eski surumde velocity "yok sayilir"di; fork Katman 2 ile artik decomposition
yoluyla guvenle destekliyor (bkz. fly_velocity_ned / fly_velocity_body).
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Optional

from pymavlink import mavutil

from ..core.geo import haversine_distance
from ..core.types import Position
from ..infra.log import get_logger
from .mavlink_io import MavlinkIO

_log = get_logger("flight_commander")
m = mavutil.mavlink

# ArduPilot eklenti komutlari (ardupilotmega.xml); common dialect'te yok
MAV_CMD_GUIDED_CHANGE_SPEED = 43000
MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
MAV_CMD_GUIDED_CHANGE_HEADING = 43002


class ArduPilotMode(IntEnum):
    MANUAL = 0
    CIRCLE = 1
    STABILIZE = 2
    TRAINING = 3
    ACRO = 4
    FBWA = 5
    FBWB = 6
    CRUISE = 7
    AUTOTUNE = 8
    AUTO = 10
    RTL = 11
    LOITER = 12
    TAKEOFF = 13
    AVOID_ADSB = 14
    GUIDED = 15
    INITIALISING = 16
    QSTABILIZE = 17
    QHOVER = 18
    QLOITER = 19
    QLAND = 20
    QRTL = 21


class HeadingType(IntEnum):
    COURSE_OVER_GROUND = 0
    HEADING = 1
    DEFAULT = 2


class AckStatus(Enum):
    NONE = "NONE"
    WAITING = "WAITING"
    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    TIMEOUT = "TIMEOUT"


@dataclass
class CommanderConfig:
    target_sys: int = 1
    target_comp: int = 1
    min_command_delta_m: float = 5.0   # ayni hedefe tekrar gonderme esigi
    ack_timeout_ms: int = 3000


class FlightCommander:
    def __init__(self, io: MavlinkIO, config: Optional[CommanderConfig] = None):
        self._io = io
        self.cfg = config or CommanderConfig()
        self._lock = threading.RLock()
        self._current_mode = ArduPilotMode.INITIALISING
        self._armed = False
        self.commands_sent = 0
        self.commands_acked = 0
        self._last_pos_cmd: Optional[Position] = None
        self._ack_status = AckStatus.NONE
        self._pending_cmd_id = 0
        self._pending_since = 0.0

    # ---------------- GLOBAL: DO_REPOSITION ----------------
    def fly_to_position(self, target: Position, ground_speed_ms: float = -1.0) -> bool:
        with self._lock:
            self._update_ack_timeout()
            if self._is_duplicate(target):
                return True
            sent = self._io.send_command_int(
                self.cfg.target_sys, self.cfg.target_comp,
                m.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                m.MAV_CMD_DO_REPOSITION,
                float(ground_speed_ms), 0.0, 0.0, 0.0,
                int(target.latitude * 1e7), int(target.longitude * 1e7),
                float(target.altitude),
            )
            if sent:
                self._last_pos_cmd = target
                self.commands_sent += 1
                self._mark_pending(m.MAV_CMD_DO_REPOSITION)
            return sent

    # ---------------- LOCAL/BODY pozisyon ofseti ----------------
    def _send_local_ned_target(self, frame: int, x: float, y: float, z: float,
                               yaw_deg: float = math.nan) -> bool:
        with self._lock:
            if not self.is_in_guided_mode():
                _log.warning("local_ned_target: GUIDED modda degil (mod=%s)",
                             self._current_mode.name)
                return False
            # POSITION_TARGET_TYPEMASK: 1 = ignore.
            tm = 0
            tm |= m.POSITION_TARGET_TYPEMASK_VX_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_VY_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_AX_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_AY_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_FORCE_SET
            tm |= m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            yaw_valid = math.isfinite(yaw_deg)
            if not yaw_valid:
                tm |= m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            yaw_rad = math.radians(yaw_deg) if yaw_valid else 0.0
            sent = self._io.send_set_position_target_local_ned(
                self.cfg.target_sys, self.cfg.target_comp, frame, tm,
                float(x), float(y), float(z),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                yaw_rad, 0.0,
            )
            if sent:
                self.commands_sent += 1
            return sent

    def fly_to_body_offset(self, forward_m: float, right_m: float, down_m: float,
                           yaw_deg: float = math.nan) -> bool:
        _log.info("body_offset fwd=%.1f right=%.1f down=%.1f yaw=%.1f",
                  forward_m, right_m, down_m, yaw_deg)
        return self._send_local_ned_target(
            m.MAV_FRAME_BODY_OFFSET_NED, forward_m, right_m, down_m, yaw_deg)

    def fly_to_local_offset(self, north_m: float, east_m: float, down_m: float,
                            yaw_deg: float = math.nan) -> bool:
        return self._send_local_ned_target(
            m.MAV_FRAME_LOCAL_OFFSET_NED, north_m, east_m, down_m, yaw_deg)

    def fly_to_local_ned(self, north_m: float, east_m: float, down_m: float,
                         yaw_deg: float = math.nan) -> bool:
        """MUTLAK lokal nokta: EKF origin'e (home) gore NED metre (Katman 0).

        OFFSET'in aksine bu konum sabit bir referansa demirlidir; ayni (N,E,D)
        tekrar gonderilince ayni dunya noktasi cikar. Firmware
        Location::from_ekf_offset_NED_m ile cozer.
        """
        return self._send_local_ned_target(
            m.MAV_FRAME_LOCAL_NED, north_m, east_m, down_m, yaw_deg)

    # ---------------- LOCAL/BODY hiz (Katman 2) ----------------
    def _send_local_ned_velocity(self, frame: int, vx: float, vy: float, vz: float,
                                 yaw_deg: float = math.nan) -> bool:
        """Hiz demand'i: pozisyon+ivme alanlari IGNORE, yalniz velocity gonderilir.

        Firmware tarafi velocity'yi heading(COG)+airspeed+climb slew'lerine ayristirir.
        """
        with self._lock:
            if not self.is_in_guided_mode():
                _log.warning("local_ned_velocity: GUIDED modda degil (mod=%s)",
                             self._current_mode.name)
                return False
            tm = 0
            # pozisyon + ivme tamamen ignore -> firmware velocity koluna girer
            tm |= m.POSITION_TARGET_TYPEMASK_X_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_Y_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_Z_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_AX_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_AY_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            tm |= m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            yaw_valid = math.isfinite(yaw_deg)
            if not yaw_valid:
                tm |= m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            yaw_rad = math.radians(yaw_deg) if yaw_valid else 0.0
            sent = self._io.send_set_position_target_local_ned(
                self.cfg.target_sys, self.cfg.target_comp, frame, tm,
                0.0, 0.0, 0.0,
                float(vx), float(vy), float(vz),
                0.0, 0.0, 0.0,
                yaw_rad, 0.0,
            )
            if sent:
                self.commands_sent += 1
            return sent

    def fly_velocity_ned(self, vn: float, ve: float, vd: float,
                         yaw_deg: float = math.nan) -> bool:
        """Yer-cercevesi (NED) hiz demand'i, m/s. vd: +asagi."""
        return self._send_local_ned_velocity(
            m.MAV_FRAME_LOCAL_NED, vn, ve, vd, yaw_deg)

    def fly_velocity_body(self, vx: float, vy: float, vz: float,
                          yaw_deg: float = math.nan) -> bool:
        """Govde-cercevesi hiz demand'i, m/s. vx:ileri, vy:sag, vz:asagi.

        Firmware yatay bileseni AHRS yaw ile yer-cercevesine dondurur.
        """
        return self._send_local_ned_velocity(
            m.MAV_FRAME_BODY_OFFSET_NED, vx, vy, vz, yaw_deg)

    def fly_to_bearing_offset(self, bearing_deg: float, distance_m: float,
                              altitude_offset_m: float = 0.0,
                              yaw_deg: float = math.nan) -> bool:
        br = math.radians(bearing_deg)
        north = math.cos(br) * distance_m
        east = math.sin(br) * distance_m
        down = -altitude_offset_m
        return self._send_local_ned_target(
            m.MAV_FRAME_LOCAL_OFFSET_NED, north, east, down, yaw_deg)

    def fly_to_vision_target(self, yaw_offset_deg: float, pitch_offset_deg: float,
                             range_m: float, yaw_deg: float = math.nan) -> bool:
        """Kamera-cercevesi acilari -> govde-cercevesi ofseti (terminal dalis)."""
        yo = math.radians(yaw_offset_deg)
        po = math.radians(pitch_offset_deg)
        fwd = math.cos(po) * math.cos(yo) * range_m
        right = math.cos(po) * math.sin(yo) * range_m
        down = math.sin(po) * range_m
        return self._send_local_ned_target(
            m.MAV_FRAME_BODY_OFFSET_NED, fwd, right, down, yaw_deg)

    # ---------------- GUIDED ext komutlari ----------------
    def change_airspeed(self, speed_ms: float, acceleration_ms2: float = 0.0,
                        use_airspeed: bool = True) -> bool:
        with self._lock:
            if not self.is_in_guided_mode():
                return False
            if not (5.0 <= speed_ms <= 100.0):
                _log.warning("change_airspeed: gecersiz hiz %.1f", speed_ms)
                return False
            speed_type = 0.0 if use_airspeed else 1.0
            sent = self._io.send_command_long(
                self.cfg.target_sys, self.cfg.target_comp,
                MAV_CMD_GUIDED_CHANGE_SPEED,
                speed_type, speed_ms, acceleration_ms2, 0, 0, 0, 0)
            if sent:
                self.commands_sent += 1
                self._mark_pending(MAV_CMD_GUIDED_CHANGE_SPEED)
            return sent

    def change_altitude(self, altitude_m: float, climb_rate_ms: float = 2.0) -> bool:
        with self._lock:
            if not self.is_in_guided_mode():
                return False
            if not (0.0 <= altitude_m <= 1000.0):
                return False
            climb_rate_ms = max(0.5, min(15.0, climb_rate_ms))
            sent = self._io.send_command_long(
                self.cfg.target_sys, self.cfg.target_comp,
                MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                0, 0, climb_rate_ms, 0, 0, 0, altitude_m)
            if sent:
                self.commands_sent += 1
                self._mark_pending(MAV_CMD_GUIDED_CHANGE_ALTITUDE)
            return sent

    def change_heading(self, heading_deg: float,
                       heading_type: HeadingType = HeadingType.HEADING,
                       accel_limit_rad_s2: float = 10.0) -> bool:
        with self._lock:
            if not self.is_in_guided_mode():
                return False
            heading_deg = heading_deg % 360.0
            sent = self._io.send_command_long(
                self.cfg.target_sys, self.cfg.target_comp,
                MAV_CMD_GUIDED_CHANGE_HEADING,
                float(int(heading_type)), heading_deg, accel_limit_rad_s2, 0, 0, 0, 0)
            if sent:
                self.commands_sent += 1
                self._mark_pending(MAV_CMD_GUIDED_CHANGE_HEADING)
            return sent

    # ---------------- mod / arm ----------------
    def set_mode(self, mode: ArduPilotMode) -> bool:
        with self._lock:
            sent = self._io.send_command_long(
                self.cfg.target_sys, self.cfg.target_comp,
                m.MAV_CMD_DO_SET_MODE,
                m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, float(int(mode)),
                0, 0, 0, 0, 0)
            if sent:
                self.commands_sent += 1
                self._mark_pending(m.MAV_CMD_DO_SET_MODE)
                _log.info("set_mode(%s)", mode.name)
            return sent

    def arm(self) -> bool:
        return self._arm_disarm(True)

    def disarm(self) -> bool:
        return self._arm_disarm(False)

    def _arm_disarm(self, arm: bool) -> bool:
        with self._lock:
            sent = self._io.send_command_long(
                self.cfg.target_sys, self.cfg.target_comp,
                m.MAV_CMD_COMPONENT_ARM_DISARM,
                1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0)
            if sent:
                self.commands_sent += 1
                self._mark_pending(m.MAV_CMD_COMPONENT_ARM_DISARM)
            return sent

    # ---------------- heartbeat'ten durum ----------------
    def update_mode_from_heartbeat(self, custom_mode: int) -> None:
        try:
            self._current_mode = ArduPilotMode(custom_mode)
        except ValueError:
            pass  # bilinmeyen mod; mevcut degeri koru

    def update_armed_from_heartbeat(self, base_mode: int) -> None:
        self._armed = bool(base_mode & m.MAV_MODE_FLAG_SAFETY_ARMED)

    def get_current_mode(self) -> ArduPilotMode:
        return self._current_mode

    def is_in_guided_mode(self) -> bool:
        return self._current_mode == ArduPilotMode.GUIDED

    def is_armed(self) -> bool:
        return self._armed

    # ---------------- ACK ----------------
    def handle_command_ack(self, cmd_id: int, result: int) -> None:
        with self._lock:
            self._update_ack_timeout()
            if self._pending_cmd_id != 0 and cmd_id != self._pending_cmd_id:
                return
            self.commands_acked += 1
            if result == m.MAV_RESULT_ACCEPTED:
                self._ack_status = AckStatus.ACCEPTED
                self._pending_cmd_id = 0
            elif result == m.MAV_RESULT_IN_PROGRESS:
                self._ack_status = AckStatus.WAITING
            else:
                self._ack_status = AckStatus.REJECTED
                self._pending_cmd_id = 0
                if cmd_id == m.MAV_CMD_DO_REPOSITION:
                    self._last_pos_cmd = None
                _log.warning("CMD %d reddedildi (result=%d)", cmd_id, result)

    def get_ack_status(self) -> AckStatus:
        with self._lock:
            self._update_ack_timeout()
            return self._ack_status

    def _mark_pending(self, cmd_id: int) -> None:
        self._pending_cmd_id = cmd_id
        self._pending_since = time.monotonic()
        self._ack_status = AckStatus.WAITING

    def _update_ack_timeout(self) -> None:
        if self._ack_status != AckStatus.WAITING or self._pending_cmd_id == 0:
            return
        if (time.monotonic() - self._pending_since) * 1000.0 >= self.cfg.ack_timeout_ms:
            self._ack_status = AckStatus.TIMEOUT
            if self._pending_cmd_id == m.MAV_CMD_DO_REPOSITION:
                self._last_pos_cmd = None
            self._pending_cmd_id = 0

    def _is_duplicate(self, target: Position) -> bool:
        if self._last_pos_cmd is None:
            return False
        return haversine_distance(self._last_pos_cmd, target) < self.cfg.min_command_delta_m

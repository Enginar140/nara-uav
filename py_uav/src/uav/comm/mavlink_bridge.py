"""MAVLink kopru: telemetri -> WorldModel + EventBus; heartbeat + watchdog.

Receive callback'leri MAVLink receive thread'inde calisir; WorldModel
(thread-guvenli) ve EventBus.publish (thread-guvenli) buradan beslenir.
Heartbeat gonderimi ve timeout izleme asyncio gorevleridir.
"""
from __future__ import annotations

import asyncio
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from ..core.types import UAVState
from ..infra.event_bus import EventBus, EventType
from ..infra.log import get_logger
from ..world_model.world_model import WorldModel
from .flight_commander import CommanderConfig, FlightCommander
from .mavlink_io import MavlinkIO
from .param_reader import ParamReader
from .statustext_parser import StatusTextParser, StatusTextType

_log = get_logger("mavlink_bridge")


@dataclass
class MavlinkStats:
    messages_received: int = 0
    messages_sent: int = 0
    last_heartbeat_us: int = 0
    pixhawk_connected: bool = False
    gcs_connected: bool = False
    heartbeat_timeout: bool = False
    armed: bool = False
    flight_mode: str = "INITIALISING"


class MavlinkBridge:
    def __init__(
        self,
        pixhawk_conn: str,
        world_model: WorldModel,
        event_bus: EventBus,
        gcs_conn: str = "",
        target_sys: int = 1,
        target_comp: int = 1,
        heartbeat_timeout_s: float = 5.0,
    ):
        self._wm = world_model
        self._bus = event_bus
        self._heartbeat_timeout_s = heartbeat_timeout_s

        self._pix = MavlinkIO(pixhawk_conn)
        self._gcs = MavlinkIO(gcs_conn) if gcs_conn else None
        self.commander = FlightCommander(
            self._pix, CommanderConfig(target_sys=target_sys, target_comp=target_comp)
        )
        self.param_reader = ParamReader(self._pix, target_sys, target_comp)
        self.param_reader.set_ready_callback(
            lambda: self._bus.publish(EventType.PARAMS_READY))
        self._parser = StatusTextParser()

        self._own_lock = threading.Lock()
        self._own = UAVState(id=0)
        self._stats_lock = threading.Lock()
        self._stats = MavlinkStats()
        self._param_lock = threading.Lock()
        self._params_requested = False
        self._param_list_requested = False
        self._last_hb = 0.0
        self._connected = False
        self._running = False
        self._hb_task: Optional[asyncio.Task] = None
        self._param_task: Optional[asyncio.Task] = None

    # -- yasam dongusu --
    async def start(self) -> bool:
        if not self._pix.connect():
            _log.error("Pixhawk baglanamadi")
            return False
        self._pix.start_receiver(self._on_pixhawk_msg)
        if self._gcs:
            if self._gcs.connect():
                self._gcs.start_receiver(self._on_gcs_msg)
            else:
                _log.warning("GCS baglantisi kurulamadi; Pixhawk koprusu devam ediyor")
        self._running = True
        self._hb_task = asyncio.create_task(self._heartbeat_loop(), name="mav_heartbeat")
        self._param_task = asyncio.create_task(self._param_request_loop(), name="mav_params")
        _log.info("MavlinkBridge basladi")
        return True

    async def _param_request_loop(self) -> None:
        """C++ akisi: kritik parametreler bir kez, gerekirse liste fallback'i."""
        await asyncio.sleep(2.0)  # otopilotun hazir olmasini bekle
        if not self._running or self.param_reader.are_critical_ready():
            return
        self._request_critical_params_once()

        await asyncio.sleep(3.0)
        if not self._running or self.param_reader.are_critical_ready():
            return
        with self._param_lock:
            if self._param_list_requested:
                return
            self._param_list_requested = True
        _log.warning("Kritik parametreler tamamlanmadi; tum parametre listesi isteniyor")
        self.param_reader.request_all()

    async def stop(self) -> None:
        self._running = False
        if self._hb_task:
            self._hb_task.cancel()
        if self._param_task:
            self._param_task.cancel()
        self._pix.close()
        if self._gcs:
            self._gcs.close()

    def send_statustext(self, text: str, severity: int = 6) -> bool:
        """Companion -> sunucu/GCS downlink STATUSTEXT (kilit/QR/mod paketleri).

        GCS baglantisi (yarisma sunucusu) varsa oraya; yoksa pixhawk hattina.
        DownlinkEncoder.send_fn bu metoda baglanir.
        """
        link = self._gcs if (self._gcs and self._gcs.is_connected()) else self._pix
        if link is None:
            return False
        link.send_statustext(text, severity)
        self._count_tx(1)
        return True

    async def _heartbeat_loop(self) -> None:
        while self._running:
            sent = 0
            if self._pix.send_heartbeat():
                sent += 1
            if self._gcs and self._gcs.send_heartbeat():
                sent += 1
            if sent:
                self._count_tx(sent)
            # timeout izleme
            if self._last_hb > 0.0:
                if time.monotonic() - self._last_hb > self._heartbeat_timeout_s:
                    if self._connected:
                        self._connected = False
                        with self._stats_lock:
                            self._stats.heartbeat_timeout = True
                        self._bus.publish(EventType.HEARTBEAT_TIMEOUT)
                        self._bus.publish(EventType.MAVLINK_DISCONNECTED)
            await asyncio.sleep(1.0)

    def _request_critical_params_once(self) -> None:
        with self._param_lock:
            if self._params_requested:
                return
            self._params_requested = True
        self.param_reader.request_critical()

    def _count_rx(self, count: int = 1) -> None:
        with self._stats_lock:
            self._stats.messages_received += count

    def _count_tx(self, count: int = 1) -> None:
        with self._stats_lock:
            self._stats.messages_sent += count

    def get_stats(self) -> MavlinkStats:
        with self._stats_lock:
            stats = MavlinkStats(**self._stats.__dict__)
        stats.pixhawk_connected = self._pix.is_connected()
        stats.gcs_connected = self._gcs.is_connected() if self._gcs else False
        return stats

    # ===================== Pixhawk mesajlari =====================
    def _on_pixhawk_msg(self, msg) -> None:
        self._count_rx()
        mtype = msg.get_type()
        try:
            if mtype == "HEARTBEAT":
                self._handle_heartbeat(msg)
            elif mtype == "GLOBAL_POSITION_INT":
                self._handle_global_position(msg)
            elif mtype == "VFR_HUD":
                self._handle_vfr_hud(msg)
            elif mtype == "GPS_RAW_INT":
                self._handle_gps(msg)
            elif mtype == "SYS_STATUS":
                self._handle_sys_status(msg)
            elif mtype == "ATTITUDE":
                self._handle_attitude(msg)
            elif mtype == "STATUSTEXT":
                self._handle_statustext(msg.text)
            elif mtype == "COMMAND_ACK":
                self.commander.handle_command_ack(msg.command, msg.result)
            elif mtype == "PARAM_VALUE":
                pid = msg.param_id
                if isinstance(pid, bytes):
                    pid = pid.decode("utf-8", errors="ignore")
                self.param_reader.handle_param_value(
                    pid.strip("\x00"), float(msg.param_value), int(msg.param_count))
        finally:
            if self._gcs and self._gcs.is_connected() and self._gcs.send_message(msg):
                self._count_tx()

    def _handle_heartbeat(self, msg) -> None:
        # C++ ile ayni: GCS ve companion heartbeat'lerini ayikla.
        from pymavlink import mavutil
        if msg.type in (
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        ):
            return
        old_mode = self.commander.get_current_mode()
        old_armed = self.commander.is_armed()
        self.commander.update_mode_from_heartbeat(msg.custom_mode)
        self.commander.update_armed_from_heartbeat(msg.base_mode)
        new_mode = self.commander.get_current_mode()
        new_armed = self.commander.is_armed()
        if new_mode != old_mode:
            _log.info("AP mod degisti: %s -> %s", old_mode.name, new_mode.name)
        if new_armed != old_armed:
            _log.info("AP armed=%s", "YES" if new_armed else "NO")
        self._last_hb = time.monotonic()
        now_us = time.time_ns() // 1000
        recovered = False
        with self._stats_lock:
            recovered = self._stats.heartbeat_timeout
            self._stats.last_heartbeat_us = now_us
            self._stats.heartbeat_timeout = False
            self._stats.armed = new_armed
            self._stats.flight_mode = new_mode.name
        if recovered:
            self._bus.publish(EventType.HEARTBEAT_RECOVERED)
        if not self._connected:
            self._connected = True
            self._bus.publish(EventType.MAVLINK_CONNECTED)
        self._bus.publish(EventType.HEARTBEAT_RECEIVED)
        self._request_critical_params_once()

    def _handle_global_position(self, msg) -> None:
        with self._own_lock:
            self._own.id = 0
            self._own.position.latitude = msg.lat / 1e7
            self._own.position.longitude = msg.lon / 1e7
            self._own.position.altitude = msg.relative_alt / 1000.0  # home'a gore
            self._own.velocity.north = msg.vx / 100.0
            self._own.velocity.east = msg.vy / 100.0
            self._own.velocity.down = msg.vz / 100.0
            self._own.groundspeed = self._own.velocity.horizontal_speed()
            self._own.heading = msg.hdg / 100.0 if msg.hdg != 65535 else self._own.heading
            self._own.timestamp_us = time.time_ns() // 1000
            snapshot = self._copy_own()
        self._wm.set_own_state(snapshot)
        self._bus.publish(EventType.OWN_STATE_UPDATED)

    def _handle_vfr_hud(self, msg) -> None:
        with self._own_lock:
            self._own.airspeed = msg.airspeed
            self._own.groundspeed = msg.groundspeed
            snap = self._copy_own() if self._own.timestamp_us else None
        if snap:
            self._wm.set_own_state(snap)

    def _handle_gps(self, msg) -> None:
        with self._own_lock:
            self._own.gps_fix_type = msg.fix_type
            self._own.satellites_visible = msg.satellites_visible
            self._own.gps_hdop = msg.eph if msg.eph != 65535 else 9999
            snap = self._copy_own() if self._own.timestamp_us else None
        if snap:
            self._wm.set_own_state(snap)

    def _handle_sys_status(self, msg) -> None:
        with self._own_lock:
            self._own.battery_voltage = msg.voltage_battery / 1000.0
            self._own.battery_remaining = msg.battery_remaining
            snap = self._copy_own() if self._own.timestamp_us else None
        # batarya failsafe icin SYS_STATUS'u WorldModel'e hemen yansit
        if snap:
            self._wm.set_own_state(snap)

    def _handle_attitude(self, msg) -> None:
        with self._own_lock:
            self._own.attitude.roll = math.degrees(msg.roll)
            self._own.attitude.pitch = math.degrees(msg.pitch)
            self._own.attitude.yaw = math.degrees(msg.yaw)

    def _copy_own(self) -> UAVState:
        import copy
        return copy.deepcopy(self._own)

    # ===================== GCS mesajlari =====================
    def _on_gcs_msg(self, msg) -> None:
        self._count_rx()
        try:
            if msg.get_type() == "STATUSTEXT":
                self._handle_statustext(msg.text)
        finally:
            if self._pix.is_connected() and self._pix.send_message(msg):
                self._count_tx()

    # ===================== STATUSTEXT dagitimi =====================
    def _handle_statustext(self, text: str) -> None:
        if isinstance(text, bytes):
            text = text.decode("utf-8", errors="ignore")
        text = text.strip("\x00").strip()
        parsed = self._parser.parse(text)
        if parsed is None:
            return
        if parsed.type == StatusTextType.RIV and parsed.opponent:
            self._wm.update_opponent(parsed.opponent)
            self._bus.publish(EventType.OPPONENT_UPDATED, parsed.opponent.id)
        elif parsed.type == StatusTextType.HSS:
            self._wm.replace_hss_zones(parsed.hss_zones)
            self._bus.publish(EventType.HSS_ZONE_UPDATED)
        elif parsed.type == StatusTextType.BND and parsed.boundary:
            self._wm.set_boundary(parsed.boundary)
            self._bus.publish(EventType.BOUNDARY_UPDATED)
        elif parsed.type == StatusTextType.MODE_CMD and parsed.mode_command:
            self._bus.publish(EventType.MODE_COMMAND_RECEIVED, parsed.mode_command)

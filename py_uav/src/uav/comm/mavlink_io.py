"""pymavlink baglanti sarmalayicisi.

C++ ConnectionManager + ham soket kodunun yerine pymavlink mavutil kullanir
(UDP/TCP/serial baglanti string'leri dogrudan desteklenir). Bloklayan receive
bir daemon thread'inde calisir; gelen her mesaj icin callback cagrilir.
Gonderme thread-guvenlidir (kilit ile).

FlightCommander'in ihtiyac duydugu dusuk-seviye gonderme metotlarini
(send_command_int / send_command_long / send_set_position_target_local_ned)
burada toplar; boylece komutlayici sahte bir gondericiyle test edilebilir.
"""
from __future__ import annotations

import threading
from typing import Callable, Optional

from pymavlink import mavutil

from ..infra.log import get_logger

_log = get_logger("mavlink_io")

MessageCallback = Callable[[object], None]


class MavlinkIO:
    def __init__(
        self,
        connection_string: str,
        source_system: int = 1,
        source_component: int = 191,  # MAV_COMP_ID_ONBOARD_COMPUTER
        baud: int = 115200,
    ):
        self.connection_string = connection_string
        self.source_system = source_system
        self.source_component = source_component
        self.baud = baud
        self.master: Optional[mavutil.mavfile] = None
        self._send_lock = threading.Lock()
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        self._callback: Optional[MessageCallback] = None

    @staticmethod
    def _cpp_compatible_connection_string(connection_string: str) -> str:
        """C++ ConnectionManager ile ayni inbound bind semantigini uygula.

        C++ tarafinda udpin/tcpin icin host bos, 0.0.0.0 veya 127.0.0.1 ise
        socket INADDR_ANY'e bind ediliyor. pymavlink ise 127.0.0.1'i birebir
        loopback'e bind eder; yarismada farkli interface'ten gelen UDP bu
        durumda gorunmez.
        """
        for prefix in ("udpin:", "tcpin:"):
            if not connection_string.startswith(prefix):
                continue
            rest = connection_string[len(prefix):]
            host, sep, port = rest.rpartition(":")
            if not sep:
                return connection_string
            if host in ("", "127.0.0.1", "localhost", "0.0.0.0"):
                return f"{prefix}0.0.0.0:{port}"
        return connection_string

    @classmethod
    def _connection_candidates(cls, connection_string: str) -> list[str]:
        bind_string = cls._cpp_compatible_connection_string(connection_string)
        if bind_string != connection_string:
            return [bind_string, connection_string]
        return [connection_string]

    # -- yasam dongusu --
    def connect(self) -> bool:
        last_error: Exception | None = None
        for bind_string in self._connection_candidates(self.connection_string):
            try:
                self.master = mavutil.mavlink_connection(
                    bind_string,
                    source_system=self.source_system,
                    source_component=self.source_component,
                    baud=self.baud,
                    autoreconnect=True,
                )
                if bind_string == self.connection_string:
                    _log.info("MAVLink baglandi: %s", self.connection_string)
                else:
                    _log.info("MAVLink baglandi: %s (bind=%s)",
                              self.connection_string, bind_string)
                return True
            except Exception as exc:  # noqa: BLE001
                last_error = exc
                self.master = None
                if bind_string != self.connection_string:
                    _log.warning("MAVLink bind basarisiz: %s (%s); fallback deneniyor: %s",
                                 bind_string, exc, self.connection_string)
                    continue
                _log.exception("MAVLink baglanti hatasi: %s (bind=%s)",
                               self.connection_string, bind_string)
                return False
        _log.error("MAVLink baglanti hatasi: %s (%s)", self.connection_string, last_error)
        return False

    def is_connected(self) -> bool:
        return self.master is not None

    def start_receiver(self, callback: MessageCallback) -> None:
        if self._running:
            return
        self._callback = callback
        self._running = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name=f"mavrx-{self.connection_string}", daemon=True
        )
        self._rx_thread.start()

    def _rx_loop(self) -> None:
        assert self.master is not None
        while self._running:
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
            except Exception:  # noqa: BLE001
                _log.exception("recv_match hatasi")
                continue
            if msg is None:
                continue
            if msg.get_type() == "BAD_DATA":
                continue
            try:
                if self._callback:
                    self._callback(msg)
            except Exception:  # noqa: BLE001
                _log.exception("MAVLink mesaj callback hatasi")

    def close(self) -> None:
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        if self.master:
            try:
                self.master.close()
            except Exception:  # noqa: BLE001
                pass

    # -- dusuk seviye gonderme (thread-guvenli) --
    def send_message(self, msg) -> bool:
        """Ham MAVLink mesajini aynen ileri yaz.

        Bridge passthrough icin mesaj yeniden encode edilmeden, alinmis paket
        buffer'i hedef linke aktarilir. Buffer yoksa pymavlink'in send yoluna
        duser.
        """
        if self.master is None:
            return False
        with self._send_lock:
            try:
                data = msg.get_msgbuf()
                if data:
                    self.master.write(data)
                else:
                    self.master.mav.send(msg)
            except Exception:  # noqa: BLE001
                _log.exception("MAVLink mesaj gonderme hatasi")
                return False
        return True

    def send_command_int(
        self, target_sys, target_comp, frame, command,
        p1, p2, p3, p4, x, y, z, current=0, autocontinue=0,
    ) -> bool:
        if self.master is None:
            return False
        with self._send_lock:
            self.master.mav.command_int_send(
                target_sys, target_comp, frame, command, current, autocontinue,
                p1, p2, p3, p4, int(x), int(y), float(z),
            )
        return True

    def send_command_long(
        self, target_sys, target_comp, command,
        p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, confirmation=0,
    ) -> bool:
        if self.master is None:
            return False
        with self._send_lock:
            self.master.mav.command_long_send(
                target_sys, target_comp, command, confirmation,
                p1, p2, p3, p4, p5, p6, p7,
            )
        return True

    def send_set_position_target_local_ned(
        self, target_sys, target_comp, frame, type_mask,
        x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
        time_boot_ms=0,
    ) -> bool:
        if self.master is None:
            return False
        with self._send_lock:
            self.master.mav.set_position_target_local_ned_send(
                time_boot_ms, target_sys, target_comp, frame, type_mask,
                x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate,
            )
        return True

    def send_heartbeat(self) -> bool:
        if self.master is None:
            return False
        with self._send_lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        return True

    def send_param_request_read(self, target_sys, target_comp, param_id: str) -> bool:
        if self.master is None:
            return False
        with self._send_lock:
            self.master.mav.param_request_read_send(
                target_sys, target_comp, param_id.encode("utf-8")[:16], -1)
        return True

    def send_param_request_list(self, target_sys, target_comp) -> bool:
        if self.master is None:
            return False
        with self._send_lock:
            self.master.mav.param_request_list_send(target_sys, target_comp)
        return True

    def send_statustext(self, text: str, severity: int = 6) -> None:
        if self.master is None:
            return
        with self._send_lock:
            self.master.mav.statustext_send(severity, text.encode("utf-8")[:50])

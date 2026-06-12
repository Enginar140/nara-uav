"""ParamReader -- ArduPilot parametrelerini oku. C++ ParamReader portu.

Kritik parametreler gelince ready callback tetiklenir (app PARAMS_READY yayinlar):
hiz zarfi, bank acisi -> donus yaricapi tahmini, fence irtifa, WP radius.
"""
from __future__ import annotations

import math
import threading
from typing import Callable, Dict, Optional

from ..infra.log import get_logger
from .mavlink_io import MavlinkIO

_log = get_logger("param_reader")

CRITICAL_PARAMS = [
    "ARSPD_FBW_MIN", "ARSPD_FBW_MAX", "TRIM_ARSPD_CM", "LIM_ROLL_CD",
    "WP_RADIUS", "WP_LOITER_RAD", "FENCE_ALT_MAX", "FENCE_ALT_MIN", "LIM_PITCH_MIN",
]


class ParamReader:
    def __init__(self, io: MavlinkIO, target_sys: int = 1, target_comp: int = 1):
        self._io = io
        self._tsys = target_sys
        self._tcomp = target_comp
        self._lock = threading.Lock()
        self._params: Dict[str, float] = {}
        self._critical_pending = set(CRITICAL_PARAMS)
        self._critical_ready = False
        self._total = -1
        self._ready_cb: Optional[Callable[[], None]] = None

    def set_ready_callback(self, cb: Callable[[], None]) -> None:
        self._ready_cb = cb

    def request_critical(self) -> None:
        for name in CRITICAL_PARAMS:
            self._io.send_param_request_read(self._tsys, self._tcomp, name)
        _log.info("%d kritik parametre istendi", len(CRITICAL_PARAMS))

    def request_all(self) -> None:
        self._io.send_param_request_list(self._tsys, self._tcomp)
        _log.info("tum parametre listesi istendi")

    def handle_param_value(self, param_id: str, value: float, param_count: int) -> None:
        cb_to_fire = None
        with self._lock:
            self._params[param_id] = value
            self._total = param_count
            if param_id in self._critical_pending:
                self._critical_pending.discard(param_id)
                _log.info("kritik param %s = %.3f", param_id, value)
            if not self._critical_ready and not self._critical_pending:
                self._critical_ready = True
                cb_to_fire = self._ready_cb
        if cb_to_fire:
            _log.info("tum kritik parametreler hazir | stall=%.1f cruise=%.1f max=%.1f "
                      "bank=%.1f turnR=%.0f WP_R=%.0f",
                      self.stall_speed(), self.cruise_speed(), self.max_airspeed(),
                      self.max_bank_angle(), self.estimate_turn_radius(), self.wp_radius())
            cb_to_fire()

    def has_param(self, name: str) -> bool:
        with self._lock:
            return name in self._params

    def get_param(self, name: str, default: float = 0.0) -> float:
        with self._lock:
            return self._params.get(name, default)

    def are_critical_ready(self) -> bool:
        return self._critical_ready

    def received_count(self) -> int:
        with self._lock:
            return len(self._params)

    def total_count(self) -> int:
        with self._lock:
            return self._total

    # -- yorumlanmis getter'lar (C++ ile ayni varsayilanlar) --
    def stall_speed(self) -> float:
        return self.get_param("ARSPD_FBW_MIN", 12.0)

    def max_airspeed(self) -> float:
        return self.get_param("ARSPD_FBW_MAX", 27.0)

    def cruise_speed(self) -> float:
        return self.get_param("TRIM_ARSPD_CM", 1500.0) / 100.0

    def max_bank_angle(self) -> float:
        return self.get_param("LIM_ROLL_CD", 6000.0) / 100.0

    def wp_radius(self) -> float:
        return self.get_param("WP_RADIUS", 80.0)

    def loiter_radius(self) -> float:
        return self.get_param("WP_LOITER_RAD", 60.0)

    def fence_alt_max(self) -> float:
        return self.get_param("FENCE_ALT_MAX", 400.0)

    def fence_alt_min(self) -> float:
        return self.get_param("FENCE_ALT_MIN", 30.0)

    def estimate_turn_radius(self) -> float:
        """r = v^2 / (g * tan(bank)). Sabit-kanat minimum donus yaricapi tahmini."""
        speed = self.cruise_speed()
        bank_rad = math.radians(self.max_bank_angle())
        g = 9.81
        if abs(math.tan(bank_rad)) < 0.01:
            return 100.0
        return (speed * speed) / (g * math.tan(bank_rad))

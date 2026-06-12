"""Zaman yardimcilari. Monotonik saat tabanli; epoch us telemetri icin."""
from __future__ import annotations

import time


def now_us() -> int:
    """Epoch'tan beri mikrosaniye (telemetri zaman damgalari icin)."""
    return time.time_ns() // 1000


def now_monotonic() -> float:
    """Monotonik saniye (sure olcumu icin; duvar saati siçramalarindan etkilenmez)."""
    return time.monotonic()


def elapsed_s(start_monotonic: float) -> float:
    return time.monotonic() - start_monotonic


def elapsed_ms(start_monotonic: float) -> float:
    return (time.monotonic() - start_monotonic) * 1000.0

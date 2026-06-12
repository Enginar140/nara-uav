"""PerceptionEngine -- modul kayit defteri + tek aktif modul. C++ portu (asyncio).

Aktif modulu periyodik poll eder, PERCEPTION_UPDATED olayi yayinlar. has_lock=false
miss olaylari (downstream deterministik fallback yapabilsin diye) de yayinlanir.
"""
from __future__ import annotations

import asyncio
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional

from ..core.time_utils import now_monotonic, now_us
from ..infra.event_bus import EventBus, EventType
from ..infra.log import get_logger
from ..world_model.world_model import WorldModel
from .module_base import IPerceptionModule
from .perception_types import PerceptionEvent

_log = get_logger("perception_engine")


@dataclass
class PerceptionEngineConfig:
    tick_rate_hz: float = 10.0
    min_confidence: float = 0.4
    lock_timeout_ms: float = 1000.0
    publish_misses: bool = True


class PerceptionEngine:
    def __init__(self, bus: EventBus, world_model: WorldModel,
                 config: Optional[PerceptionEngineConfig] = None):
        self._bus = bus
        self._wm = world_model
        self.cfg = config or PerceptionEngineConfig()
        self._modules: Dict[str, IPerceptionModule] = {}
        self._active: Optional[IPerceptionModule] = None
        self._active_id = ""
        self._lock = threading.RLock()
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._last_lock_t = 0.0
        self._last_miss_t = 0.0
        self._last_was_lock = False

    def register_module(self, module: IPerceptionModule) -> None:
        if module is None:
            return
        with self._lock:
            self._modules[module.id()] = module
        _log.info("modul kayitli: '%s'", module.id())

    def set_active(self, module_id: str) -> bool:
        prev = None
        nxt = None
        with self._lock:
            if not module_id:
                prev, prev_id = self._active, self._active_id
                self._active, self._active_id = None, ""
            else:
                m = self._modules.get(module_id)
                if m is None:
                    _log.warning("set_active('%s'): bilinmeyen modul", module_id)
                    return False
                if self._active_id == module_id:
                    return True
                prev, prev_id = self._active, self._active_id
                nxt = m
                self._active, self._active_id = m, module_id
        if prev:
            try:
                prev.stop()
            except Exception:  # noqa: BLE001
                _log.exception("modul stop() hatasi")
        if nxt:
            try:
                nxt.start()
            except Exception:  # noqa: BLE001
                _log.exception("modul start() hatasi")
        _log.info("aktif modul -> '%s'", module_id or "<none>")
        return True

    def active_id(self) -> str:
        with self._lock:
            return self._active_id

    def registered(self) -> List[str]:
        with self._lock:
            return sorted(self._modules.keys())

    async def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._last_lock_t = now_monotonic()
        self._last_miss_t = self._last_lock_t
        self._last_was_lock = False
        self._task = asyncio.create_task(self._loop(), name="perception")
        _log.info("PerceptionEngine basladi @ %.1f Hz", self.cfg.tick_rate_hz)

    async def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._task:
            self._task.cancel()
        self.set_active("")
        _log.info("PerceptionEngine durdu")

    async def _loop(self) -> None:
        period = max(0.02, 1.0 / max(1.0, self.cfg.tick_rate_hz))
        while self._running:
            self.tick_once()
            await asyncio.sleep(period)

    # Test edilebilir tek adim
    def tick_once(self) -> None:
        with self._lock:
            module = self._active
        result = None
        if module is not None:
            try:
                result = module.poll(self._wm.get_own_state())
            except Exception:  # noqa: BLE001
                _log.exception("poll() hatasi")
                result = None

        if result is not None and result.confidence >= self.cfg.min_confidence:
            if result.stamp_us == 0:
                result.stamp_us = now_us()
            self._bus.publish(EventType.PERCEPTION_UPDATED,
                              PerceptionEvent(has_lock=True, target=result))
            self._last_lock_t = now_monotonic()
            self._last_miss_t = self._last_lock_t
            self._last_was_lock = True
        else:
            now = now_monotonic()
            since_lock_ms = (now - self._last_lock_t) * 1000.0
            since_miss_ms = (now - self._last_miss_t) * 1000.0
            stale = since_lock_ms >= self.cfg.lock_timeout_ms
            miss_due = self._last_was_lock or since_miss_ms >= self.cfg.lock_timeout_ms
            if stale and miss_due and (self.cfg.publish_misses or self._last_was_lock):
                self._bus.publish(EventType.PERCEPTION_UPDATED,
                                  PerceptionEvent(has_lock=False))
                self._last_miss_t = now
                self._last_was_lock = False

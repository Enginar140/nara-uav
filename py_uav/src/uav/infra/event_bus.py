"""Olay veriyolu (event bus) -- asyncio tabanli pub/sub.

C++ EventBus'in dispatch-thread modelinin asyncio karsiligi:
  - publish() HERHANGI bir thread'den cagrilabilir (ornegin bloklayan MAVLink
    receive thread'inden). Olay, loop'a guvenli sekilde aktarilir.
  - tek bir dispatch task'i kuyrugu isler ve abonelere dagitir.
  - handler'lar sync veya coroutine olabilir.
"""
from __future__ import annotations

import asyncio
import itertools
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Awaitable, Callable, Dict, Optional

from .log import get_logger

_log = get_logger("event_bus")


class EventType(Enum):
    # Safety
    SAFETY_VIOLATION = auto()
    BOUNDARY_VIOLATION = auto()
    HSS_VIOLATION = auto()
    COLLISION_WARNING = auto()
    # Data
    OPPONENT_UPDATED = auto()
    HSS_ZONE_UPDATED = auto()
    BOUNDARY_UPDATED = auto()
    OWN_STATE_UPDATED = auto()
    WAYPOINTS_UPDATED = auto()
    # Mission
    MODE_CHANGED = auto()
    MODE_COMMAND_RECEIVED = auto()
    # Perception
    PERCEPTION_UPDATED = auto()
    LOCK_EVENT = auto()        # kilitlenme durum makinesi olayi (achieved/lost)
    QR_DECODED = auto()        # kamikaze QR cozuldu (payload)
    # Comm
    MAVLINK_CONNECTED = auto()
    MAVLINK_DISCONNECTED = auto()
    HEARTBEAT_TIMEOUT = auto()
    HEARTBEAT_RECOVERED = auto()
    HEARTBEAT_RECEIVED = auto()
    # Failsafe
    GPS_QUALITY_LOW = auto()
    GPS_QUALITY_CRITICAL = auto()
    BATTERY_LOW = auto()
    BATTERY_CRITICAL = auto()
    # System
    PARAMS_READY = auto()
    SYSTEM_STARTED = auto()
    SYSTEM_STOPPING = auto()


@dataclass
class Event:
    type: EventType
    payload: Any = None
    timestamp_us: int = 0


EventHandler = Callable[[Event], Optional[Awaitable[None]]]


@dataclass
class _Subscription:
    id: int
    event_type: EventType
    handler: EventHandler


class EventBus:
    def __init__(self, max_queue: int = 1000):
        self._subs: Dict[EventType, Dict[int, _Subscription]] = {}
        self._id_gen = itertools.count(1)
        self._max_queue = max_queue
        self._queue: Optional[asyncio.Queue] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._task: Optional[asyncio.Task] = None
        self._running = False

    # -- abonelik (loop disindan da cagrilabilir, sadece dict islemi) --
    def subscribe(self, event_type: EventType, handler: EventHandler) -> int:
        sub_id = next(self._id_gen)
        self._subs.setdefault(event_type, {})[sub_id] = _Subscription(
            sub_id, event_type, handler
        )
        return sub_id

    def unsubscribe(self, sub_id: int) -> None:
        for subs in self._subs.values():
            subs.pop(sub_id, None)

    # -- yasam dongusu --
    async def start(self) -> None:
        if self._running:
            return
        self._loop = asyncio.get_running_loop()
        self._queue = asyncio.Queue(maxsize=self._max_queue)
        self._running = True
        self._task = asyncio.create_task(self._dispatch_loop(), name="event_bus")
        _log.debug("EventBus basladi")

    async def stop(self) -> None:
        self._running = False
        if self._task:
            await self._queue.put(None)  # sentinel
            try:
                await asyncio.wait_for(self._task, timeout=2.0)
            except asyncio.TimeoutError:
                self._task.cancel()
        _log.debug("EventBus durdu")

    # -- yayinlama (THREAD-GUVENLI) --
    def publish(self, event_type: EventType, payload: Any = None) -> None:
        from ..core.time_utils import now_us

        ev = Event(type=event_type, payload=payload, timestamp_us=now_us())
        if not self._running or self._loop is None or self._queue is None:
            # Loop henuz yokken senkron dagit (init asamasi)
            self._dispatch_sync(ev)
            return
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is self._loop:
            self._enqueue_nowait(ev)
        else:
            self._loop.call_soon_threadsafe(self._enqueue_nowait, ev)

    def _enqueue_nowait(self, ev: Event) -> None:
        try:
            self._queue.put_nowait(ev)
        except asyncio.QueueFull:
            _log.warning("EventBus kuyrugu dolu, olay dusuruldu: %s", ev.type.name)

    def _dispatch_sync(self, ev: Event) -> None:
        for sub in list(self._subs.get(ev.type, {}).values()):
            try:
                res = sub.handler(ev)
                if asyncio.iscoroutine(res):
                    res.close()  # loop yokken coroutine calistirilamaz
            except Exception:  # noqa: BLE001
                _log.exception("Sync handler hatasi: %s", ev.type.name)

    async def _dispatch_loop(self) -> None:
        assert self._queue is not None
        while True:
            ev = await self._queue.get()
            if ev is None:
                break
            for sub in list(self._subs.get(ev.type, {}).values()):
                try:
                    res = sub.handler(ev)
                    if asyncio.iscoroutine(res):
                        await res
                except Exception:  # noqa: BLE001
                    _log.exception("Handler hatasi: %s", ev.type.name)

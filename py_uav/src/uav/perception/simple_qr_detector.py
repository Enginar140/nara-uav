"""SimpleQRDetector -- kamerasiz QR modulu. C++ portu.

Enjekte edilen tespiti (VISION_LOS veya LOCAL_NED) dondurur; beklenen payload
ayarliysa eslesmeyenleri eler. Gercek kamera entegrasyonu opencv modullerinde.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional

from ..core.time_utils import elapsed_s, now_monotonic, now_us
from ..core.types import UAVState
from .module_base import IPerceptionModule
from .perception_types import PerceptionFrame, PerceptionTarget


@dataclass
class QRDetectorConfig:
    detection_timeout_s: float = 0.7
    default_range_m: float = 60.0
    min_confidence: float = 0.6
    expected_payload: str = ""   # bos = her payload kabul


class SimpleQRDetector(IPerceptionModule):
    def __init__(self, config: Optional[QRDetectorConfig] = None):
        self.cfg = config or QRDetectorConfig()
        self._running = False
        self._lock = threading.Lock()
        self._injected: Optional[PerceptionTarget] = None
        self._injected_at = 0.0
        self._last_payload: Optional[str] = None

    def id(self) -> str:
        return "qr_simple"

    def inject_vision_los(self, yaw_off_deg: float, pitch_off_deg: float, range_m: float,
                          confidence: float, payload: str = "") -> None:
        t = PerceptionTarget(frame=PerceptionFrame.VISION_LOS, a=yaw_off_deg,
                             b=pitch_off_deg, c=range_m, confidence=confidence,
                             stamp_us=now_us(), source_tag=self.id(), label=payload)
        with self._lock:
            self._injected = t
            self._injected_at = now_monotonic()
            if payload:
                self._last_payload = payload

    def inject_local_offset(self, north_m: float, east_m: float, down_m: float,
                            confidence: float, payload: str = "") -> None:
        t = PerceptionTarget(frame=PerceptionFrame.LOCAL_NED, a=north_m, b=east_m,
                             c=down_m, confidence=confidence, stamp_us=now_us(),
                             source_tag=self.id(), label=payload)
        with self._lock:
            self._injected = t
            self._injected_at = now_monotonic()
            if payload:
                self._last_payload = payload

    def clear(self) -> None:
        with self._lock:
            self._injected = None

    def has_fresh_detection(self) -> bool:
        with self._lock:
            return self._injected is not None and \
                elapsed_s(self._injected_at) < self.cfg.detection_timeout_s

    def last_payload(self) -> Optional[str]:
        with self._lock:
            return self._last_payload

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False
        self.clear()

    def poll(self, own_state: UAVState) -> Optional[PerceptionTarget]:
        if not self._running:
            return None
        with self._lock:
            if self._injected is None:
                return None
            if elapsed_s(self._injected_at) >= self.cfg.detection_timeout_s:
                self._injected = None
                return None
            if self.cfg.expected_payload and self._injected.label != self.cfg.expected_payload:
                return None
            return self._injected

"""SimpleVisionTracker -- kamerasiz vision modulu. C++ portu.

Enjekte edilen sentetik tespiti dondurur; yoksa WorldModel'deki rakipten
VISION_LOS hedefi uretir (kamera yokken FOLLOW fallback'i icin).
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional

from ..core import geo
from ..core.time_utils import elapsed_s, now_monotonic, now_us
from ..core.types import UAVState
from ..world_model.world_model import WorldModel
from .module_base import IPerceptionModule
from .perception_types import PerceptionFrame, PerceptionTarget


@dataclass
class VisionTrackerConfig:
    detection_timeout_s: float = 1.0
    default_range_m: float = 100.0
    min_confidence: float = 0.5
    target_id: int = -1          # -1 = en yakin rakip
    use_world_fallback: bool = True


class SimpleVisionTracker(IPerceptionModule):
    def __init__(self, world_model: WorldModel, config: Optional[VisionTrackerConfig] = None):
        self._wm = world_model
        self.cfg = config or VisionTrackerConfig()
        self._running = False
        self._lock = threading.Lock()
        self._injected: Optional[PerceptionTarget] = None
        self._injected_at = 0.0

    def id(self) -> str:
        return "vision_tracker"

    def set_target_id(self, tid: int) -> None:
        self.cfg.target_id = tid

    def set_synthetic_detection(self, yaw_off_deg: float, pitch_off_deg: float,
                                range_m: float, confidence: float) -> None:
        t = PerceptionTarget(frame=PerceptionFrame.VISION_LOS, a=yaw_off_deg,
                             b=pitch_off_deg, c=range_m, confidence=confidence,
                             stamp_us=now_us(), source_tag=self.id())
        with self._lock:
            self._injected = t
            self._injected_at = now_monotonic()

    def clear_synthetic_detection(self) -> None:
        with self._lock:
            self._injected = None

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False
        self.clear_synthetic_detection()

    def poll(self, own_state: UAVState) -> Optional[PerceptionTarget]:
        if not self._running:
            return None
        with self._lock:
            if self._injected and elapsed_s(self._injected_at) < self.cfg.detection_timeout_s:
                return self._injected
            self._injected = None
        if not self.cfg.use_world_fallback:
            return None
        return self._poll_world_model(own_state)

    def _poll_world_model(self, own: UAVState) -> Optional[PerceptionTarget]:
        if not own.is_valid():
            return None
        if self.cfg.target_id >= 0:
            target = self._wm.get_opponent(self.cfg.target_id)
        else:
            target = self._wm.get_nearest_opponent(own.position)
        if target is None or not target.is_valid():
            return None
        bearing_to = geo.bearing(own.position, target.position)
        yaw_offset = geo.angle_difference(bearing_to, own.heading)
        horiz = geo.haversine_distance(own.position, target.position)
        down = own.position.altitude - target.position.altitude
        pitch_offset = math.degrees(math.atan2(down, max(1.0, horiz)))
        slant = math.sqrt(horiz * horiz + down * down)
        if slant < 1.0:
            slant = self.cfg.default_range_m
        return PerceptionTarget(
            frame=PerceptionFrame.VISION_LOS, a=yaw_offset, b=pitch_offset, c=slant,
            confidence=self.cfg.min_confidence, stamp_us=now_us(),
            source_tag=self.id(), label="world_fallback")

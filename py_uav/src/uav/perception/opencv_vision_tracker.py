"""OpenCvVisionTracker -- gercek kamera + HoughCircles yuvarlak dedektor.

C++ opencv_camera + round_detector + opencv_vision_tracker portu, tek dosyada.
OPSIYONEL: cv2 yoksa import kirilmaz; modul kurulurken anlamli hata verir.
Kamera okuma bloklayici oldugundan kendi worker thread'inde calisir; poll()
son tespiti dondurur (PerceptionEngine asyncio tick'ini bloklamaz).
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional

from ..core.time_utils import elapsed_s, now_monotonic, now_us
from ..core.types import UAVState
from ..infra.log import get_logger
from .module_base import IPerceptionModule
from .perception_types import PerceptionFrame, PerceptionTarget

_log = get_logger("opencv_vision")

try:
    import cv2  # noqa: F401
    import numpy as np
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False


@dataclass
class CameraConfig:
    type: str = "device"        # device | file | rtsp | http
    device_index: int = 0
    url: str = ""
    file_path: str = ""
    width: int = 640
    height: int = 480
    fps: float = 30.0
    loop_file: bool = True


@dataclass
class RoundDetectorConfig:
    dp: float = 1.2
    min_dist_factor: float = 0.25
    param1: float = 100.0
    param2: float = 30.0
    min_radius_px: int = 15
    max_radius_px: int = 200
    gaussian_kernel: int = 9
    use_hsv_mask: bool = False
    hsv_lo_h1: int = 0
    hsv_hi_h1: int = 10
    hsv_lo_h2: int = 170
    hsv_hi_h2: int = 179
    hsv_lo_s: int = 100
    hsv_hi_s: int = 255
    hsv_lo_v: int = 80
    hsv_hi_v: int = 255
    base_confidence: float = 0.7


@dataclass
class OpenCvVisionConfig:
    id: str = "vision_tracker_opencv"
    h_fov_deg: float = 60.0
    v_fov_deg: float = 45.0
    detection_timeout_s: float = 1.0
    min_confidence: float = 0.55
    worker_sleep_ms: int = 5
    range_mode: str = "fixed"   # fixed | size | altitude
    fixed_range_m: float = 50.0
    real_diameter_m: float = 0.30
    min_range_m: float = 1.0
    max_range_m: float = 1000.0
    camera: CameraConfig = None
    detector: RoundDetectorConfig = None

    def __post_init__(self):
        if self.camera is None:
            self.camera = CameraConfig()
        if self.detector is None:
            self.detector = RoundDetectorConfig()


@dataclass
class _FrameDetection:
    cx_px: float
    cy_px: float
    radius_px: float
    confidence: float
    label: str = "circle"


class OpenCvVisionTracker(IPerceptionModule):
    def __init__(self, config: Optional[OpenCvVisionConfig] = None):
        if not _HAS_CV2:
            raise RuntimeError(
                "OpenCvVisionTracker icin opencv-python gerekli: pip install opencv-python")
        self.cfg = config or OpenCvVisionConfig()
        dk = self.cfg.detector.gaussian_kernel
        if dk > 0 and dk % 2 == 0:
            self.cfg.detector.gaussian_kernel = dk + 1
        self._running = False
        self._cap = None
        self._worker: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._last_target: Optional[PerceptionTarget] = None
        self._last_target_t = 0.0
        self._actual_w = self.cfg.camera.width
        self._actual_h = self.cfg.camera.height

    def id(self) -> str:
        return self.cfg.id

    # -- kamera --
    def _open_camera(self) -> bool:
        c = self.cfg.camera
        src = c.device_index if c.type == "device" else (c.file_path if c.type == "file" else c.url)
        self._cap = cv2.VideoCapture(src)
        if not self._cap.isOpened():
            _log.error("kamera acilamadi: %s", src)
            return False
        if c.width:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, c.width)
        if c.height:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c.height)
        if c.fps:
            self._cap.set(cv2.CAP_PROP_FPS, c.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or c.width
        self._actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or c.height
        return True

    def start(self) -> None:
        if self._running:
            return
        if not self._open_camera():
            return
        self._running = True
        self._worker = threading.Thread(target=self._worker_loop, name="cv-vision", daemon=True)
        self._worker.start()
        _log.info("OpenCvVisionTracker basladi (%dx%d, fov %.0fx%.0f)",
                  self._actual_w, self._actual_h, self.cfg.h_fov_deg, self.cfg.v_fov_deg)

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._worker:
            self._worker.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        with self._lock:
            self._last_target = None

    def _worker_loop(self) -> None:
        while self._running:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                if self.cfg.camera.type == "file" and self.cfg.camera.loop_file:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                threading.Event().wait(0.02)
                continue
            det = self._detect(frame)
            if det and det.confidence >= self.cfg.min_confidence:
                cx0 = self._actual_w * 0.5
                cy0 = self._actual_h * 0.5
                fx = cx0 / math.tan(math.radians(self.cfg.h_fov_deg * 0.5))
                fy = cy0 / math.tan(math.radians(self.cfg.v_fov_deg * 0.5))
                yaw_off = math.degrees(math.atan2(det.cx_px - cx0, fx))
                pitch_off = math.degrees(math.atan2(det.cy_px - cy0, fy))
                rng = self._estimate_range(det)
                t = PerceptionTarget(
                    frame=PerceptionFrame.VISION_LOS, a=yaw_off, b=pitch_off, c=rng,
                    confidence=det.confidence, stamp_us=now_us(),
                    source_tag=self.id(), label=det.label)
                with self._lock:
                    self._last_target = t
                    self._last_target_t = now_monotonic()
            if self.cfg.worker_sleep_ms > 0:
                threading.Event().wait(self.cfg.worker_sleep_ms / 1000.0)

    def _detect(self, bgr) -> Optional[_FrameDetection]:
        d = self.cfg.detector
        if d.use_hsv_mask:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            m1 = cv2.inRange(hsv, (d.hsv_lo_h1, d.hsv_lo_s, d.hsv_lo_v),
                             (d.hsv_hi_h1, d.hsv_hi_s, d.hsv_hi_v))
            m2 = cv2.inRange(hsv, (d.hsv_lo_h2, d.hsv_lo_s, d.hsv_lo_v),
                             (d.hsv_hi_h2, d.hsv_hi_s, d.hsv_hi_v))
            gray = cv2.bitwise_or(m1, m2)
            gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN,
                                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
            gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE,
                                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
        else:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if d.gaussian_kernel >= 3:
            gray = cv2.GaussianBlur(gray, (d.gaussian_kernel, d.gaussian_kernel), 0)
        min_dim = min(bgr.shape[1], bgr.shape[0])
        min_dist = max(10.0, d.min_dist_factor * min_dim)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, d.dp, min_dist,
                                   param1=d.param1, param2=d.param2,
                                   minRadius=d.min_radius_px, maxRadius=d.max_radius_px)
        if circles is None:
            return None
        circles = circles[0]
        best = max(circles, key=lambda c: c[2])
        rel = min(1.0, best[2] / float(min_dim) * 4.0)
        conf = min(1.0, d.base_confidence + 0.3 * rel)
        return _FrameDetection(cx_px=float(best[0]), cy_px=float(best[1]),
                               radius_px=float(best[2]), confidence=conf)

    def _estimate_range(self, det: _FrameDetection) -> float:
        mode = self.cfg.range_mode
        r = self.cfg.fixed_range_m
        if mode == "size" and det.radius_px > 1.0:
            cx0 = self._actual_w * 0.5
            fx = cx0 / math.tan(math.radians(self.cfg.h_fov_deg * 0.5))
            r = self.cfg.real_diameter_m * fx / (2.0 * det.radius_px)
        return max(self.cfg.min_range_m, min(self.cfg.max_range_m, r))

    def poll(self, own_state: UAVState) -> Optional[PerceptionTarget]:
        if not self._running:
            return None
        with self._lock:
            snap = self._last_target
            t = self._last_target_t
        if snap is None:
            return None
        if elapsed_s(t) > self.cfg.detection_timeout_s:
            return None
        return snap

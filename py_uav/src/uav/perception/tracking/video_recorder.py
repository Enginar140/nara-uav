"""LockVideoRecorder -- sartname uyumlu degerlendirme videosu yazici.

Sartname (6.1.1) gereksinimleri:
  - Sabit frame rate, min 15 fps.
  - Min 640x480, 4:3/5:4/16:9.
  - Kilit suresince her frame'e AH (kilit dortgeni) KIRMIZI (#FF0000), <=3 px.
  - Sag ust kosede ms hassasiyetinde SUNUCU saati.
  - H264/MP4, OpenCV 4.5 + FFPLAY ile acilabilir.
  - Postprocessing/zoom/cizim (AH/AV disinda) YOK; ham kamera goruntusu.

Bu sinif yalniz AV (referans) + AH (kilit) dortgenleri ve sunucu saatini cizer.
cv2 yoksa import kirilmaz; open() False doner.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from ...infra.log import get_logger
from .detection import BBox

_log = get_logger("lock_video")

try:
    import cv2  # type: ignore
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

RED_BGR = (0, 0, 255)      # #FF0000
WHITE_BGR = (255, 255, 255)


@dataclass
class VideoConfig:
    path: str = "lock_record.mp4"
    fps: float = 15.0                 # sabit (sartname min 15)
    fourcc: str = "mp4v"             # yarisma icin 'avc1' (H264) tercih edilir
    draw_av: bool = True             # AV referans dortgenini ciz (acik gri)
    ah_thickness: int = 2            # <= 3 px (sartname)
    clock_scale: float = 0.6
    write_when_unlocked: bool = True  # kilit yokken de ham frame yaz


class LockVideoRecorder:
    def __init__(self, config: Optional[VideoConfig] = None):
        self.cfg = config or VideoConfig()
        self._writer = None
        self._size: Tuple[int, int] = (0, 0)
        self.frames_written = 0

    @property
    def is_open(self) -> bool:
        return self._writer is not None

    def open(self, width: int, height: int) -> bool:
        if not _HAS_CV2:
            _log.warning("cv2 yok; video kaydi devre disi")
            return False
        fourcc = cv2.VideoWriter_fourcc(*self.cfg.fourcc)
        self._writer = cv2.VideoWriter(self.cfg.path, fourcc, self.cfg.fps,
                                       (width, height))
        if not self._writer.isOpened():
            _log.error("VideoWriter acilamadi: %s (%s)", self.cfg.path, self.cfg.fourcc)
            self._writer = None
            return False
        self._size = (width, height)
        _log.info("Video kaydi: %s %dx%d @%.0ffps (%s)", self.cfg.path,
                  width, height, self.cfg.fps, self.cfg.fourcc)
        return True

    def write(self, frame_bgr, server_ms: int,
              av_box: Optional[BBox] = None,
              lock_box: Optional[BBox] = None,
              locked: bool = False) -> bool:
        """Bir frame'i (gerekli anotasyonlarla) yaz."""
        if self._writer is None:
            return False
        if not locked and not self.cfg.write_when_unlocked:
            return False
        canvas = frame_bgr  # ham frame uzerine yalniz AV/AH + saat
        if self.cfg.draw_av and av_box is not None:
            self._rect(canvas, av_box, (160, 160, 160), 1)
        if locked and lock_box is not None:
            self._rect(canvas, lock_box, RED_BGR, max(1, min(3, self.cfg.ah_thickness)))
        self._draw_clock(canvas, server_ms)
        self._writer.write(canvas)
        self.frames_written += 1
        return True

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None

    # ---- cizim ----
    def _rect(self, img, box: BBox, color, thickness: int) -> None:
        l, t, r, b = box.to_ltrb()
        cv2.rectangle(img, (int(l), int(t)), (int(r), int(b)), color, thickness)

    def _draw_clock(self, img, server_ms: int) -> None:
        text = str(int(server_ms))
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = self.cfg.clock_scale
        (tw, th), _ = cv2.getTextSize(text, font, scale, 2)
        w = self._size[0]
        x = max(0, w - tw - 8)
        y = th + 8
        # okunabilirlik icin koyu zemin + beyaz yazi
        cv2.rectangle(img, (x - 4, 2), (x + tw + 4, y + 6), (0, 0, 0), -1)
        cv2.putText(img, text, (x, y), font, scale, WHITE_BGR, 2, cv2.LINE_AA)


def video_available() -> bool:
    return _HAS_CV2

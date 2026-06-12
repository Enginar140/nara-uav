"""LockVideoRecorder -- sartname uyumlu degerlendirme videosu yazici.

Sartname (6.1.1) gereksinimleri:
  - SABIT frame rate, min 15 fps. -> duvar-saati (sunucu ms) bazli pacing:
    kamera hizindan bagimsiz, her 1/fps periyodu icin TAM BIR frame yazilir
    (kamera yavassa son kare cogaltilir, hizliysa ara kareler atlanir).
  - Min 640x480, 4:3/5:4/16:9. -> writer ILK GERCEK frame boyutuyla acilir
    (kamera istenen cozunurlugu vermezse sessiz bos-video tuzagi olmaz).
  - Kilit suresince her frame'e AH (kilit dortgeni) KIRMIZI (#FF0000), <=3 px.
  - Sag ust kosede ms hassasiyetinde SUNUCU saati.
  - H264/MP4 (avc1); acilamazsa mp4v'ye duser (uyari ile).
  - Var olan dosya ASLA ezilmez: ayni yol doluysa zaman damgali yeni dosya
    acilir (modul yeniden baslarsa onceki kanit korunur).

Bu sinif yalniz AV (referans) + AH (kilit) dortgenleri ve sunucu saatini cizer.
cv2 yoksa import kirilmaz; open() False doner.
"""
from __future__ import annotations

import os
import time
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
    fourcc: str = "avc1"             # H264; acilamazsa mp4v fallback
    draw_av: bool = True             # AV referans dortgenini ciz (acik gri)
    ah_thickness: int = 2            # <= 3 px (sartname)
    clock_scale: float = 0.6
    write_when_unlocked: bool = True  # kilit yokken de ham frame yaz
    max_dup_per_call: int = 8        # tek cagrida en fazla kare cogaltma
    rebase_gap_frames: int = 60      # saat sicramasinda pacing'i yeniden kur


class LockVideoRecorder:
    def __init__(self, config: Optional[VideoConfig] = None):
        self.cfg = config or VideoConfig()
        self._writer = None
        self._pending = False         # open() cagrildi, writer ilk frame'de kurulur
        self._size: Tuple[int, int] = (0, 0)
        self._actual_path = ""
        # sabit-fps pacing durumu
        self._start_ms: Optional[int] = None
        self.frames_written = 0
        self.frames_skipped = 0

    @property
    def is_open(self) -> bool:
        return self._writer is not None or self._pending

    @property
    def actual_path(self) -> str:
        return self._actual_path

    def open(self, width: int, height: int) -> bool:
        """Kaydi 'ac': writer ilk gercek frame'in boyutuyla kurulur.

        width/height yalniz bilgi amaclidir; kamera farkli cozunurluk verirse
        gercek boyut kullanilir (cv2.VideoWriter boyut uyusmazliginda frame'leri
        SESSIZCE atar -- bos kanit videosu tuzagi).
        """
        if not _HAS_CV2:
            _log.warning("cv2 yok; video kaydi devre disi")
            return False
        self._pending = True
        self._start_ms = None
        return True

    def write(self, frame_bgr, server_ms: int,
              av_box: Optional[BBox] = None,
              lock_box: Optional[BBox] = None,
              locked: bool = False) -> bool:
        """Bir kamera frame'ini sabit-fps pacing ile yaz.

        Donus: en az bir kare yazildiysa True. Kamera fps > kayit fps ise ara
        cagrilar False doner (kare atlandi) -- bu normaldir.
        """
        if self._writer is None:
            if not self._pending:
                return False
            if not self._create_writer(frame_bgr):
                return False
        if not locked and not self.cfg.write_when_unlocked:
            return False

        period_ms = 1000.0 / max(1.0, self.cfg.fps)
        if self._start_ms is None:
            self._start_ms = int(server_ms)
        due = int((server_ms - self._start_ms) / period_ms) + 1
        if due - self.frames_written > self.cfg.rebase_gap_frames:
            # sunucu saati sicradi (sync) -> pacing'i yeniden kur, tek kare yaz
            _log.warning("video pacing yeniden kuruldu (saat sicramasi)")
            self._start_ms = int(server_ms - self.frames_written * period_ms)
            due = self.frames_written + 1
        if due <= self.frames_written:
            self.frames_skipped += 1
            return False  # bu periyodun karesi zaten yazildi

        canvas = frame_bgr  # ham frame uzerine yalniz AV/AH + saat
        if self.cfg.draw_av and av_box is not None:
            self._rect(canvas, av_box, (160, 160, 160), 1)
        if locked and lock_box is not None:
            self._rect(canvas, lock_box, RED_BGR, max(1, min(3, self.cfg.ah_thickness)))
        self._draw_clock(canvas, server_ms)
        n = min(due - self.frames_written, self.cfg.max_dup_per_call)
        for _ in range(n):
            self._writer.write(canvas)
            self.frames_written += 1
        return True

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            _log.info("Video kapandi: %s (%d kare)", self._actual_path,
                      self.frames_written)
        self._pending = False

    # ---- writer kurulum ----
    def _create_writer(self, frame_bgr) -> bool:
        h, w = frame_bgr.shape[:2]
        path = self._unique_path(self.cfg.path)
        writer = self._try_fourcc(path, self.cfg.fourcc, w, h)
        if writer is None and self.cfg.fourcc.lower() != "mp4v":
            _log.warning("fourcc '%s' acilamadi, mp4v'ye dusuluyor (yarisma "
                         "H264 ister; openh264/ffmpeg kurulumu kontrol edin)",
                         self.cfg.fourcc)
            writer = self._try_fourcc(path, "mp4v", w, h)
        if writer is None:
            _log.error("VideoWriter acilamadi: %s", path)
            self._pending = False
            return False
        self._writer = writer
        self._size = (w, h)
        self._actual_path = path
        self._pending = False
        self.frames_written = 0
        self.frames_skipped = 0
        self._start_ms = None
        _log.info("Video kaydi: %s %dx%d @%.0ffps", path, w, h, self.cfg.fps)
        return True

    def _try_fourcc(self, path: str, fourcc: str, w: int, h: int):
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc),
                                 float(max(1.0, self.cfg.fps)), (w, h))
        return writer if writer.isOpened() else None

    @staticmethod
    def _unique_path(path: str) -> str:
        """Var olan dosyayi ezme: doluysa zaman damgali yeni ad uret."""
        if not os.path.exists(path):
            return path
        stem, ext = os.path.splitext(path)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        candidate = f"{stem}_{stamp}{ext}"
        i = 1
        while os.path.exists(candidate):
            candidate = f"{stem}_{stamp}_{i}{ext}"
            i += 1
        return candidate

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

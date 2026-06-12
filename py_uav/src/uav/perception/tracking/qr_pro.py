"""QrProDetector -- profesyonel QR okuma (kamikaze gorevi).

- cv2.QRCodeDetector birincil; pyzbar (varsa) yedek -> daha yuksek okuma orani.
- ROI: onceki tespit etrafinda kirpip arar -> yuksek FPS.
- Multi-scale: kucuk/uzak QR icin ROI'yi buyuterek tekrar dener.
- Sartname: QR 2.5x2.5 m, 45 derece plakali; dalista okunur, mesaj <=2 sn icinde.

cv2 yoksa import kirilmaz.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from ...core.time_utils import now_us
from ...infra.log import get_logger
from .detection import BBox

_log = get_logger("qr_pro")

try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

try:
    from pyzbar import pyzbar  # type: ignore
    _HAS_PYZBAR = True
except Exception:  # noqa: BLE001
    _HAS_PYZBAR = False


@dataclass
class QrProConfig:
    use_pyzbar_fallback: bool = True
    roi_pad_frac: float = 0.6      # son kutu etrafinda +%60 dolgu ile ROI
    upscale_factors: Tuple[float, ...] = (1.0, 2.0)  # multi-scale ROI denemeleri
    min_box_px: int = 16
    roi_miss_limit: int = 8        # bu kadar ardisik ROI miss -> tam kareye don


@dataclass
class QrResult:
    payload: str
    box: BBox
    stamp_us: int


class QrProDetector:
    def __init__(self, config: Optional[QrProConfig] = None):
        if not _HAS_CV2:
            raise RuntimeError("QrProDetector icin opencv-python gerekli")
        self.cfg = config or QrProConfig()
        self._cv = cv2.QRCodeDetector()
        self._last_box: Optional[BBox] = None
        self._miss = 0

    def detect(self, frame_bgr) -> Optional[QrResult]:
        if frame_bgr is None:
            return None
        h, w = frame_bgr.shape[:2]
        # 1) ROI denemesi (varsa) -> 2) tam kare
        regions = []
        if self._last_box is not None and self._miss < self.cfg.roi_miss_limit:
            regions.append(self._roi_rect(self._last_box, w, h))
        regions.append((0, 0, w, h))

        for (rx, ry, rw, rh) in regions:
            crop = frame_bgr[ry:ry + rh, rx:rx + rw]
            if crop.size == 0:
                continue
            for scale in self.cfg.upscale_factors:
                res = self._decode(crop, scale, offset=(rx, ry), frame_wh=(w, h))
                if res is not None:
                    self._last_box = res.box
                    self._miss = 0
                    return res
        self._miss += 1
        if self._miss >= self.cfg.roi_miss_limit:
            self._last_box = None
        return None

    # ---- ic ----
    def _decode(self, crop, scale: float, offset, frame_wh) -> Optional[QrResult]:
        img = crop
        if scale != 1.0:
            img = cv2.resize(crop, None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_LINEAR)
        ox, oy = offset
        # birincil: OpenCV
        data, pts = self._decode_cv(img)
        if not data and self.cfg.use_pyzbar_fallback and _HAS_PYZBAR:
            data, pts = self._decode_pyzbar(img)
        if not data or pts is None:
            return None
        box = self._pts_to_box(pts, scale, ox, oy)
        if box is None or box.w < self.cfg.min_box_px or box.h < self.cfg.min_box_px:
            return None
        return QrResult(payload=data, box=box, stamp_us=now_us())

    def _decode_cv(self, img):
        try:
            data, points, _ = self._cv.detectAndDecode(img)
        except cv2.error:
            return "", None
        if data and points is not None and len(points) > 0:
            return data, points.reshape(-1, 2)
        return "", None

    def _decode_pyzbar(self, img):
        try:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            for sym in pyzbar.decode(gray):
                if sym.type == "QRCODE" and sym.data:
                    r = sym.rect
                    pts = np.array([[r.left, r.top], [r.left + r.width, r.top],
                                    [r.left + r.width, r.top + r.height],
                                    [r.left, r.top + r.height]], dtype=float)
                    return sym.data.decode("utf-8", "replace"), pts
        except Exception:  # noqa: BLE001
            return "", None
        return "", None

    def _pts_to_box(self, pts, scale: float, ox: float, oy: float) -> Optional[BBox]:
        try:
            xs = [float(p[0]) / scale + ox for p in pts]
            ys = [float(p[1]) / scale + oy for p in pts]
        except (TypeError, IndexError):
            return None
        return BBox.from_ltrb(min(xs), min(ys), max(xs), max(ys))

    def _roi_rect(self, box: BBox, w: int, h: int):
        pad = self.cfg.roi_pad_frac
        bw = box.w * (1.0 + 2 * pad)
        bh = box.h * (1.0 + 2 * pad)
        x = int(max(0, box.cx - bw * 0.5))
        y = int(max(0, box.cy - bh * 0.5))
        rw = int(min(w - x, bw))
        rh = int(min(h - y, bh))
        return (x, y, max(1, rw), max(1, rh))

    def reset(self) -> None:
        self._last_box = None
        self._miss = 0


def qr_pro_available() -> bool:
    return _HAS_CV2

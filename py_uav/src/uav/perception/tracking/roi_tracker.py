"""RoiTracker -- OpenCV gorsel takipci ile YOLO frame'leri ARASINI doldurur.

Amac: YOLO her frame'de pahali; onun yerine N frame'de bir YOLO calistir, arada
hafif bir gorsel takipci (CSRT > KCF > MOSSE) ile kutuyu surdur -> yuksek FPS.
cv2 yoksa import kirilmaz.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ...infra.log import get_logger
from .detection import BBox

_log = get_logger("roi_tracker")

try:
    import cv2  # type: ignore
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False


def _try_create(kind: str):
    """Bir takipci turunu farkli OpenCV API stilleriyle olusturmayi dene.

    Surum/paket farklari: yeni 'cv2.TrackerCSRT.create()', eski
    'cv2.TrackerCSRT_create()', contrib 'cv2.legacy.TrackerCSRT_create()'.
    """
    paths = (
        f"Tracker{kind}.create",       # 4.5.2+ ana namespace, classmethod
        f"Tracker{kind}_create",       # eski fabrika fonksiyonu
        f"legacy.Tracker{kind}_create",  # opencv-contrib legacy
    )
    for path in paths:
        obj = cv2
        try:
            for part in path.split("."):
                obj = getattr(obj, part)
            return obj()
        except (AttributeError, cv2.error):
            continue
    return None


# CSRT/KCF contrib'de; temel opencv-python'da yoksa MIL/GOTURN'a dus.
_FALLBACK_ORDER = ("CSRT", "KCF", "MOSSE", "MIL")


def _make_tracker(kind: str):
    """Istenen turu, yoksa siradaki uygun turu olustur (en az MIL temelde var)."""
    kind = kind.upper()
    order = [kind] + [k for k in _FALLBACK_ORDER if k != kind]
    for k in order:
        trk = _try_create(k)
        if trk is not None:
            if k != kind:
                _log.warning("Takipci '%s' yok; '%s' kullaniliyor", kind, k)
            return trk
    return None


@dataclass
class RoiConfig:
    tracker_type: str = "CSRT"   # CSRT (dogru) | KCF (hizli) | MOSSE (cok hizli)
    max_age: int = 30            # bu kadar frame'den sonra YOLO ile yenile
    min_box_px: int = 6


class RoiTracker:
    def __init__(self, config: Optional[RoiConfig] = None):
        if not _HAS_CV2:
            raise RuntimeError("RoiTracker icin opencv-python gerekli")
        self.cfg = config or RoiConfig()
        self._tracker = None
        self._age = 0
        self._ok = False
        self._frame_wh = (0, 0)

    @property
    def active(self) -> bool:
        return self._ok and self._tracker is not None

    @property
    def needs_refresh(self) -> bool:
        """YOLO ile yeniden tespit gerekiyor mu (takip koptu ya da yaslandi)."""
        return (not self.active) or (self._age >= self.cfg.max_age)

    def init(self, frame_bgr, box: BBox) -> bool:
        """YOLO tespiti ile takipciyi (yeniden) baslat."""
        self._tracker = _make_tracker(self.cfg.tracker_type)
        if self._tracker is None:
            _log.warning("OpenCV takipci olusturulamadi (%s)", self.cfg.tracker_type)
            self._ok = False
            return False
        h, w = frame_bgr.shape[:2]
        self._frame_wh = (w, h)
        x = max(0.0, box.left)
        y = max(0.0, box.top)
        bw = min(box.w, w - x)
        bh = min(box.h, h - y)
        if bw < self.cfg.min_box_px or bh < self.cfg.min_box_px:
            self._ok = False
            return False
        # OpenCV 4.x init/update tamsayi bbox bekler (int piksel)
        self._tracker.init(frame_bgr, (int(x), int(y), int(bw), int(bh)))
        self._ok = True
        self._age = 0
        return True

    def update(self, frame_bgr) -> Optional[BBox]:
        """Yeni frame'de kutuyu surdur; basarisizsa None."""
        if not self.active:
            return None
        ok, rect = self._tracker.update(frame_bgr)
        self._age += 1
        if not ok:
            self._ok = False
            return None
        x, y, bw, bh = rect
        if bw < self.cfg.min_box_px or bh < self.cfg.min_box_px:
            self._ok = False
            return None
        return BBox.from_xywh(float(x), float(y), float(bw), float(bh))

    def clear(self) -> None:
        self._tracker = None
        self._ok = False
        self._age = 0


def roi_available() -> bool:
    return _HAS_CV2

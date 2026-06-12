"""Takip cekirdek tipleri + sartname geometrisi.

Saf veri/geometri; cv2/numpy gerektirmez (kamerasiz test edilebilir).

Sartname terimleri (2025 Savasan IHA):
  AK : Kamera Gorus Alani (tum kare)
  AV : Hedef Vurus Alani  -> karenin merkezindeki dortgen; hedef MERKEZI burada olmali
  AH : Kilitlenme Dortgeni -> hedefin etrafina cizilen kutu (bizde = tespit kutusu)
  HH : Hedef Hava Araci

Kilit gecerlilik kosullari (her frame icin):
  1) Hedef merkezi AV icinde.
  2) Hedef, yatay/dikey eksenlerden EN AZ BIRINDE kareyi >= lock_min_frac kaplar
     (sartname %5; algoritma esigi olarak %6+ onerilir -> config'te).
  3) AH hedefin >= %90'ini icermeli ve AH merkezi ile hedef merkezi yatayda
     hedef_genisligi/2, dikeyde hedef_yuksekligi/2'den yakin olmali. Bizde AH =
     tespit kutusu oldugundan bu sart yapisal olarak saglanir; yine de dis
     kaynakli AH icin dogrulama fonksiyonu verilir.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class BBox:
    """Piksel cinsinden eksen-hizali kutu; (cx,cy) merkez, (w,h) boyut."""
    cx: float
    cy: float
    w: float
    h: float

    @property
    def left(self) -> float:
        return self.cx - self.w * 0.5

    @property
    def right(self) -> float:
        return self.cx + self.w * 0.5

    @property
    def top(self) -> float:
        return self.cy - self.h * 0.5

    @property
    def bottom(self) -> float:
        return self.cy + self.h * 0.5

    @property
    def area(self) -> float:
        return max(0.0, self.w) * max(0.0, self.h)

    @property
    def center(self) -> Tuple[float, float]:
        return (self.cx, self.cy)

    @staticmethod
    def from_xywh(x: float, y: float, w: float, h: float) -> "BBox":
        """Sol-ust kosesi (x,y) + boyut -> merkezli BBox."""
        return BBox(cx=x + w * 0.5, cy=y + h * 0.5, w=w, h=h)

    @staticmethod
    def from_ltrb(left: float, top: float, right: float, bottom: float) -> "BBox":
        return BBox(cx=(left + right) * 0.5, cy=(top + bottom) * 0.5,
                    w=right - left, h=bottom - top)

    def to_ltrb(self) -> Tuple[float, float, float, float]:
        return (self.left, self.top, self.right, self.bottom)

    def intersection_area(self, other: "BBox") -> float:
        lx = max(self.left, other.left)
        ly = max(self.top, other.top)
        rx = min(self.right, other.right)
        ry = min(self.bottom, other.bottom)
        if rx <= lx or ry <= ly:
            return 0.0
        return (rx - lx) * (ry - ly)


@dataclass
class Detection:
    """Bir frame'deki tek tespit. box piksel cinsinden; frame boyutu birlikte tasinir."""
    box: BBox
    frame_w: int
    frame_h: int
    confidence: float = 0.0
    class_id: int = 0
    label: str = ""
    track_id: int = -1
    stamp_us: int = 0

    def axis_fraction(self) -> float:
        """Hedefin yatay/dikey eksenlerden EN BUYUGUNDE kapladigi kare orani.

        Sartname: 'yatay ve dikey eksenlerden en az birinde en az %5'.
        """
        fw = self.box.w / self.frame_w if self.frame_w > 0 else 0.0
        fh = self.box.h / self.frame_h if self.frame_h > 0 else 0.0
        return max(fw, fh)

    def area_fraction(self) -> float:
        denom = float(self.frame_w * self.frame_h)
        return self.box.area / denom if denom > 0 else 0.0

    def center_offset_norm(self) -> Tuple[float, float]:
        """Merkeze gore normalize ofset [-1,1]; (+x sag, +y asagi). PID girdisi."""
        if self.frame_w <= 0 or self.frame_h <= 0:
            return (0.0, 0.0)
        nx = (self.box.cx - self.frame_w * 0.5) / (self.frame_w * 0.5)
        ny = (self.box.cy - self.frame_h * 0.5) / (self.frame_h * 0.5)
        return (nx, ny)


@dataclass(frozen=True)
class FrameGeometry:
    """Kare boyutu + AV (hedef vurus alani) tanimi.

    AV, karenin merkezinde, genislik/yukseklik kare boyutunun bir orani kadar
    olan bir dortgendir. Kesin oran sartname Sekil 2'ye gore ayarlanir (config).
    """
    width: int
    height: int
    av_w_frac: float = 0.40   # AV genisligi / kare genisligi
    av_h_frac: float = 0.40   # AV yuksekligi / kare yuksekligi

    def av_box(self) -> BBox:
        return BBox(cx=self.width * 0.5, cy=self.height * 0.5,
                    w=self.width * self.av_w_frac, h=self.height * self.av_h_frac)

    def center_in_av(self, cx: float, cy: float) -> bool:
        av = self.av_box()
        return (av.left <= cx <= av.right) and (av.top <= cy <= av.bottom)


def lock_box_valid(target: BBox, lock_box: BBox, min_cover: float = 0.90) -> bool:
    """AH (lock_box) kilitlenme dortgeni gecerli mi (sartname).

    - lock_box hedefin >= min_cover (%90) kadarini icermeli,
    - lock_box merkezi ile hedef merkezi yatayda hedef_w/2, dikeyde hedef_h/2'den
      daha uzak olmamali.
    """
    if target.area <= 0.0:
        return False
    cover = lock_box.intersection_area(target) / target.area
    if cover < min_cover:
        return False
    if abs(lock_box.cx - target.cx) > target.w * 0.5:
        return False
    if abs(lock_box.cy - target.cy) > target.h * 0.5:
        return False
    return True


def frame_lock_valid(det: Detection, geom: FrameGeometry,
                     lock_min_frac: float = 0.06) -> bool:
    """Tek frame kilit-uygun mu: hedef merkezi AV icinde VE boyut >= esik.

    lock_min_frac sartname %5'in uzerinde (tolerans icin %6 onerilir).
    """
    if det.frame_w != geom.width or det.frame_h != geom.height:
        # boyut uyusmazligi -> normalize oranlarla yine de degerlendir
        pass
    if det.axis_fraction() < lock_min_frac:
        return False
    return geom.center_in_av(det.box.cx, det.box.cy)

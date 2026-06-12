"""CostMap -- sert duvar + yumusak maliyet alani (analitik, raster yok).

Hybrid A* icin iki katman:
  HARD  : HSS yaricapi + hss_buffer + izleme hatasi payi VE sinir poligonu
          (boundary_buffer kadar iceri cekilmis). ASLA ihlal edilmez.
  SOFT  : hard kenardan soft_band_m boyunca dogrusal azalan maliyet. Genis
          alan varsa planlayici uzaktan dolasir; koridor tek secenekse maliyeti
          odeyerek icinden gecer ("yol yok" yalniz gercek imkansizlikta).

Ek olarak homotopi yan-tercihi (side bias): onceki planin gectigi yan
korunsun diye, bolgenin "diger yanindaki" noktalar soft banda ek maliyet alir.
HSS daireleri saf matematikle (hizli), sinir shapely prepared ile kontrol edilir.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from shapely.geometry import LineString, Point, Polygon
from shapely.prepared import prep


@dataclass(frozen=True)
class ZoneCircle:
    zone_id: int
    cx: float
    cy: float
    hard_r: float       # yaricap + hss_buffer + izleme payi (sert duvar)


@dataclass
class CostMapConfig:
    soft_band_m: float = 130.0          # hard kenardan itibaren maliyet bandi
    soft_cost_max: float = 4.0          # band ic kenarindaki birim-maliyet carpani
    boundary_soft_band_m: float = 60.0  # sinira yakinlik maliyet bandi
    boundary_cost_max: float = 1.5
    side_bias_cost: float = 1.0         # homotopi yan-degisim cezasi (banda ek)


class CostMap:
    """Tek planlama cagrisi icin sert/yumusak uzay sorgulari (xy metre)."""

    def __init__(self, zones: Sequence[ZoneCircle],
                 boundary_xy: Optional[Sequence[Tuple[float, float]]],
                 boundary_buffer_m: float,
                 cfg: Optional[CostMapConfig] = None,
                 side_bias: Optional[Dict[int, int]] = None,
                 chord: Optional[Tuple[float, float, float, float]] = None):
        self.cfg = cfg or CostMapConfig()
        self.zones: List[ZoneCircle] = list(zones)
        self._side_bias = dict(side_bias or {})
        self._chord = chord  # (sx, sy, gx, gy): yan isareti icin referans dogru

        self.boundary_poly: Optional[Polygon] = None
        self.allowed: Optional[Polygon] = None
        self._allowed_prep = None
        self._allowed_exterior = None
        # sinir-yakinlik maliyeti icin hucre onbellegi (ayni hucre, cok heading)
        self._bcost_cache: Dict[Tuple[int, int], float] = {}
        if boundary_xy is not None and len(boundary_xy) >= 3:
            poly = Polygon(boundary_xy)
            if not poly.is_valid:
                poly = poly.buffer(0)
            self.boundary_poly = poly
            allowed = poly.buffer(-max(0.0, boundary_buffer_m))
            if allowed.is_empty:
                allowed = poly
            self.allowed = allowed
            self._allowed_prep = prep(allowed)
            self._allowed_exterior = allowed.boundary

    # ===================== HARD =====================
    def hard_free(self, x: float, y: float) -> bool:
        for z in self.zones:
            if math.hypot(x - z.cx, y - z.cy) < z.hard_r:
                return False
        if self._allowed_prep is not None:
            if not self._allowed_prep.contains(Point(x, y)):
                return False
        return True

    def polyline_in_allowed(self, pts: Sequence[Tuple[float, float]]) -> bool:
        """Kirik cizgi izinli bolgenin (sinir - buffer) icinde mi?"""
        if self._allowed_prep is None or len(pts) < 2:
            return True
        return self._allowed_prep.covers(LineString(pts))

    def segment_hard_free(self, ax: float, ay: float, bx: float, by: float) -> bool:
        for z in self.zones:
            if _seg_circle_hit(ax, ay, bx, by, z.cx, z.cy, z.hard_r):
                return False
        if self._allowed_prep is not None:
            if not self._allowed_prep.covers(LineString([(ax, ay), (bx, by)])):
                return False
        return True

    # ===================== SOFT =====================
    def point_cost(self, x: float, y: float) -> float:
        """Birim mesafe basina EK maliyet (0 = tamamen serbest alan)."""
        c = 0.0
        band = self.cfg.soft_band_m
        for z in self.zones:
            d = math.hypot(x - z.cx, y - z.cy) - z.hard_r
            if d < band:
                frac = 1.0 - max(0.0, d) / band
                c += self.cfg.soft_cost_max * frac * frac
                bias = self._side_bias.get(z.zone_id)
                if bias and self._chord is not None:
                    if self._point_side(x, y, z) == -bias:
                        c += self.cfg.side_bias_cost
        if self._allowed_exterior is not None and self.cfg.boundary_soft_band_m > 0.0:
            ck = (int(x // 20.0), int(y // 20.0))
            bc = self._bcost_cache.get(ck)
            if bc is None:
                db = self._allowed_exterior.distance(Point(x, y))
                band = self.cfg.boundary_soft_band_m
                bc = self.cfg.boundary_cost_max * (1.0 - db / band) if db < band else 0.0
                self._bcost_cache[ck] = bc
            c += bc
        return c

    def segment_cost(self, ax: float, ay: float, bx: float, by: float) -> float:
        """Orta + uc nokta ornekli yaklasik ortalama birim maliyet."""
        mx, my = (ax + bx) * 0.5, (ay + by) * 0.5
        return 0.5 * (self.point_cost(mx, my) + self.point_cost(bx, by))

    def _point_side(self, x: float, y: float, z: ZoneCircle) -> int:
        """start->goal dogrusunun bolge merkezine gore noktanin yani (+1/-1)."""
        sx, sy, gx, gy = self._chord
        cross = (gx - sx) * (y - z.cy) - (gy - sy) * (x - z.cx)
        return 1 if cross >= 0.0 else -1

    # ===================== homotopi imzasi =====================
    def signature_of(self, pts: Sequence[Tuple[float, float]]) -> Tuple[Tuple[int, int], ...]:
        """Yolun her bolgeyi hangi yandan gectigi: (zone_id, +1/-1) listesi.

        Bolge merkezi etrafinda suprulen toplam aci isareti kullanilir; cok
        uzaktan/etkisiz gecisler (suprulen aci kucuk) imzaya girmez.
        """
        sig: List[Tuple[int, int]] = []
        for z in self.zones:
            total = 0.0
            prev_ang: Optional[float] = None
            relevant = False
            for (x, y) in pts:
                dx, dy = x - z.cx, y - z.cy
                if math.hypot(dx, dy) < z.hard_r + self.cfg.soft_band_m * 2.0:
                    relevant = True
                ang = math.atan2(dy, dx)
                if prev_ang is not None:
                    d = ang - prev_ang
                    while d > math.pi:
                        d -= 2.0 * math.pi
                    while d < -math.pi:
                        d += 2.0 * math.pi
                    total += d
                prev_ang = ang
            if relevant and abs(total) > 0.15:
                # isaret _point_side ile tutarli: +1 = start->goal dogrusunun
                # sol yani (merkez etrafinda CW suprulen yol solda kalir)
                sig.append((z.zone_id, 1 if total < 0.0 else -1))
        return tuple(sig)


def _seg_circle_hit(ax: float, ay: float, bx: float, by: float,
                    cx: float, cy: float, r: float) -> bool:
    """[a,b] dogru parcasi (cx,cy,r) dairesine girer mi (saf matematik)."""
    abx, aby = bx - ax, by - ay
    ab_sq = abx * abx + aby * aby
    if ab_sq < 1e-12:
        return math.hypot(ax - cx, ay - cy) < r
    t = ((cx - ax) * abx + (cy - ay) * aby) / ab_sq
    t = max(0.0, min(1.0, t))
    px, py = ax + abx * t, ay + aby * t
    return math.hypot(px - cx, py - cy) < r

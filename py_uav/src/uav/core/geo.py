"""Cografi hesaplamalar.

Iki katman:
  1) Modul fonksiyonlari (haversine_distance, bearing, ...): C++ `geo.cpp` ile
     birebir anlam esligi. Kucuk yaris alani icin equirektangular NED yaklasimi.
  2) LocalFrame: planlama (shapely) icin tek, tutarli metre-duzlemi projeksiyonu.
     pyproj Azimuthal-Equidistant ile bir datum etrafinda; tum planlama geometrisi
     bu duzlemde calisir, boylece safety kontrolleri ile planlayici ayni metrikte olur.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from .types import Boundary, Position

EARTH_RADIUS_M = 6_371_000.0
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


@dataclass
class NED:
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0


def normalize_angle(degrees: float) -> float:
    """[0, 360) araligina normalize et."""
    d = math.fmod(degrees, 360.0)
    if d < 0.0:
        d += 360.0
    return d


def angle_difference(a: float, b: float) -> float:
    """a - b farki [-180, 180]."""
    return math.fmod(a - b + 540.0, 360.0) - 180.0


def haversine_distance(a: Position, b: Position) -> float:
    """3B mesafe degil; yatay haversine (C++ ile ayni). Metre."""
    dlat = (b.latitude - a.latitude) * DEG_TO_RAD
    dlon = (b.longitude - a.longitude) * DEG_TO_RAD
    lat1 = a.latitude * DEG_TO_RAD
    lat2 = b.latitude * DEG_TO_RAD
    s = math.sin(dlat / 2.0)
    c = math.sin(dlon / 2.0)
    h = s * s + math.cos(lat1) * math.cos(lat2) * c * c
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(h))


def bearing(from_p: Position, to_p: Position) -> float:
    """Ileri azimut, [0, 360) derece."""
    lat1 = from_p.latitude * DEG_TO_RAD
    lat2 = to_p.latitude * DEG_TO_RAD
    dlon = (to_p.longitude - from_p.longitude) * DEG_TO_RAD
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return normalize_angle(math.atan2(y, x) * RAD_TO_DEG)


def destination_point(from_p: Position, bearing_deg: float, distance_m: float) -> Position:
    """from_p'den bearing yonunde distance_m metre uzaktaki nokta."""
    lat1 = from_p.latitude * DEG_TO_RAD
    lon1 = from_p.longitude * DEG_TO_RAD
    brng = bearing_deg * DEG_TO_RAD
    d = distance_m / EARTH_RADIUS_M
    lat2 = math.asin(math.sin(lat1) * math.cos(d) + math.cos(lat1) * math.sin(d) * math.cos(brng))
    lon2 = lon1 + math.atan2(
        math.sin(brng) * math.sin(d) * math.cos(lat1),
        math.cos(d) - math.sin(lat1) * math.sin(lat2),
    )
    return Position(lat2 * RAD_TO_DEG, lon2 * RAD_TO_DEG, from_p.altitude)


def position_to_ned(pos: Position, reference: Position) -> NED:
    """reference'a gore equirektangular NED ofset (metre)."""
    dlat = (pos.latitude - reference.latitude) * DEG_TO_RAD
    dlon = (pos.longitude - reference.longitude) * DEG_TO_RAD
    cos_lat = math.cos(reference.latitude * DEG_TO_RAD)
    return NED(
        north=dlat * EARTH_RADIUS_M,
        east=dlon * EARTH_RADIUS_M * cos_lat,
        down=-(pos.altitude - reference.altitude),
    )


def ned_to_position(ned: NED, reference: Position) -> Position:
    cos_lat = math.cos(reference.latitude * DEG_TO_RAD)
    dlat = ned.north / EARTH_RADIUS_M
    dlon = ned.east / (EARTH_RADIUS_M * cos_lat) if cos_lat > 1e-12 else 0.0
    return Position(
        reference.latitude + dlat * RAD_TO_DEG,
        reference.longitude + dlon * RAD_TO_DEG,
        reference.altitude - ned.down,
    )


def point_in_polygon(point: Position, polygon: Sequence[Position]) -> bool:
    """Ray-casting; lat/lon duzleminde (kucuk alan icin yeterli)."""
    n = len(polygon)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        yi, xi = polygon[i].latitude, polygon[i].longitude
        yj, xj = polygon[j].latitude, polygon[j].longitude
        if ((yi > point.latitude) != (yj > point.latitude)) and (
            point.longitude < (xj - xi) * (point.latitude - yi) / (yj - yi) + xi
        ):
            inside = not inside
        j = i
    return inside


def nearest_point_on_segment(p: Position, a: Position, b: Position) -> Position:
    ap = position_to_ned(p, a)
    ab = position_to_ned(b, a)
    ab_sq = ab.north * ab.north + ab.east * ab.east
    if ab_sq < 1e-12:
        return a
    t = (ap.north * ab.north + ap.east * ab.east) / ab_sq
    t = max(0.0, min(1.0, t))
    return ned_to_position(NED(ab.north * t, ab.east * t, 0.0), a)


def distance_to_segment(point: Position, seg_a: Position, seg_b: Position) -> float:
    return haversine_distance(point, nearest_point_on_segment(point, seg_a, seg_b))


def segment_intersects_circle(p1: Position, p2: Position, center: Position, radius: float) -> bool:
    return distance_to_segment(center, p1, p2) <= radius


def segments_intersect(a1: Position, a2: Position, b1: Position, b2: Position) -> bool:
    """a1 referansli yerel XY duzleminde dogru-parcasi kesisimi."""
    def to_xy(p: Position) -> Tuple[float, float]:
        ned = position_to_ned(p, a1)
        return (ned.east, ned.north)

    ax1, ay1 = to_xy(a1)
    ax2, ay2 = to_xy(a2)
    bx1, by1 = to_xy(b1)
    bx2, by2 = to_xy(b2)
    d1x, d1y = ax2 - ax1, ay2 - ay1
    d2x, d2y = bx2 - bx1, by2 - by1
    denom = d1x * d2y - d1y * d2x
    if abs(denom) < 1e-12:
        return False
    t = ((bx1 - ax1) * d2y - (by1 - ay1) * d2x) / denom
    u = ((bx1 - ax1) * d1y - (by1 - ay1) * d1x) / denom
    return 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0


# =============================================================================
# LocalFrame: planlama icin tutarli metre-duzlemi (pyproj AEQD)
# =============================================================================

class LocalFrame:
    """Bir datum etrafinda yerel teget-duzlem (East=x, North=y, metre).

    Planlayici tum geometriyi (shapely) bu duzlemde yapar. lat/lon <-> metre
    donusumu pyproj Azimuthal-Equidistant ile; kucuk yaris alaninda yon ve
    mesafe hatalari ihmal edilebilir, equirektangular'dan daha saglam.
    """

    def __init__(self, datum: Position):
        # Gec import: pyproj agir; modul yuklenirken zorunlu olmasin.
        from pyproj import Transformer

        self.datum = datum
        proj = (
            f"+proj=aeqd +lat_0={datum.latitude} +lon_0={datum.longitude} "
            "+x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
        )
        self._fwd = Transformer.from_crs("EPSG:4326", proj, always_xy=True)
        self._inv = Transformer.from_crs(proj, "EPSG:4326", always_xy=True)

    def to_xy(self, pos: Position) -> Tuple[float, float]:
        x, y = self._fwd.transform(pos.longitude, pos.latitude)
        return (float(x), float(y))

    def to_position(self, x: float, y: float, altitude: float = 0.0) -> Position:
        lon, lat = self._inv.transform(x, y)
        return Position(float(lat), float(lon), altitude)

    @staticmethod
    def from_boundary_or_point(boundary: Boundary | None, fallback: Position) -> "LocalFrame":
        if boundary is not None and boundary.is_valid():
            return LocalFrame(boundary.get_center())
        return LocalFrame(fallback)

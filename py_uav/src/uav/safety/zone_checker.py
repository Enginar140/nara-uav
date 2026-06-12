"""Bolge kontrol yardimcilari (boundary + HSS). C++ ZoneChecker portu.

geo.py fonksiyonlariyla calisir (safety_gate ve nav ile tutarli metrik).
"""
from __future__ import annotations

import math
from typing import List

from ..core import geo
from ..core.types import Boundary, HSSZone, Position


class ZoneChecker:
    def is_inside_boundary(self, pos: Position, boundary: Boundary) -> bool:
        if not boundary.is_valid():
            return True
        return geo.point_in_polygon(pos, boundary.vertices)

    def is_inside_any_hss(self, pos: Position, zones: List[HSSZone],
                          extra_buffer: float = 0.0) -> bool:
        for z in zones:
            if not z.active:
                continue
            if geo.haversine_distance(pos, z.center) < z.radius + extra_buffer:
                return True
        return False

    def distance_to_boundary(self, pos: Position, boundary: Boundary) -> float:
        if not boundary.is_valid():
            return math.inf
        n = len(boundary.vertices)
        return min(
            geo.distance_to_segment(pos, boundary.vertices[i], boundary.vertices[(i + 1) % n])
            for i in range(n)
        )

    def distance_to_nearest_hss(self, pos: Position, zones: List[HSSZone]) -> float:
        best = math.inf
        for z in zones:
            if not z.active:
                continue
            best = min(best, geo.haversine_distance(pos, z.center) - z.radius)
        return best

    def segment_intersects_any_hss(self, frm: Position, to: Position,
                                   zones: List[HSSZone], buffer: float = 0.0) -> bool:
        for z in zones:
            if not z.active:
                continue
            if geo.segment_intersects_circle(frm, to, z.center, z.radius + buffer):
                return True
        return False

    def segment_exits_boundary(self, frm: Position, to: Position, boundary: Boundary) -> bool:
        if not boundary.is_valid():
            return False
        n = len(boundary.vertices)
        for i in range(n):
            if geo.segments_intersect(frm, to, boundary.vertices[i], boundary.vertices[(i + 1) % n]):
                return True
        return not self.is_inside_boundary(to, boundary)

    def is_position_safe(self, pos: Position, boundary: Boundary,
                         zones: List[HSSZone], hss_buffer: float = 40.0) -> bool:
        if not self.is_inside_boundary(pos, boundary):
            return False
        if self.is_inside_any_hss(pos, zones, hss_buffer):
            return False
        return True

"""SafetyGate -- her komut oncesi son savunma hatti.

C++ SafetyGate portu, ANCAK velocity validasyonu/clamp YOK (sistem hiz komutu
gondermiyor). Yalniz pozisyon dogrulama + guvenli noktaya cekme (clamp).
"""
from __future__ import annotations

from dataclasses import dataclass

from ..core import geo
from ..core.types import Position
from ..world_model.world_model import WorldModel
from .zone_checker import ZoneChecker


@dataclass
class SafetyGateConfig:
    hss_buffer_m: float = 40.0
    boundary_buffer_m: float = 20.0
    min_altitude_m: float = 30.0
    max_altitude_m: float = 400.0


class SafetyGate:
    def __init__(self, checker: ZoneChecker, world_model: WorldModel,
                 config: SafetyGateConfig | None = None):
        self.checker = checker
        self._wm = world_model
        self.cfg = config or SafetyGateConfig()

    def validate_position(self, target: Position) -> bool:
        if not (self.cfg.min_altitude_m <= target.altitude <= self.cfg.max_altitude_m):
            return False
        boundary = self._wm.get_boundary()
        zones = self._wm.get_active_hss_zones()
        if boundary is not None:
            if not self.checker.is_inside_boundary(target, boundary):
                return False
            if self.checker.distance_to_boundary(target, boundary) < self.cfg.boundary_buffer_m:
                return False
        if self.checker.is_inside_any_hss(target, zones, self.cfg.hss_buffer_m):
            return False
        return True

    def validate_segment(self, frm: Position, target: Position) -> bool:
        if not self.validate_position(target):
            return False
        boundary = self._wm.get_boundary()
        if boundary is not None:
            if not self.checker.is_inside_boundary(frm, boundary):
                return False
            if self.checker.segment_exits_boundary(frm, target, boundary):
                return False
        zones = self._wm.get_active_hss_zones()
        if self.checker.is_inside_any_hss(frm, zones, self.cfg.hss_buffer_m):
            return False
        if self.checker.segment_intersects_any_hss(frm, target, zones, self.cfg.hss_buffer_m):
            return False
        return True

    def validate_escape_segment(self, frm: Position, target: Position) -> bool:
        """Kacis segmenti: HSS'ten UZAKLASAN harekete izin verir (C++ portu).

        frm HSS icindeyse, hedef zone'dan daha uzak ve guvenli mesafedeyse kabul.
        Aksi halde normal segment kesisim kurali.
        """
        if not self.validate_position(target):
            return False
        boundary = self._wm.get_boundary()
        if boundary is not None and not self.checker.is_inside_boundary(target, boundary):
            return False
        zones = self._wm.get_active_hss_zones()
        for z in zones:
            if not z.active:
                continue
            safe_dist = z.radius + self.cfg.hss_buffer_m
            from_dist = geo.haversine_distance(frm, z.center)
            target_dist = geo.haversine_distance(target, z.center)
            if from_dist < safe_dist:
                # zone icinde basliyoruz: hedef daha uzak VE guvenli olmali
                if target_dist < safe_dist or target_dist <= from_dist:
                    return False
                continue
            if self.checker.segment_intersects_any_hss(frm, target, [z], 0.0) or \
                    geo.segment_intersects_circle(frm, target, z.center, safe_dist):
                return False
        return True

    def clamp_to_safe(self, target: Position) -> Position:
        safe = Position(target.latitude, target.longitude, target.altitude)
        safe.altitude = min(max(safe.altitude, self.cfg.min_altitude_m), self.cfg.max_altitude_m)

        boundary = self._wm.get_boundary()
        if boundary is not None:
            if not self.checker.is_inside_boundary(safe, boundary):
                center = boundary.get_center()
                brng = geo.bearing(safe, center)
                dist = geo.haversine_distance(safe, center)
                step = dist * 0.1
                for _ in range(15):
                    safe = geo.destination_point(safe, brng, step)
                    if self.checker.is_inside_boundary(safe, boundary):
                        d = self.checker.distance_to_boundary(safe, boundary)
                        if d < self.cfg.boundary_buffer_m:
                            safe = geo.destination_point(safe, brng, self.cfg.boundary_buffer_m - d)
                        break
            else:
                d = self.checker.distance_to_boundary(safe, boundary)
                if d < self.cfg.boundary_buffer_m:
                    center = boundary.get_center()
                    brng = geo.bearing(safe, center)
                    safe = geo.destination_point(safe, brng, self.cfg.boundary_buffer_m - d)

        zones = self._wm.get_active_hss_zones()
        for z in zones:
            if not z.active:
                continue
            dist = geo.haversine_distance(safe, z.center)
            safe_dist = z.radius + self.cfg.hss_buffer_m
            if dist < safe_dist:
                brng = geo.bearing(z.center, safe)
                safe = geo.destination_point(z.center, brng, safe_dist + 10.0)
                safe.altitude = target.altitude

        safe.altitude = min(max(safe.altitude, self.cfg.min_altitude_m), self.cfg.max_altitude_m)
        return safe

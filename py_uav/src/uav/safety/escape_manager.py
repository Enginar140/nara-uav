"""EscapeManager -- acil kacis hedefi uretimi. C++ EscapeManager portu.

boundary / HSS / collision senaryolarina gore guvenli kacis konumu uretir;
SafetyGate ile dogrular, gerekirse alternatif bearing arar.
"""
from __future__ import annotations

import math
from typing import List, Optional

from ..core import geo
from ..core.types import Boundary, HSSZone, Position, Threat, UAVState
from ..infra.log import get_logger
from ..world_model.world_model import WorldModel
from .safety_gate import SafetyGate
from .zone_checker import ZoneChecker

_log = get_logger("escape_manager")

_ALT_OFFSETS = [30, -30, 60, -60, 90, -90, 120, -120, 150, -150]


class EscapeManager:
    def __init__(self, checker: ZoneChecker, world_model: WorldModel, safety_gate: SafetyGate):
        self._checker = checker
        self._wm = world_model
        self._gate = safety_gate

    def _validate_or_alternative(self, candidate: Position, own_pos: Position,
                                 base_bearing: float, escape_dist: float) -> Optional[Position]:
        if self._gate.validate_position(candidate):
            return candidate
        for offset in _ALT_OFFSETS:
            alt_bearing = geo.normalize_angle(base_bearing + offset)
            alt = geo.destination_point(own_pos, alt_bearing, escape_dist)
            alt.altitude = candidate.altitude
            if self._gate.validate_position(alt):
                return alt
        boundary = self._wm.get_boundary()
        if boundary is not None:
            center = boundary.get_center()
            center.altitude = own_pos.altitude
            if self._gate.validate_position(center):
                return center
        _log.warning("Guvenli kacis bulunamadi, clamp ile en iyi aday donuyor")
        return self._gate.clamp_to_safe(candidate)

    def generate_escape_target(self, own: UAVState) -> Optional[Position]:
        boundary = self._wm.get_boundary()
        zones = self._wm.get_active_hss_zones()
        if boundary is not None and not self._checker.is_inside_boundary(own.position, boundary):
            return self.generate_boundary_escape(own, boundary)
        for z in zones:
            dist = geo.haversine_distance(own.position, z.center)
            if dist < z.radius + self._gate.cfg.hss_buffer_m:
                return self.generate_hss_escape(own, zones)
        return None

    def generate_boundary_escape(self, own: UAVState, bnd: Boundary) -> Optional[Position]:
        center = bnd.get_center()
        brg = geo.bearing(own.position, center)
        escape = geo.destination_point(own.position, brg, 200.0)
        escape.altitude = own.position.altitude
        return self._validate_or_alternative(escape, own.position, brg, 200.0)

    def generate_hss_escape(self, own: UAVState, zones: List[HSSZone]) -> Optional[Position]:
        nearest_dist = math.inf
        nearest: Optional[HSSZone] = None
        nearest_safe_radius = 0.0
        for z in zones:
            dist = geo.haversine_distance(own.position, z.center)
            safe_radius = z.radius + self._gate.cfg.hss_buffer_m
            if dist < safe_radius and dist < nearest_dist:
                nearest_dist = dist
                nearest = z
                nearest_safe_radius = safe_radius
        if nearest is None:
            return None
        brg_away = geo.bearing(nearest.center, own.position)
        escape_dist = nearest_safe_radius - nearest_dist + 80.0
        escape = geo.destination_point(own.position, brg_away, escape_dist)
        escape.altitude = own.position.altitude
        return self._validate_or_alternative(escape, own.position, brg_away, escape_dist)

    def generate_collision_escape(self, own: UAVState, threat: Threat) -> Optional[Position]:
        escape_bearing = geo.normalize_angle(threat.bearing + 90.0)
        if own.velocity.horizontal_speed() > 5.0:
            own_heading = math.degrees(math.atan2(own.velocity.east, own.velocity.north))
            if geo.angle_difference(threat.bearing, own_heading) > 0:
                escape_bearing = geo.normalize_angle(own_heading + 45.0)
            else:
                escape_bearing = geo.normalize_angle(own_heading - 45.0)
        escape = geo.destination_point(own.position, escape_bearing, 150.0)
        escape.altitude = own.position.altitude + 20.0
        return self._validate_or_alternative(escape, own.position, escape_bearing, 150.0)

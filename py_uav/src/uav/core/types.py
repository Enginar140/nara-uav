"""Temel veri yapilari ve enum'lar.

C++ `core/types.hpp` ile birebir anlam esligi; Python dataclass'lariyla.
Aci birimi: derece. Mesafe: metre. Konum: WGS84 lat/lon + AMSL/relative alt (m).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import List, Optional

# Konum esitlik toleranslari (C++ Position::operator== ile ayni)
_LL_EPS = 1e-9
_ALT_EPS = 0.01


@dataclass
class Position:
    """WGS84 cografi konum."""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Position):
            return NotImplemented
        return (
            abs(self.latitude - other.latitude) < _LL_EPS
            and abs(self.longitude - other.longitude) < _LL_EPS
            and abs(self.altitude - other.altitude) < _ALT_EPS
        )

    def __hash__(self) -> int:
        return hash((round(self.latitude, 9), round(self.longitude, 9)))

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.latitude, self.longitude, self.altitude)


@dataclass
class Velocity:
    """NED hiz bileseni (m/s)."""
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0

    def horizontal_speed(self) -> float:
        return math.hypot(self.north, self.east)

    def speed_3d(self) -> float:
        return math.sqrt(self.north**2 + self.east**2 + self.down**2)


@dataclass
class Attitude:
    """Govde duruşu (derece)."""
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class UAVState:
    """Kendi veya rakip ucagin anlik durumu."""
    id: int = 0
    position: Position = field(default_factory=Position)
    velocity: Velocity = field(default_factory=Velocity)
    attitude: Attitude = field(default_factory=Attitude)
    heading: float = 0.0
    airspeed: float = 0.0
    groundspeed: float = 0.0
    timestamp_us: int = 0

    gps_fix_type: int = 0
    satellites_visible: int = 0
    gps_hdop: int = 9999

    battery_voltage: float = 0.0
    battery_remaining: int = -1

    def is_valid(self) -> bool:
        return self.timestamp_us > 0

    def age_seconds(self) -> float:
        if self.timestamp_us == 0:
            return math.inf
        now_us = time.time_ns() // 1000
        return max(0.0, (now_us - self.timestamp_us) / 1e6)


@dataclass
class HSSZone:
    """Hava Savunma Sistemi (yasak daire bolgesi)."""
    id: int = 0
    center: Position = field(default_factory=Position)
    radius: float = 0.0
    active: bool = False
    buffer_distance: float = 40.0

    def effective_radius(self) -> float:
        return self.radius + self.buffer_distance


@dataclass
class Boundary:
    """Yarisma alani sinir poligonu + irtifa zarfi."""
    vertices: List[Position] = field(default_factory=list)
    min_altitude: float = 30.0
    max_altitude: float = 400.0

    def is_valid(self) -> bool:
        return len(self.vertices) >= 3

    def get_center(self) -> Position:
        if not self.vertices:
            return Position()
        lat = sum(v.latitude for v in self.vertices) / len(self.vertices)
        lon = sum(v.longitude for v in self.vertices) / len(self.vertices)
        alt = sum(v.altitude for v in self.vertices) / len(self.vertices)
        return Position(lat, lon, alt)


# =============================================================================
# Komut & Gorev tipleri
# =============================================================================

class GoalType(Enum):
    NONE = "NONE"
    PATROL = "PATROL"
    TRACK_TARGET = "TRACK_TARGET"
    DIVE_TARGET = "DIVE_TARGET"
    ESCAPE = "ESCAPE"


@dataclass
class MissionGoal:
    type: GoalType = GoalType.NONE
    target_position: Optional[Position] = None
    target_id: Optional[int] = None


class MissionMode(Enum):
    ARDUPILOT = "ARDUPILOT"
    PATROL = "PATROL"
    FOLLOW = "FOLLOW"
    QR_DIVE = "QR_DIVE"


# =============================================================================
# Guvenlik tipleri
# =============================================================================

class RiskLevel(IntEnum):
    NONE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4


class BoundaryStatus(IntEnum):
    INSIDE = 0
    NEAR_BOUNDARY = 1
    OUTSIDE = 2


@dataclass
class Threat:
    aircraft_id: int = 0
    distance: float = 0.0
    bearing: float = 0.0
    closing_speed: float = 0.0
    time_to_collision: float = math.inf
    risk_level: RiskLevel = RiskLevel.NONE


@dataclass
class SafetyState:
    collision_risk: RiskLevel = RiskLevel.NONE
    boundary_status: BoundaryStatus = BoundaryStatus.INSIDE
    hss_violations: List[int] = field(default_factory=list)
    active_threats: List[Threat] = field(default_factory=list)
    comm_loss: bool = False

    def is_safe(self) -> bool:
        return (
            self.collision_risk < RiskLevel.HIGH
            and self.boundary_status == BoundaryStatus.INSIDE
            and not self.hss_violations
            and not self.comm_loss
        )

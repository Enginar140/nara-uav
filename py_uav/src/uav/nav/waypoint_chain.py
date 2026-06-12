"""WaypointChain -- GUIDED DO_REPOSITION sequencing icin waypoint zinciri.

C++ WaypointChain portu: acceptance radius ile ilerleme, smart replace, validate.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional

from ..core.geo import haversine_distance
from ..core.types import Boundary, HSSZone, Position


@dataclass
class Waypoint:
    position: Position
    airspeed: float = -1.0
    acceptance_radius_m: float = 80.0
    label: str = ""


class ChainStatus(Enum):
    EMPTY = auto()
    IN_PROGRESS = auto()
    WAYPOINT_REACHED = auto()
    CHAIN_COMPLETE = auto()


class WaypointChain:
    def __init__(self):
        self._chain: List[Waypoint] = []
        self._idx = 0
        self._circular = False
        self._complete = False

    def load(self, chain: List[Waypoint], circular: bool = False) -> None:
        self._chain = list(chain)
        self._circular = circular
        self._idx = 0
        self._complete = not chain

    def update(self, own_pos: Position) -> ChainStatus:
        if not self._chain:
            return ChainStatus.EMPTY
        if self._complete and not self._circular:
            return ChainStatus.CHAIN_COMPLETE
        target = self._chain[self._idx]
        if haversine_distance(own_pos, target.position) <= target.acceptance_radius_m:
            self._idx += 1
            if self._idx >= len(self._chain):
                if self._circular:
                    self._idx = 0
                else:
                    self._complete = True
                    return ChainStatus.CHAIN_COMPLETE
            return ChainStatus.WAYPOINT_REACHED
        return ChainStatus.IN_PROGRESS

    def current_target(self) -> Optional[Waypoint]:
        if not self._chain or (self._complete and not self._circular):
            return None
        return self._chain[self._idx]

    def peek_next(self) -> Optional[Waypoint]:
        if not self._chain or (self._complete and not self._circular):
            return None
        nxt = self._idx + 1
        if nxt >= len(self._chain):
            if self._circular:
                nxt = 0
            else:
                return None
        return self._chain[nxt]

    def advance_to_next(self) -> bool:
        if not self._chain or (self._complete and not self._circular):
            return False
        nxt = self._idx + 1
        if nxt >= len(self._chain):
            if self._circular:
                self._idx = 0
                return True
            self._complete = True
            return False
        self._idx = nxt
        return True

    def remaining_waypoints(self) -> List[Waypoint]:
        if not self._chain or self._complete:
            return []
        return self._chain[self._idx:]

    def clear(self) -> None:
        self._chain = []
        self._idx = 0
        self._circular = False
        self._complete = False

    def replace(self, new_chain: List[Waypoint]) -> None:
        if not new_chain:
            self.clear()
            return
        self._chain = list(new_chain)
        self._complete = False
        self._idx = 0

    def is_empty(self) -> bool:
        return len(self._chain) == 0

    def is_complete(self) -> bool:
        return self._complete

    def current_index(self) -> int:
        return self._idx

    def size(self) -> int:
        return len(self._chain)

    def validate(self, checker, boundary: Boundary, zones: List[HSSZone],
                 hss_buffer: float = 40.0) -> bool:
        for i in range(self._idx, len(self._chain)):
            if not checker.is_position_safe(self._chain[i].position, boundary, zones, hss_buffer):
                return False
            if i > self._idx:
                a, b = self._chain[i - 1].position, self._chain[i].position
                if checker.segment_intersects_any_hss(a, b, zones, hss_buffer):
                    return False
                if checker.segment_exits_boundary(a, b, boundary):
                    return False
        return True

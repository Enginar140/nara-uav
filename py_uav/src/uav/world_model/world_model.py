"""Dunya modeli: kendi durum, rakipler, HSS bolgeleri, sinir + waypoint listesi.

Thread-guvenli (threading.RLock): MAVLink receive thread'i yazar, asyncio
loop'taki gorevler okur. Tum erisim kilitle korunur; kopyalar disari dondurulur.
"""
from __future__ import annotations

import copy
import threading
from typing import Dict, List, Optional

from ..core.geo import haversine_distance
from ..core.types import Boundary, HSSZone, Position, UAVState


class WorldModel:
    def __init__(self):
        self._lock = threading.RLock()
        self._own: UAVState = UAVState()
        self._opponents: Dict[int, UAVState] = {}
        self._hss: Dict[int, HSSZone] = {}
        self._boundary: Optional[Boundary] = None
        # GCS'ten gelen ham gorev waypoint listesi (HSS-guvenli hale getirilmeden once)
        self._mission_waypoints: List[Position] = []

    # -- kendi durum --
    def set_own_state(self, state: UAVState) -> None:
        with self._lock:
            self._own = copy.deepcopy(state)

    def get_own_state(self) -> UAVState:
        with self._lock:
            return copy.deepcopy(self._own)

    # -- rakipler --
    def update_opponent(self, state: UAVState) -> None:
        with self._lock:
            self._opponents[state.id] = copy.deepcopy(state)

    def remove_opponent(self, opp_id: int) -> None:
        with self._lock:
            self._opponents.pop(opp_id, None)

    def get_opponent(self, opp_id: int) -> Optional[UAVState]:
        with self._lock:
            opp = self._opponents.get(opp_id)
            return copy.deepcopy(opp) if opp else None

    def get_all_opponents(self) -> List[UAVState]:
        with self._lock:
            return [copy.deepcopy(o) for o in self._opponents.values()]

    def get_nearest_opponent(self, pos: Position) -> Optional[UAVState]:
        with self._lock:
            best: Optional[UAVState] = None
            best_d = float("inf")
            for o in self._opponents.values():
                d = haversine_distance(pos, o.position)
                if d < best_d:
                    best_d = d
                    best = o
            return copy.deepcopy(best) if best else None

    def remove_stale_opponents(self, max_age_s: float) -> List[int]:
        removed: List[int] = []
        with self._lock:
            for oid, o in list(self._opponents.items()):
                if o.age_seconds() > max_age_s:
                    del self._opponents[oid]
                    removed.append(oid)
        return removed

    # -- HSS bolgeleri --
    def update_hss_zone(self, zone: HSSZone) -> None:
        with self._lock:
            self._hss[zone.id] = copy.deepcopy(zone)

    def replace_hss_zones(self, zones: List[HSSZone]) -> None:
        """Gelen HSS listesini otorite kabul et; eskileri temizle."""
        with self._lock:
            self._hss = {z.id: copy.deepcopy(z) for z in zones}

    def remove_hss_zone(self, zone_id: int) -> None:
        with self._lock:
            self._hss.pop(zone_id, None)

    def get_active_hss_zones(self) -> List[HSSZone]:
        with self._lock:
            return [copy.deepcopy(z) for z in self._hss.values() if z.active]

    def get_all_hss_zones(self) -> List[HSSZone]:
        with self._lock:
            return [copy.deepcopy(z) for z in self._hss.values()]

    # -- sinir --
    def set_boundary(self, boundary: Boundary) -> None:
        with self._lock:
            self._boundary = copy.deepcopy(boundary)

    def get_boundary(self) -> Optional[Boundary]:
        with self._lock:
            return copy.deepcopy(self._boundary) if self._boundary else None

    def has_boundary(self) -> bool:
        with self._lock:
            return self._boundary is not None and self._boundary.is_valid()

    # -- gorev waypoint listesi (GCS'ten) --
    def set_mission_waypoints(self, waypoints: List[Position]) -> None:
        with self._lock:
            self._mission_waypoints = [copy.deepcopy(w) for w in waypoints]

    def get_mission_waypoints(self) -> List[Position]:
        with self._lock:
            return [copy.deepcopy(w) for w in self._mission_waypoints]

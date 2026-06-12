"""PatrolMode -- boundary inset devriyesi. C++ PatrolMode portu.

Boundary kose noktalarini merkeze dogru iceri ceker, aci sirasina dizer,
en yakindan baslar; her hedefe HSS-guvenli rota planlar (NavigationEngine).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

from ..core import geo
from ..core.types import GoalType, MissionGoal, Position, UAVState
from ..infra.log import get_logger
from ..nav.navigation_engine import NavigationEngine
from ..nav.waypoint_chain import ChainStatus
from ..world_model.world_model import WorldModel
from .mode_base import IMissionMode

_log = get_logger("patrol_mode")


@dataclass
class PatrolConfig:
    boundary_inset_m: float = 50.0
    cruise_altitude_m: float = 100.0
    patrol_speed_ms: float = 18.0


class PatrolMode(IMissionMode):
    def __init__(self, nav: NavigationEngine, world_model: WorldModel,
                 config: PatrolConfig | None = None):
        self._nav = nav
        self._wm = world_model
        self.cfg = config or PatrolConfig()
        self._waypoints: List[Position] = []
        self._idx = 0

    def on_enter(self) -> None:
        _log.info("PATROL: giriliyor")
        self._idx = 0
        self._generate_waypoints()
        if self._waypoints:
            self._plan_to_next()

    def on_exit(self) -> None:
        self._nav.clear_route()

    def tick(self, own_state: UAVState) -> None:
        if not self._waypoints:
            self._generate_waypoints()
            if not self._waypoints:
                return
        status = self._nav.update(own_state.position)
        if status in (ChainStatus.CHAIN_COMPLETE, ChainStatus.EMPTY):
            self._idx = (self._idx + 1) % len(self._waypoints)
            self._plan_to_next()

    def get_mission_goal(self) -> MissionGoal:
        return MissionGoal(type=GoalType.PATROL)

    def name(self) -> str:
        return "PATROL"

    def _generate_waypoints(self) -> None:
        boundary = self._wm.get_boundary()
        if boundary is None or not boundary.is_valid():
            _log.warning("PATROL: gecerli boundary yok, waypoint uretilemiyor")
            return
        center = boundary.get_center()
        wps: List[Position] = []
        for v in boundary.vertices:
            brg = geo.bearing(v, center)
            dist = geo.haversine_distance(v, center)
            inset = min(self.cfg.boundary_inset_m, dist * 0.3)
            wp = geo.destination_point(v, brg, inset)
            wp.altitude = self.cfg.cruise_altitude_m
            wps.append(wp)
        wps.sort(key=lambda p: geo.bearing(center, p))
        own = self._wm.get_own_state()
        if own.is_valid() and wps:
            self._idx = min(range(len(wps)),
                            key=lambda i: geo.haversine_distance(own.position, wps[i]))
        self._waypoints = wps
        _log.info("PATROL: %d devriye waypoint uretildi", len(wps))

    def _plan_to_next(self) -> None:
        if not self._waypoints:
            return
        self._nav.plan_route(self._waypoints[self._idx], self.cfg.patrol_speed_ms)

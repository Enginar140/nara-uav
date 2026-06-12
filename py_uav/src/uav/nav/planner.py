"""Yol planlayici -- shapely tabanli, sabit-kanat koridor-farkinda visibility graph.

Felsefe: ArduPlane arc'lari kendisi ucar; biz yalniz HSS/fence-guvenli, donus
yaricapina gore yeterli MARJI olan bir waypoint LISTESI uretiriz.

Iki katman marj:
  - HARD safety: HSS radius + hss_buffer_m. ASLA ihlal edilmez (final dogrulama).
  - SOFT corridor: hard + turn_safety_margin_m + escape_prevention_margin_m.
    Planlayici detour'lari bu genis koridorun disindan uretir; boylece ucak
    WP'leri keserken (turn radius/acceptance) hard koridora girmez.

Akis (her sey LocalFrame metre-duzleminde, shapely ile):
  start->goal soft-clear ise [start, goal]
  degilse: HSS dairelerini soft poligonlara genislet -> visibility graph
  -> networkx Dijkstra -> line-of-sight simplify -> densify
  -> HARD HSS/boundary final dogrulama (sampling)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import networkx as nx
from shapely.geometry import LineString, Point, Polygon

from ..core import geo
from ..core.time_utils import elapsed_ms, now_monotonic
from ..core.types import Boundary, HSSZone, Position
from ..infra.log import get_logger

_log = get_logger("planner")


@dataclass
class PlannerConfig:
    hss_buffer_m: float = 40.0
    boundary_buffer_m: float = 20.0
    min_turn_radius_m: float = 50.0
    turn_safety_margin_m: float = 50.0
    escape_prevention_margin_m: float = 80.0
    max_segment_length_m: float = 80.0
    polygon_segments: int = 16        # shapely buffer cozunurlugu (quad_segs ~ /4)
    sample_step_m: float = 15.0       # final dogrulama ornekleme adimi


@dataclass
class PathResult:
    success: bool = False
    waypoints: List[Position] = None
    total_distance_m: float = 0.0
    planning_time_ms: float = 0.0
    error: str = ""

    def __post_init__(self):
        if self.waypoints is None:
            self.waypoints = []


@dataclass(frozen=True)
class _VNode:
    xy: Tuple[float, float]
    obstacle_id: int = -1
    vertex_id: int = -1


class PathPlanner:
    def __init__(self, config: Optional[PlannerConfig] = None):
        self.cfg = config or PlannerConfig()

    # SOFT koridor marji = hss_buffer + turn_safety + escape_prevention.
    # Ek marj, waypoint acceptance/lookahead kaynakli kose kesmeyi de absorbe eder.
    def planning_margin(self) -> float:
        return self.cfg.hss_buffer_m + max(self.cfg.turn_safety_margin_m,
                                           self.cfg.min_turn_radius_m) + max(
                                               0.0, self.cfg.escape_prevention_margin_m)

    def set_min_turn_radius(self, r: float) -> None:
        self.cfg.min_turn_radius_m = max(0.0, r)
        # marji yaricaptan kucuk birakma
        self.cfg.turn_safety_margin_m = max(self.cfg.turn_safety_margin_m, r)

    # ===================== Genel API =====================
    def plan(self, start: Position, goal: Position,
             boundary: Optional[Boundary], hss_zones: List[HSSZone]) -> PathResult:
        t0 = now_monotonic()
        frame = geo.LocalFrame.from_boundary_or_point(boundary, start)
        ctx = _PlanCtx(self.cfg, frame, boundary, hss_zones)

        # 1) Direkt soft-clear?
        if ctx.segment_soft_clear(start, goal):
            return self._finish([start, goal], start, goal, t0)

        # 2) Visibility graph
        path_xy = self._visibility_path(ctx, start, goal)
        if path_xy is None:
            return PathResult(success=False, error="visibility graph: yol yok",
                              planning_time_ms=elapsed_ms(t0))

        # 3) line-of-sight simplify
        path_xy = self._simplify(ctx, path_xy)

        # xy -> Position (irtifa: along-track interpolasyon)
        positions = self._xy_to_positions(ctx, path_xy, start, goal)

        # 4) densify
        positions = self._densify(positions)

        # 5) HARD final dogrulama
        if not self.is_path_safe(positions, boundary, hss_zones, frame=frame):
            return PathResult(success=False,
                              error="uretilen yol hard guvenlik dogrulamasini gecemedi",
                              planning_time_ms=elapsed_ms(t0))

        return self._finish(positions, start, goal, t0, already=True)

    def make_safe(self, waypoints: List[Position],
                  boundary: Optional[Boundary], hss_zones: List[HSSZone]) -> PathResult:
        """Verilen waypoint LISTESINI HSS/fence-guvenli hale getir.

        Her ardisik bacagi plan() ile detour'la, sonuclari birlestir.
        "Surekli waypoint listesi vereceğiz" senaryosunun ana giris noktasi.
        """
        t0 = now_monotonic()
        if len(waypoints) < 2:
            return PathResult(success=False, error="en az 2 waypoint gerekli",
                              planning_time_ms=elapsed_ms(t0))
        stitched: List[Position] = [waypoints[0]]
        for i in range(1, len(waypoints)):
            leg = self.plan(stitched[-1], waypoints[i], boundary, hss_zones)
            if not leg.success:
                return PathResult(success=False,
                                  error=f"bacak {i} planlanamadi: {leg.error}",
                                  planning_time_ms=elapsed_ms(t0))
            # ilk nokta onceki bacagin son noktasiyla ayni -> atla
            stitched.extend(leg.waypoints[1:])
        total = sum(geo.haversine_distance(stitched[i - 1], stitched[i])
                    for i in range(1, len(stitched)))
        return PathResult(success=True, waypoints=stitched,
                          total_distance_m=total, planning_time_ms=elapsed_ms(t0))

    def is_direct_clear(self, frm: Position, to: Position,
                        boundary: Optional[Boundary], hss_zones: List[HSSZone]) -> bool:
        frame = geo.LocalFrame.from_boundary_or_point(boundary, frm)
        ctx = _PlanCtx(self.cfg, frame, boundary, hss_zones)
        return ctx.segment_soft_clear(frm, to)

    # ===================== Visibility graph =====================
    def _visibility_path(self, ctx: "_PlanCtx", start: Position, goal: Position):
        start_xy = ctx.frame.to_xy(start)
        goal_xy = ctx.frame.to_xy(goal)

        nodes: List[_VNode] = [_VNode(start_xy), _VNode(goal_xy)]
        for obs_idx, poly in enumerate(ctx.soft_obstacles):
            for vertex_idx, xy in enumerate(list(poly.exterior.coords)[:-1]):
                nodes.append(_VNode(xy, obs_idx, vertex_idx))

        start_containing = ctx.find_containing_soft_obstacle(start_xy)
        goal_containing = ctx.find_containing_soft_obstacle(goal_xy)

        g = nx.Graph()
        for i, _ in enumerate(nodes):
            g.add_node(i)

        n = len(nodes)
        for i in range(n):
            for j in range(i + 1, n):
                a = nodes[i]
                b = nodes[j]
                if a.obstacle_id >= 0 and a.obstacle_id == b.obstacle_id:
                    vertex_count = len(list(ctx.soft_obstacles[a.obstacle_id].exterior.coords)) - 1
                    diff = abs(a.vertex_id - b.vertex_id)
                    adjacent = diff == 1 or diff == vertex_count - 1
                    if not adjacent:
                        continue

                skip_a = -1
                skip_b = -1
                if i == 0 and start_containing >= 0:
                    skip_a = start_containing
                elif i == 1 and goal_containing >= 0:
                    skip_a = goal_containing
                if j == 0 and start_containing >= 0:
                    skip_b = start_containing
                elif j == 1 and goal_containing >= 0:
                    skip_b = goal_containing

                if not ctx.edge_hard_safe_xy(a.xy, b.xy):
                    continue
                if ctx.edge_visible_xy(a.xy, b.xy, skip_a=skip_a, skip_b=skip_b):
                    d = math.dist(a.xy, b.xy)
                    g.add_edge(i, j, weight=d)

        try:
            idx_path = nx.shortest_path(g, 0, 1, weight="weight")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return None
        return [nodes[i].xy for i in idx_path]

    def _simplify(self, ctx: "_PlanCtx", path_xy):
        if len(path_xy) <= 2:
            return path_xy
        out = [path_xy[0]]
        cur = 0
        while cur < len(path_xy) - 1:
            nxt = cur + 1
            for cand in range(len(path_xy) - 1, cur + 1, -1):
                skip_a = ctx.find_containing_soft_obstacle(path_xy[cur])
                skip_b = ctx.find_containing_soft_obstacle(path_xy[cand])
                if (ctx.edge_hard_safe_xy(path_xy[cur], path_xy[cand])
                        and ctx.edge_visible_xy(path_xy[cur], path_xy[cand],
                                                skip_a=skip_a, skip_b=skip_b)):
                    nxt = cand
                    break
            out.append(path_xy[nxt])
            cur = nxt
        return out

    def _xy_to_positions(self, ctx: "_PlanCtx", path_xy, start: Position, goal: Position):
        # along-track mesafeye gore irtifa interpolasyonu
        cum = [0.0]
        for i in range(1, len(path_xy)):
            cum.append(cum[-1] + math.dist(path_xy[i - 1], path_xy[i]))
        total = cum[-1] if cum[-1] > 1e-6 else 1.0
        positions: List[Position] = []
        for i, (x, y) in enumerate(path_xy):
            frac = cum[i] / total
            alt = start.altitude + (goal.altitude - start.altitude) * frac
            positions.append(ctx.frame.to_position(x, y, alt))
        # uc noktalari tam start/goal'a sabitle
        positions[0] = Position(start.latitude, start.longitude, start.altitude)
        positions[-1] = Position(goal.latitude, goal.longitude, goal.altitude)
        return positions

    def _densify(self, path: List[Position]) -> List[Position]:
        if len(path) < 2 or self.cfg.max_segment_length_m <= 1.0:
            return path
        dense = [path[0]]
        for i in range(1, len(path)):
            a, b = dense[-1], path[i]
            dist = geo.haversine_distance(a, b)
            pieces = max(1, math.ceil(dist / self.cfg.max_segment_length_m))
            ned = geo.position_to_ned(b, a)
            for k in range(1, pieces + 1):
                t = k / pieces
                dense.append(geo.ned_to_position(
                    geo.NED(ned.north * t, ned.east * t, ned.down * t), a))
        return dense

    # ===================== HARD dogrulama =====================
    def is_path_safe(self, path: List[Position], boundary: Optional[Boundary],
                     hss_zones: List[HSSZone], frame=None) -> bool:
        if not path:
            return False
        if frame is None:
            frame = geo.LocalFrame.from_boundary_or_point(boundary, path[0])
        ctx = _PlanCtx(self.cfg, frame, boundary, hss_zones)
        for p in path:
            if not ctx.point_hard_safe(p):
                return False
        for i in range(1, len(path)):
            if not ctx.segment_hard_safe(path[i - 1], path[i]):
                return False
        return True

    def _finish(self, waypoints, start, goal, t0, already=False) -> PathResult:
        total = sum(geo.haversine_distance(waypoints[i - 1], waypoints[i])
                    for i in range(1, len(waypoints)))
        return PathResult(success=True, waypoints=waypoints,
                          total_distance_m=total, planning_time_ms=elapsed_ms(t0))


class _PlanCtx:
    """Tek bir planlama cagrisi icin shapely geometrisi (LocalFrame metre-duzlemi)."""

    def __init__(self, cfg: PlannerConfig, frame: geo.LocalFrame,
                 boundary: Optional[Boundary], hss_zones: List[HSSZone]):
        self.cfg = cfg
        self.frame = frame
        # SOFT koridor: HSS buffer + turn marji. turn marji donus yaricapindan
        # kucuk kalmaz -> ucak WP'yi keserken HARD koridora girmez.
        self.soft_margin = (
            cfg.hss_buffer_m
            + max(cfg.turn_safety_margin_m, cfg.min_turn_radius_m)
            + max(0.0, cfg.escape_prevention_margin_m)
        )
        self.hard_margin = cfg.hss_buffer_m
        quad = max(2, cfg.polygon_segments // 4)
        poly_segments = quad * 4
        chord_compensation = 1.0 / math.cos(math.pi / poly_segments)

        active = [z for z in hss_zones if z.active]
        self.soft_obstacles: List[Polygon] = []
        self.soft_interiors: List[Polygon] = []
        self.hard_circles: List[Tuple[Tuple[float, float], float]] = []
        for z in active:
            cx, cy = frame.to_xy(z.center)
            soft_radius = (z.radius + self.soft_margin) * chord_compensation
            self.soft_obstacles.append(
                Point(cx, cy).buffer(soft_radius, quad_segs=quad))
            self.soft_interiors.append(
                Point(cx, cy).buffer(max(0.0, soft_radius - 0.5), quad_segs=quad))
            self.hard_circles.append(((cx, cy), z.radius + self.hard_margin))

        # boundary -> izinli bolge (ic buffer)
        self.boundary_poly: Optional[Polygon] = None
        self.allowed_region: Optional[Polygon] = None
        if boundary is not None and boundary.is_valid():
            coords = [frame.to_xy(v) for v in boundary.vertices]
            self.boundary_poly = Polygon(coords)
            if not self.boundary_poly.is_valid:
                self.boundary_poly = self.boundary_poly.buffer(0)
            self.allowed_region = self.boundary_poly.buffer(-cfg.boundary_buffer_m)
            if self.allowed_region.is_empty:
                self.allowed_region = self.boundary_poly

    # -- SOFT (planlama) --
    def edge_visible_xy(self, a: Tuple[float, float], b: Tuple[float, float],
                        skip_a: int = -1, skip_b: int = -1) -> bool:
        line = LineString([a, b])
        for idx, interior in enumerate(self.soft_interiors):
            if idx == skip_a or idx == skip_b:
                continue
            if line.intersects(interior):
                return False
        if self.allowed_region is not None:
            if not self.allowed_region.covers(line):
                return False
        return True

    def segment_soft_clear(self, a: Position, b: Position) -> bool:
        return self.edge_visible_xy(self.frame.to_xy(a), self.frame.to_xy(b))

    def find_containing_soft_obstacle(self, xy: Tuple[float, float]) -> int:
        pt = Point(xy)
        for idx, poly in enumerate(self.soft_obstacles):
            if poly.contains(pt):
                return idx
        return -1

    def edge_hard_safe_xy(self, a: Tuple[float, float], b: Tuple[float, float]) -> bool:
        line = LineString([a, b])
        for (cx, cy), r in self.hard_circles:
            if line.distance(Point(cx, cy)) < r:
                return False
        if self.boundary_poly is not None and not self.boundary_poly.covers(line):
            return False
        return True

    # -- HARD (final dogrulama) --
    def point_hard_safe(self, p: Position) -> bool:
        x, y = self.frame.to_xy(p)
        for (cx, cy), r in self.hard_circles:
            if math.hypot(x - cx, y - cy) < r:
                return False
        if self.boundary_poly is not None:
            pt = Point(x, y)
            if not self.boundary_poly.covers(pt):
                return False
            if self.boundary_poly.exterior.distance(pt) < self.cfg.boundary_buffer_m:
                return False
        return True

    def segment_hard_safe(self, a: Position, b: Position) -> bool:
        ax, ay = self.frame.to_xy(a)
        bx, by = self.frame.to_xy(b)
        line = LineString([(ax, ay), (bx, by)])
        for (cx, cy), r in self.hard_circles:
            if line.distance(Point(cx, cy)) < r:
                return False
        if self.boundary_poly is not None:
            if not self.boundary_poly.covers(line):
                return False
        return True

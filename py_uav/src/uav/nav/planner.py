"""Yol planlayici -- uc kademeli sabit-kanat planlama orkestrasyonu.

Felsefe degismedi: "Python plans, ArduPlane flies". Cikti her zaman duz bir
waypoint LISTESI (DO_REPOSITION zinciri); fark, listenin artik kinematik-
farkindali uretilmesi.

Kademeler (plan()):
  0) Direkt konfor-marjli temiz ise -> [start, goal] (en hizli yol).
  1) Konforlu görünürlük grafi (eski davranis): HSS'ler TAM soft marjla
     sisirilir; genis alanda bugunku kanitlanmis genis-klerans rotalari.
  2) Hybrid A* (cost-field + Dubins primitifleri): sert duvar yalniz
     HSS + hss_buffer + izleme payi; konfor marji MALIYET. Dar koridorlar
     uculabilir, baslangic/varis heading'i hesaba katilir. Kademe 1 yoksa
     (koridor muhurlu) veya varis heading'i istendiyse devreye girer.
  3) Son care: indirgenmis-marjli görünürlük grafi (heading-kor ama dar
     koridoru acabilen). Hybrid butce asiminda "yol yok" yerine yol verir.

Iki guvenlik kavrami:
  HARD  : HSS yaricap + hss_buffer_m, sinir - boundary_buffer_m. Final
          dogrulama (is_path_safe) ve SafetyGate ile ayni metrik.
  KONFOR: hard + max(turn_safety, donus yaricapi) + escape_prevention.
          Kademe 1'de duvar, kademe 2'de maliyet bandi.

Homotopi: PathResult.signature yolun her HSS'i hangi yandan gectigini tasir;
plan(prefer_signature=...) ile onceki yan tercih edilir (flip-flop onleme).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import networkx as nx
from shapely.geometry import LineString, Point, Polygon

from ..core import geo
from ..core.time_utils import elapsed_ms, now_monotonic
from ..core.types import Boundary, HSSZone, Position
from ..infra.log import get_logger
from .cost_field import CostMap, CostMapConfig, ZoneCircle
from .hybrid_astar import HybridAStarConfig
from .hybrid_astar import search as hybrid_search

_log = get_logger("planner")

_G = 9.80665


def heading_to_theta(heading_deg: float) -> float:
    """Pusula (0=Kuzey, CW) -> matematik acisi (radyan, +x'ten CCW)."""
    return math.radians(90.0 - heading_deg)


def theta_to_heading(theta: float) -> float:
    return (90.0 - math.degrees(theta)) % 360.0


@dataclass
class PlannerConfig:
    # HARD katman (SafetyGate ile ayni anlam)
    hss_buffer_m: float = 40.0
    boundary_buffer_m: float = 20.0
    # kinematik
    min_turn_radius_m: float = 50.0
    max_bank_deg: float = 35.0
    # konfor marji bilesenleri (kademe 1 duvari, kademe 2 maliyet bandi)
    turn_safety_margin_m: float = 50.0
    escape_prevention_margin_m: float = 80.0
    # planlayicinin sert duvarina eklenen izleme hatasi payi (L1 sapmasi vb.)
    tracking_error_m: float = 15.0
    # direkt-ucus (pursuit/look-ahead) kontrolu icin hard'a ek marj
    direct_clear_margin_m: float = 25.0
    # cikti ornekleme
    max_segment_length_m: float = 80.0
    max_arc_step_deg: float = 25.0
    # gorunurluk grafi cozunurlugu
    polygon_segments: int = 16
    sample_step_m: float = 15.0
    # hybrid A* / maliyet alani
    dubins_smoothing_enabled: bool = True
    soft_cost_max: float = 4.0
    boundary_soft_band_m: float = 60.0
    boundary_cost_max: float = 1.5
    side_bias_cost: float = 1.0
    cell_size_m: float = 15.0
    heading_bins: int = 16
    max_expansions: int = 30000
    time_budget_ms: float = 600.0


@dataclass
class PathResult:
    success: bool = False
    waypoints: List[Position] = None
    total_distance_m: float = 0.0
    planning_time_ms: float = 0.0
    error: str = ""
    signature: Tuple[Tuple[int, int], ...] = ()
    stage: str = ""   # "direct" | "graph" | "hybrid" | "graph_reduced"

    def __post_init__(self):
        if self.waypoints is None:
            self.waypoints = []


class PathPlanner:
    def __init__(self, config: Optional[PlannerConfig] = None):
        self.cfg = config or PlannerConfig()

    # ===================== marj/kinematik yardimcilari =====================
    def planning_margin(self) -> float:
        """KONFOR marji: hard buffer + donus + escape (kademe 1 duvari)."""
        return self.cfg.hss_buffer_m + max(self.cfg.turn_safety_margin_m,
                                           self.cfg.min_turn_radius_m) + max(
                                               0.0, self.cfg.escape_prevention_margin_m)

    def direct_margin(self) -> float:
        """Direkt-ucus (pursuit/look-ahead) icin HSS'e ek marj.

        Konfor marjindan kasitli olarak kucuk: koridor icinde takibi
        engellemez; SafetyGate hard katmani ayrica korur.
        """
        return (self.cfg.hss_buffer_m + self.cfg.tracking_error_m
                + self.cfg.direct_clear_margin_m)

    def turn_radius_for(self, speed_ms: Optional[float]) -> float:
        """Hiza bagli minimum donus yaricapi: R = v^2 / (g*tan(bank))."""
        r = self.cfg.min_turn_radius_m
        if speed_ms is not None and speed_ms > 1.0:
            bank = math.radians(max(10.0, self.cfg.max_bank_deg))
            r = max(r, speed_ms * speed_ms / (_G * math.tan(bank)))
        return r

    def set_min_turn_radius(self, r: float) -> None:
        self.cfg.min_turn_radius_m = max(0.0, r)
        # konfor marjini yaricaptan kucuk birakma
        self.cfg.turn_safety_margin_m = max(self.cfg.turn_safety_margin_m, r)

    # ===================== Genel API =====================
    def plan(self, start: Position, goal: Position,
             boundary: Optional[Boundary], hss_zones: List[HSSZone],
             start_heading_deg: Optional[float] = None,
             goal_heading_deg: Optional[float] = None,
             speed_ms: Optional[float] = None,
             prefer_signature: Tuple[Tuple[int, int], ...] = ()) -> PathResult:
        t0 = now_monotonic()
        frame = geo.LocalFrame.from_boundary_or_point(boundary, start)
        ctx = _PlanCtx(self.cfg, frame, boundary, hss_zones)

        want_goal_heading = goal_heading_deg is not None

        # Kademe 0: direkt konfor-marjli temiz (varis heading'i istenmiyorsa)
        if not want_goal_heading and ctx.segment_comfort_clear(start, goal):
            return self._finish([start, goal], t0, "direct", ctx)

        # Kademe 1: konforlu gorunurluk grafi (eski kanitlanmis davranis)
        if not want_goal_heading:
            res = self._graph_plan(ctx, start, goal, ctx.comfort_obstacles,
                                   ctx.comfort_interiors, t0, "graph")
            if res is not None:
                return res

        # Kademe 2: hybrid A* (koridor + heading farkindaligi)
        if self.cfg.dubins_smoothing_enabled:
            res = self._hybrid_plan(ctx, start, goal, start_heading_deg,
                                    goal_heading_deg, speed_ms,
                                    prefer_signature, t0)
            if res is not None:
                return res

        # Kademe 3: indirgenmis-marjli graf (son care, heading-kor)
        res = self._graph_plan(ctx, start, goal, ctx.reduced_obstacles,
                               ctx.reduced_interiors, t0, "graph_reduced")
        if res is not None:
            return res

        return PathResult(success=False, error="yol bulunamadi (tum kademeler)",
                          planning_time_ms=elapsed_ms(t0))

    def make_safe(self, waypoints: List[Position],
                  boundary: Optional[Boundary], hss_zones: List[HSSZone],
                  start_heading_deg: Optional[float] = None,
                  speed_ms: Optional[float] = None) -> PathResult:
        """Verilen waypoint LISTESINI HSS/fence-guvenli hale getir.

        Bacaklar plan() ile detour'lanir; heading surekliligi korunur (bir
        bacagin cikis yonu sonrakinin baslangic heading'i olur).
        """
        t0 = now_monotonic()
        if len(waypoints) < 2:
            return PathResult(success=False, error="en az 2 waypoint gerekli",
                              planning_time_ms=elapsed_ms(t0))
        stitched: List[Position] = [waypoints[0]]
        heading = start_heading_deg
        for i in range(1, len(waypoints)):
            leg = self.plan(stitched[-1], waypoints[i], boundary, hss_zones,
                            start_heading_deg=heading, speed_ms=speed_ms)
            if not leg.success:
                return PathResult(success=False,
                                  error=f"bacak {i} planlanamadi: {leg.error}",
                                  planning_time_ms=elapsed_ms(t0))
            stitched.extend(leg.waypoints[1:])
            if len(stitched) >= 2:
                heading = geo.bearing(stitched[-2], stitched[-1])
        total = sum(geo.haversine_distance(stitched[i - 1], stitched[i])
                    for i in range(1, len(stitched)))
        return PathResult(success=True, waypoints=stitched,
                          total_distance_m=total, planning_time_ms=elapsed_ms(t0))

    def is_direct_clear(self, frm: Position, to: Position,
                        boundary: Optional[Boundary], hss_zones: List[HSSZone],
                        margin_m: Optional[float] = None) -> bool:
        """Direkt segment temiz mi?

        margin_m None -> KONFOR marji (eski davranis, planlama tercihi).
        margin_m verilirse -> HSS yaricap + margin_m sisirmesiyle kontrol
        (pursuit/look-ahead icin direct_margin() onerilir).
        """
        frame = geo.LocalFrame.from_boundary_or_point(boundary, frm)
        ctx = _PlanCtx(self.cfg, frame, boundary, hss_zones)
        if margin_m is None:
            return ctx.segment_comfort_clear(frm, to)
        return ctx.segment_clear_with_margin(frm, to, margin_m)

    # ===================== Kademe 1/3: gorunurluk grafi =====================
    def _graph_plan(self, ctx: "_PlanCtx", start: Position, goal: Position,
                    obstacles: List[Polygon], interiors: List[Polygon],
                    t0: float, stage: str) -> Optional[PathResult]:
        path_xy = self._visibility_path(ctx, start, goal, obstacles, interiors)
        if path_xy is None:
            return None
        path_xy = self._simplify(ctx, path_xy, obstacles, interiors)
        positions = self._xy_to_positions(ctx, [(x, y) for x, y in path_xy],
                                          start, goal)
        positions = self._densify(positions)
        if not self.is_path_safe(positions, ctx.boundary, ctx.zones, frame=ctx.frame):
            return None
        return self._finish(positions, t0, stage, ctx)

    def _visibility_path(self, ctx: "_PlanCtx", start: Position, goal: Position,
                         obstacles: List[Polygon], interiors: List[Polygon]):
        start_xy = ctx.frame.to_xy(start)
        goal_xy = ctx.frame.to_xy(goal)

        nodes: List[Tuple[Tuple[float, float], int, int]] = [
            (start_xy, -1, -1), (goal_xy, -1, -1)]
        for obs_idx, poly in enumerate(obstacles):
            for vertex_idx, xy in enumerate(list(poly.exterior.coords)[:-1]):
                nodes.append((xy, obs_idx, vertex_idx))

        start_containing = _containing_idx(obstacles, start_xy)
        goal_containing = _containing_idx(obstacles, goal_xy)

        g = nx.Graph()
        for i in range(len(nodes)):
            g.add_node(i)
        n = len(nodes)
        for i in range(n):
            for j in range(i + 1, n):
                a_xy, a_obs, a_v = nodes[i]
                b_xy, b_obs, b_v = nodes[j]
                if a_obs >= 0 and a_obs == b_obs:
                    vertex_count = len(list(obstacles[a_obs].exterior.coords)) - 1
                    diff = abs(a_v - b_v)
                    if not (diff == 1 or diff == vertex_count - 1):
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
                if not ctx.edge_hard_safe_xy(a_xy, b_xy):
                    continue
                if _edge_visible(interiors, ctx.allowed_region, a_xy, b_xy,
                                 skip_a, skip_b):
                    g.add_edge(i, j, weight=math.dist(a_xy, b_xy))
        try:
            idx_path = nx.shortest_path(g, 0, 1, weight="weight")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return None
        return [nodes[i][0] for i in idx_path]

    def _simplify(self, ctx: "_PlanCtx", path_xy, obstacles, interiors):
        if len(path_xy) <= 2:
            return path_xy
        out = [path_xy[0]]
        cur = 0
        while cur < len(path_xy) - 1:
            nxt = cur + 1
            for cand in range(len(path_xy) - 1, cur + 1, -1):
                skip_a = _containing_idx(obstacles, path_xy[cur])
                skip_b = _containing_idx(obstacles, path_xy[cand])
                if (ctx.edge_hard_safe_xy(path_xy[cur], path_xy[cand])
                        and _edge_visible(interiors, ctx.allowed_region,
                                          path_xy[cur], path_xy[cand],
                                          skip_a, skip_b)):
                    nxt = cand
                    break
            out.append(path_xy[nxt])
            cur = nxt
        return out

    # ===================== Kademe 2: hybrid A* =====================
    def _hybrid_plan(self, ctx: "_PlanCtx", start: Position, goal: Position,
                     start_heading_deg: Optional[float],
                     goal_heading_deg: Optional[float],
                     speed_ms: Optional[float],
                     prefer_signature: Tuple[Tuple[int, int], ...],
                     t0: float) -> Optional[PathResult]:
        start_xy = ctx.frame.to_xy(start)
        goal_xy = ctx.frame.to_xy(goal)
        if start_heading_deg is None:
            start_heading_deg = geo.bearing(start, goal)
        q0 = (start_xy[0], start_xy[1], heading_to_theta(start_heading_deg))
        goal_theta = (heading_to_theta(goal_heading_deg)
                      if goal_heading_deg is not None else None)

        side_bias: Dict[int, int] = {zid: side for zid, side in prefer_signature}
        cmap = ctx.build_cost_map(side_bias=side_bias,
                                  chord=(start_xy[0], start_xy[1],
                                         goal_xy[0], goal_xy[1]))
        radius = self.turn_radius_for(speed_ms)
        hcfg = HybridAStarConfig(
            cell_size_m=self.cfg.cell_size_m,
            heading_bins=self.cfg.heading_bins,
            max_expansions=self.cfg.max_expansions,
            time_budget_s=self.cfg.time_budget_ms / 1000.0,
        )
        pts = hybrid_search(q0, goal_xy, goal_theta, radius, cmap, hcfg)
        if pts is None:
            return None
        sparse = self._downsample(pts)
        positions = self._xy_to_positions(ctx, sparse, start, goal)
        if not self.is_path_safe(positions, ctx.boundary, ctx.zones, frame=ctx.frame):
            return None
        return self._finish(positions, t0, "hybrid", ctx, cmap=cmap, fine_pts=pts)

    def _downsample(self, pts) -> List[Tuple[float, float]]:
        """Ince cozunurluklu (x,y,theta) izini seyrek waypoint'lere indir.

        Kural: son tutulan noktadan bu yana |heading degisimi| >=
        max_arc_step_deg VEYA mesafe >= max_segment_length_m ise noktayi tut.
        Arc'larda sik, duzlerde seyrek waypoint uretir.
        """
        if len(pts) <= 2:
            return [(p[0], p[1]) for p in pts]
        max_turn = math.radians(max(5.0, self.cfg.max_arc_step_deg))
        max_seg = max(20.0, self.cfg.max_segment_length_m)
        out = [(pts[0][0], pts[0][1])]
        last_theta = pts[0][2]
        acc_dist = 0.0
        for prev, cur in zip(pts, pts[1:]):
            acc_dist += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            dth = abs((cur[2] - last_theta + math.pi) % (2.0 * math.pi) - math.pi)
            if dth >= max_turn or acc_dist >= max_seg:
                out.append((cur[0], cur[1]))
                last_theta = cur[2]
                acc_dist = 0.0
        if out[-1] != (pts[-1][0], pts[-1][1]):
            out.append((pts[-1][0], pts[-1][1]))
        return out

    # ===================== ortak cikti yardimcilari =====================
    def _xy_to_positions(self, ctx: "_PlanCtx", path_xy, start: Position,
                         goal: Position, snap_goal: bool = True) -> List[Position]:
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
        positions[0] = Position(start.latitude, start.longitude, start.altitude)
        if snap_goal:
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

    def _finish(self, waypoints: List[Position], t0: float, stage: str,
                ctx: "_PlanCtx", cmap: Optional[CostMap] = None,
                fine_pts=None) -> PathResult:
        total = sum(geo.haversine_distance(waypoints[i - 1], waypoints[i])
                    for i in range(1, len(waypoints)))
        signature: Tuple[Tuple[int, int], ...] = ()
        if ctx.zones:
            if cmap is None:
                cmap = ctx.build_cost_map()
            pts_xy = (fine_pts if fine_pts is not None
                      else [ctx.frame.to_xy(p) for p in waypoints])
            signature = cmap.signature_of([(p[0], p[1]) for p in pts_xy])
        return PathResult(success=True, waypoints=waypoints,
                          total_distance_m=total, planning_time_ms=elapsed_ms(t0),
                          signature=signature, stage=stage)

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


# =============================================================================
# _PlanCtx: tek planlama cagrisinin geometrisi (LocalFrame metre-duzlemi)
# =============================================================================

class _PlanCtx:
    def __init__(self, cfg: PlannerConfig, frame: geo.LocalFrame,
                 boundary: Optional[Boundary], hss_zones: List[HSSZone]):
        self.cfg = cfg
        self.frame = frame
        self.boundary = boundary
        self.zones = [z for z in hss_zones if z.active]

        self.hard_margin = cfg.hss_buffer_m
        self.wall_margin = cfg.hss_buffer_m + cfg.tracking_error_m
        self.comfort_margin = (cfg.hss_buffer_m
                               + max(cfg.turn_safety_margin_m, cfg.min_turn_radius_m)
                               + max(0.0, cfg.escape_prevention_margin_m))
        self.reduced_margin = self.wall_margin + 10.0

        quad = max(2, cfg.polygon_segments // 4)
        poly_segments = quad * 4
        chord_comp = 1.0 / math.cos(math.pi / poly_segments)

        self.hard_circles: List[Tuple[Tuple[float, float], float]] = []
        self.comfort_obstacles: List[Polygon] = []
        self.comfort_interiors: List[Polygon] = []
        self.reduced_obstacles: List[Polygon] = []
        self.reduced_interiors: List[Polygon] = []
        for z in self.zones:
            cx, cy = frame.to_xy(z.center)
            self.hard_circles.append(((cx, cy), z.radius + self.hard_margin))
            for margin, obs, ints in (
                    (self.comfort_margin, self.comfort_obstacles, self.comfort_interiors),
                    (self.reduced_margin, self.reduced_obstacles, self.reduced_interiors)):
                r = (z.radius + margin) * chord_comp
                obs.append(Point(cx, cy).buffer(r, quad_segs=quad))
                ints.append(Point(cx, cy).buffer(max(0.0, r - 0.5), quad_segs=quad))

        self.boundary_poly: Optional[Polygon] = None
        self.allowed_region: Optional[Polygon] = None
        self._boundary_xy: Optional[List[Tuple[float, float]]] = None
        if boundary is not None and boundary.is_valid():
            coords = [frame.to_xy(v) for v in boundary.vertices]
            self._boundary_xy = coords
            self.boundary_poly = Polygon(coords)
            if not self.boundary_poly.is_valid:
                self.boundary_poly = self.boundary_poly.buffer(0)
            self.allowed_region = self.boundary_poly.buffer(-cfg.boundary_buffer_m)
            if self.allowed_region.is_empty:
                self.allowed_region = self.boundary_poly

    # -- hybrid A* icin maliyet haritasi --
    def build_cost_map(self, side_bias: Optional[Dict[int, int]] = None,
                       chord: Optional[Tuple[float, float, float, float]] = None) -> CostMap:
        soft_band = max(10.0, self.comfort_margin - self.wall_margin)
        zones = [ZoneCircle(z.id, *self.frame.to_xy(z.center),
                            hard_r=z.radius + self.wall_margin)
                 for z in self.zones]
        return CostMap(
            zones, self._boundary_xy, self.cfg.boundary_buffer_m,
            CostMapConfig(soft_band_m=soft_band,
                          soft_cost_max=self.cfg.soft_cost_max,
                          boundary_soft_band_m=self.cfg.boundary_soft_band_m,
                          boundary_cost_max=self.cfg.boundary_cost_max,
                          side_bias_cost=self.cfg.side_bias_cost),
            side_bias=side_bias, chord=chord)

    # -- konfor / marjli kontroller --
    def segment_comfort_clear(self, a: Position, b: Position) -> bool:
        return _edge_visible(self.comfort_interiors, self.allowed_region,
                             self.frame.to_xy(a), self.frame.to_xy(b), -1, -1)

    def segment_clear_with_margin(self, a: Position, b: Position,
                                  margin_m: float) -> bool:
        ax, ay = self.frame.to_xy(a)
        bx, by = self.frame.to_xy(b)
        line = LineString([(ax, ay), (bx, by)])
        for z in self.zones:
            cx, cy = self.frame.to_xy(z.center)
            if line.distance(Point(cx, cy)) < z.radius + margin_m:
                return False
        if self.allowed_region is not None and not self.allowed_region.covers(line):
            return False
        return True

    def edge_hard_safe_xy(self, a: Tuple[float, float], b: Tuple[float, float]) -> bool:
        line = LineString([a, b])
        for (cx, cy), r in self.hard_circles:
            if line.distance(Point(cx, cy)) < r:
                return False
        if self.boundary_poly is not None and not self.boundary_poly.covers(line):
            return False
        return True

    # -- HARD (final dogrulama; SafetyGate ile ayni metrik) --
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


# =============================================================================
# modul-ici saf yardimcilar
# =============================================================================

def _containing_idx(polys: List[Polygon], xy: Tuple[float, float]) -> int:
    pt = Point(xy)
    for idx, poly in enumerate(polys):
        if poly.contains(pt):
            return idx
    return -1


def _edge_visible(interiors: List[Polygon], allowed_region: Optional[Polygon],
                  a: Tuple[float, float], b: Tuple[float, float],
                  skip_a: int, skip_b: int) -> bool:
    line = LineString([a, b])
    for idx, interior in enumerate(interiors):
        if idx == skip_a or idx == skip_b:
            continue
        if line.intersects(interior):
            return False
    if allowed_region is not None and not allowed_region.covers(line):
        return False
    return True

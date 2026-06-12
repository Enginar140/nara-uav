"""Hybrid A* -- (x, y, heading) lattice'inde Dubins-primitifli arama.

Sabit-kanat icin kinematik-uyumlu planlayici cekirdegi:
  - Durum: surekli (x, y, theta); kapali kume (hucre, heading-dilimi) ile.
  - Primitifler: duz / sol arc / sag arc; arc bir heading dilimi kadar dondurur
    (arc uzunlugu = R * 2pi/bins), boylece lattice tutarli kalir.
  - Maliyet: uzunluk * (1 + CostMap.point_cost) + donus/yon-degisim cezalari.
  - Dubins shot: periyodik olarak dugumden hedefe analitik Dubins denenir;
    sert-uzay dogrulamasini gecerse arama erken biter (hedef heading'i garanti).
  - Heuristik: hedef heading'liyse Dubins uzunlugu, degilse Oklid (ikisi de
    engelsiz alt-sinir -> kabul edilebilir).

Butce: max_expansions + time_budget_s; asilirsa None (cagiran fallback'e duser).
"""
from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .cost_field import CostMap
from .dubins import (DubinsPath, mod2pi, sample_path, shortest_path,
                     shortest_path_to_point)

State = Tuple[float, float, float]   # (x, y, theta)
Key = Tuple[int, int, int]           # (ix, iy, heading_bin)


@dataclass
class HybridAStarConfig:
    cell_size_m: float = 15.0
    heading_bins: int = 16
    turn_cost_factor: float = 1.05        # arc'lar duzlerden hafifce pahali
    direction_switch_penalty: float = 8.0  # L<->R gecisinde ek (zigzag onleme)
    max_expansions: int = 30000
    time_budget_s: float = 0.6
    shot_interval: int = 8                 # hedefe yakinken shot deneme sikligi
    shot_interval_far: int = 24            # uzaktayken (shot ornekleme pahali)
    shot_near_distance_m: float = 600.0
    sample_step_m: float = 12.0            # primitif/shot sert-uzay ornekleme
    goal_xy_tolerance_m: float = 30.0      # shot'siz hedef kabulu
    # Agirlikli A*: h carpani >1 aramayi keskin hizlandirir; sonuc en fazla
    # ~bu oranda alt-optimal olur (incumbent f>=best sonlandirmasiyla birlikte).
    heuristic_weight: float = 1.3
    # Incumbent'in birim-maliyeti bu orandan azsa (soft maliyet ~yok) erken cik.
    early_exit_cost_slack: float = 0.10


class _Node:
    __slots__ = ("state", "g", "parent", "samples", "action")

    def __init__(self, state: State, g: float, parent: Optional[Key],
                 samples: List[State], action: str):
        self.state = state
        self.g = g
        self.parent = parent
        self.samples = samples   # parent'tan bu duguma ara ornekler (uc dahil)
        self.action = action     # 'S' | 'L' | 'R' | '' (start)


def search(start: State, goal_xy: Tuple[float, float],
           goal_theta: Optional[float], radius: float, cmap: CostMap,
           cfg: Optional[HybridAStarConfig] = None) -> Optional[List[State]]:
    """start'tan hedefe sert-guvenli, maliyet-farkindali, ucusabilir yol.

    Donus: ~sample_step_m cozunurlugunde (x, y, theta) listesi (start dahil)
    veya None (yol yok / butce asildi).
    """
    cfg = cfg or HybridAStarConfig()
    if radius <= 1e-6 or not cmap.hard_free(start[0], start[1]):
        return None

    dtheta = 2.0 * math.pi / cfg.heading_bins
    arc_len = radius * dtheta
    step_len = max(arc_len, cfg.cell_size_m * 1.25)

    def key_of(s: State) -> Key:
        return (int(math.floor(s[0] / cfg.cell_size_m)),
                int(math.floor(s[1] / cfg.cell_size_m)),
                int(mod2pi(s[2]) / dtheta) % cfg.heading_bins)

    def heuristic(s: State) -> float:
        if goal_theta is not None:
            p = shortest_path(s, (goal_xy[0], goal_xy[1], goal_theta), radius)
            if p is not None:
                return p.length * cfg.heuristic_weight
        return math.hypot(goal_xy[0] - s[0], goal_xy[1] - s[1]) * cfg.heuristic_weight

    def try_shot(s: State) -> Optional[Tuple[List[State], float]]:
        """Hedefe analitik Dubins; sert-guvenliyse (ornekler, soft maliyet)."""
        if goal_theta is not None:
            p = shortest_path(s, (goal_xy[0], goal_xy[1], goal_theta), radius)
        else:
            p = shortest_path_to_point(s, goal_xy, radius)
        if p is None:
            return None
        pts = sample_path(p, cfg.sample_step_m)
        if not _polyline_hard_free(pts, cmap):
            return None
        cost = 0.0
        for (ax, ay, _), (bx, by, _) in zip(pts, pts[1:]):
            seg = math.hypot(bx - ax, by - ay)
            cost += seg * (1.0 + cmap.point_cost(bx, by))
        return pts, cost

    start_key = key_of(start)
    nodes: Dict[Key, _Node] = {start_key: _Node(start, 0.0, None, [start], "")}
    open_heap: List[Tuple[float, int, Key]] = []
    counter = 0
    heapq.heappush(open_heap, (heuristic(start), counter, start_key))
    closed: Dict[Key, float] = {}
    deadline = time.monotonic() + cfg.time_budget_s
    pops = 0
    # Incumbent: shot/hedef-kabul adaylarinin en iyisi. Heap min'i (f) bir alt
    # sinir oldugundan, f >= best_cost olunca incumbent optimaldir (bu metrikte).
    best_path: Optional[List[State]] = None
    best_cost = math.inf

    while open_heap:
        f, _, key = heapq.heappop(open_heap)
        if f >= best_cost:
            return best_path  # incumbent artik iyilestirilemez
        node = nodes.get(key)
        if node is None or key in closed:
            continue
        closed[key] = node.g
        pops += 1
        if pops > cfg.max_expansions:
            return best_path
        if (pops & 0xFF) == 0 and time.monotonic() > deadline:
            return best_path

        s = node.state
        # Dubins shot: hedefe yakinken sik, uzaktayken seyrek (ornekleme pahali)
        dist_goal = math.hypot(goal_xy[0] - s[0], goal_xy[1] - s[1])
        interval = (cfg.shot_interval if dist_goal <= cfg.shot_near_distance_m
                    else cfg.shot_interval_far)
        if pops == 1 or (pops % interval) == 0:
            shot = try_shot(s)
            if shot is not None:
                pts, shot_cost = shot
                total = node.g + shot_cost
                if total < best_cost:
                    best_cost = total
                    best_path = _reconstruct(nodes, key) + pts[1:]
                    # soft maliyeti ihmal edilebilir incumbent: pratikte optimal
                    if shot_cost <= pts_len(pts) * (1.0 + cfg.early_exit_cost_slack):
                        return best_path

        # shot'siz hedef kabulu (heading serbest ya da dilim toleransi)
        if math.hypot(goal_xy[0] - s[0], goal_xy[1] - s[1]) <= cfg.goal_xy_tolerance_m:
            if goal_theta is None or abs(
                    mod2pi(s[2] - goal_theta + math.pi) - math.pi) <= dtheta * 1.5:
                if node.g < best_cost:
                    best_cost = node.g
                    best_path = _reconstruct(nodes, key)

        for action in ("S", "L", "R"):
            samples = _primitive(s, action, step_len, radius, cfg.sample_step_m)
            end = samples[-1]
            if not _polyline_hard_free(samples, cmap, skip_first=True):
                continue
            ekey = key_of(end)
            cost = step_len * (1.0 + cmap.point_cost(end[0], end[1]))
            if action != "S":
                cost *= cfg.turn_cost_factor
                if node.action not in ("", action, "S"):
                    cost += cfg.direction_switch_penalty
            g2 = node.g + cost
            existing = nodes.get(ekey)
            if ekey in closed or (existing is not None and existing.g <= g2):
                continue
            nodes[ekey] = _Node(end, g2, key, samples[1:], action)
            counter += 1
            heapq.heappush(open_heap, (g2 + heuristic(end), counter, ekey))
    return best_path


def pts_len(pts: List[State]) -> float:
    """Ornek listesinin toplam yatay uzunlugu (m)."""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def _primitive(s: State, action: str, length: float, radius: float,
               sample_step: float) -> List[State]:
    """Tek hareket primitifinin ara ornekleri (baslangic dahil)."""
    x, y, theta = s
    n = max(1, int(math.ceil(length / sample_step)))
    sub = length / n
    out: List[State] = [s]
    if action == "S":
        for i in range(1, n + 1):
            d = sub * i
            out.append((x + d * math.cos(theta), y + d * math.sin(theta), theta))
        return out
    sign = 1.0 if action == "L" else -1.0
    for i in range(1, n + 1):
        nt = theta + sign * (sub * i) / radius
        if action == "L":
            nx = x + radius * (math.sin(nt) - math.sin(theta))
            ny = y - radius * (math.cos(nt) - math.cos(theta))
        else:
            nx = x - radius * (math.sin(nt) - math.sin(theta))
            ny = y + radius * (math.cos(nt) - math.cos(theta))
        out.append((nx, ny, nt))
    return out


def _polyline_hard_free(pts: List[State], cmap: CostMap,
                        skip_first: bool = False) -> bool:
    """Nokta-bazli HSS kontrolu + sinir icin tek polyline covers."""
    start_i = 1 if skip_first else 0
    for i in range(start_i, len(pts)):
        x, y = pts[i][0], pts[i][1]
        for z in cmap.zones:
            if math.hypot(x - z.cx, y - z.cy) < z.hard_r:
                return False
    return cmap.polyline_in_allowed([(p[0], p[1]) for p in pts])


def _reconstruct(nodes: Dict[Key, _Node], key: Key) -> List[State]:
    chunks: List[List[State]] = []
    k: Optional[Key] = key
    while k is not None:
        node = nodes[k]
        chunks.append(node.samples)
        k = node.parent
    chunks.reverse()
    out: List[State] = []
    for c in chunks:
        out.extend(c)
    return out

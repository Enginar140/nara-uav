"""Dubins yol primitifleri -- sabit-kanat minimum donus yaricapi kinematigi.

Saf xy-duzlemi (LocalFrame metre; x=Dogu, y=Kuzey degil -- BURASI matematik
duzlemi: theta radyan, +x ekseninden CCW). Pusula <-> matematik donusumu
planner katmaninda yapilir: theta = radians(90 - heading_deg).

Iki cozucu:
  shortest_path(q0, q1, R)      : klasik 6-vaka Dubins (LSL,RSR,LSR,RSL,RLR,LRL)
  shortest_path_to_point(q0, p) : varis heading'i SERBEST (CS: don + duz git)

Cikti DubinsPath: ('L'|'S'|'R', uzunluk_m) segman listesi; sample_path ile
sabit adimda (x, y, theta) ornekleme.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

_TWO_PI = 2.0 * math.pi


def mod2pi(theta: float) -> float:
    """[0, 2*pi) araligina indirger."""
    t = math.fmod(theta, _TWO_PI)
    if t < 0.0:
        t += _TWO_PI
    return t


@dataclass(frozen=True)
class DubinsPath:
    q0: Tuple[float, float, float]          # (x, y, theta) baslangic
    radius: float                           # donus yaricapi (m)
    segments: Tuple[Tuple[str, float], ...]  # ('L'|'S'|'R', uzunluk_m)
    kind: str                               # "LSL" vb.

    @property
    def length(self) -> float:
        return sum(s for _, s in self.segments)


def _step(x: float, y: float, theta: float, seg_type: str, dist: float,
          radius: float) -> Tuple[float, float, float]:
    """Tek segman uzerinde dist metre ilerlet."""
    if dist <= 0.0:
        return (x, y, theta)
    if seg_type == "S":
        return (x + dist * math.cos(theta), y + dist * math.sin(theta), theta)
    dphi = dist / radius
    if seg_type == "L":
        nt = theta + dphi
        return (x + radius * (math.sin(nt) - math.sin(theta)),
                y - radius * (math.cos(nt) - math.cos(theta)), nt)
    # 'R'
    nt = theta - dphi
    return (x - radius * (math.sin(nt) - math.sin(theta)),
            y + radius * (math.cos(nt) - math.cos(theta)), nt)


def path_endpoint(path: DubinsPath) -> Tuple[float, float, float]:
    x, y, theta = path.q0
    for seg_type, seg_len in path.segments:
        x, y, theta = _step(x, y, theta, seg_type, seg_len, path.radius)
    return (x, y, theta)


def sample_path(path: DubinsPath, step_m: float) -> List[Tuple[float, float, float]]:
    """Yol boyunca ~step_m araliklarla (x, y, theta) noktalari (uclar dahil)."""
    step_m = max(0.5, step_m)
    pts: List[Tuple[float, float, float]] = [path.q0]
    x, y, theta = path.q0
    for seg_type, seg_len in path.segments:
        if seg_len <= 1e-9:
            continue
        n = max(1, int(math.ceil(seg_len / step_m)))
        sub = seg_len / n
        for _ in range(n):
            x, y, theta = _step(x, y, theta, seg_type, sub, path.radius)
            pts.append((x, y, theta))
    return pts


# ===================== 6-vaka klasik Dubins =====================
# Standart normalize form (Walker dubins.c esdegeri): mesafe d = |q1-q0| / R,
# a = mod2pi(theta0 - phi), b = mod2pi(theta1 - phi). Donen (t, p, q) yari-
# cap birimindedir; metreye R ile carparak cevrilir.

def _lsl(a: float, b: float, d: float):
    tmp = d + math.sin(a) - math.sin(b)
    p_sq = 2.0 + d * d - 2.0 * math.cos(a - b) + 2.0 * d * (math.sin(a) - math.sin(b))
    if p_sq < 0.0:
        return None
    ang = math.atan2(math.cos(b) - math.cos(a), tmp)
    return (mod2pi(ang - a), math.sqrt(p_sq), mod2pi(b - ang))


def _rsr(a: float, b: float, d: float):
    tmp = d - math.sin(a) + math.sin(b)
    p_sq = 2.0 + d * d - 2.0 * math.cos(a - b) + 2.0 * d * (math.sin(b) - math.sin(a))
    if p_sq < 0.0:
        return None
    ang = math.atan2(math.cos(a) - math.cos(b), tmp)
    return (mod2pi(a - ang), math.sqrt(p_sq), mod2pi(ang - b))


def _lsr(a: float, b: float, d: float):
    p_sq = -2.0 + d * d + 2.0 * math.cos(a - b) + 2.0 * d * (math.sin(a) + math.sin(b))
    if p_sq < 0.0:
        return None
    p = math.sqrt(p_sq)
    ang = math.atan2(-math.cos(a) - math.cos(b), d + math.sin(a) + math.sin(b)) \
        - math.atan2(-2.0, p)
    return (mod2pi(ang - a), p, mod2pi(ang - mod2pi(b)))


def _rsl(a: float, b: float, d: float):
    p_sq = -2.0 + d * d + 2.0 * math.cos(a - b) - 2.0 * d * (math.sin(a) + math.sin(b))
    if p_sq < 0.0:
        return None
    p = math.sqrt(p_sq)
    ang = math.atan2(math.cos(a) + math.cos(b), d - math.sin(a) - math.sin(b)) \
        - math.atan2(2.0, p)
    return (mod2pi(a - ang), p, mod2pi(b - ang))


def _rlr(a: float, b: float, d: float):
    tmp = (6.0 - d * d + 2.0 * math.cos(a - b) + 2.0 * d * (math.sin(a) - math.sin(b))) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = mod2pi(_TWO_PI - math.acos(tmp))
    t = mod2pi(a - math.atan2(math.cos(a) - math.cos(b), d - math.sin(a) + math.sin(b))
               + mod2pi(p / 2.0))
    return (t, p, mod2pi(a - b - t + mod2pi(p)))


def _lrl(a: float, b: float, d: float):
    tmp = (6.0 - d * d + 2.0 * math.cos(a - b) + 2.0 * d * (math.sin(b) - math.sin(a))) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = mod2pi(_TWO_PI - math.acos(tmp))
    t = mod2pi(-a - math.atan2(math.cos(a) - math.cos(b), d + math.sin(a) - math.sin(b))
               + p / 2.0)
    return (t, p, mod2pi(mod2pi(b) - a - t + mod2pi(p)))


_WORDS = (
    ("LSL", _lsl, ("L", "S", "L")),
    ("RSR", _rsr, ("R", "S", "R")),
    ("LSR", _lsr, ("L", "S", "R")),
    ("RSL", _rsl, ("R", "S", "L")),
    ("RLR", _rlr, ("R", "L", "R")),
    ("LRL", _lrl, ("L", "R", "L")),
)

# Analitik formul + yeniden-insa dogrulamasi icin uc nokta toleransi (m).
_ENDPOINT_TOL_M = 0.5


def shortest_path(q0: Tuple[float, float, float], q1: Tuple[float, float, float],
                  radius: float) -> Optional[DubinsPath]:
    """q0 -> q1 (her ikisi de heading'li) en kisa Dubins yolu."""
    if radius <= 1e-6:
        return None
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    big_d = math.hypot(dx, dy)
    d = big_d / radius
    phi = math.atan2(dy, dx) if big_d > 1e-9 else 0.0
    a = mod2pi(q0[2] - phi)
    b = mod2pi(q1[2] - phi)

    best: Optional[DubinsPath] = None
    best_len = math.inf
    for kind, fn, types in _WORDS:
        res = fn(a, b, d)
        if res is None:
            continue
        t, p, q = res
        # t, p, q yaricap birimindedir (CSC'de p duz, CCC'de orta arc acisi)
        seg_lens = (t * radius, p * radius, q * radius)
        total = seg_lens[0] + seg_lens[1] + seg_lens[2]
        if total >= best_len:
            continue
        cand = DubinsPath(q0=q0, radius=radius,
                          segments=tuple(zip(types, seg_lens)), kind=kind)
        ex, ey, et = path_endpoint(cand)
        if (math.hypot(ex - q1[0], ey - q1[1]) > _ENDPOINT_TOL_M
                or abs(mod2pi(et - q1[2] + math.pi) - math.pi) > 1e-3):
            continue  # sayisal sinir vakasi: bu kelimeyi atla
        best = cand
        best_len = total
    return best


def shortest_path_to_point(q0: Tuple[float, float, float],
                           target: Tuple[float, float],
                           radius: float) -> Optional[DubinsPath]:
    """Varis heading'i serbest: don (C) + duz (S) ile noktaya en kisa yol.

    Hedef her iki donus cemberinin de icindeyse None (cagiran tam Dubins'e
    veya baska stratejiye duser).
    """
    if radius <= 1e-6:
        return None
    x, y, theta = q0
    px, py = target
    if math.hypot(px - x, py - y) < 1e-6:
        return DubinsPath(q0=q0, radius=radius, segments=(("S", 0.0),), kind="S")

    best: Optional[DubinsPath] = None
    best_len = math.inf
    for turn in ("L", "R"):
        if turn == "L":
            cx, cy = x - radius * math.sin(theta), y + radius * math.cos(theta)
        else:
            cx, cy = x + radius * math.sin(theta), y - radius * math.cos(theta)
        dx, dy = px - cx, py - cy
        dist_c = math.hypot(dx, dy)
        if dist_c < radius - 1e-9:
            continue  # hedef cemberin icinde, bu yonle CS yok
        gamma = math.acos(min(1.0, radius / dist_c))
        psi = math.atan2(dy, dx)
        tangent_len = math.sqrt(max(0.0, dist_c * dist_c - radius * radius))
        phi0 = math.atan2(y - cy, x - cx)
        for phi_t in (psi - gamma, psi + gamma):
            tx = cx + radius * math.cos(phi_t)
            ty = cy + radius * math.sin(phi_t)
            # teget noktasindaki hareket yonu hedefe bakmali
            head = phi_t + math.pi / 2.0 if turn == "L" else phi_t - math.pi / 2.0
            vx, vy = px - tx, py - ty
            vlen = math.hypot(vx, vy)
            if vlen < 1e-9:
                continue
            if (vx * math.cos(head) + vy * math.sin(head)) / vlen < 0.999:
                continue
            arc = mod2pi(phi_t - phi0) if turn == "L" else mod2pi(phi0 - phi_t)
            total = arc * radius + tangent_len
            if total >= best_len:
                continue
            cand = DubinsPath(q0=q0, radius=radius,
                              segments=((turn, arc * radius), ("S", tangent_len)),
                              kind=turn + "S")
            ex, ey, _ = path_endpoint(cand)
            if math.hypot(ex - px, ey - py) > _ENDPOINT_TOL_M:
                continue
            best = cand
            best_len = total
    return best

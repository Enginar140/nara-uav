"""CenteringController -- hedefi kare merkezine getiren PID guduml.

Girdi: hedefin normalize merkez ofseti (nx, ny) in [-1,1] (detection.center_
offset_norm). Cikti: GOVDE-cercevesi hiz demand'i (vx ileri, vy sag, vz asagi).
Bu demand dogrudan FlightCommander.fly_velocity_body / NavigationEngine.
fly_velocity_body'e verilebilir; ArduPlane fork'u yatay (vx,vy)'yi rotaya,
vz'yi tirmanis oranina ayristirir. Boylece sabit-kanat icin anlamli bir
'merkeze yonel' guduml elde edilir (yana kayma yok, kurs degisimi var).

Saf mantik; cv2/numpy gerektirmez.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from ...core.time_utils import now_monotonic


class PID:
    """Anti-windup'li, cikti-sinirli, turev-filtreli basit PID."""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_limit: float = 1.0, integral_limit: float = 0.5,
                 d_filter: float = 0.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = abs(out_limit)
        self.integral_limit = abs(integral_limit)
        self.d_filter = min(1.0, max(0.0, d_filter))
        self._i = 0.0
        self._prev_err = 0.0
        self._d = 0.0
        self._have_prev = False

    def reset(self) -> None:
        self._i = 0.0
        self._prev_err = 0.0
        self._d = 0.0
        self._have_prev = False

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return self._clamp(self.kp * error + self.ki * self._i)
        # integral (anti-windup clamp)
        self._i = _clip(self._i + error * dt, -self.integral_limit, self.integral_limit)
        # turev (olcum turevi yerine hata turevi; dusuk-gecirgen filtre)
        raw_d = (error - self._prev_err) / dt if self._have_prev else 0.0
        self._d = self.d_filter * self._d + (1.0 - self.d_filter) * raw_d
        self._prev_err = error
        self._have_prev = True
        out = self.kp * error + self.ki * self._i + self.kd * self._d
        return self._clamp(out)

    def _clamp(self, v: float) -> float:
        return _clip(v, -self.out_limit, self.out_limit)


def _clip(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


@dataclass
class CenteringConfig:
    # yatay (nx -> yana/kurs) ve dikey (ny -> tirmanis) PID kazanclari
    yaw_kp: float = 1.2
    yaw_ki: float = 0.05
    yaw_kd: float = 0.20
    pitch_kp: float = 0.8
    pitch_ki: float = 0.03
    pitch_kd: float = 0.15
    cruise_speed_ms: float = 20.0     # ileri hiz (vx)
    max_lateral_ms: float = 14.0      # vy tavani (kurs agresifligi)
    max_vertical_ms: float = 4.0      # vz tavani (tirmanis/dalis)
    deadband: float = 0.02            # |n| < deadband -> sifir (titremeyi onle)
    slow_on_turn: float = 0.4         # buyuk yatay hatada ileri hizi kis (0..1)


class CenteringController:
    def __init__(self, config: CenteringConfig | None = None):
        self.cfg = config or CenteringConfig()
        c = self.cfg
        self._yaw = PID(c.yaw_kp, c.yaw_ki, c.yaw_kd, out_limit=1.0)
        self._pitch = PID(c.pitch_kp, c.pitch_ki, c.pitch_kd, out_limit=1.0)
        self._last_t = 0.0

    def reset(self) -> None:
        self._yaw.reset()
        self._pitch.reset()
        self._last_t = 0.0

    def update(self, nx: float, ny: float, t: float | None = None,
               cruise_speed: float | None = None) -> Tuple[float, float, float]:
        """nx,ny in [-1,1] -> (vx, vy, vz) govde hizi (m/s)."""
        now = t if t is not None else now_monotonic()
        dt = 0.0 if self._last_t == 0.0 else max(0.0, now - self._last_t)
        self._last_t = now
        c = self.cfg
        ex = 0.0 if abs(nx) < c.deadband else nx
        ey = 0.0 if abs(ny) < c.deadband else ny
        u_yaw = self._yaw.update(ex, dt)      # [-1,1]
        u_pitch = self._pitch.update(ey, dt)  # [-1,1]
        vy = u_yaw * c.max_lateral_ms
        vz = u_pitch * c.max_vertical_ms      # ny>0 (hedef asagida) -> vz>0 (dal)
        base = cruise_speed if cruise_speed is not None else c.cruise_speed_ms
        vx = base * (1.0 - c.slow_on_turn * min(1.0, abs(u_yaw)))
        return (vx, vy, vz)

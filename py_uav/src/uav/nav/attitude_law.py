"""AttitudeDirectLaw -- GUIDED icinde "sanal FBWA" kontrol kanunu.

FBWA'nin kontrol yuzeyi (roll/pitch ACI talepleri) GUIDED'de SET_ATTITUDE_TARGET
ile birebir mevcuttur; mod GUIDED kaldigi icin sartname acisindan OTONOM sayilir
(FBWA'nin kendisi yari-oto = manuel sayilirdi, sayfa 15 / 6.1.7).

Iki parca:
  1) PD kanunu: gorus-hatti (LOS) hatalari (yaw_err, pitch_err derece) ->
     zarf-kisitli roll/pitch talepleri. Kamera merkezleme (kilitlenme) ve
     QR dalisi terminal fazi icin.
  2) Ileri-projeksiyon emniyet kontrolu (NavigationEngine cagirir): komut
     edilen yatisla olusacak yorunge ufuk boyunca HSS/sinir/irtifa-bandina
     girecekse komut GONDERILMEZ -> cagiran pozisyon gudumune duser.

Stall korumasi: hava hizi (min + marj) altina dustugunde burun-yukari talepler
kirpilir (yalniz duz/asagi pitch'e izin verilir).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from ..core import geo
from ..core.types import Position, UAVState

_G = 9.80665


@dataclass
class AttitudeLawConfig:
    enabled: bool = False
    # PD kazanclari: LOS hatasi (derece) -> aci talebi (derece)
    roll_kp: float = 1.6
    roll_kd: float = 0.35
    pitch_kp: float = 0.9
    pitch_kd: float = 0.20
    # zarf
    max_bank_deg: float = 45.0
    pitch_min_deg: float = -30.0
    pitch_max_deg: float = 18.0
    pitch_trim_deg: float = 2.0       # duz ucus icin kucuk burun-yukari trim
    # stream
    min_interval_s: float = 0.08      # ~12 Hz ust sinir
    # emniyet projeksiyonu
    horizon_s: float = 4.0
    horizon_step_s: float = 0.8
    floor_margin_m: float = 15.0      # min irtifaya ek tampon
    ceiling_margin_m: float = 10.0
    min_airspeed_margin: float = 2.0  # stall korumasi devreye girme marji


class AttitudeDirectLaw:
    """Durum tutan PD kanunu (tek hedef akisi icin)."""

    def __init__(self, config: Optional[AttitudeLawConfig] = None):
        self.cfg = config or AttitudeLawConfig()
        self._prev_yaw_err: Optional[float] = None
        self._prev_pitch_err: Optional[float] = None
        self._prev_t: Optional[float] = None

    def reset(self) -> None:
        self._prev_yaw_err = None
        self._prev_pitch_err = None
        self._prev_t = None

    def compute(self, yaw_err_deg: float, pitch_err_deg: float,
                now_s: float) -> Tuple[float, float]:
        """LOS hatalari -> (roll_deg, pitch_deg) zarf-kisitli talepler.

        yaw_err > 0: hedef sagda -> saga yatis (+roll).
        pitch_err > 0: hedef yukarida -> burun yukari (+pitch).
        """
        c = self.cfg
        d_yaw = 0.0
        d_pitch = 0.0
        if self._prev_t is not None:
            dt = max(1e-3, now_s - self._prev_t)
            if dt < 1.0:  # eski turev bayatsa kullanma
                d_yaw = (yaw_err_deg - self._prev_yaw_err) / dt
                d_pitch = (pitch_err_deg - self._prev_pitch_err) / dt
        self._prev_yaw_err = yaw_err_deg
        self._prev_pitch_err = pitch_err_deg
        self._prev_t = now_s

        roll = c.roll_kp * yaw_err_deg + c.roll_kd * d_yaw
        pitch = c.pitch_trim_deg + c.pitch_kp * pitch_err_deg + c.pitch_kd * d_pitch
        roll = max(-c.max_bank_deg, min(c.max_bank_deg, roll))
        pitch = max(c.pitch_min_deg, min(c.pitch_max_deg, pitch))
        return roll, pitch

    # ===================== emniyet yardimcilari =====================
    def clamp_for_airspeed(self, pitch_deg: float, airspeed: float,
                           min_airspeed: float) -> float:
        """Hava hizi marjin altindaysa burun-yukari talebi kirp (stall korumasi)."""
        if airspeed > 0.0 and airspeed < min_airspeed + self.cfg.min_airspeed_margin:
            return min(pitch_deg, 0.0)
        return pitch_deg

    def project_trajectory(self, own: UAVState, roll_deg: float,
                           pitch_deg: float) -> List[Position]:
        """Komut edilen yatis/pitch ile ufuk boyunca yorunge ornekleri.

        Sabit-kanat kinematigi: donus hizi = g*tan(roll)/V; dikey hiz =
        V*sin(pitch). Komutun SURDURULMESI varsayimiyla en kotu durumu
        ornekler -- emniyet kontrolu icin yeterli ve muhafazakar.
        """
        c = self.cfg
        speed = max(8.0, own.airspeed if own.airspeed > 1.0
                    else own.velocity.horizontal_speed())
        psi_dot = _G * math.tan(math.radians(roll_deg)) / speed  # rad/s (+saga)
        vz = speed * math.sin(math.radians(pitch_deg))           # +yukari
        pts: List[Position] = []
        heading = math.radians(own.heading)
        north = 0.0
        east = 0.0
        t = 0.0
        while t < c.horizon_s - 1e-9:
            step = min(c.horizon_step_s, c.horizon_s - t)
            # adim ortasi heading ile entegre (daha iyi arc yaklasimi)
            mid = heading + psi_dot * step * 0.5
            north += speed * math.cos(mid) * step
            east += speed * math.sin(mid) * step
            heading += psi_dot * step
            t += step
            p = geo.ned_to_position(geo.NED(north, east, -vz * t), own.position)
            pts.append(p)
        return pts

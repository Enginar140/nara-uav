"""TargetEstimator -- hedef merkez/boyut icin sabit-hizli Kalman tahmini.

Amac: tespitler gurultulu/aralikli geldiginde hedefin bir sonraki konumunu
yumusak ve dayanikli tahmin etmek ('iyi tahminler'). Eksen basina 2-durumlu
(konum, hiz) Kalman; olcum yoksa yalniz predict (dropout sirasinda surdurur).
numpy gerektirmez (saf Python) -> kamerasiz test edilebilir.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .detection import BBox, Detection


class _Kalman1D:
    """Sabit-hiz 2-durum Kalman: x=[pos, vel]. Skaler olcum (pos)."""

    def __init__(self, q: float = 50.0, r: float = 9.0):
        self.q = q            # surec gurultusu (ivme belirsizligi)
        self.r = r            # olcum gurultusu (px^2)
        self.pos = 0.0
        self.vel = 0.0
        self.p00 = 1e3
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 1e3
        self._init = False

    def reset(self, pos: float) -> None:
        self.pos, self.vel = pos, 0.0
        self.p00, self.p01, self.p10, self.p11 = self.r, 0.0, 0.0, self.r
        self._init = True

    def predict(self, dt: float) -> None:
        if not self._init or dt <= 0.0:
            return
        # x = F x ;  F = [[1, dt],[0,1]]
        self.pos += self.vel * dt
        # P = F P F^T + Q
        p00 = self.p00 + dt * (self.p10 + self.p01) + dt * dt * self.p11
        p01 = self.p01 + dt * self.p11
        p10 = self.p10 + dt * self.p11
        p11 = self.p11
        q = self.q
        self.p00 = p00 + q * dt * dt * dt / 3.0
        self.p01 = p01 + q * dt * dt / 2.0
        self.p10 = p10 + q * dt * dt / 2.0
        self.p11 = p11 + q * dt

    def correct(self, z: float) -> None:
        if not self._init:
            self.reset(z)
            return
        s = self.p00 + self.r
        k0 = self.p00 / s
        k1 = self.p10 / s
        y = z - self.pos
        self.pos += k0 * y
        self.vel += k1 * y
        p00 = (1 - k0) * self.p00
        p01 = (1 - k0) * self.p01
        p10 = self.p10 - k1 * self.p00
        p11 = self.p11 - k1 * self.p01
        self.p00, self.p01, self.p10, self.p11 = p00, p01, p10, p11


@dataclass
class EstimatorConfig:
    q_pos: float = 80.0      # merkez surec gurultusu
    r_pos: float = 9.0       # merkez olcum gurultusu
    q_size: float = 20.0     # boyut surec gurultusu
    r_size: float = 16.0     # boyut olcum gurultusu
    max_coast_s: float = 0.6 # olcumsuz surdurme suresi (sonra tahmin gecersiz)


class TargetEstimator:
    """cx, cy, w, h icin dort bagimsiz Kalman; predict + correct + coast."""

    def __init__(self, config: Optional[EstimatorConfig] = None):
        self.cfg = config or EstimatorConfig()
        self._kx = _Kalman1D(self.cfg.q_pos, self.cfg.r_pos)
        self._ky = _Kalman1D(self.cfg.q_pos, self.cfg.r_pos)
        self._kw = _Kalman1D(self.cfg.q_size, self.cfg.r_size)
        self._kh = _Kalman1D(self.cfg.q_size, self.cfg.r_size)
        self._last_meas_t: Optional[float] = None
        self._frame_w = 0
        self._frame_h = 0
        self._initialised = False

    @property
    def initialised(self) -> bool:
        return self._initialised

    def update(self, t: float, det: Optional[Detection]) -> Optional[Detection]:
        """Olcum varsa correct, yoksa coast. Tahmin edilen Detection dondurur
        (coast suresi asilirsa None)."""
        if det is not None:
            dt = 0.0 if self._last_meas_t is None else max(0.0, t - self._last_meas_t)
            for k, z in ((self._kx, det.box.cx), (self._ky, det.box.cy),
                         (self._kw, det.box.w), (self._kh, det.box.h)):
                k.predict(dt)
                k.correct(z)
            self._last_meas_t = t
            self._frame_w = det.frame_w
            self._frame_h = det.frame_h
            self._initialised = True
            return self._as_detection(t, det.confidence, det.track_id, det.label)

        # olcum yok -> coast
        if not self._initialised or self._last_meas_t is None:
            return None
        coast = t - self._last_meas_t
        if coast > self.cfg.max_coast_s:
            return None
        for k in (self._kx, self._ky, self._kw, self._kh):
            k.predict(coast)
        # predict sonrasi last_meas_t'yi guncellemiyoruz; coast referansi olarak tut
        return self._as_detection(t, confidence=0.0, track_id=-1, label="coast")

    def predict_at(self, t: float) -> Optional[Detection]:
        """Olcumu degistirmeden t aninda ileri tahmin (look-ahead)."""
        if not self._initialised or self._last_meas_t is None:
            return None
        dt = max(0.0, t - self._last_meas_t)
        cx = self._kx.pos + self._kx.vel * dt
        cy = self._ky.pos + self._ky.vel * dt
        w = max(1.0, self._kw.pos + self._kw.vel * dt)
        h = max(1.0, self._kh.pos + self._kh.vel * dt)
        return Detection(box=BBox(cx, cy, w, h), frame_w=self._frame_w,
                         frame_h=self._frame_h, confidence=0.0, label="predict")

    def _as_detection(self, t: float, confidence: float, track_id: int,
                      label: str) -> Detection:
        w = max(1.0, self._kw.pos)
        h = max(1.0, self._kh.pos)
        return Detection(box=BBox(self._kx.pos, self._ky.pos, w, h),
                         frame_w=self._frame_w, frame_h=self._frame_h,
                         confidence=confidence, track_id=track_id, label=label)

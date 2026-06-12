"""Perception veri tipleri. C++ perception_types.hpp portu."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


class PerceptionFrame(Enum):
    """Tespit sonucunun ofseti hangi cercevede ifade ettigi.

    NavigationEngine bu degere gore farkli local-control wrapper'ina yonlendirir.
    """
    BODY = auto()       # a=forward_m, b=right_m, c=down_m (AHRS yaw'a gore)
    LOCAL_NED = auto()  # a=north_m,   b=east_m,  c=down_m (mevcut konuma gore)
    POLAR = auto()      # a=bearing_deg, b=distance_m, c=altitude_offset_m (+=tirman)
    VISION_LOS = auto() # a=yaw_offset_deg, b=pitch_offset_deg, c=range_m (kamera LOS)
    VELOCITY_NED = auto()   # a=vN, b=vE, c=vD (m/s, yer cercevesi) -- Katman 2
    VELOCITY_BODY = auto()  # a=vx ileri, b=vy sag, c=vz asagi (m/s) -- Katman 2


def frame_to_str(frame: PerceptionFrame) -> str:
    return frame.name


@dataclass
class PerceptionTarget:
    frame: PerceptionFrame = PerceptionFrame.BODY
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    yaw_deg: Optional[float] = None   # opsiyonel komut yaw (deg, true)
    confidence: float = 0.0           # 0..1
    stamp_us: int = 0
    source_tag: str = ""              # ornek "vision_tracker", "qr_simple"
    label: str = ""                   # cozulmus QR metni vb.


@dataclass
class PerceptionEvent:
    has_lock: bool = False
    target: Optional[PerceptionTarget] = None

"""Hiz yonetimi (C++ SpeedManager portu)."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SpeedConfig:
    cruise: float = 20.0
    max_speed: float = 35.0
    min_speed: float = 12.0
    patrol: float = 18.0
    pursuit: float = 30.0
    dive: float = 25.0


class SpeedManager:
    def __init__(self, config: SpeedConfig | None = None):
        self.cfg = config or SpeedConfig()

    def clamp(self, speed: float) -> float:
        return min(max(speed, self.cfg.min_speed), self.cfg.max_speed)

    def cruise_speed(self) -> float:
        return self.clamp(self.cfg.cruise)

    def patrol_speed(self) -> float:
        return self.clamp(self.cfg.patrol)

    def pursuit_speed(self) -> float:
        return self.clamp(self.cfg.pursuit)

    def dive_speed(self) -> float:
        return self.clamp(self.cfg.dive)

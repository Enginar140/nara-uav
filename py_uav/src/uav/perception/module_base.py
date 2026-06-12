"""IPerceptionModule arayuzu. C++ iperception_module.hpp portu."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from ..core.types import UAVState
from .perception_types import PerceptionTarget


class IPerceptionModule(ABC):
    @abstractmethod
    def id(self) -> str:
        """config (yaml) icin kararli kimlik, snake_case."""

    @abstractmethod
    def start(self) -> None:
        """Engine modulu aktiflestirince bir kez (kamera ac, model yukle)."""

    @abstractmethod
    def stop(self) -> None:
        """Engine modulu pasiflestirince."""

    @abstractmethod
    def poll(self, own_state: UAVState) -> Optional[PerceptionTarget]:
        """Bir tespit dongusu; bloklamaz, hizli doner."""

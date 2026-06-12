"""IMissionMode arayuzu."""
from __future__ import annotations

from abc import ABC, abstractmethod

from ..core.types import MissionGoal, UAVState


class IMissionMode(ABC):
    @abstractmethod
    def on_enter(self) -> None: ...

    @abstractmethod
    def on_exit(self) -> None: ...

    @abstractmethod
    def tick(self, own_state: UAVState) -> None: ...

    @abstractmethod
    def get_mission_goal(self) -> MissionGoal: ...

    @abstractmethod
    def name(self) -> str: ...

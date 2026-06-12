"""ArduPilotPassthrough -- operator/ArduPilot kontrolu (no-op tick)."""
from __future__ import annotations

from ..comm.flight_commander import FlightCommander
from ..core.types import GoalType, MissionGoal, UAVState
from ..infra.log import get_logger
from .mode_base import IMissionMode

_log = get_logger("ardupilot_mode")


class ArduPilotPassthrough(IMissionMode):
    def __init__(self, commander: FlightCommander):
        self._cmd = commander

    def on_enter(self) -> None:
        _log.info("ARDUPILOT passthrough: kontrol operatorde")

    def on_exit(self) -> None:
        pass

    def tick(self, own_state: UAVState) -> None:
        pass  # ArduPilot/operator uçuruyor

    def get_mission_goal(self) -> MissionGoal:
        return MissionGoal(type=GoalType.NONE)

    def name(self) -> str:
        return "ARDUPILOT"

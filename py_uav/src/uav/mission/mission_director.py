"""MissionDirector -- gorev modu durum makinesi. C++ MissionDirector portu.

Gecis kurallari:
  ARDUPILOT -> her sey (operator override)
  her sey   -> ARDUPILOT (guvenlik fallback)
  QR_DIVE   -> yalniz PATROL (dalis temiz cikis)
  FOLLOW    -> PATROL veya QR_DIVE
  PATROL    -> her sey

GUIDED gecisi non-blocking: GUIDED komutu gonderilir, onaylanana kadar (heartbeat)
kisa bir grace periyodu taninir; bu sure icinde "GUIDED degil" durumu operator
override sayilmaz (asyncio loop'u bloklamadan C++'taki bekleme-onayini taklit eder).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from ..comm.flight_commander import ArduPilotMode, FlightCommander
from ..comm.statustext_parser import ParsedModeCommand
from ..core.time_utils import elapsed_s, now_monotonic
from ..core.types import MissionMode, Position
from ..infra.event_bus import EventBus, EventType
from ..infra.log import get_logger
from ..nav.navigation_engine import NavigationEngine
from ..world_model.world_model import WorldModel
from .ardupilot_mode import ArduPilotPassthrough
from .follow_mode import FollowConfig, FollowMode
from .patrol_mode import PatrolConfig, PatrolMode
from .qr_dive_mode import QRDiveConfig, QRDiveMode

_log = get_logger("mission_director")


class PostDiveAction(Enum):
    RTL = "rtl"
    PATROL = "patrol"
    LOITER = "loiter"


def parse_post_dive_action(s: str, fallback: PostDiveAction = PostDiveAction.RTL) -> PostDiveAction:
    try:
        return PostDiveAction(s.strip().lower())
    except ValueError:
        return fallback


@dataclass
class MissionConfig:
    patrol: PatrolConfig = field(default_factory=PatrolConfig)
    follow: FollowConfig = field(default_factory=FollowConfig)
    qr_dive: QRDiveConfig = field(default_factory=QRDiveConfig)
    qr_dive_on_complete: PostDiveAction = PostDiveAction.RTL
    guided_confirm_grace_s: float = 2.0


class MissionDirector:
    def __init__(self, nav: NavigationEngine, commander: FlightCommander,
                 world_model: WorldModel, event_bus: EventBus,
                 config: MissionConfig | None = None, perception=None):
        self._nav = nav
        self._cmd = commander
        self._wm = world_model
        self._bus = event_bus
        self._perception = perception
        self.cfg = config or MissionConfig()
        self._current = MissionMode.ARDUPILOT
        self._transitions = 0
        self._guided_request_t = 0.0
        self._ardupilot: Optional[ArduPilotPassthrough] = None
        self._patrol: Optional[PatrolMode] = None
        self._follow: Optional[FollowMode] = None
        self._qr_dive: Optional[QRDiveMode] = None

    def initialize(self) -> None:
        self._ardupilot = ArduPilotPassthrough(self._cmd)
        self._patrol = PatrolMode(self._nav, self._wm, self.cfg.patrol)
        self._follow = FollowMode(self._nav, self._wm, self.cfg.follow)
        self._qr_dive = QRDiveMode(self._nav, self._wm, self._cmd, self.cfg.qr_dive)
        if self._perception is not None:
            self._follow.set_perception_engine(self._perception, self._bus)
            self._qr_dive.set_perception_engine(self._perception, self._bus)
        self._qr_dive.set_completion_callback(self._on_qr_dive_complete)
        self._follow.set_target_lost_callback(
            lambda: self.request_mode(MissionMode.PATROL))
        self._ardupilot.on_enter()

    # ===================== mod istekleri =====================
    def request_mode(self, mode: MissionMode) -> bool:
        if mode == self._current:
            return True
        if not self._validate_transition(self._current, mode):
            _log.warning("Gecersiz gecis %s -> %s", self._current.value, mode.value)
            return False
        return self._execute_transition(self._current, mode)

    def force_mode(self, mode: MissionMode) -> None:
        if mode == self._current:
            return
        _log.info("Operator force %s -> %s", self._current.value, mode.value)
        self._execute_transition(self._current, mode)

    def get_current_mode(self) -> MissionMode:
        return self._current

    def set_follow_target(self, target_id: int) -> None:
        self._follow.set_target_id(target_id)

    def set_dive_target(self, target: Position) -> None:
        self._qr_dive.set_dive_target(target)

    # ===================== tick =====================
    def tick(self) -> None:
        own = self._wm.get_own_state()
        if not own.is_valid():
            return
        # Operator GUIDED'den cikardiysa ve grace doluysa -> ARDUPILOT'a devret
        if self._current != MissionMode.ARDUPILOT and not self._cmd.is_in_guided_mode():
            if elapsed_s(self._guided_request_t) > self.cfg.guided_confirm_grace_s:
                _log.warning("AP GUIDED'den cikti (mod=%s); kontrol operatore (ARDUPILOT)",
                             self._cmd.get_current_mode().name)
                self.force_mode(MissionMode.ARDUPILOT)
                return
            # grace icinde: GUIDED onayini bekle, handler'i calistirma
            return
        handler = self._handler_for(self._current)
        if handler:
            handler.tick(own)

    # ===================== GCS komutu =====================
    def handle_mode_command(self, cmd: ParsedModeCommand) -> None:
        _log.info("Mod komutu alindi -> %s", cmd.target_mode.value)
        if cmd.target_id is not None:
            self.set_follow_target(cmd.target_id)
        if cmd.target_pos is not None:
            self.set_dive_target(cmd.target_pos)
        self.force_mode(cmd.target_mode)

    # ===================== RTL =====================
    def trigger_rtl(self, reason: str) -> None:
        _log.warning("RTL tetiklendi - %s", reason)
        handler = self._handler_for(self._current)
        if handler:
            handler.on_exit()
        self._current = MissionMode.ARDUPILOT
        self._transitions += 1
        self._ardupilot.on_enter()
        self._cmd.set_mode(ArduPilotMode.RTL)
        self._bus.publish(EventType.MODE_CHANGED)

    # ===================== ic =====================
    def _on_qr_dive_complete(self) -> None:
        action = self.cfg.qr_dive_on_complete
        _log.info("QR_DIVE tamamlandi, on_complete=%s", action.value)
        # Sonraki mod devralmadan once stale GUIDED durumunu birak
        self._nav.release_guided_hold()
        if action == PostDiveAction.PATROL:
            self.request_mode(MissionMode.PATROL)
        elif action == PostDiveAction.LOITER:
            self.force_mode(MissionMode.ARDUPILOT)
            self._cmd.set_mode(ArduPilotMode.LOITER)
        else:
            self.trigger_rtl("qr_dive_complete")

    def _handler_for(self, mode: MissionMode):
        return {
            MissionMode.ARDUPILOT: self._ardupilot,
            MissionMode.PATROL: self._patrol,
            MissionMode.FOLLOW: self._follow,
            MissionMode.QR_DIVE: self._qr_dive,
        }.get(mode, self._ardupilot)

    def _validate_transition(self, frm: MissionMode, to: MissionMode) -> bool:
        if frm == MissionMode.ARDUPILOT:
            return True
        if to == MissionMode.ARDUPILOT:
            return True
        if frm == MissionMode.QR_DIVE:
            return to == MissionMode.PATROL
        if frm == MissionMode.FOLLOW:
            return to in (MissionMode.PATROL, MissionMode.QR_DIVE)
        return True

    def _execute_transition(self, frm: MissionMode, to: MissionMode) -> bool:
        _log.info("Gecis %s -> %s", frm.value, to.value)
        old = self._handler_for(self._current)
        if old:
            old.on_exit()
        # GUIDED gerekiyorsa komutu gonder (non-blocking; grace ile onay beklenir)
        if to != MissionMode.ARDUPILOT and not self._cmd.is_in_guided_mode():
            if not self._cmd.set_mode(ArduPilotMode.GUIDED):
                _log.warning("GUIDED komutu gonderilemedi; ARDUPILOT'ta kaliniyor")
                self._current = MissionMode.ARDUPILOT
                self._transitions += 1
                self._ardupilot.on_enter()
                self._bus.publish(EventType.MODE_CHANGED, MissionMode.ARDUPILOT)
                return False
            self._guided_request_t = now_monotonic()
        # Onceki modun stale GUIDED durumunu (yaw/waypoint) temizle
        if frm != to and to != MissionMode.ARDUPILOT:
            self._nav.release_guided_hold()
        self._current = to
        self._transitions += 1
        new = self._handler_for(to)
        if new:
            new.on_enter()
        self._bus.publish(EventType.MODE_CHANGED, to)
        return True

    # ===================== istatistik =====================
    def get_stats(self) -> dict:
        phase = ""
        if self._current == MissionMode.FOLLOW and self._follow:
            phase = self._follow.get_phase().name
        elif self._current == MissionMode.QR_DIVE and self._qr_dive:
            phase = self._qr_dive.get_phase().name
        return {
            "current_mode": self._current.value,
            "mode_transitions": self._transitions,
            "active_phase": phase,
        }

"""STATUSTEXT mesaj cozumleyici (GCS -> companion).

Formatlar (iki nokta ayrac):
  RIV:ID:LAT:LON:ALT:HDG:SPD:TS              -> rakip durumu
  HSS:COUNT:ID:LAT:LON:RAD:...               -> HSS bolgeleri
  BND:COUNT:LAT:LON:LAT:LON:...              -> sinir poligonu
  FREE | ARDUPILOT | LOCK:ID | KAMIKAZE:LAT:LON   -> mod komutu
Cok parcali: PREFIX[PART/TOTAL]:icerik       -> yeniden birlestirilir

Mod komutu -> gorev modu eslemesi:
  FREE      -> PATROL
  ARDUPILOT -> ARDUPILOT
  LOCK:id   -> FOLLOW (target_id)
  KAMIKAZE  -> QR_DIVE (target_pos)
"""
from __future__ import annotations

import math
import re
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional

from ..core.types import Boundary, HSSZone, MissionMode, Position, UAVState
from ..infra.log import get_logger

_log = get_logger("statustext_parser")

_MULTIPART = re.compile(r"^(HSS|BND|RIV)\[(\d+)/(\d+)\]:(.*)$")


class StatusTextType(Enum):
    RIV = auto()
    HSS = auto()
    BND = auto()
    MODE_CMD = auto()


@dataclass
class ParsedModeCommand:
    target_mode: MissionMode
    target_id: Optional[int] = None
    target_pos: Optional[Position] = None


@dataclass
class ParsedStatusText:
    type: StatusTextType
    opponent: Optional[UAVState] = None
    hss_zones: List[HSSZone] = field(default_factory=list)
    boundary: Optional[Boundary] = None
    mode_command: Optional[ParsedModeCommand] = None


@dataclass
class _BufferEntry:
    parts: List[str]
    total: int
    received: int
    ts: float


class StatusTextParser:
    def __init__(self):
        self._buffer: Dict[str, _BufferEntry] = {}
        self._lock = threading.Lock()

    def parse(self, text: str) -> Optional[ParsedStatusText]:
        if not text:
            return None

        mp = _MULTIPART.match(text)
        if mp:
            combined = self._accumulate(mp)
            return self.parse(combined) if combined is not None else None

        if text.startswith("RIV:"):
            return self._parse_riv(text)
        if text.startswith("HSS:"):
            return self._parse_hss(text)
        if text.startswith("BND:"):
            return self._parse_bnd(text)
        if text.startswith("LOCK:") or text.startswith("KAMIKAZE:"):
            return self._parse_mode(text)
        if text in ("FREE", "ARDUPILOT"):
            return self._parse_mode(text)
        return None

    # -- cok parcali birlestirme --
    def _accumulate(self, mp: re.Match) -> Optional[str]:
        prefix, part, total, content = mp.group(1), int(mp.group(2)), int(mp.group(3)), mp.group(4)
        with self._lock:
            self._cleanup_stale()
            key = f"{prefix}_{total}"
            entry = self._buffer.get(key)
            if entry is None:
                entry = _BufferEntry(parts=[""] * total, total=total, received=0, ts=time.monotonic())
                self._buffer[key] = entry
            idx = part - 1
            if 0 <= idx < total and not entry.parts[idx]:
                entry.parts[idx] = content
                entry.received += 1
            entry.ts = time.monotonic()
            if entry.received >= entry.total:
                del self._buffer[key]
                return "".join(entry.parts)
        return None

    def _cleanup_stale(self) -> None:
        now = time.monotonic()
        for key in [k for k, e in self._buffer.items() if now - e.ts > 10.0]:
            del self._buffer[key]

    def clear_buffer(self) -> None:
        with self._lock:
            self._buffer.clear()

    # -- RIV --
    def _parse_riv(self, text: str) -> Optional[ParsedStatusText]:
        parts = text.split(":")
        if len(parts) < 8:
            _log.warning("Gecersiz RIV (8 alan gerekli): %s", text)
            return None
        try:
            opp = UAVState()
            opp.id = int(parts[1])
            opp.position = Position(float(parts[2]), float(parts[3]), float(parts[4]))
            heading = float(parts[5])
            speed = float(parts[6])
            rad = math.radians(heading)
            opp.velocity.north = speed * math.cos(rad)
            opp.velocity.east = speed * math.sin(rad)
            opp.heading = heading
            opp.groundspeed = speed
            opp.airspeed = speed
            opp.timestamp_us = time.time_ns() // 1000
            return ParsedStatusText(type=StatusTextType.RIV, opponent=opp)
        except (ValueError, IndexError):
            _log.exception("RIV cozulemedi: %s", text)
            return None

    # -- HSS --
    def _parse_hss(self, text: str) -> Optional[ParsedStatusText]:
        parts = text.split(":")
        if len(parts) < 2:
            return None
        try:
            count = int(parts[1])
            zones: List[HSSZone] = []
            idx = 2
            for i in range(count):
                if idx + 4 > len(parts):
                    _log.warning("HSS: zone %d'de kesildi", i)
                    break
                z = HSSZone(
                    id=int(parts[idx]),
                    center=Position(float(parts[idx + 1]), float(parts[idx + 2]), 0.0),
                    radius=float(parts[idx + 3]),
                    active=True,
                )
                zones.append(z)
                idx += 4
            return ParsedStatusText(type=StatusTextType.HSS, hss_zones=zones)
        except (ValueError, IndexError):
            _log.exception("HSS cozulemedi: %s", text)
            return None

    # -- BND --
    def _parse_bnd(self, text: str) -> Optional[ParsedStatusText]:
        parts = text.split(":")
        if len(parts) < 8:
            _log.warning("Gecersiz BND (>=3 vertex gerekli): %s", text)
            return None
        try:
            count = int(parts[1])
            verts: List[Position] = []
            idx = 2
            for i in range(count):
                if idx + 2 > len(parts):
                    break
                verts.append(Position(float(parts[idx]), float(parts[idx + 1]), 0.0))
                idx += 2
            if len(verts) < 3:
                return None
            return ParsedStatusText(type=StatusTextType.BND, boundary=Boundary(vertices=verts))
        except (ValueError, IndexError):
            _log.exception("BND cozulemedi: %s", text)
            return None

    # -- mod komutu --
    def _parse_mode(self, text: str) -> Optional[ParsedStatusText]:
        try:
            if text == "FREE":
                cmd = ParsedModeCommand(target_mode=MissionMode.PATROL)
            elif text == "ARDUPILOT":
                cmd = ParsedModeCommand(target_mode=MissionMode.ARDUPILOT)
            elif text.startswith("LOCK:"):
                parts = text.split(":")
                if len(parts) < 2:
                    return None
                cmd = ParsedModeCommand(target_mode=MissionMode.FOLLOW, target_id=int(parts[1]))
            elif text.startswith("KAMIKAZE:"):
                parts = text.split(":")
                if len(parts) < 3:
                    return None
                cmd = ParsedModeCommand(
                    target_mode=MissionMode.QR_DIVE,
                    target_pos=Position(float(parts[1]), float(parts[2]), 0.0),
                )
            else:
                return None
            return ParsedStatusText(type=StatusTextType.MODE_CMD, mode_command=cmd)
        except (ValueError, IndexError):
            _log.exception("Mod komutu cozulemedi: %s", text)
            return None

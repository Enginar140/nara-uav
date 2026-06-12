"""TrackingController -- basit centroid takip (SORT benzeri). C++ portu.

Goruntu-cercevesi tespitlerini kalici parcalara (track) baglar; sabit hiz
ile bir adim ileri tahmin, en yakin eslesme, yas-tabanli silme.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class Detection:
    x: float
    y: float
    confidence: float = 0.0


@dataclass
class TrackedObject:
    id: int = 0
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    confidence: float = 0.0
    consecutive_frames: int = 0
    frames_since_seen: int = 0


@dataclass
class TrackingConfig:
    max_age_frames: int = 30
    min_hits: int = 3
    dist_threshold: float = 100.0


class TrackingController:
    def __init__(self, config: Optional[TrackingConfig] = None):
        self.cfg = config or TrackingConfig()
        self._tracks: Dict[int, TrackedObject] = {}
        self._next_id = 1

    def update(self, detections: List[Detection]) -> None:
        # tahmin + yaslandirma
        for t in self._tracks.values():
            t.frames_since_seen += 1
            t.x += t.vx
            t.y += t.vy

        matched = [False] * len(detections)
        for t in self._tracks.values():
            best_d = self.cfg.dist_threshold
            best_i = -1
            for i, det in enumerate(detections):
                if matched[i]:
                    continue
                d = math.hypot(t.x - det.x, t.y - det.y)
                if d < best_d:
                    best_d = d
                    best_i = i
            if best_i >= 0:
                det = detections[best_i]
                t.vx = det.x - t.x
                t.vy = det.y - t.y
                t.x, t.y = det.x, det.y
                t.confidence = det.confidence
                t.consecutive_frames += 1
                t.frames_since_seen = 0
                matched[best_i] = True

        for i, det in enumerate(detections):
            if matched[i]:
                continue
            t = TrackedObject(id=self._next_id, x=det.x, y=det.y,
                              confidence=det.confidence, consecutive_frames=1)
            self._next_id += 1
            self._tracks[t.id] = t

        for tid in [tid for tid, t in self._tracks.items()
                    if t.frames_since_seen > self.cfg.max_age_frames]:
            del self._tracks[tid]

    def get_active_tracks(self) -> List[TrackedObject]:
        return [t for t in self._tracks.values()
                if t.consecutive_frames >= self.cfg.min_hits]

    def get_track(self, tid: int) -> Optional[TrackedObject]:
        return self._tracks.get(tid)

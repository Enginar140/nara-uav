"""Config -> CombatVisionTracker fabrikasi + downlink baglama.

Mimari: agir bagimliliklar (YOLO/cv2) burada KURULMAZ; tracker.start() icinde
lazy kurulur. Boylece config okuma/registrasyon her ortamda calisir.
"""
from __future__ import annotations

from typing import Callable, Optional

from ...infra.event_bus import EventBus, EventType
from ...infra.log import get_logger
from ..opencv_vision_tracker import CameraConfig
from .centering_pid import CenteringConfig
from .combat_tracker import CombatTrackerConfig, CombatVisionTracker
from .lock_evaluator import LockEvaluatorConfig

_log = get_logger("combat_factory")


def build_combat_tracker(cfg, *, server_ms_fn: Optional[Callable[[], int]] = None,
                         autonomous_fn: Optional[Callable[[], bool]] = None
                         ) -> CombatVisionTracker:
    """ConfigManager'dan 'perception.combat.*' okuyup tracker uretir.

    Dedektor/ROI/QR/recorder lazy (start()'ta) kurulur; burada None birakilir.
    """
    g = cfg.get_node("perception.combat") or {}
    cam = g.get("camera", {}) or {}
    pid = g.get("centering", {}) or {}
    lock = g.get("lock", {}) or {}

    tcfg = CombatTrackerConfig(
        id=str(g.get("id", "combat_vision")),
        mode=str(g.get("mode", "lock")),
        av_w_frac=float(g.get("av_w_frac", 0.40)),
        av_h_frac=float(g.get("av_h_frac", 0.40)),
        lock_min_frac=float(g.get("lock_min_frac", 0.06)),
        detect_every_n=int(g.get("detect_every_n", 6)),
        record_video=bool(g.get("record_video", False)),
        camera=CameraConfig(
            type=str(cam.get("type", "device")),
            device_index=int(cam.get("device_index", 0)),
            url=str(cam.get("url", "")),
            file_path=str(cam.get("file_path", "")),
            width=int(cam.get("width", 640)),
            height=int(cam.get("height", 480)),
            fps=float(cam.get("fps", 30.0)),
            loop_file=bool(cam.get("loop_file", True)),
        ),
        centering=CenteringConfig(
            yaw_kp=float(pid.get("yaw_kp", 1.2)),
            yaw_ki=float(pid.get("yaw_ki", 0.05)),
            yaw_kd=float(pid.get("yaw_kd", 0.20)),
            pitch_kp=float(pid.get("pitch_kp", 0.8)),
            pitch_ki=float(pid.get("pitch_ki", 0.03)),
            pitch_kd=float(pid.get("pitch_kd", 0.15)),
            cruise_speed_ms=float(pid.get("cruise_speed_ms", 20.0)),
            max_lateral_ms=float(pid.get("max_lateral_ms", 14.0)),
            max_vertical_ms=float(pid.get("max_vertical_ms", 4.0)),
        ),
        lock=LockEvaluatorConfig(
            lock_min_frac=float(g.get("lock_min_frac", 0.06)),
            min_lock_s=float(lock.get("min_lock_s", 4.0)),
            max_gap_s=float(lock.get("max_gap_s", 0.5)),
            invalid_tolerance_s=float(lock.get("invalid_tolerance_s", 0.8)),
            require_autonomous=bool(lock.get("require_autonomous", True)),
            prevent_same_target_twice=bool(lock.get("prevent_same_target_twice", True)),
            require_airborne=bool(lock.get("require_airborne", True)),
        ),
    )

    recorder = None
    if tcfg.record_video:
        try:
            from .video_recorder import LockVideoRecorder, VideoConfig
            vid = g.get("video", {}) or {}
            recorder = LockVideoRecorder(VideoConfig(
                path=str(vid.get("path", "lock_record.mp4")),
                fps=float(vid.get("fps", 15.0)),
                fourcc=str(vid.get("fourcc", "mp4v")),
            ))
        except Exception:  # noqa: BLE001
            _log.exception("video recorder kurulamadi")

    return CombatVisionTracker(tcfg, recorder=recorder, server_ms_fn=server_ms_fn,
                               autonomous_fn=autonomous_fn)


def attach_downlink(tracker: CombatVisionTracker, encoder,
                    bus: Optional[EventBus] = None) -> None:
    """Tracker kilit/QR callback'lerini downlink encoder'a (ve EventBus'a) bagla."""

    def _on_lock(ev) -> None:
        from .lock_evaluator import LockEventKind
        active = ev.kind == LockEventKind.ACHIEVED
        # kilit paketi (KLT) + kilit dortgeni konumu (KBX)
        encoder.send_lock(active, lock_end_ms=encoder.server_ms())
        if ev.frame_w and ev.frame_h:
            encoder.send_lock_box(ev.box_cx, ev.box_cy, ev.box_w, ev.box_h,
                                  ev.frame_w, ev.frame_h)
        if bus is not None:
            bus.publish(EventType.LOCK_EVENT, ev)

    def _on_qr(payload: str, box) -> None:
        encoder.send_qr(payload)
        if bus is not None:
            bus.publish(EventType.QR_DECODED, payload)

    tracker.on_lock_event = _on_lock
    tracker.on_qr = _on_qr

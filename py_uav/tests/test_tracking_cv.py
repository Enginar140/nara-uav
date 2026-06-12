"""cv2/yolo-bagimli tracking modulleri: ROI tracker, pro QR, YOLO availability.

Bagimlilik yoksa testler atlanir (skip). Ag/indirme gerektiren YOLO cikarimi
calistirilmaz; yalniz arayuz dogrulanir.
"""
import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from uav.perception.tracking.detection import BBox, FrameGeometry  # noqa: E402
from uav.perception.tracking.roi_tracker import (RoiConfig, RoiTracker,  # noqa: E402
                                                 roi_available)


def _frame_with_rect(x, y, w=80, h=60, size=(480, 640)):
    img = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    img[:] = (30, 30, 30)
    cv2.rectangle(img, (x, y), (x + w, y + h), (200, 220, 240), -1)
    cv2.rectangle(img, (x + 10, y + 10), (x + w - 10, y + h - 10), (10, 10, 10), 3)
    return img


def test_roi_tracker_follows_moving_box():
    if not roi_available():
        pytest.skip("cv2 yok")
    trk = RoiTracker(RoiConfig(tracker_type="CSRT", max_age=100))
    x0, y0, w, h = 100, 100, 80, 60
    f0 = _frame_with_rect(x0, y0, w, h)
    assert trk.init(f0, BBox.from_xywh(x0, y0, w, h))
    assert trk.active
    last = None
    for dx in range(0, 120, 20):
        f = _frame_with_rect(x0 + dx, y0, w, h)
        box = trk.update(f)
        if box is not None:
            last = box
    assert last is not None
    # kutu hareketi takip etti (saga kaydi)
    assert last.cx > x0 + 40


def test_qr_pro_roundtrip_if_encoder_available():
    enc_create = getattr(cv2, "QRCodeEncoder_create", None)
    if enc_create is None:
        pytest.skip("cv2.QRCodeEncoder yok (contrib gerekli)")
    from uav.perception.tracking.qr_pro import QrProDetector
    enc = enc_create()
    qr = enc.encode("NARA-TEST-42")
    qr = cv2.resize(qr, (300, 300), interpolation=cv2.INTER_NEAREST)
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    canvas[90:390, 170:470] = qr
    bgr = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    det = QrProDetector()
    res = det.detect(bgr)
    assert res is not None and res.payload == "NARA-TEST-42"
    # ikinci karede ROI yolu da calismali
    res2 = det.detect(bgr)
    assert res2 is not None and res2.payload == "NARA-TEST-42"


def test_yolo_module_importable():
    from uav.perception.tracking.yolo_detector import yolo_available
    # ultralytics kurulu olabilir/olmayabilir; arayuz bayragi bool donmeli
    assert isinstance(yolo_available(), bool)


# ---------------- orkestrator (process_frame) ----------------
from uav.perception.tracking.combat_tracker import (CombatTrackerConfig,  # noqa: E402
                                                    CombatVisionTracker)
from uav.perception.tracking.detection import Detection  # noqa: E402
from uav.perception.tracking.lock_evaluator import LockEventKind  # noqa: E402
from uav.perception.perception_types import PerceptionFrame  # noqa: E402


class _FakeDet:
    """Hep merkezde, kilit-uygun bir tespit dondurur."""
    def __init__(self, w=640, h=480):
        self.w, self.h = w, h

    def best(self, frame):
        return Detection(box=BBox(self.w * 0.5, self.h * 0.5, 60, 40),
                         frame_w=self.w, frame_h=self.h, confidence=0.9, track_id=-1)


def test_combat_lock_and_guidance_via_process_frame():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    trk = CombatVisionTracker(CombatTrackerConfig(mode="lock"),
                              detector=_FakeDet(), roi=None)
    t = 0.0
    achieved = None
    last_guidance = None
    for _ in range(60):  # ~6 s @ 0.1
        res = trk.process_frame(frame, t, server_ms=int(t * 1000),
                                is_autonomous=True)
        if res.lock_event and res.lock_event.kind == LockEventKind.ACHIEVED:
            achieved = res.lock_event
        if res.guidance is not None:
            last_guidance = res.guidance
        t += 0.1
    assert achieved is not None
    assert last_guidance is not None
    assert last_guidance.frame == PerceptionFrame.VELOCITY_BODY


def test_combat_not_autonomous_no_lock():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    trk = CombatVisionTracker(CombatTrackerConfig(mode="lock"),
                              detector=_FakeDet(), roi=None)
    t = 0.0
    for _ in range(60):
        res = trk.process_frame(frame, t, int(t * 1000), is_autonomous=False)
        assert res.lock_event is None
        t += 0.1


class _FakeQr:
    def __init__(self):
        from uav.perception.tracking.qr_pro import QrResult
        self._R = QrResult

    def detect(self, frame):
        return self._R(payload="QR-NARA-7", box=BBox(320, 240, 80, 80), stamp_us=1)


def test_combat_qr_mode_emits_payload():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    trk = CombatVisionTracker(CombatTrackerConfig(mode="qr"), qr_detector=_FakeQr())
    res = trk.process_frame(frame, 0.0, 0, is_autonomous=True)
    assert res.qr_payload == "QR-NARA-7"
    assert res.guidance is not None and res.guidance.frame == PerceptionFrame.VELOCITY_BODY


# ---------------- video recorder ----------------
def test_video_recorder_writes_file(tmp_path):
    from uav.perception.tracking.video_recorder import (LockVideoRecorder,
                                                        VideoConfig, video_available)
    if not video_available():
        pytest.skip("cv2 yok")
    out = tmp_path / "rec.mp4"
    rec = LockVideoRecorder(VideoConfig(path=str(out), fps=15.0))
    assert rec.open(640, 480)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    av = FrameGeometry(640, 480).av_box()
    for i in range(10):
        rec.write(frame, server_ms=1_718_000_000_000 + i,
                  av_box=av, lock_box=BBox(320, 240, 60, 40), locked=True)
    rec.close()
    assert rec.frames_written == 10
    assert out.exists() and out.stat().st_size > 0

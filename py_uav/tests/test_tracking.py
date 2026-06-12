"""tracking cekirdegi: detection geometri, lock durum makinesi, Kalman, PID,
downlink encoder. Hepsi saf mantik (cv2/yolo gerektirmez)."""
from uav.comm.downlink import (STATUSTEXT_MAX, DownlinkEncoder, decode_downlink)
from uav.perception.tracking.centering_pid import CenteringController, PID
from uav.perception.tracking.detection import (BBox, Detection, FrameGeometry,
                                               frame_lock_valid, lock_box_valid)
from uav.perception.tracking.lock_evaluator import (LockEventKind,
                                                    LockEvaluatorConfig,
                                                    LockOnEvaluator, LockState)
from uav.perception.tracking.target_estimator import TargetEstimator

W, H = 640, 480
GEOM = FrameGeometry(W, H, av_w_frac=0.40, av_h_frac=0.40)


def _det(cx=320.0, cy=240.0, w=60.0, h=40.0, tid=1, conf=0.9):
    return Detection(box=BBox(cx, cy, w, h), frame_w=W, frame_h=H,
                     confidence=conf, track_id=tid)


# ---------------- detection / geometri ----------------
def test_bbox_geometry():
    b = BBox.from_xywh(10, 20, 100, 50)
    assert b.center == (60.0, 45.0)
    assert b.area == 5000.0
    assert b.to_ltrb() == (10.0, 20.0, 110.0, 70.0)


def test_axis_fraction_and_av():
    d = _det(w=64, h=24)  # yatay 64/640=0.10, dikey 24/480=0.05 -> max 0.10
    assert abs(d.axis_fraction() - 0.10) < 1e-6
    assert GEOM.center_in_av(320, 240)
    assert not GEOM.center_in_av(40, 240)   # AV disinda


def test_lock_box_valid_rules():
    target = BBox(320, 240, 60, 40)
    assert lock_box_valid(target, target)                 # tam ortusme
    off = BBox(320 + 40, 240, 60, 40)                     # merkez yatayda w/2'den uzak
    assert not lock_box_valid(target, off)


def test_frame_lock_valid_threshold():
    assert frame_lock_valid(_det(w=64), GEOM, lock_min_frac=0.06)
    assert not frame_lock_valid(_det(w=20, h=20), GEOM, lock_min_frac=0.06)  # cok kucuk
    assert not frame_lock_valid(_det(cx=40), GEOM)        # AV disinda


# ---------------- lock evaluator ----------------
def _feed(ev, dur, dt=0.1, t0=0.0, **kw):
    """t0..t0+dur araliginda gecerli frame besle; son event'i dondur."""
    last = None
    t = t0
    n = int(round(dur / dt))
    for _ in range(n + 1):
        ev_out = ev.update(t, _det(**{k: v for k, v in kw.items() if k in
                                      ("cx", "cy", "w", "h", "tid")}),
                           GEOM, is_autonomous=kw.get("auto", True),
                           airborne=kw.get("airborne", True))
        if ev_out is not None:
            last = ev_out   # feed boyunca gorulen son anlamli olay
        t += dt
    return last, t


def test_lock_achieved_after_4s():
    ev = LockOnEvaluator()
    last, _ = _feed(ev, 4.0)
    assert ev.state == LockState.LOCKED
    assert last is not None and last.kind == LockEventKind.ACHIEVED
    assert last.duration_s >= 4.0 - 1e-6


def test_no_lock_when_target_small():
    ev = LockOnEvaluator()
    last, _ = _feed(ev, 5.0, w=20, h=20)
    assert ev.state == LockState.IDLE and last is None


def test_no_lock_when_manual():
    ev = LockOnEvaluator()
    last, _ = _feed(ev, 5.0, auto=False)
    assert ev.state == LockState.IDLE and last is None


def test_no_lock_when_ground():
    ev = LockOnEvaluator()
    last, _ = _feed(ev, 5.0, airborne=False)
    assert ev.state == LockState.IDLE and last is None


def test_autonomy_gate_target_already_in_av_during_manual():
    ev = LockOnEvaluator()
    t = 0.0
    # hedef AV icinde ama MANUEL -> prev_in_av True olur, otonoma gecince
    # pencere baslamamali (sartname: AV'ye girmeden once otonom olunmali)
    for _ in range(5):
        ev.update(t, _det(), GEOM, is_autonomous=False)
        t += 0.1
    last = None
    for _ in range(60):
        last = ev.update(t, _det(), GEOM, is_autonomous=True)
        t += 0.1
    assert ev.state == LockState.IDLE and last is None


def test_small_gap_tolerated_big_gap_breaks():
    ev = LockOnEvaluator()
    _feed(ev, 4.0)                       # kilit
    assert ev.state == LockState.LOCKED
    # kisa kayip (0.1s < max_gap 0.15, sartname 200ms butcesinin altinda)
    ev.update(4.1, None, GEOM)
    assert ev.state == LockState.LOCKED
    ev.update(4.15, _det(), GEOM)
    assert ev.state == LockState.LOCKED
    # butce ustu kayip -> kopar (LOST)
    last = ev.update(4.5, None, GEOM)
    assert ev.state == LockState.IDLE
    assert last is not None and last.kind == LockEventKind.LOST


def test_gap_budget_is_spec_compliant():
    """Sartname: 4 sn kilitte kesinti butcesi %5 = 200 ms. Varsayilanlar altinda."""
    cfg = LockEvaluatorConfig()
    assert cfg.max_gap_s <= 0.2
    assert cfg.invalid_tolerance_s <= 0.2


def test_lock_requires_valid_coverage_not_just_elapsed():
    """Kayipli pencerede kilit, kaybin telafisine kadar ERTELENMELI.

    0.1 sn'lik kayip varsa 4.0 sn'de degil ~4.1 sn'de ACHIEVED beklenir
    (hakem kesintisiz 4 sn arar; gecen-sure saymak yanlis pakete yol acar).
    """
    ev = LockOnEvaluator()
    t = 0.0
    achieved_at = None
    while t <= 4.3:
        det = None if abs(t - 2.0) < 0.049 else _det()   # 2.0'da tek kayip frame
        out = ev.update(round(t, 3), det, GEOM)
        if out is not None and out.kind == LockEventKind.ACHIEVED:
            achieved_at = t
            break
        t = round(t + 0.05, 3)
    assert achieved_at is not None
    assert achieved_at >= 4.05 - 1e-9   # 4.0'dan ERTELENMIS olmali


def test_prevent_same_target_twice():
    ev = LockOnEvaluator()
    _feed(ev, 4.0, tid=1)
    ev.update(99.0, None, GEOM)          # kopar -> tid=1 blokeli
    # tid=1 tekrar denenir ama bloklu -> kilit yok
    last, t = _feed(ev, 5.0, t0=100.0, tid=1)
    assert ev.state == LockState.IDLE and last is None
    # farkli hedef (tid=2) -> kilit olur, blok kalkar
    last2, _ = _feed(ev, 4.5, t0=110.0, tid=2)
    assert ev.state == LockState.LOCKED and last2.kind == LockEventKind.ACHIEVED


# ---------------- Kalman estimator ----------------
def test_estimator_tracks_linear_motion():
    est = TargetEstimator()
    t = 0.0
    out = None
    for i in range(30):
        out = est.update(t, _det(cx=100 + 10 * i, cy=200))
        t += 0.1
    assert out is not None
    assert abs(out.box.cx - (100 + 10 * 29)) < 15.0   # son olcume yakin


def test_estimator_coasts_then_expires():
    est = TargetEstimator()
    t = 0.0
    for i in range(10):
        est.update(t, _det(cx=100 + 10 * i))
        t += 0.1
    # olcumsuz coast: kisa surede tahmin var
    coast = est.update(t + 0.1, None)
    assert coast is not None
    # max_coast asilinca None
    gone = est.update(t + 5.0, None)
    assert gone is None


# ---------------- PID / centering ----------------
def test_pid_output_limits_and_sign():
    pid = PID(2.0, 0.0, 0.0, out_limit=1.0)
    assert pid.update(10.0, 0.1) == 1.0       # doygunluk
    assert pid.update(-10.0, 0.1) == -1.0


def test_centering_signs_and_deadband():
    c = CenteringController()
    # hedef sagda (nx>0) -> saga don (vy>0); asagida (ny>0) -> dal (vz>0)
    vx, vy, vz = c.update(0.5, 0.5, t=0.0)
    _, vy2, vz2 = c.update(0.5, 0.5, t=0.1)
    assert vy2 > 0 and vz2 > 0 and vx > 0
    # deadband
    c2 = CenteringController()
    _, vyz, vzz = c2.update(0.01, 0.01, t=0.0)
    assert vyz == 0.0 and vzz == 0.0


# ---------------- downlink encoder ----------------
def test_downlink_formats_under_limit():
    enc = DownlinkEncoder()
    enc.sync_server_clock(1_718_000_000_000)
    klt = enc.encode_lock(True, lock_end_ms=1_718_000_004_000)
    assert klt.startswith("KLT:") and len(klt) <= STATUSTEXT_MAX
    kbx = enc.encode_lock_box(320, 240, 60, 40, 640, 480)
    assert decode_downlink(kbx).kind == "KBX" and len(kbx) <= STATUSTEXT_MAX
    mod = enc.encode_mode("GUIDED")
    assert decode_downlink(mod).fields[-1] == "GUIDED"


def test_downlink_chunk_long_qr():
    long_payload = "X" * 120
    frames = DownlinkEncoder.chunk("QRR", f"123:{long_payload}")
    assert len(frames) > 1
    assert all(len(f) <= STATUSTEXT_MAX for f in frames)
    assert frames[0].startswith("QRR[1/")


def test_downlink_server_clock_offset():
    enc = DownlinkEncoder()
    enc.sync_server_clock(2_000_000_000_000, local_epoch_ms=1_000_000_000_000)
    # offset = +1e12 ms
    assert abs(enc.clock.offset_ms - 1_000_000_000_000) < 1.0


# ---------------- combat tracker sartname uyumu ----------------

class _StubDetector:
    def __init__(self):
        self.det = None

    def best(self, frame):
        return self.det


class _StubRoi:
    active = False
    needs_refresh = False

    def update(self, frame):
        return None

    def init(self, frame, box):
        pass


class _SpyRecorder:
    is_open = True

    def __init__(self):
        self.calls = []   # (server_ms, locked, has_lock_box)

    def write(self, frame, server_ms, av_box=None, lock_box=None, locked=False):
        self.calls.append((server_ms, locked, lock_box is not None))
        return True


def _np_frame():
    import numpy as np
    return np.zeros((480, 640, 3), dtype=np.uint8)


def _make_tracker(mode="lock", qr=None):
    from uav.perception.tracking.combat_tracker import (CombatTrackerConfig,
                                                        CombatVisionTracker)
    det = _StubDetector()
    rec = _SpyRecorder()
    trk = CombatVisionTracker(CombatTrackerConfig(mode=mode, detect_every_n=1),
                              detector=det, roi=_StubRoi(), qr_detector=qr,
                              recorder=rec)
    return trk, det, rec


def test_tracker_draws_lock_box_during_tracking_phase():
    """AH dortgeni 4 sn DOLMADAN da (TRACKING) cizilmeli.

    Hakem bildirilen bitisten geriye sarip 4 sn'lik pencereyi inceler; o
    pencere TRACKING fazidir -- dortgensiz frame kilidi gecersiz kilar.
    """
    trk, det, rec = _make_tracker()
    frame = _np_frame()
    t = 0.0
    for i in range(10):                      # 1 sn: TRACKING (< 4 sn)
        det.det = _det(tid=-1)
        trk.process_frame(frame, t, server_ms=int(t * 1000), is_autonomous=True)
        t += 0.1
    from uav.perception.tracking.lock_evaluator import LockState
    assert trk.lock_state() == LockState.TRACKING
    # ilk frame'den itibaren kirmizi AH cizilmis olmali
    assert all(locked and has_box for _, locked, has_box in rec.calls)


def test_tracker_guidance_is_vision_los_with_fov_mapping():
    from uav.perception.perception_types import PerceptionFrame
    trk, det, rec = _make_tracker()
    frame = _np_frame()
    # hedef sag-altta: nx=0.5 -> yaw=+15 (h_fov 60); ny=0.5 -> pitch=+11.25 (asagi)
    det.det = _det(cx=480, cy=360, tid=-1)
    res = trk.process_frame(frame, 0.0, server_ms=0, is_autonomous=True)
    g = res.guidance
    assert g is not None and g.frame == PerceptionFrame.VISION_LOS
    assert abs(g.a - 15.0) < 1.5
    assert abs(g.b - 11.25) < 1.5
    assert g.c == 80.0


def test_tracker_qr_mode_records_video_evidence():
    """Kamikaze de videoyla dogrulanir -> QR modunda her frame yazilmali."""
    class _QrStub:
        result = None

        def detect(self, frame):
            return self.result

    qr = _QrStub()
    trk, det, rec = _make_tracker(mode="qr", qr=qr)
    frame = _np_frame()
    trk.process_frame(frame, 0.0, server_ms=0, is_autonomous=True)
    assert len(rec.calls) == 1 and rec.calls[0][1] is False   # QR yok: ham frame
    from types import SimpleNamespace
    qr.result = SimpleNamespace(box=BBox(320, 240, 80, 80), payload="KOD123")
    res = trk.process_frame(frame, 0.1, server_ms=100, is_autonomous=True)
    assert res.qr_payload == "KOD123"
    assert rec.calls[-1][1] is True and rec.calls[-1][2] is True  # kirmizi kutu


def test_tracker_runtime_mode_switch_keeps_module_alive():
    class _QrStub:
        def detect(self, frame):
            return None

    trk, det, rec = _make_tracker(mode="lock", qr=_QrStub())
    frame = _np_frame()
    det.det = _det(tid=-1)
    trk.process_frame(frame, 0.0, server_ms=0, is_autonomous=True)
    trk.set_mode("qr")
    assert trk.cfg.mode == "qr"
    res = trk.process_frame(frame, 0.1, server_ms=100, is_autonomous=True)
    assert res.lock_event is None
    trk.set_mode("lock")
    assert trk.cfg.mode == "lock"
    # kayit nesnesi ayni kaldi (video kaniti kesintisiz)
    assert len(rec.calls) >= 2


def test_video_recorder_fixed_fps_pacing(tmp_path):
    pytest = __import__("pytest")
    cv2 = pytest.importorskip("cv2")
    import numpy as np
    from uav.perception.tracking.video_recorder import (LockVideoRecorder,
                                                        VideoConfig)
    path = str(tmp_path / "lock.mp4")
    rec = LockVideoRecorder(VideoConfig(path=path, fps=15.0, fourcc="mp4v"))
    assert rec.open(640, 480)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    # 2 sn boyunca 30 fps kamera beslemesi -> dosyaya ~30 kare (15 fps * 2 sn)
    for ms in range(0, 2000, 33):
        rec.write(frame, server_ms=ms)
    written = rec.frames_written
    rec.close()
    assert abs(written - 30) <= 2, written
    # ayni yol tekrar acilirsa ESKI DOSYA EZILMEZ (zaman damgali yeni ad)
    rec2 = LockVideoRecorder(VideoConfig(path=path, fps=15.0, fourcc="mp4v"))
    assert rec2.open(640, 480)
    rec2.write(frame, server_ms=0)
    assert rec2.actual_path != path
    rec2.close()


def test_video_recorder_uses_actual_frame_size(tmp_path):
    pytest = __import__("pytest")
    pytest.importorskip("cv2")
    import numpy as np
    from uav.perception.tracking.video_recorder import (LockVideoRecorder,
                                                        VideoConfig)
    rec = LockVideoRecorder(VideoConfig(path=str(tmp_path / "v.mp4"),
                                        fps=15.0, fourcc="mp4v"))
    rec.open(640, 480)                          # istek 640x480...
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)   # kamera 1280x720 verdi
    assert rec.write(frame, server_ms=0)        # sessiz dusme YOK, yazildi
    assert rec.frames_written == 1
    rec.close()


# ---------------- downlink wiring (factory) ----------------
def test_attach_downlink_emits_klt_and_qrr():
    from uav.perception.tracking.combat_tracker import (CombatTrackerConfig,
                                                        CombatVisionTracker)
    from uav.perception.tracking.factory import attach_downlink
    from uav.perception.tracking.lock_evaluator import LockEvent, LockEventKind

    sent = []
    enc = DownlinkEncoder(send_fn=lambda text, sev=6: sent.append(text))
    enc.sync_server_clock(1_718_000_000_000)
    trk = CombatVisionTracker(CombatTrackerConfig(mode="lock"), detector=object())
    attach_downlink(trk, enc, bus=None)

    ev = LockEvent(kind=LockEventKind.ACHIEVED, target_id=1, start_t=0.0, end_t=4.0,
                   box_cx=320, box_cy=240, box_w=60, box_h=40, frame_w=640, frame_h=480)
    trk.on_lock_event(ev)
    assert any(t.startswith("KLT:") for t in sent)
    assert any(t.startswith("KBX:") for t in sent)

    trk.on_qr("NARA-QR-9", None)
    assert any(t.startswith("QRR") for t in sent)

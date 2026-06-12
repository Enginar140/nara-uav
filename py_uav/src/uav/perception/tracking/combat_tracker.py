"""CombatVisionTracker -- Savasan/Kamikaze gorus orkestratoru.

Tum tracking parcalarini birlestiren PerceptionEngine modulu:

  kamera -> dedektor(YOLO, seyrek) + ROI takipci(sik) -> Kalman tahmin
         -> kilit durum makinesi (lock event)        \\
         -> merkeze-getirme PID  -> guidance target   -> poll()/nav
         -> video kayit (AV/AH + sunucu saati)
  (qr modunda: QR dedektoru -> payload callback + dalis guidance)

Tasarim:
  - Cekirdek adim process_frame() saf/enjekte-edilebilir -> kamerasiz test.
  - Agir bagimliliklar (YOLO/cv2) opsiyonel; enjekte edilirse hic kurulmaz.
  - Kilit/QR olaylari callback ile disari verilir (downlink'e baglanir).
"""
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Callable, Optional

from ...core.time_utils import now_monotonic, now_us
from ...core.types import UAVState
from ...infra.log import get_logger
from ..module_base import IPerceptionModule
from ..opencv_vision_tracker import CameraConfig
from ..perception_types import PerceptionFrame, PerceptionTarget
from .centering_pid import CenteringConfig, CenteringController
from .detection import BBox, Detection, FrameGeometry
from .lock_evaluator import (LockEvaluatorConfig, LockEvent, LockOnEvaluator,
                             LockState)
from .target_estimator import EstimatorConfig, TargetEstimator

_log = get_logger("combat_tracker")

try:
    import cv2  # type: ignore
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False


@dataclass
class CombatTrackerConfig:
    id: str = "combat_vision"
    mode: str = "lock"               # lock | qr (runtime'da set_mode ile degisir)
    # AV oranlari sartname Sekil 1'den: yatay %25+%25 kenar -> genislik %50,
    # dikey %10+%10 kenar -> yukseklik %80.
    av_w_frac: float = 0.50
    av_h_frac: float = 0.80
    lock_min_frac: float = 0.06
    detect_every_n: int = 6          # her N frame'de bir YOLO (arada ROI)
    record_video: bool = False
    worker_sleep_ms: int = 1
    airborne_default: bool = True
    # guidance cercevesi: "los" -> VISION_LOS aci hatalari (sanal FBWA attitude
    # kanunu kullanir); "velocity" -> eski merkeze-getirme PID govde hizi.
    guidance: str = "los"
    h_fov_deg: float = 60.0
    v_fov_deg: float = 45.0
    default_range_m: float = 80.0    # LOS hedefi icin nominal menzil
    camera: CameraConfig = None
    estimator: EstimatorConfig = field(default_factory=EstimatorConfig)
    centering: CenteringConfig = field(default_factory=CenteringConfig)
    lock: LockEvaluatorConfig = field(default_factory=LockEvaluatorConfig)

    def __post_init__(self):
        if self.camera is None:
            self.camera = CameraConfig()
        # lock esigini tek kaynaktan yonet
        self.lock.lock_min_frac = self.lock_min_frac


@dataclass
class ProcessResult:
    guidance: Optional[PerceptionTarget] = None
    lock_event: Optional[LockEvent] = None
    detection: Optional[Detection] = None
    qr_payload: Optional[str] = None


class CombatVisionTracker(IPerceptionModule):
    """detector: .best(frame)->Detection|None  veya  .detect(frame)->List.
       qr_detector: .detect(frame)->QrResult|None.  recorder: LockVideoRecorder."""

    def __init__(self, config: Optional[CombatTrackerConfig] = None,
                 detector=None, roi=None, qr_detector=None, recorder=None,
                 server_ms_fn: Optional[Callable[[], int]] = None,
                 autonomous_fn: Optional[Callable[[], bool]] = None):
        self.cfg = config or CombatTrackerConfig()
        self._detector = detector
        self._roi = roi
        self._qr = qr_detector
        self._recorder = recorder
        self._server_ms = server_ms_fn or (lambda: now_us() // 1000)
        # canli otonomi okumasi (ornek: commander.is_in_guided_mode)
        self._autonomous_fn = autonomous_fn

        self._est = TargetEstimator(self.cfg.estimator)
        self._pid = CenteringController(self.cfg.centering)
        self._lock_eval = LockOnEvaluator(self.cfg.lock)

        self._frame_n = 0
        self._cur_tid = -1
        self._next_tid = 1
        self._had_target = False
        self._geom: Optional[FrameGeometry] = None

        # disari acik durum
        self._lock = threading.RLock()
        self._latest: Optional[PerceptionTarget] = None
        self._latest_t = 0.0
        self._is_autonomous = True
        self._airborne = self.cfg.airborne_default
        self._running = False
        self._cap = None
        self._worker: Optional[threading.Thread] = None

        # callback'ler (entegrasyon baglar)
        self.on_lock_event: Optional[Callable[[LockEvent], None]] = None
        self.on_qr: Optional[Callable[[str, BBox], None]] = None

    def id(self) -> str:
        return self.cfg.id

    # ---- disari kontrol ----
    def set_autonomous(self, value: bool) -> None:
        with self._lock:
            self._is_autonomous = bool(value)

    def set_airborne(self, value: bool) -> None:
        with self._lock:
            self._airborne = bool(value)

    def lock_state(self):
        return self._lock_eval.state

    def set_mode(self, mode: str) -> None:
        """Calisma aninda lock <-> qr gecisi (kamera ve video kaydi ACIK kalir).

        Sartname ayni turda hem kilitlenme hem kamikaze'ye izin verir ve ayni
        kamerayi sart kosar; modul degistirmek yerine mod degistirilir, boylece
        tek parca video kaniti korunur.
        """
        if mode not in ("lock", "qr"):
            return
        with self._lock:
            if mode == self.cfg.mode:
                return
            self.cfg.mode = mode
            self._lazy_build()
            self._est = TargetEstimator(self.cfg.estimator)
            self._pid.reset()
            self._lock_eval.reset()
            self._had_target = False
            self._cur_tid = -1
        _log.info("CombatVisionTracker mod degisti -> %s", mode)

    # ---- cekirdek adim (test edilebilir) ----
    def process_frame(self, frame_bgr, t: float, server_ms: int,
                      is_autonomous: bool) -> ProcessResult:
        h, w = frame_bgr.shape[:2]
        if self._geom is None or self._geom.width != w or self._geom.height != h:
            self._geom = FrameGeometry(w, h, self.cfg.av_w_frac, self.cfg.av_h_frac)
        self._frame_n += 1

        if self.cfg.mode == "qr":
            return self._process_qr(frame_bgr, t, server_ms)

        det = self._acquire(frame_bgr, w, h)
        est_det = self._est.update(t, det)

        ev = self._lock_eval.update(
            t, est_det, self._geom, is_autonomous=is_autonomous,
            target_id=self._cur_tid, airborne=self._airborne)

        guidance = None
        if est_det is not None:
            guidance = self._make_guidance(est_det, t, label="lock_pursuit")

        if self._recorder is not None and self._recorder.is_open:
            # AH dortgeni KILIT PENCERESI boyunca cizilir (TRACKING + LOCKED).
            # Hakem, bildirilen bitisten 5 sn geriye sarip 4 sn'lik kilidi
            # arar -- o pencere TRACKING fazidir; dortgensiz frame'ler kilidi
            # gecersiz kilar (sartname: 'kilitlenme suresince her frame'e').
            window_active = self._lock_eval.state in (LockState.TRACKING,
                                                      LockState.LOCKED)
            lock_box = est_det.box if (est_det is not None and window_active) else None
            self._recorder.write(frame_bgr, server_ms, av_box=self._geom.av_box(),
                                 lock_box=lock_box, locked=lock_box is not None)

        return ProcessResult(guidance=guidance, lock_event=ev, detection=est_det)

    def _process_qr(self, frame_bgr, t: float, server_ms: int) -> ProcessResult:
        res = self._qr.detect(frame_bgr) if self._qr is not None else None
        det = None
        guidance = None
        if res is not None:
            det = Detection(box=res.box, frame_w=self._geom.width,
                            frame_h=self._geom.height, confidence=0.9,
                            label=res.payload)
            guidance = self._make_guidance(det, t, label=res.payload)
        # Kamikaze kaniti da VIDEO ile dogrulanir (sartname 6.2: 'QR mesaji
        # video uzerinden kontrol edilecektir') -> QR modunda da her frame yaz;
        # cozulen QR kutusu kirmizi isaretlenir.
        if self._recorder is not None and self._recorder.is_open:
            self._recorder.write(frame_bgr, server_ms, av_box=None,
                                 lock_box=det.box if det is not None else None,
                                 locked=det is not None)
        if res is None:
            return ProcessResult()
        return ProcessResult(guidance=guidance, detection=det,
                             qr_payload=res.payload)

    def _make_guidance(self, det: Detection, t: float,
                       label: str) -> PerceptionTarget:
        """Tespitten guidance hedefi uret (cfg.guidance secimine gore).

        "los": normalize merkez ofsetini kamera FOV'uyla aci hatasina cevirir
        -> VISION_LOS. NavigationEngine.dispatch bunu (aciksa) sanal-FBWA
        attitude kanununa, degilse pozisyon-adimina yonlendirir.
        "velocity": eski merkeze-getirme PID -> govde hiz demand'i.
        """
        nx, ny = det.center_offset_norm()
        conf = det.confidence or 0.5
        if self.cfg.guidance == "velocity":
            vx, vy, vz = self._pid.update(nx, ny, t)
            return PerceptionTarget(
                frame=PerceptionFrame.VELOCITY_BODY, a=vx, b=vy, c=vz,
                confidence=conf, stamp_us=now_us(), source_tag=self.id(),
                label=label)
        yaw_off = nx * self.cfg.h_fov_deg * 0.5
        pitch_off = ny * self.cfg.v_fov_deg * 0.5   # +asagi (VISION_LOS kurali)
        return PerceptionTarget(
            frame=PerceptionFrame.VISION_LOS, a=yaw_off, b=pitch_off,
            c=self.cfg.default_range_m, confidence=conf, stamp_us=now_us(),
            source_tag=self.id(), label=label)

    # ---- dedektor + ROI birlesimi ----
    def _acquire(self, frame_bgr, w: int, h: int) -> Optional[Detection]:
        # 1) ROI ile ucuz takip (gecerli + yenileme gerekmiyorsa)
        if self._roi is not None and self._roi.active and not self._roi.needs_refresh \
                and (self._frame_n % self.cfg.detect_every_n != 0):
            box = self._roi.update(frame_bgr)
            if box is not None:
                self._had_target = True
                return Detection(box=box, frame_w=w, frame_h=h, confidence=0.6,
                                 track_id=self._cur_tid, label="roi")
        # 2) YOLO ile (yeniden) tespit
        yd = self._detect_best(frame_bgr)
        if yd is None:
            # tespit yok -> hedef kaybi (bir sonraki edinim yeni kimlik alir)
            self._had_target = False
            self._cur_tid = -1
            return None
        if not self._had_target or self._cur_tid < 0:
            self._cur_tid = self._next_tid
            self._next_tid += 1
        self._had_target = True
        yd.track_id = self._cur_tid
        if self._roi is not None:
            self._roi.init(frame_bgr, yd.box)
        return yd

    def _detect_best(self, frame_bgr) -> Optional[Detection]:
        d = self._detector
        if d is None:
            return None
        if hasattr(d, "best"):
            return d.best(frame_bgr)
        if hasattr(d, "detect"):
            dets = d.detect(frame_bgr)
            return dets[0] if dets else None
        return None

    # ---- IPerceptionModule ----
    def poll(self, own_state: UAVState) -> Optional[PerceptionTarget]:
        with self._lock:
            return self._latest

    def start(self) -> None:
        if self._running:
            return
        self._lazy_build()
        if not self._open_camera():
            _log.warning("kamera acilamadi; combat tracker pasif")
            return
        if self.cfg.record_video and self._recorder is not None:
            self._recorder.open(self.cfg.camera.width, self.cfg.camera.height)
        self._running = True
        self._worker = threading.Thread(target=self._loop, name="combat-vision",
                                        daemon=True)
        self._worker.start()
        _log.info("CombatVisionTracker basladi (mode=%s)", self.cfg.mode)

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._worker:
            self._worker.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()
        if self._recorder is not None:
            self._recorder.close()
        with self._lock:
            self._latest = None

    # ---- worker ----
    def _loop(self) -> None:
        while self._running:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                if self.cfg.camera.type == "file" and self.cfg.camera.loop_file:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                threading.Event().wait(0.02)
                continue
            t = now_monotonic()
            if self._autonomous_fn is not None:
                try:
                    self.set_autonomous(self._autonomous_fn())
                except Exception:  # noqa: BLE001
                    pass
            with self._lock:
                is_auto = self._is_autonomous
            res = self.process_frame(frame, t, self._server_ms(), is_auto)
            with self._lock:
                self._latest = res.guidance
                self._latest_t = t
            if res.lock_event is not None and self.on_lock_event is not None:
                self.on_lock_event(res.lock_event)
            if res.qr_payload and self.on_qr is not None:
                self.on_qr(res.qr_payload, res.detection.box if res.detection else None)
            if self.cfg.worker_sleep_ms > 0:
                threading.Event().wait(self.cfg.worker_sleep_ms / 1000.0)

    def _lazy_build(self) -> None:
        """Enjekte edilmediyse gercek dedektorleri kur (agir importlar)."""
        if self.cfg.mode == "lock" and self._detector is None:
            from .yolo_detector import YoloAircraftDetector
            self._detector = YoloAircraftDetector()
        if self.cfg.mode == "lock" and self._roi is None:
            from .roi_tracker import RoiTracker
            self._roi = RoiTracker()
        if self.cfg.mode == "qr" and self._qr is None:
            from .qr_pro import QrProDetector
            self._qr = QrProDetector()

    def _open_camera(self) -> bool:
        if not _HAS_CV2:
            return False
        c = self.cfg.camera
        src = c.device_index if c.type == "device" else (
            c.file_path if c.type == "file" else c.url)
        self._cap = cv2.VideoCapture(src)
        if not self._cap.isOpened():
            return False
        if c.width:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, c.width)
        if c.height:
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c.height)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return True

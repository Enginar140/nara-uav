"""YoloAircraftDetector -- ultralytics YOLO ile rakip IHA tespiti.

Kaliteli dedektor: frame -> List[Detection]. ultralytics yoksa import kirilmaz;
nesne olusturulurken anlamli hata verir (optional bagimlilik). Hafif/yuksek-FPS
icin ROI tracker (roi_tracker.py) ile birlikte kullanilir: YOLO seyrek, ROI sik.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from ...core.time_utils import now_us
from ...infra.log import get_logger
from .detection import BBox, Detection

_log = get_logger("yolo_detector")

try:
    from ultralytics import YOLO  # type: ignore
    _HAS_YOLO = True
except Exception:  # noqa: BLE001  (ultralytics agir; her turlu import hatasini yut)
    _HAS_YOLO = False


@dataclass
class YoloConfig:
    model_path: str = "yolov8n.pt"   # kucuk model -> yuksek FPS; egitilmis ucak modeli onerilir
    conf: float = 0.35
    iou: float = 0.5
    imgsz: int = 640
    device: str = ""                 # "" oto, "cpu", "0" (cuda)
    half: bool = False               # FP16 (cuda'da hizli)
    max_det: int = 10
    # kabul edilen sinif id'leri (bos = hepsi). COCO'da 'aeroplane'=4.
    class_ids: List[int] = field(default_factory=list)
    min_box_px: int = 6              # cok kucuk kutulari ele


class YoloAircraftDetector:
    def __init__(self, config: Optional[YoloConfig] = None):
        if not _HAS_YOLO:
            raise RuntimeError(
                "YoloAircraftDetector icin ultralytics gerekli: pip install ultralytics")
        self.cfg = config or YoloConfig()
        self._model = YOLO(self.cfg.model_path)
        self._warmed = False
        _log.info("YOLO yuklendi: %s (imgsz=%d conf=%.2f)",
                  self.cfg.model_path, self.cfg.imgsz, self.cfg.conf)

    def detect(self, frame_bgr) -> List[Detection]:
        """BGR frame -> Detection listesi (guven sirali, en yuksek ilk)."""
        if frame_bgr is None:
            return []
        h, w = frame_bgr.shape[:2]
        kwargs = dict(conf=self.cfg.conf, iou=self.cfg.iou, imgsz=self.cfg.imgsz,
                      max_det=self.cfg.max_det, verbose=False)
        if self.cfg.device:
            kwargs["device"] = self.cfg.device
        if self.cfg.half:
            kwargs["half"] = True
        if self.cfg.class_ids:
            kwargs["classes"] = list(self.cfg.class_ids)
        res = self._model.predict(frame_bgr, **kwargs)
        out: List[Detection] = []
        if not res:
            return out
        boxes = getattr(res[0], "boxes", None)
        if boxes is None:
            return out
        stamp = now_us()
        for b in boxes:
            xyxy = b.xyxy[0].tolist()
            x1, y1, x2, y2 = xyxy
            bw, bh = x2 - x1, y2 - y1
            if bw < self.cfg.min_box_px or bh < self.cfg.min_box_px:
                continue
            conf = float(b.conf[0]) if b.conf is not None else 0.0
            cls = int(b.cls[0]) if b.cls is not None else 0
            out.append(Detection(
                box=BBox.from_ltrb(x1, y1, x2, y2), frame_w=w, frame_h=h,
                confidence=conf, class_id=cls, label=self._name(cls), stamp_us=stamp))
        out.sort(key=lambda d: d.confidence, reverse=True)
        return out

    def best(self, frame_bgr) -> Optional[Detection]:
        dets = self.detect(frame_bgr)
        return dets[0] if dets else None

    def _name(self, cls: int) -> str:
        try:
            return self._model.names.get(cls, str(cls))  # type: ignore
        except Exception:  # noqa: BLE001
            return str(cls)


def yolo_available() -> bool:
    return _HAS_YOLO

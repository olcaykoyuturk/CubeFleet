"""
detector.py — Ultralytics YOLO ile küp tespiti.

Default model: yolo11s.pt (Ultralytics resmi pre-trained COCO weights, 2024 nesli).
  - YOLO11s: 9.4M parametre, mAP@50-95 = 47.0 (YOLOv8s = 44.9), CPU'da ~30-40 fps
  - Daha hızli isteyen yolo11n (2.6M, 60 fps) kullanabilir
  - Daha hassas isteyen yolo11m (20.1M, 15-20 fps) kullanabilir
  - Custom-trained .pt agirligi varsa MODEL_PATH'i degistir; ayni sinif (cube)
    icin runs/detect/train*/weights/best.pt yolu gecilebilir

Tek-class kullanim ("cube"):
  - Pre-trained COCO weights'ta "cube" sinifi YOK — sadece pipeline testi icin
  - Gerçek tespit için Roboflow ile etiketle + Ultralytics CLI ile fine-tune:
      yolo task=detect mode=train model=yolo11s.pt data=cubes.yaml epochs=50 imgsz=640
    Bittiginde runs/detect/train/weights/best.pt'i MODEL_PATH yap

Lazy load: model ilk detect() cagrisinda yuklenir (UI startup'i bloklamaz).
Thread-safe degil — tek thread (PC ana UI loop / _process_camera) icin tasarlandi.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np


# Varsayilan ayarlar — env override edilebilir
# Custom-trained: cube-ajbii/v1, YOLO11s @ 480x480, mAP50=0.983 (2026-05-24)
_DEFAULT_MODEL = str(Path(__file__).resolve().parent.parent / "models" / "cube_best.pt")
MODEL_PATH       = os.environ.get("CUBE_MODEL", _DEFAULT_MODEL)
CONF_THRESHOLD   = float(os.environ.get("CUBE_CONF", "0.6"))
IOU_THRESHOLD    = float(os.environ.get("CUBE_IOU",  "0.45"))
TARGET_CLASSES   = os.environ.get("CUBE_CLASSES", "Cube").strip()   # data.yaml'da 'Cube'
INFER_IMGSZ      = int(os.environ.get("CUBE_IMGSZ", "480"))         # train imgsz ile ayni

# Renkler (BGR)
COL_BBOX    = (0, 255, 0)        # yesil bbox
COL_CENTER  = (0, 0, 255)        # kirmizi merkez
COL_LABEL   = (255, 255, 255)    # beyaz yazi
COL_LBL_BG  = (0, 120, 0)        # koyu yesil arkaplan


@dataclass
class Detection:
    """Tek bir tespit: bbox + güven + sinif + merkez."""
    x1: int
    y1: int
    x2: int
    y2: int
    conf: float
    cls_id: int
    cls_name: str

    @property
    def cx(self) -> int:
        return (self.x1 + self.x2) // 2

    @property
    def cy(self) -> int:
        return (self.y1 + self.y2) // 2

    @property
    def area(self) -> int:
        return max(0, (self.x2 - self.x1) * (self.y2 - self.y1))


class CubeDetector:
    """Lazy-loaded YOLO wrapper. Ilk detect() cagrisinda model yuklenir."""

    def __init__(self,
                 model_path: str = MODEL_PATH,
                 conf: float = CONF_THRESHOLD,
                 iou: float = IOU_THRESHOLD,
                 imgsz: int = INFER_IMGSZ):
        self.model_path = model_path
        self.conf = conf
        self.iou = iou
        self.imgsz = imgsz
        self._model = None
        self._target_class_ids: Optional[List[int]] = None
        self._last_infer_ms = 0.0

    # ------ Model yükleme ----------------------------------------------------

    def _ensure_loaded(self) -> bool:
        if self._model is not None:
            return True
        try:
            from ultralytics import YOLO  # type: ignore
        except ImportError:
            print("[detector] ultralytics kurulu degil: pip install ultralytics")
            return False
        try:
            self._model = YOLO(self.model_path)
            # Class filter (env: CUBE_CLASSES="cube" veya "cube,top_face")
            if TARGET_CLASSES:
                wanted = {n.strip().lower() for n in TARGET_CLASSES.split(",")}
                names = self._model.names    # dict {id: name}
                self._target_class_ids = [i for i, n in names.items()
                                          if n.lower() in wanted]
                if not self._target_class_ids:
                    print(f"[detector] UYARI: {TARGET_CLASSES} model'de bulunamadi. "
                          f"Mevcut siniflar: {list(names.values())[:10]}...")
            return True
        except Exception as e:
            print(f"[detector] Model yuklenemedi ({self.model_path}): {e}")
            self._model = None
            return False

    @property
    def loaded(self) -> bool:
        return self._model is not None

    @property
    def last_infer_ms(self) -> float:
        return self._last_infer_ms

    # ------ Tespit -----------------------------------------------------------

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """BGR frame -> tespit listesi (boş liste = hiç bulamadi).
        Model yuklenemediyse veya hata olursa bos liste doner."""
        if not self._ensure_loaded():
            return []

        assert self._model is not None
        t0 = time.perf_counter()
        try:
            results = self._model.predict(
                source=frame,
                conf=self.conf,
                iou=self.iou,
                imgsz=self.imgsz,
                classes=self._target_class_ids,
                verbose=False,
            )
        except Exception as e:
            print(f"[detector] predict hata: {e}")
            return []
        self._last_infer_ms = (time.perf_counter() - t0) * 1000.0

        if not results:
            return []

        r = results[0]
        if r.boxes is None or len(r.boxes) == 0:
            return []

        names = self._model.names
        dets: List[Detection] = []
        for box in r.boxes:
            xyxy = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0].cpu().numpy())
            cls  = int(box.cls[0].cpu().numpy())
            dets.append(Detection(
                x1=int(xyxy[0]), y1=int(xyxy[1]),
                x2=int(xyxy[2]), y2=int(xyxy[3]),
                conf=conf, cls_id=cls,
                cls_name=names.get(cls, f"class_{cls}"),
            ))
        return dets

    # ------ Overlay ----------------------------------------------------------

    def draw(self, frame: np.ndarray, dets: List[Detection]) -> np.ndarray:
        """Frame uzerine bbox + merkez + etiket cizer. In-place."""
        for d in dets:
            cv2.rectangle(frame, (d.x1, d.y1), (d.x2, d.y2), COL_BBOX, 2)
            cv2.drawMarker(frame, (d.cx, d.cy), COL_CENTER,
                           markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
            label = f"{d.cls_name} {d.conf:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (d.x1, d.y1 - th - 6),
                          (d.x1 + tw + 6, d.y1), COL_LBL_BG, -1)
            cv2.putText(frame, label, (d.x1 + 3, d.y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_LABEL, 1, cv2.LINE_AA)
        return frame

    # ------ En iyi tespit (magnet hedefi için) ------------------------------

    def best(self, dets: List[Detection]) -> Optional[Detection]:
        """En buyuk + en yuksek conf'lu tespiti dondurur (yakin/buyuk cube)."""
        if not dets:
            return None
        return max(dets, key=lambda d: (d.area, d.conf))


# Modul-seviyesi singleton — UI tek bir detector paylaşır
_singleton: Optional[CubeDetector] = None


def get_detector() -> CubeDetector:
    """Process boyunca tek instance. Ilk cagrida CubeDetector() yaratir."""
    global _singleton
    if _singleton is None:
        _singleton = CubeDetector()
    return _singleton

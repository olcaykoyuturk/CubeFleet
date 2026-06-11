"""Ultralytics YOLO ile cube tespiti. Lazy load, tek thread."""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np


# Custom-trained: cube-ajbii/v1, YOLO11s @ 480x480, mAP50=0.983
_DEFAULT_MODEL = str(Path(__file__).resolve().parent.parent / "models" / "cube_best.pt")
MODEL_PATH       = os.environ.get("CUBE_MODEL", _DEFAULT_MODEL)
CONF_THRESHOLD   = float(os.environ.get("CUBE_CONF", "0.6"))
IOU_THRESHOLD    = float(os.environ.get("CUBE_IOU",  "0.45"))
TARGET_CLASSES   = os.environ.get("CUBE_CLASSES", "Cube").strip()
INFER_IMGSZ      = int(os.environ.get("CUBE_IMGSZ", "480"))

COL_BBOX    = (0, 255, 0)
COL_CENTER  = (0, 0, 255)
COL_LABEL   = (255, 255, 255)
COL_LBL_BG  = (0, 120, 0)
COL_AIM     = (0, 215, 255)   # aim-point (BGR sari) — magnet hedef noktasi


@dataclass
class Detection:
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
    def height(self) -> int:
        return max(0, self.y2 - self.y1)

    @property
    def area(self) -> int:
        return max(0, (self.x2 - self.x1) * (self.y2 - self.y1))

    def aim_point(self, k: float) -> tuple:
        """Magnet hedef noktasi: bbox merkezi yerine biraz YUKARISI (cy - height*k).
        Kamera kola monteli ve kupe yaklasinca magnet kupun ust yuzune denk gelsin diye
        dikey offset uygulanir. k=0 → tam merkez, k arttikca daha yukari (bbox ustune)."""
        return (self.cx, int(self.cy - self.height * k))


class CubeDetector:
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
            if TARGET_CLASSES:
                wanted = {n.strip().lower() for n in TARGET_CLASSES.split(",")}
                names = self._model.names
                self._target_class_ids = [i for i, n in names.items()
                                          if n.lower() in wanted]
                if not self._target_class_ids:
                    print(f"[detector] {TARGET_CLASSES} model'de yok. "
                          f"Mevcut: {list(names.values())[:10]}")
            return True
        except Exception as e:
            print(f"[detector] yuklenemedi ({self.model_path}): {e}")
            self._model = None
            return False

    @property
    def loaded(self) -> bool:
        return self._model is not None

    @property
    def last_infer_ms(self) -> float:
        return self._last_infer_ms

    def detect(self, frame: np.ndarray) -> List[Detection]:
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
            dets.append(Detection(
                x1=int(xyxy[0]), y1=int(xyxy[1]),
                x2=int(xyxy[2]), y2=int(xyxy[3]),
                conf=float(box.conf[0].cpu().numpy()),
                cls_id=int(box.cls[0].cpu().numpy()),
                cls_name=names.get(int(box.cls[0].cpu().numpy()),
                                   f"class_{int(box.cls[0].cpu().numpy())}"),
            ))
        return dets

    def draw(self, frame: np.ndarray, dets: List[Detection],
             aim_k: Optional[float] = None) -> np.ndarray:
        """bbox + merkez crosshair + label cizer. aim_k verilirse her tespit icin
        ayrica magnet aim-point'i (sari) cizer — otonom kapma hedef noktasini gosterir."""
        for d in dets:
            cv2.rectangle(frame, (d.x1, d.y1), (d.x2, d.y2), COL_BBOX, 2)
            cv2.drawMarker(frame, (d.cx, d.cy), COL_CENTER,
                           markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)
            if aim_k is not None:
                ax, ay = d.aim_point(aim_k)
                cv2.drawMarker(frame, (ax, ay), COL_AIM,
                               markerType=cv2.MARKER_TILTED_CROSS, markerSize=16, thickness=2)
            label = f"{d.cls_name} {d.conf:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (d.x1, d.y1 - th - 6),
                          (d.x1 + tw + 6, d.y1), COL_LBL_BG, -1)
            cv2.putText(frame, label, (d.x1 + 3, d.y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COL_LABEL, 1, cv2.LINE_AA)
        return frame

    def best(self, dets: List[Detection]) -> Optional[Detection]:
        if not dets:
            return None
        return max(dets, key=lambda d: (d.area, d.conf))


_singleton: Optional[CubeDetector] = None


def get_detector() -> CubeDetector:
    global _singleton
    if _singleton is None:
        _singleton = CubeDetector()
    return _singleton

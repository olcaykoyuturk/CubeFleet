"""
HSV tabanli kup tespit modulu — kol-takip senaryosuna optimize.

Iyilestirmeler (mevcut API geriye uyumlu):
  * CLAHE on isleme (LAB L kanali) — isiklandirma degisimine dayanikli HSV
  * minAreaRect tabanli rotasyon-bagimsiz aspect (perspektif distorsiyonu OK)
  * Doluluk orani filtrei (concav/dagınık sekilleri eler)
  * 4-kose acı kontrolu (~90°) — perspektif icin tolerans, ucgen/yildiz eler
  * Confidence skoru (0..1) — kol kontrolu daha stabil bbox alir
  * Siralama: confidence × area (yalniz alan degil "ne kadar kare oldugu" da)
"""

import json
import math
import os
from dataclasses import dataclass, asdict, field
from typing import List, Tuple, Optional

import cv2
import numpy as np


# =============================================================================
# Esik yapilari
# =============================================================================

@dataclass
class RedThresholds:
    h1_low:  int = 0     # 0..10 arasi kirmizi (alt aralik)
    h1_high: int = 10
    h2_low:  int = 170   # 170..180 arasi kirmizi (ust aralik)
    h2_high: int = 180
    s_min:   int = 100
    v_min:   int = 60


@dataclass
class BlueThresholds:
    h_low:   int = 100
    h_high:  int = 130
    s_min:   int = 100
    v_min:   int = 60


@dataclass
class CommonParams:
    # Maske / morfoloji
    min_area:        int   = 400    # bu pikselin altindaki konturlar yok sayilir
    morph_kernel:    int   = 5      # acma/kapama icin kernel boyutu

    # Sekil filtresi (rotated bbox aspect — kola yakin perspektif icin gevsek)
    aspect_min:      float = 0.5
    aspect_max:      float = 2.0

    # Doluluk orani — contour alanı / rotated bbox alani
    fill_min:        float = 0.55   # bu degerin altinda concav/parcali sayilir

    # 4-kose acı toleransi (perspektif duslunce 90'dan sapma normal)
    corner_angle_min: float = 60.0
    corner_angle_max: float = 120.0

    # Confidence esigi (0..1) — bunun altindaki detection'lar dusurulur
    min_confidence:  float = 0.35

    # Isiklandirma normallestirme
    use_clahe:       bool  = True
    clahe_clip:      float = 2.0    # CLAHE clipLimit


@dataclass
class Presets:
    red:    RedThresholds   = field(default_factory=RedThresholds)
    blue:   BlueThresholds  = field(default_factory=BlueThresholds)
    common: CommonParams    = field(default_factory=CommonParams)


# =============================================================================
# Tespit veri yapisi
# =============================================================================

@dataclass
class Detection:
    color:      str                                    # "red" veya "blue"
    center:     Tuple[int, int]
    bbox:       Tuple[int, int, int, int]              # eksenle hizali x, y, w, h
    area:       int
    aspect:     float                                  # rotated bbox aspect (>=1)
    corners:    Optional[List[Tuple[int, int]]] = None # 4'lu polygon (varsa)
    confidence: float = 1.0                            # 0..1 squareness skoru


# =============================================================================
# Yardimcilar — CLAHE, geometri
# =============================================================================

def _apply_clahe(frame_bgr: np.ndarray, clip_limit: float) -> np.ndarray:
    """LAB L kanalina CLAHE — local kontrast artirir, isik dengesizligini azaltir."""
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
    l = clahe.apply(l)
    return cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)


def _post_process(mask: np.ndarray, kernel_size: int) -> np.ndarray:
    """Acma + kapama ile gurultuyu temizle."""
    k = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def _angle_at_corner(prev_pt, pt, next_pt) -> float:
    """Bir kose noktasinda olusan ic acı (derece). 90° kareler icin ideal."""
    v1 = (prev_pt[0] - pt[0], prev_pt[1] - pt[1])
    v2 = (next_pt[0] - pt[0], next_pt[1] - pt[1])
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    n1  = math.hypot(*v1)
    n2  = math.hypot(*v2)
    if n1 == 0 or n2 == 0:
        return 0.0
    cos_a = max(-1.0, min(1.0, dot / (n1 * n2)))
    return math.degrees(math.acos(cos_a))


def _evaluate_contour(cnt: np.ndarray, color_name: str,
                      common: CommonParams) -> Optional[Detection]:
    """Bir contour'un kare olma puanını hesapla. Detection veya None doner."""
    area = cv2.contourArea(cnt)
    if area < common.min_area:
        return None

    # 1. Rotasyon-bagimsiz bounding rect (perspektif/dondurulmus kareler icin)
    rect = cv2.minAreaRect(cnt)              # ((cx, cy), (w, h), angle)
    (_cx, _cy), (rw, rh), _ang = rect
    if rw < 4 or rh < 4:
        return None

    long_side  = max(rw, rh)
    short_side = min(rw, rh)
    aspect     = long_side / short_side       # her zaman >= 1
    if aspect > common.aspect_max:
        return None

    # 2. Doluluk orani — contour alanı / rotated bbox alani
    bbox_area = rw * rh
    fill = area / bbox_area
    if fill < common.fill_min:
        return None

    # 3. 4-kose yaklasim — perspektif distorsiyonu icin 3-6 arasi kabul, en iyi 4
    epsilon = 0.04 * cv2.arcLength(cnt, True)
    approx  = cv2.approxPolyDP(cnt, epsilon, True)
    n_corners = len(approx)

    corners: Optional[List[Tuple[int, int]]] = None
    angle_score = 0.5   # 4 kose bulunamazsa noter puan
    if 3 <= n_corners <= 6:
        if n_corners == 4:
            pts: List[Tuple[int, int]] = [
                (int(p[0][0]), int(p[0][1])) for p in approx
            ]
            angles = [
                _angle_at_corner(pts[(i - 1) % 4], pts[i], pts[(i + 1) % 4])
                for i in range(4)
            ]
            if all(common.corner_angle_min <= a <= common.corner_angle_max for a in angles):
                corners = pts
                dev = sum(abs(a - 90) for a in angles) / 4
                angle_score = max(0.0, 1.0 - dev / 30.0)   # 30° sapma → 0
            else:
                return None

    # 4. Confidence: aspect/fill/angle puanlarinin ortalamasi (0..1)
    aspect_score = max(0.0, 1.0 - (aspect - 1.0) / max(0.01, common.aspect_max - 1.0))
    fill_score   = min(1.0, fill / 0.95)
    confidence   = (aspect_score + fill_score + angle_score) / 3.0
    if confidence < common.min_confidence:
        return None

    # Eksenle hizali bbox — UI bbox cizimi ve merkez hesabi icin
    x, y, w, h = cv2.boundingRect(cnt)
    return Detection(
        color      = color_name,
        center     = (x + w // 2, y + h // 2),
        bbox       = (x, y, w, h),
        area       = int(area),
        aspect     = aspect,
        corners    = corners,
        confidence = confidence,
    )


def _detect_from_mask(mask: np.ndarray, color_name: str,
                       common: CommonParams) -> List[Detection]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out: List[Detection] = []
    for cnt in contours:
        det = _evaluate_contour(cnt, color_name, common)
        if det is not None:
            out.append(det)
    # Siralama: confidence × area — kucuk-ama-net kareyi buyuk-ama-bozuk kareye tercih et
    out.sort(key=lambda d: d.confidence * d.area, reverse=True)
    return out


# =============================================================================
# Renk tespit (CLAHE on isleme dahil)
# =============================================================================

def _prep(frame_bgr: np.ndarray, common: CommonParams) -> np.ndarray:
    if common.use_clahe:
        return _apply_clahe(frame_bgr, common.clahe_clip)
    return frame_bgr


def detect_red(frame_bgr: np.ndarray, t: RedThresholds, common: CommonParams):
    frame = _prep(frame_bgr, common)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv,
                     np.array([t.h1_low,  t.s_min, t.v_min]),
                     np.array([t.h1_high, 255,     255]))
    m2 = cv2.inRange(hsv,
                     np.array([t.h2_low,  t.s_min, t.v_min]),
                     np.array([t.h2_high, 255,     255]))
    mask = cv2.bitwise_or(m1, m2)
    mask = _post_process(mask, common.morph_kernel)
    return _detect_from_mask(mask, "red", common), mask


def detect_blue(frame_bgr: np.ndarray, t: BlueThresholds, common: CommonParams):
    frame = _prep(frame_bgr, common)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array([t.h_low,  t.s_min, t.v_min]),
                       np.array([t.h_high, 255,     255]))
    mask = _post_process(mask, common.morph_kernel)
    return _detect_from_mask(mask, "blue", common), mask


# =============================================================================
# Cizim
# =============================================================================

_RED_BGR  = (0,   0,   255)
_BLUE_BGR = (255, 100, 0)
_LABEL_TR = {"red": "KIRMIZI", "blue": "MAVI"}


def draw_detections(frame_bgr: np.ndarray, detections: List[Detection]) -> np.ndarray:
    out = frame_bgr.copy()
    for det in detections:
        color_bgr = _RED_BGR if det.color == "red" else _BLUE_BGR

        # Cizim: 4-koseli polygon varsa gercek kare cevresi, yoksa eksenle hizali bbox
        if det.corners is not None and len(det.corners) == 4:
            pts = np.array(det.corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(out, [pts], isClosed=True, color=color_bgr, thickness=2)
        else:
            x, y, w, h = det.bbox
            cv2.rectangle(out, (x, y), (x + w, y + h), color_bgr, 2)

        # Merkez nokta
        cv2.circle(out, det.center, 4, color_bgr, -1)

        # Etiket — kare adı + alan + confidence
        x, y, w, h = det.bbox
        label = (f"{_LABEL_TR.get(det.color, det.color.upper())}  "
                 f"{det.area}px  c={det.confidence:.2f}")
        font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.50, 2
        (tw, th), _ = cv2.getTextSize(label, font, scale, thick)

        ly = max(th + 6, y - 4)              # bbox ustune sigmazsa asagi kaydir
        cv2.rectangle(out, (x, ly - th - 6), (x + tw + 6, ly + 2), color_bgr, -1)
        cv2.putText(out, label, (x + 3, ly - 3), font, scale,
                    (255, 255, 255), thick, cv2.LINE_AA)
    return out


# =============================================================================
# Preset I/O — geriye uyumlu (eski JSON'lardaki eksik alanlar default'tan dolar)
# =============================================================================

def save_presets(presets: Presets, path: str):
    payload = {
        "red":    asdict(presets.red),
        "blue":   asdict(presets.blue),
        "common": asdict(presets.common),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def load_presets(path: str) -> Presets:
    if not os.path.exists(path):
        return Presets()
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return Presets(
            red    = RedThresholds(**data.get("red", {})),
            blue   = BlueThresholds(**data.get("blue", {})),
            common = _common_from_dict(data.get("common", {})),
        )
    except Exception as e:
        print(f"Preset yuklenemedi ({e}); default kullanilacak.")
        return Presets()


def _common_from_dict(d: dict) -> CommonParams:
    """Eski preset dosyalarinda yeni alanlar yok — sadece bilinen alanlari geçir."""
    defaults = CommonParams()
    valid_keys = set(defaults.__dict__.keys())
    filtered = {k: v for k, v in d.items() if k in valid_keys}
    return CommonParams(**{**asdict(defaults), **filtered})

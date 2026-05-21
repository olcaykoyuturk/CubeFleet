"""
Calibration Presets — PC tarafinda isimli kalibrasyon profillerini yonetir.

Akis:
1. AGV manuel kalibrasyonu bitirir → WS uzerinden 'calibrationData' mesajini PC'ye yollar
2. PC kullanicidan isim ister → preset olarak JSON'a kaydeder
3. Kullanici sonradan preset secebilir → PC 'applyCalibration' mesajiyla AGV'ye yollar
4. AGV reconnect olunca PC otomatik son aktif preset'i geri yukler (RAM'de uctugu icin)

Veri formati (pc/calibration_presets.json):

    {
      "presets": {
        "<preset_name>": {
            "createdAt": "2026-05-19T14:30:00",
            "agvSource": "AGV_1",
            "note":      "kullanici notu",
            "sensorMin": [int, int, ... 8 adet],
            "sensorMax": [int, int, ... 8 adet]
        },
        ...
      },
      "activePerAgv": { "AGV_1": "<preset_name>", "AGV_2": "..." }
    }
"""

from __future__ import annotations

import json
import os
import re
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional


SENSOR_COUNT = 8
MIN_RANGE    = 500   # firmware'deki CALIB_MIN_RANGE ile eslesir


@dataclass
class Preset:
    name:      str
    sensorMin: List[int]
    sensorMax: List[int]
    createdAt: str   = ""
    agvSource: str   = ""
    note:      str   = ""

    def is_valid(self) -> bool:
        """Tum sensorlerde yeterli kontrast var mi?"""
        if len(self.sensorMin) != SENSOR_COUNT or len(self.sensorMax) != SENSOR_COUNT:
            return False
        for lo, hi in zip(self.sensorMin, self.sensorMax):
            if hi - lo < MIN_RANGE:
                return False
        return True

    def to_dict(self) -> dict:
        return {
            "createdAt": self.createdAt,
            "agvSource": self.agvSource,
            "note":      self.note,
            "sensorMin": list(self.sensorMin),
            "sensorMax": list(self.sensorMax),
        }

    @classmethod
    def from_dict(cls, name: str, d: dict) -> "Preset":
        return cls(
            name      = name,
            sensorMin = list(d.get("sensorMin", []))[:SENSOR_COUNT],
            sensorMax = list(d.get("sensorMax", []))[:SENSOR_COUNT],
            createdAt = d.get("createdAt", ""),
            agvSource = d.get("agvSource", ""),
            note      = d.get("note", ""),
        )


class PresetStore:
    """JSON'a kalici, RAM'de hizli erisim. Tum mutasyonlardan sonra save()."""

    def __init__(self, json_path: str):
        self.path: str = json_path
        self.presets: Dict[str, Preset] = {}
        self.active_per_agv: Dict[str, str] = {}
        self.load()

    # ---- IO ---------------------------------------------------------------
    def load(self) -> None:
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                doc = json.load(f)
        except (OSError, json.JSONDecodeError):
            return

        self.presets = {
            name: Preset.from_dict(name, d)
            for name, d in (doc.get("presets") or {}).items()
        }
        self.active_per_agv = dict(doc.get("activePerAgv") or {})

    def save(self) -> None:
        doc = {
            "presets":      {n: p.to_dict() for n, p in self.presets.items()},
            "activePerAgv": dict(self.active_per_agv),
        }
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=2, ensure_ascii=False)
        os.replace(tmp, self.path)

    # ---- Mutasyonlar ------------------------------------------------------
    @staticmethod
    def normalize_name(raw: str) -> str:
        """Dosyaya / UI'a uygun ad: bosluklar _'e, ASCII disi karakter atilir."""
        s = re.sub(r"\s+", "_", raw.strip())
        s = re.sub(r"[^A-Za-z0-9_\-]", "", s)
        return s[:48]

    def add(self, name: str, sensorMin: List[int], sensorMax: List[int],
            agvSource: str = "", note: str = "") -> Preset:
        """Yeni preset olustur veya ayni isimli olani uzerine yaz."""
        clean = self.normalize_name(name) or f"preset_{int(time.time())}"
        p = Preset(
            name      = clean,
            sensorMin = list(sensorMin)[:SENSOR_COUNT],
            sensorMax = list(sensorMax)[:SENSOR_COUNT],
            createdAt = datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
            agvSource = agvSource,
            note      = note,
        )
        self.presets[clean] = p
        self.save()
        return p

    def delete(self, name: str) -> bool:
        if name not in self.presets:
            return False
        del self.presets[name]
        # Bu preset aktifse referansi temizle
        for agv_id, active in list(self.active_per_agv.items()):
            if active == name:
                del self.active_per_agv[agv_id]
        self.save()
        return True

    def set_active(self, agv_id: str, name: str) -> bool:
        if name not in self.presets:
            return False
        self.active_per_agv[agv_id] = name
        self.save()
        return True

    # ---- Sorgulama --------------------------------------------------------
    def get(self, name: str) -> Optional[Preset]:
        return self.presets.get(name)

    def active_for(self, agv_id: str) -> Optional[Preset]:
        name = self.active_per_agv.get(agv_id)
        if name is None:
            return None
        return self.presets.get(name)

    def names(self) -> List[str]:
        return sorted(self.presets.keys())

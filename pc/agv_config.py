"""
AGV Config Store — Her AGV icin kalici ayarlar (camera URL, vs).

Calibration preset'lerinden ayri ama benzer pattern: JSON dosyasi, atomik save,
runtime'da RAM'de hizli erisim. Her AGV'nin kendi ESP32-CAM'i oldugu icin
cam_url + cam_control_url gibi degerler AGV bazinda saklanir.

Format (pc/agv_config.json):

    {
      "agvs": {
        "AGV_1": {
            "cam_stream_url":  "http://192.168.4.50:81/stream",
            "cam_control_url": "http://192.168.4.50:80"
        },
        "AGV_2": {
            "cam_stream_url":  "http://192.168.4.51:81/stream",
            "cam_control_url": "http://192.168.4.51:80"
        }
      }
    }

NOT: active_preset zaten calibration_presets.json'da (PresetStore.activePerAgv).
Bunu burada duplike etmiyoruz — iki dosya farkli sorumlulukta:
  - calibration_presets.json: sensör kalibrasyon profilleri
  - agv_config.json:           AGV donanim/baglanti ayarlari (cam, vs)
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Dict, Optional


# AGV_N icin varsayilan IP eki — N=1 → 50, N=2 → 51, ... CAM_IP_OCT_4 ile uyumlu
DEFAULT_CAM_IP_BASE = 50
DEFAULT_CAM_STREAM_PORT  = 81
DEFAULT_CAM_CONTROL_PORT = 80


def default_cam_urls(agv_id: str) -> tuple[str, str]:
    """AGV_1, AGV_2, ... isminden default kamera URL'lerini tahmin et.
    Kullanici manuel override yapabilir."""
    try:
        n = int(agv_id.split("_")[-1])
    except (ValueError, IndexError):
        n = 1
    ip = f"192.168.4.{DEFAULT_CAM_IP_BASE + n - 1}"
    return (
        f"http://{ip}:{DEFAULT_CAM_STREAM_PORT}/stream",
        f"http://{ip}:{DEFAULT_CAM_CONTROL_PORT}",
    )


@dataclass
class AGVConfig:
    agv_id:           str
    cam_stream_url:   str = ""
    cam_control_url:  str = ""

    def to_dict(self) -> dict:
        return {
            "cam_stream_url":  self.cam_stream_url,
            "cam_control_url": self.cam_control_url,
        }

    @classmethod
    def from_dict(cls, agv_id: str, d: dict) -> "AGVConfig":
        return cls(
            agv_id          = agv_id,
            cam_stream_url  = d.get("cam_stream_url", ""),
            cam_control_url = d.get("cam_control_url", ""),
        )

    @classmethod
    def default_for(cls, agv_id: str) -> "AGVConfig":
        """Yeni AGV ilk kez gorunce default URL'lerle olustur."""
        s, c = default_cam_urls(agv_id)
        return cls(agv_id=agv_id, cam_stream_url=s, cam_control_url=c)


class AGVConfigStore:
    """JSON tabanli AGV config persistence."""

    def __init__(self, json_path: str):
        self.path: str = json_path
        self.agvs: Dict[str, AGVConfig] = {}
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
        self.agvs = {
            agv_id: AGVConfig.from_dict(agv_id, d)
            for agv_id, d in (doc.get("agvs") or {}).items()
        }

    def save(self) -> None:
        doc = {"agvs": {n: c.to_dict() for n, c in self.agvs.items()}}
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=2, ensure_ascii=False)
        os.replace(tmp, self.path)

    # ---- Sorgulama --------------------------------------------------------
    def get(self, agv_id: str) -> AGVConfig:
        """AGV config dondurur — yoksa default ile olusturup save eder."""
        if agv_id not in self.agvs:
            self.agvs[agv_id] = AGVConfig.default_for(agv_id)
            self.save()
        return self.agvs[agv_id]

    def update(self, agv_id: str, **fields) -> AGVConfig:
        """Belirli alanlari guncelle + kaydet."""
        cfg = self.get(agv_id)
        for k, v in fields.items():
            if hasattr(cfg, k):
                setattr(cfg, k, v)
        self.save()
        return cfg

    def remove(self, agv_id: str) -> bool:
        if agv_id not in self.agvs:
            return False
        del self.agvs[agv_id]
        self.save()
        return True

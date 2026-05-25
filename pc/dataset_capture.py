"""
dataset_capture.py — ESP32-CAM stream'inden YOLO dataset toplama UI.

Standalone tool — agv_control.py'den bagimsiz. Sadece bir CAM URL gerek.

Kullanim:
    & "c:\\Projelerim\\AGV\\.venv\\Scripts\\python.exe" `
        c:\\Projelerim\\AGV\\pc\\dataset_capture.py

Akis:
    1. Stream URL gir (default: http://192.168.4.50:81/stream)
    2. Cikti klasoru sec (default: ./dataset)
    3. "Yayin Ac" → frame'ler gelmeye baslar
    4. "Kare Cek" → tek frame kaydet
       VEYA "Auto" + interval saniye → otomatik periyodik kaydet
    5. Sayac + son kaydedilen dosya adi UI'de gosterilir

Dosya adi formati: cube_YYYYMMDD_HHMMSS_NNN.jpg
    YYYYMMDD_HHMMSS: capture zamani
    NNN: o klasordeki sira no (otomatik artar)

Roboflow uplomak icin: cikti klasorunu zip'le, roboflow.com → upload.
"""

from __future__ import annotations

import os
import sys
import time
import datetime
from pathlib import Path
from typing import Optional

import customtkinter as ctk
import cv2
from PIL import Image

# Same-dir imports
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from stream_reader import StreamReader  # noqa: E402


# --- Renkler / fontlar -------------------------------------------------------
COL_BG_DEEP   = "#0F1419"
COL_PANEL     = "#1A1F2E"
COL_SUCCESS   = "#22C55E"
COL_SUCCESS_H = "#16A34A"
COL_DANGER    = "#EF4444"
COL_DANGER_H  = "#DC2626"
COL_INFO      = "#3B82F6"
COL_INFO_H    = "#2563EB"
COL_MUTED     = "#94A3B8"

DEFAULT_URL = "http://192.168.4.50:81/stream"
DEFAULT_DIR = str(SCRIPT_DIR.parent / "dataset")


class DatasetCaptureApp(ctk.CTk):
    PREVIEW_W = 640
    PREVIEW_H = 480

    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        self.title("CubeFleet — Dataset Capture")
        self.geometry("1100x640")
        self.minsize(900, 560)
        self.configure(fg_color=COL_BG_DEEP)

        # State
        self.reader: Optional[StreamReader] = None
        self.last_frame = None              # son okunan numpy BGR frame
        self.last_count = 0                 # son okunan frame index'i
        self.saved_count = 0                # bu oturumda kaydedilen sayı
        self.last_saved: str = "—"          # son dosya adi
        self.session_seq = 1                # dosya numarasi
        self.auto_active = False
        self.last_auto_save = 0.0
        self._img_ref = None                # CTkImage GC korumasi

        self._build_ui()
        self._tick()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------ UI ---------------------------------------------------------------

    def _build_ui(self):
        root = ctk.CTkFrame(self, fg_color="transparent")
        root.pack(fill="both", expand=True, padx=12, pady=12)

        # Sol: preview
        left = ctk.CTkFrame(root, fg_color=COL_PANEL, corner_radius=8)
        left.pack(side="left", fill="both", expand=True, padx=(0, 8))

        ctk.CTkLabel(left, text="ESP32-CAM Akisi",
                     font=ctk.CTkFont(size=14, weight="bold")
                     ).pack(anchor="w", padx=12, pady=(8, 4))

        self.preview = ctk.CTkLabel(
            left, text="Akış kapalı", width=self.PREVIEW_W, height=self.PREVIEW_H,
            fg_color="#000000", text_color=COL_MUTED,
            font=ctk.CTkFont(size=12),
        )
        self.preview.pack(padx=12, pady=8)

        self.fps_lbl = ctk.CTkLabel(left, text="FPS: -   Frame: -",
                                     font=ctk.CTkFont(size=11),
                                     text_color=COL_MUTED, anchor="w")
        self.fps_lbl.pack(fill="x", padx=12, pady=(0, 12))

        # Sag: kontroller
        right = ctk.CTkFrame(root, fg_color=COL_PANEL, corner_radius=8, width=340)
        right.pack(side="left", fill="y")
        right.pack_propagate(False)

        ctk.CTkLabel(right, text="KONTROL",
                     font=ctk.CTkFont(size=14, weight="bold")
                     ).pack(anchor="w", padx=12, pady=(10, 6))

        # URL
        ctk.CTkLabel(right, text="Stream URL",
                     font=ctk.CTkFont(size=11), anchor="w"
                     ).pack(fill="x", padx=12)
        self.url_var = ctk.StringVar(value=DEFAULT_URL)
        ctk.CTkEntry(right, textvariable=self.url_var,
                     font=ctk.CTkFont(size=11)
                     ).pack(fill="x", padx=12, pady=(2, 8))

        # Output dir
        ctk.CTkLabel(right, text="Cikti Klasoru",
                     font=ctk.CTkFont(size=11), anchor="w"
                     ).pack(fill="x", padx=12)
        self.dir_var = ctk.StringVar(value=DEFAULT_DIR)
        ctk.CTkEntry(right, textvariable=self.dir_var,
                     font=ctk.CTkFont(size=11)
                     ).pack(fill="x", padx=12, pady=(2, 8))

        # Connect/disconnect
        btn_row = ctk.CTkFrame(right, fg_color="transparent")
        btn_row.pack(fill="x", padx=12, pady=(4, 12))
        self.btn_connect = ctk.CTkButton(
            btn_row, text="Yayin Ac", fg_color=COL_SUCCESS,
            hover_color=COL_SUCCESS_H, height=32,
            command=self._on_connect)
        self.btn_connect.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.btn_disconnect = ctk.CTkButton(
            btn_row, text="Kapat", fg_color="#374151",
            hover_color="#1F2937", height=32, state="disabled",
            command=self._on_disconnect)
        self.btn_disconnect.pack(side="left", fill="x", expand=True, padx=(4, 0))

        # Capture
        ctk.CTkLabel(right, text="YAKALAMA",
                     font=ctk.CTkFont(size=12, weight="bold")
                     ).pack(anchor="w", padx=12, pady=(8, 4))

        self.btn_capture = ctk.CTkButton(
            right, text="📷 Kare Cek (Space)", fg_color=COL_INFO,
            hover_color=COL_INFO_H, height=44,
            font=ctk.CTkFont(size=13, weight="bold"),
            command=self._capture_one, state="disabled")
        self.btn_capture.pack(fill="x", padx=12, pady=4)
        # Space tusu binding
        self.bind("<space>", lambda e: self._capture_one())

        # Auto-capture
        auto_row = ctk.CTkFrame(right, fg_color="transparent")
        auto_row.pack(fill="x", padx=12, pady=(8, 4))
        self.auto_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(auto_row, text="Auto",
                        variable=self.auto_var,
                        command=self._toggle_auto,
                        font=ctk.CTkFont(size=11)
                        ).pack(side="left")
        ctk.CTkLabel(auto_row, text="her",
                     font=ctk.CTkFont(size=11)
                     ).pack(side="left", padx=(8, 4))
        self.auto_interval = ctk.DoubleVar(value=2.0)
        ctk.CTkEntry(auto_row, textvariable=self.auto_interval,
                     width=50, font=ctk.CTkFont(size=11)
                     ).pack(side="left")
        ctk.CTkLabel(auto_row, text="sn",
                     font=ctk.CTkFont(size=11)
                     ).pack(side="left", padx=(4, 0))

        # Stats
        ctk.CTkLabel(right, text="DURUM",
                     font=ctk.CTkFont(size=12, weight="bold")
                     ).pack(anchor="w", padx=12, pady=(16, 4))

        stat_card = ctk.CTkFrame(right, fg_color="#0F1419", corner_radius=6)
        stat_card.pack(fill="x", padx=12)
        self.saved_lbl = ctk.CTkLabel(
            stat_card, text="Kaydedilen: 0",
            font=ctk.CTkFont(size=20, weight="bold"),
            text_color=COL_SUCCESS, anchor="w")
        self.saved_lbl.pack(fill="x", padx=10, pady=(8, 2))
        self.last_lbl = ctk.CTkLabel(
            stat_card, text="Son: —",
            font=ctk.CTkFont(size=10),
            text_color=COL_MUTED, anchor="w")
        self.last_lbl.pack(fill="x", padx=10, pady=(0, 8))

        # Status bar (alt)
        self.status_lbl = ctk.CTkLabel(
            right, text="Hazir", font=ctk.CTkFont(size=10),
            text_color=COL_MUTED, anchor="w")
        self.status_lbl.pack(side="bottom", fill="x", padx=12, pady=8)

    # ------ Bağlanma ---------------------------------------------------------

    def _on_connect(self):
        url = self.url_var.get().strip()
        if not url:
            self._status("URL bos", err=True)
            return
        self._status(f"Bağlaniyor: {url}")
        self.reader = StreamReader(url)
        if not self.reader.start():
            self._status(f"Bağlanti basarisiz: {url}", err=True)
            self.reader = None
            return
        self.btn_connect.configure(state="disabled")
        self.btn_disconnect.configure(state="normal")
        self.btn_capture.configure(state="normal")
        self._status("Bağlandi")

    def _on_disconnect(self):
        if self.reader is not None:
            try:
                self.reader.stop()
            except Exception:
                pass
            self.reader = None
        self.btn_connect.configure(state="normal")
        self.btn_disconnect.configure(state="disabled")
        self.btn_capture.configure(state="disabled")
        self.preview.configure(image=None, text="Akış kapalı")
        self._img_ref = None
        self._status("Kapatildi")

    # ------ Yakalama ---------------------------------------------------------

    def _ensure_dir(self) -> Optional[Path]:
        d = self.dir_var.get().strip()
        if not d:
            self._status("Cikti klasoru bos", err=True)
            return None
        path = Path(d)
        try:
            path.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            self._status(f"Klasor olusturma hata: {e}", err=True)
            return None
        return path

    def _next_filename(self, out_dir: Path) -> Path:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        # Dosya numarasi: klasordeki mevcut cube_*.jpg sayisi + 1
        while True:
            fname = f"cube_{ts}_{self.session_seq:03d}.jpg"
            full = out_dir / fname
            if not full.exists():
                return full
            self.session_seq += 1

    def _capture_one(self):
        if self.last_frame is None:
            self._status("Henuz frame yok", err=True)
            return
        out_dir = self._ensure_dir()
        if out_dir is None:
            return
        fpath = self._next_filename(out_dir)
        try:
            ok = cv2.imwrite(str(fpath), self.last_frame)
            if not ok:
                self._status(f"Yazma basarisiz: {fpath.name}", err=True)
                return
        except Exception as e:
            self._status(f"Hata: {e}", err=True)
            return
        self.saved_count += 1
        self.session_seq += 1
        self.last_saved = fpath.name
        self.saved_lbl.configure(text=f"Kaydedilen: {self.saved_count}")
        self.last_lbl.configure(text=f"Son: {fpath.name}")
        self._status(f"OK: {fpath.name}")

    def _toggle_auto(self):
        self.auto_active = self.auto_var.get()
        if self.auto_active:
            self.last_auto_save = time.time()
            self._status(f"Auto: her {self.auto_interval.get():.1f} sn")
        else:
            self._status("Auto kapatildi")

    # ------ Tick -------------------------------------------------------------

    def _tick(self):
        try:
            if self.reader is not None:
                frame, count = self.reader.read()
                if frame is not None:
                    self.last_frame = frame
                    self.last_count = count

                    # Preview goster
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    pil = Image.fromarray(rgb)
                    pil.thumbnail((self.PREVIEW_W, self.PREVIEW_H),
                                  Image.Resampling.LANCZOS)
                    img = ctk.CTkImage(light_image=pil, dark_image=pil,
                                       size=pil.size)
                    self.preview.configure(image=img, text="")
                    self._img_ref = img

                    self.fps_lbl.configure(
                        text=f"Frame: {count}   |   Boyut: {frame.shape[1]}x{frame.shape[0]}")

                # Auto-capture
                if self.auto_active:
                    try:
                        interval = max(0.1, float(self.auto_interval.get()))
                    except Exception:
                        interval = 2.0
                    if time.time() - self.last_auto_save >= interval:
                        self._capture_one()
                        self.last_auto_save = time.time()
        except Exception as e:
            self._status(f"Tick hata: {e}", err=True)
        self.after(33, self._tick)   # ~30 Hz UI

    # ------ Util -------------------------------------------------------------

    def _status(self, msg: str, err: bool = False):
        self.status_lbl.configure(
            text=msg,
            text_color=COL_DANGER if err else COL_MUTED)

    def _on_close(self):
        try:
            if self.reader is not None:
                self.reader.stop()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    app = DatasetCaptureApp()
    app.mainloop()

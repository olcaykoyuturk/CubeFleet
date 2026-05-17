"""
AGV Renk Tespit Uygulamasi (CustomTkinter UI)
==============================================
ESP32-CAM MJPEG akisini cekip kirmizi/mavi 3D kupleri tespit eder.

Slider'larla HSV esiklerini canli ayarla, "Kaydet" ile JSON'a yaz.
"""

import os
import time

import cv2
import customtkinter as ctk
import numpy as np
from PIL import Image

from detector import (
    Presets, RedThresholds, BlueThresholds, CommonParams,
    detect_red, detect_blue, draw_detections,
    save_presets, load_presets,
)
from stream_reader import StreamReader


CAM_URL_DEFAULT = "http://192.168.4.50:81/stream"
PRESET_PATH     = os.path.join(os.path.dirname(__file__), "hsv_presets.json")


# =============================================================================
# UI Yardimcilari
# =============================================================================

class LabelSlider(ctk.CTkFrame):
    """Slider + dinamik deger etiketi + sol etiket (tek satirda)."""

    def __init__(self, master, label, mn, mx, init, on_change=None, step=1):
        super().__init__(master, fg_color="transparent")
        self.on_change = on_change

        self.label = ctk.CTkLabel(self, text=label, width=70, anchor="w")
        self.label.grid(row=0, column=0, sticky="w", padx=(0, 4))

        steps = max(1, int((mx - mn) / step))
        self.slider = ctk.CTkSlider(self, from_=mn, to=mx, number_of_steps=steps,
                                    command=self._cb, width=180)
        self.slider.set(init)
        self.slider.grid(row=0, column=1, padx=4)

        self.value_lbl = ctk.CTkLabel(self, text=str(init), width=40, anchor="e")
        self.value_lbl.grid(row=0, column=2, sticky="e")

    def _cb(self, val):
        v = int(val)
        self.value_lbl.configure(text=str(v))
        if self.on_change is not None:
            self.on_change(v)

    def get(self):
        return int(self.slider.get())

    def set(self, v):
        self.slider.set(v)
        self.value_lbl.configure(text=str(int(v)))


# =============================================================================
# Ana Uygulama
# =============================================================================

class VisionApp(ctk.CTk):
    UI_TICK_MS = 30   # ~33 fps UI guncelleme

    def __init__(self):
        super().__init__()

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.title("AGV Renk Tespit")
        self.geometry("1100x680")
        self.minsize(1000, 620)

        # Durum
        self.presets:  Presets             = load_presets(PRESET_PATH)
        self.reader:   StreamReader | None = None
        self.show_red:    bool = True
        self.show_blue:   bool = True
        self.show_mask:   bool = False
        self.detections      = []
        self.last_count: int  = 0
        self.last_fps:   float = 0.0
        self.last_show_time   = time.time()

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self.UI_TICK_MS, self._tick)

    # -------------------------------------------------------------------------
    # Layout
    # -------------------------------------------------------------------------
    def _build_ui(self):
        # Ust seritte URL ve baglanti butonlari
        topbar = ctk.CTkFrame(self)
        topbar.pack(fill="x", padx=8, pady=8)

        ctk.CTkLabel(topbar, text="Stream:").pack(side="left", padx=(8, 4))
        self.url_var = ctk.StringVar(value=CAM_URL_DEFAULT)
        ctk.CTkEntry(topbar, textvariable=self.url_var, width=320).pack(side="left", padx=4)

        self.connect_btn    = ctk.CTkButton(topbar, text="Baglan",  width=80, command=self._connect)
        self.disconnect_btn = ctk.CTkButton(topbar, text="Kes",     width=60, command=self._disconnect, state="disabled")
        self.connect_btn.pack(side="left", padx=4)
        self.disconnect_btn.pack(side="left", padx=4)

        self.status_lbl = ctk.CTkLabel(topbar, text="Bagli degil", text_color="orange")
        self.status_lbl.pack(side="left", padx=12)

        # Govde: sol = video, sag = kontrol paneli
        body = ctk.CTkFrame(self)
        body.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        # ----- Sol: video alani -----
        left = ctk.CTkFrame(body)
        left.pack(side="left", fill="both", expand=True, padx=(0, 8))

        self.video_lbl = ctk.CTkLabel(left, text="Akis bekleniyor...", width=640, height=480,
                                      fg_color="#1a1a1a")
        self.video_lbl.pack(padx=8, pady=8, expand=True)

        info_row = ctk.CTkFrame(left, fg_color="transparent")
        info_row.pack(fill="x", padx=8, pady=(0, 8))
        self.info_lbl = ctk.CTkLabel(info_row, text="FPS: 0.0  |  -", anchor="w")
        self.info_lbl.pack(side="left")

        # ----- Sag: kontrol paneli (kaydirilabilir) -----
        right = ctk.CTkScrollableFrame(body, width=380, label_text="Ayarlar")
        right.pack(side="right", fill="y")

        # Renk acma/kapama + maske gosterimi
        toggles = ctk.CTkFrame(right, fg_color="transparent")
        toggles.pack(fill="x", pady=(4, 8))

        self.red_var  = ctk.BooleanVar(value=True)
        self.blue_var = ctk.BooleanVar(value=True)
        self.mask_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(toggles, text="Kirmizi", variable=self.red_var,
                        command=self._update_toggles).grid(row=0, column=0, padx=4, sticky="w")
        ctk.CTkCheckBox(toggles, text="Mavi",    variable=self.blue_var,
                        command=self._update_toggles).grid(row=0, column=1, padx=4, sticky="w")
        ctk.CTkCheckBox(toggles, text="Maske goster", variable=self.mask_var,
                        command=self._update_toggles).grid(row=0, column=2, padx=4, sticky="w")

        # ---- KIRMIZI ----
        red_frame = ctk.CTkFrame(right)
        red_frame.pack(fill="x", pady=4)
        ctk.CTkLabel(red_frame, text="KIRMIZI HSV", text_color="#ff5050",
                     font=ctk.CTkFont(weight="bold")).pack(pady=(4, 2))

        r = self.presets.red
        self.s_r_h1l = LabelSlider(red_frame, "H1 low",  0, 30,  r.h1_low)
        self.s_r_h1h = LabelSlider(red_frame, "H1 high", 0, 30,  r.h1_high)
        self.s_r_h2l = LabelSlider(red_frame, "H2 low",  150, 180, r.h2_low)
        self.s_r_h2h = LabelSlider(red_frame, "H2 high", 150, 180, r.h2_high)
        self.s_r_smin = LabelSlider(red_frame, "S min",  0, 255, r.s_min)
        self.s_r_vmin = LabelSlider(red_frame, "V min",  0, 255, r.v_min)
        for s in (self.s_r_h1l, self.s_r_h1h, self.s_r_h2l, self.s_r_h2h,
                  self.s_r_smin, self.s_r_vmin):
            s.pack(fill="x", padx=8, pady=2)

        ctk.CTkButton(red_frame, text="Kirmizi Kaydet", fg_color="#aa3030",
                      command=self._save_red).pack(pady=(4, 8), padx=8)

        # ---- MAVI ----
        blue_frame = ctk.CTkFrame(right)
        blue_frame.pack(fill="x", pady=4)
        ctk.CTkLabel(blue_frame, text="MAVI HSV", text_color="#5090ff",
                     font=ctk.CTkFont(weight="bold")).pack(pady=(4, 2))

        b = self.presets.blue
        self.s_b_hl   = LabelSlider(blue_frame, "H low",  60, 140, b.h_low)
        self.s_b_hh   = LabelSlider(blue_frame, "H high", 60, 140, b.h_high)
        self.s_b_smin = LabelSlider(blue_frame, "S min",  0, 255, b.s_min)
        self.s_b_vmin = LabelSlider(blue_frame, "V min",  0, 255, b.v_min)
        for s in (self.s_b_hl, self.s_b_hh, self.s_b_smin, self.s_b_vmin):
            s.pack(fill="x", padx=8, pady=2)

        ctk.CTkButton(blue_frame, text="Mavi Kaydet", fg_color="#3050aa",
                      command=self._save_blue).pack(pady=(4, 8), padx=8)

        # ---- ORTAK ----
        common_frame = ctk.CTkFrame(right)
        common_frame.pack(fill="x", pady=4)
        ctk.CTkLabel(common_frame, text="ORTAK FILTRELER",
                     font=ctk.CTkFont(weight="bold")).pack(pady=(4, 2))

        c = self.presets.common
        self.s_c_area  = LabelSlider(common_frame, "Min alan", 50,  5000, c.min_area, step=50)
        self.s_c_ar_lo = LabelSlider(common_frame, "AR min",   30,  100, int(c.aspect_min * 100), step=5)
        self.s_c_ar_hi = LabelSlider(common_frame, "AR max",   100, 250, int(c.aspect_max * 100), step=5)
        self.s_c_kern  = LabelSlider(common_frame, "Kernel",   1,   15,  c.morph_kernel)
        for s in (self.s_c_area, self.s_c_ar_lo, self.s_c_ar_hi, self.s_c_kern):
            s.pack(fill="x", padx=8, pady=2)

        ctk.CTkButton(common_frame, text="Tum Preset'leri Kaydet",
                      command=self._save_all).pack(pady=(4, 8), padx=8)

        # ---- Yardim ----
        info = ctk.CTkLabel(
            right,
            text="Ipuclari:\n"
                 "• Kirmizi OpenCV'de iki H araliginda (0-10 ve 170-180)\n"
                 "• S min'i artir -> sadece doygun renkler\n"
                 "• V min'i artir -> sadece parlak renkler\n"
                 "• AR (aspect ratio) 0.6-1.7 = yaklasik kare",
            justify="left", font=ctk.CTkFont(size=10),
            text_color="#aaaaaa", wraplength=340)
        info.pack(fill="x", padx=8, pady=8)

    # -------------------------------------------------------------------------
    # Olaylar
    # -------------------------------------------------------------------------
    def _update_toggles(self):
        self.show_red  = self.red_var.get()
        self.show_blue = self.blue_var.get()
        self.show_mask = self.mask_var.get()

    def _connect(self):
        if self.reader is not None:
            return
        self.reader = StreamReader(self.url_var.get())
        if not self.reader.start():
            self.reader = None
            self.status_lbl.configure(text="HATA: Baglanilamadi", text_color="red")
            return
        self.status_lbl.configure(text="Bagli", text_color="lightgreen")
        self.connect_btn.configure(state="disabled")
        self.disconnect_btn.configure(state="normal")
        self.last_count = 0

    def _disconnect(self):
        if self.reader is None:
            return
        self.reader.stop()
        self.reader = None
        self.status_lbl.configure(text="Bagli degil", text_color="orange")
        self.connect_btn.configure(state="normal")
        self.disconnect_btn.configure(state="disabled")

    def _on_close(self):
        if self.reader is not None:
            self.reader.stop()
        self.destroy()

    # -------------------------------------------------------------------------
    # Slider'lardan preset olusturma
    # -------------------------------------------------------------------------
    def _current_red(self) -> RedThresholds:
        return RedThresholds(
            h1_low  = self.s_r_h1l.get(),
            h1_high = self.s_r_h1h.get(),
            h2_low  = self.s_r_h2l.get(),
            h2_high = self.s_r_h2h.get(),
            s_min   = self.s_r_smin.get(),
            v_min   = self.s_r_vmin.get(),
        )

    def _current_blue(self) -> BlueThresholds:
        return BlueThresholds(
            h_low  = self.s_b_hl.get(),
            h_high = self.s_b_hh.get(),
            s_min  = self.s_b_smin.get(),
            v_min  = self.s_b_vmin.get(),
        )

    def _current_common(self) -> CommonParams:
        return CommonParams(
            min_area     = self.s_c_area.get(),
            aspect_min   = self.s_c_ar_lo.get() / 100.0,
            aspect_max   = self.s_c_ar_hi.get() / 100.0,
            morph_kernel = max(1, self.s_c_kern.get()),
        )

    def _save_red(self):
        self.presets.red    = self._current_red()
        self.presets.common = self._current_common()
        save_presets(self.presets, PRESET_PATH)
        self._toast("Kirmizi preset kaydedildi")

    def _save_blue(self):
        self.presets.blue   = self._current_blue()
        self.presets.common = self._current_common()
        save_presets(self.presets, PRESET_PATH)
        self._toast("Mavi preset kaydedildi")

    def _save_all(self):
        self.presets = Presets(
            red    = self._current_red(),
            blue   = self._current_blue(),
            common = self._current_common(),
        )
        save_presets(self.presets, PRESET_PATH)
        self._toast("Tum preset'ler kaydedildi")

    def _toast(self, text):
        self.status_lbl.configure(text=text, text_color="lightgreen")
        self.after(2000, lambda: self.status_lbl.configure(
            text="Bagli" if self.reader else "Bagli degil",
            text_color=("lightgreen" if self.reader else "orange")
        ))

    # -------------------------------------------------------------------------
    # Ana dongu (UI tick)
    # -------------------------------------------------------------------------
    def _tick(self):
        try:
            self._tick_once()
        except Exception as e:
            print("Tick hata:", e)
        self.after(self.UI_TICK_MS, self._tick)

    def _tick_once(self):
        if self.reader is None:
            return

        frame, count = self.reader.read()
        if frame is None:
            return

        # FPS hesabi (kaynaktan gelen frame'lere gore)
        now = time.time()
        if now - self.last_show_time >= 0.5:
            self.last_fps = (count - self.last_count) / (now - self.last_show_time)
            self.last_count     = count
            self.last_show_time = now

        # Tespit
        red_dets,  red_mask  = ([], None)
        blue_dets, blue_mask = ([], None)
        common = self._current_common()

        if self.show_red:
            red_dets, red_mask = detect_red(frame, self._current_red(), common)
        if self.show_blue:
            blue_dets, blue_mask = detect_blue(frame, self._current_blue(), common)

        all_dets = red_dets + blue_dets

        # Maske gosterimi modu
        if self.show_mask:
            combined = np.zeros(frame.shape[:2], dtype=np.uint8)
            if red_mask is not None:
                combined = cv2.bitwise_or(combined, red_mask)
            if blue_mask is not None:
                combined = cv2.bitwise_or(combined, blue_mask)
            display = cv2.cvtColor(combined, cv2.COLOR_GRAY2BGR)
            display = draw_detections(display, all_dets)
        else:
            display = draw_detections(frame, all_dets)

        # Bilgi etiketi
        red_n, blue_n = len(red_dets), len(blue_dets)
        info = (f"FPS: {self.last_fps:.1f}  |  Kirmizi: {red_n}  Mavi: {blue_n}  "
                f"|  Toplam frame: {count}")
        if all_dets:
            top = all_dets[0]
            info += f"  |  En buyuk {top.color}: ({top.center[0]},{top.center[1]}) {top.area}px"
        self.info_lbl.configure(text=info)

        # Tk'ye uyumlu PIL imaji ve gosterim
        rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(rgb)

        # Video alanini frame boyutuna gore olcekle (UI'yi smasmaz)
        target_w, target_h = 640, 480
        pil.thumbnail((target_w, target_h), Image.LANCZOS)
        ctk_img = ctk.CTkImage(light_image=pil, dark_image=pil, size=pil.size)
        self.video_lbl.configure(image=ctk_img, text="")
        self.video_lbl._image = ctk_img   # GC engelle


# =============================================================================
# Entry point
# =============================================================================

if __name__ == "__main__":
    VisionApp().mainloop()

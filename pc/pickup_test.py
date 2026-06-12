"""
OTONOM KAPMA TEST ARACI — bagimsiz teshis UI'si.

agv_control.py'den BAGIMSIZ calisir: ESP32-CAM'e dogrudan baglanir
(WS server gerekmez). Amac: kapma sekansini adim adim izleyip sorunu bulmak.

Iceren:
  * Canli MJPEG + YOLO overlay + KINEMATIK dogrulama katmani:
      - macenta arti = MIKNATISIN YERE IZDUSUMU (FK'den hesaplanir — kol
        inerse miknatis oraya iner; arti kupun ustundeyse model dogru)
      - kupun HESAPLANAN dunya konumu (cm) sol ustte (cetvelle kontrol edilebilir)
  * 4 servo slider (kolu konumlamak icin — kontrolorun baslangic pozu buradan)
  * ▶ BASLAT / ⏹ DURDUR / 🛑 ACIL DUR (Esc) — acil dur: freeze + miknatis OFF
  * ⏯ ADIM MODU + detayli log; her kosu pc/pickup_logs/run_*.jsonl'a TAM kayit
  * KINEMATIK kalibrasyon paneli (olculer + 4 referans + odak)

Kontrolor: pickup_controller.PickupController v4 (GERCEK KINEMATIK):
  AIM (kup dunya konumu olculur, IK ile hover pozu hesaplanir, dogrulanir) ->
  DESCEND (tip_z cm cm dusurulur, IK her adimda; miknatis 2 cm'de ACIK;
  buton sonlandirir) -> GRABBED.

KALIBRASYON SIRASI (bir kez, ~5 dk):
  1) arm_sim sinir sihirbazi (guvenli zarf)
  2) OLCULER (cetvel, cm): L1 omuz->dirsek, L2 dirsek->bilek, L3 bilek->miknatis
     ucu, H0 omuz yerden, R0 omuz-base yatay ofset, kup kenari -> 💾 Olculeri kaydet
  3) REFERANSLAR (kolu tarif edilen duruma getir, butona bas):
     📐 L1 DIK (omuz uzvu dik yukari) / 📐 KOL DUZ (L2, L1 dogrultusunda) /
     📐 MIKNATIS ASAGI (miknatis yuzu tam asagi) / 📐 BASE ILERI (tam karsi)
  4) ODAK: kupu kameradan olculen mesafeye koy (orn. 20 cm), 🎯 Odak kalibre
  5) DOGRULAMA: macenta arti gercekten miknatisin altindaki noktada mi; kup
     koyunca yazilan (x, y) cm cetvelle tutuyor mu. 💾 Config Kaydet.

Calistir:  & ".venv\\Scripts\\python.exe" pc\\pickup_test.py
"""

from __future__ import annotations

import json
import os
import queue
import threading
import time
import urllib.request

import customtkinter as ctk
import cv2
from PIL import Image

from arm_kinematics import (ArmModel, DEFAULTS as KIN_DEFAULTS,
                            point_from_height, calibrate_camera)
from detector import get_detector
from pickup_controller import PickupController, interp_band, gr_band
from stream_reader import StreamReader

PICKUP_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pickup_config.json")
DATASET_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "dataset2")   # repo koku / dataset2
AGV = "TEST"   # tek kol — sabit anahtar

SERVO_NAMES = ("Base", "Shoulder", "Elbow", "Gripper")
SERVO_HOME = (93, 120, 60, 110)


class PickupTest(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("OTONOM KAPMA — Test & Teşhis")
        self.geometry("1280x760")
        ctk.set_appearance_mode("dark")

        # --- PickupController'in bekledigi app arayuzu ---
        self.detection_state: dict = {}
        self.grip_state: dict = {}
        self.arm_state: dict = {}   # /poll arm: servos(anlik)+targets — gercek settle
        self.pickup_cfg: dict = self._load_cfg()
        self.arm_servo_vars: dict = {AGV: [ctk.IntVar(value=v) for v in SERVO_HOME]}
        self.servo_sliders: dict = {}   # idx -> (slider, label, isim)

        self.cam_url = "http://192.168.4.50"
        self.reader: StreamReader | None = None
        self.detect_on = False
        self.pc: PickupController | None = None
        self.step_mode = ctk.BooleanVar(value=False)
        self._img_ref = None
        self._poll_stop = threading.Event()
        self._poll_thread = None
        self._last_seq = 0
        self._grip_last_seq = 0

        self._raw_frame = None      # son HAM kare (overlay'siz) — dataset cekimi
        self._shot_n = 0
        self._last_auto_shot = 0.0

        # SIRALI HTTP kuyrugu — komut sirasi garantili (bkz. _http)
        self._http_q: queue.Queue = queue.Queue()
        threading.Thread(target=self._http_worker, daemon=True).start()

        self._build()
        self.bind("<Escape>", lambda e: self.estop())
        self.bind("<space>", lambda e: self._capture_shot())
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(80, self._loop)
        auto = self.pickup_cfg.get("autonomous", {}) or {}
        env = auto.get("safe_envelope") or {}
        self.log("✅ güvenli bölge kayıtlı" if env.get("sh_grid")
                 else "⚠ güvenli bölge YOK — önce arm_sim sihirbazı")
        miss = ArmModel(auto.get("kinematics") or {}).missing()
        self.log("✅ kinematik kalibrasyon hazır" if not miss
                 else "⚠ KİNEMATİK eksik: " + ", ".join(miss) +
                      " — ölçüleri gir + 📐 referansları yakala")

    # ------------------------------------------------------------------ cfg
    def _load_cfg(self) -> dict:
        try:
            with open(PICKUP_CFG, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def _save_cfg(self):
        # ATOMIK yazim (tmp + os.replace): agv_control da ayni dosyaya yazar;
        # yarida kesilen json.dump dosyayi bozuyordu.
        try:
            tmp = PICKUP_CFG + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self.pickup_cfg, f, indent=2, ensure_ascii=False)
            os.replace(tmp, PICKUP_CFG)
            self.log(f"💾 config kaydedildi → {PICKUP_CFG}")
        except Exception as e:
            self.log(f"❌ config kaydedilemedi: {e}")

    # ------------------------------------------------------------------ UI
    def _build(self):
        left = ctk.CTkScrollableFrame(self, width=330)
        left.pack(side="left", fill="y", padx=6, pady=6)
        right = ctk.CTkFrame(self, fg_color="transparent")
        right.pack(side="right", fill="both", expand=True, padx=6, pady=6)

        # SAG: video + log
        self.video = ctk.CTkLabel(right, text="Yayın kapalı", width=720, height=480,
                                  fg_color="#111")
        self.video.pack(side="top", pady=(0, 4))
        self.logbox = ctk.CTkTextbox(right, font=("Consolas", 11),
                                     fg_color="#101010", text_color="#ccc")
        self.logbox.pack(side="bottom", fill="both", expand=True)
        self.logbox.configure(state="disabled")

        # SOL: baglanti
        ctk.CTkLabel(left, text="📷 ESP32-CAM", font=("", 13, "bold")
                     ).pack(anchor="w", pady=(2, 2))
        self.url = ctk.CTkEntry(left)
        self.url.insert(0, self.cam_url)
        self.url.pack(fill="x", padx=2)
        row = ctk.CTkFrame(left, fg_color="transparent")
        row.pack(fill="x", pady=2)
        self.stream_btn = ctk.CTkButton(row, text="📡 Yayını Aç", fg_color="#46a",
                                        command=self._toggle_stream)
        self.stream_btn.pack(side="left", expand=True, fill="x", padx=(0, 2))
        self.detect_btn = ctk.CTkButton(row, text="🎯 Tespit: KAPALI", fg_color="#444",
                                        command=self._toggle_detect)
        self.detect_btn.pack(side="left", expand=True, fill="x", padx=(2, 0))

        # SOL: servolar
        ctk.CTkLabel(left, text="SERVOLAR (kolu buradan konumla)",
                     font=("", 13, "bold")).pack(anchor="w", pady=(10, 2))
        for i, name in enumerate(SERVO_NAMES):
            self._slider(left, i, name)
        mrow = ctk.CTkFrame(left, fg_color="transparent")
        mrow.pack(fill="x", pady=2)
        ctk.CTkButton(mrow, text="🧲 Mıknatıs AÇ", fg_color="#3a7", width=100,
                      command=lambda: self._http("/magnet?s=1", log=True)
                      ).pack(side="left", expand=True, fill="x", padx=(0, 2))
        ctk.CTkButton(mrow, text="Mıknatıs KAPAT", fg_color="#444", width=100,
                      command=lambda: self._http("/magnet?s=0", log=True)
                      ).pack(side="left", expand=True, fill="x", padx=(2, 0))

        lrow = ctk.CTkFrame(left, fg_color="transparent")
        lrow.pack(fill="x", pady=2)
        ctk.CTkLabel(lrow, text="💡 LED", width=60).pack(side="left")
        self.led_lbl = ctk.CTkLabel(lrow, text="0", width=34)
        ls = ctk.CTkSlider(lrow, from_=0, to=255, number_of_steps=255,
                           command=self._set_led)
        ls.set(0)
        ls.pack(side="left", expand=True, fill="x", padx=4)
        self.led_lbl.pack(side="left")

        # SOL: dataset cekimi (ham kare -> dataset2/)
        ctk.CTkLabel(left, text="DATASET (dataset2/)", font=("", 13, "bold")
                     ).pack(anchor="w", pady=(10, 2))
        dsrow = ctk.CTkFrame(left, fg_color="transparent")
        dsrow.pack(fill="x", pady=2)
        ctk.CTkButton(dsrow, text="📸 Kare Çek (Space)", fg_color="#46a",
                      command=self._capture_shot).pack(side="left", expand=True,
                                                       fill="x", padx=(0, 2))
        self.auto_shot = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(dsrow, text="Auto 5sn", variable=self.auto_shot, width=80
                        ).pack(side="left", padx=(2, 0))

        # SOL: arama pozu — servolari kayitli goruntuleme pozuna otomatik getirir
        ctk.CTkLabel(left, text="ARAMA MODU", font=("", 13, "bold")
                     ).pack(anchor="w", pady=(10, 2))
        prow = ctk.CTkFrame(left, fg_color="transparent")
        prow.pack(fill="x", pady=2)
        ctk.CTkButton(prow, text="🔍 ARAMA POZUNA GİT", fg_color="#3a7",
                      command=self._goto_search).pack(side="left", expand=True,
                                                      fill="x", padx=(0, 2))
        ctk.CTkButton(prow, text="📍 Bu pozu kaydet", fg_color="#46a", width=110,
                      command=self._save_search).pack(side="left", padx=(2, 0))

        # SOL: KINEMATIK KALIBRASYON — adim adim SIHIRBAZ (tek dugme)
        ctk.CTkLabel(left, text="KİNEMATİK KALİBRASYON",
                     font=("", 13, "bold")).pack(anchor="w", pady=(10, 2))
        ctk.CTkButton(left, text="🧙 KALİBRASYON SİHİRBAZI", height=36,
                      fg_color="#7a4fbf", hover_color="#623da0",
                      command=self._open_wizard).pack(fill="x", pady=2)
        self.kin_status = ctk.CTkLabel(left, text="", font=("", 11),
                                       text_color="#ffd24a", justify="left",
                                       wraplength=300)
        self.kin_status.pack(anchor="w", pady=(2, 0))
        self._refresh_kin_status()
        ctk.CTkButton(left, text="💾 Config Kaydet",
                      command=self._save_cfg).pack(fill="x", pady=2)

        # SOL: kontrol
        ctk.CTkLabel(left, text="OTONOM KAPMA", font=("", 13, "bold")
                     ).pack(anchor="w", pady=(10, 2))
        crow = ctk.CTkFrame(left, fg_color="transparent")
        crow.pack(fill="x", pady=2)
        ctk.CTkButton(crow, text="▶ BAŞLAT", fg_color="#3fbf66",
                      command=self.start).pack(side="left", expand=True,
                                               fill="x", padx=(0, 2))
        ctk.CTkButton(crow, text="⏹ DURDUR", fg_color="#777",
                      command=self.stop).pack(side="left", expand=True,
                                              fill="x", padx=(2, 0))
        ctk.CTkButton(left, text="🛑 ACİL DUR (Esc) — freeze + mıknatıs OFF",
                      fg_color="#c62828", hover_color="#a32020", height=40,
                      command=self.estop).pack(fill="x", pady=(4, 2))

        srow = ctk.CTkFrame(left, fg_color="transparent")
        srow.pack(fill="x", pady=2)
        ctk.CTkCheckBox(srow, text="⏯ Adım modu", variable=self.step_mode,
                        command=self._on_step_mode).pack(side="left", padx=2)
        ctk.CTkButton(srow, text="⏭ TEK ADIM", fg_color="#46a", width=110,
                      command=self._single_tick).pack(side="left", expand=True,
                                                      fill="x", padx=2)

        # MIKNATIS MERKEZ INCE AYAR: koşu sonrası mıknatıs küpün ortasına tam
        # inmiyorsa, indiği YÖNE bas (mıknatısı oraya kaydırır), buton küpün
        # tam üstüne gelene kadar 0.3'er nudge'la. Çalışan kontrolöre de uygulanır.
        ctk.CTkLabel(left, text="🎯 MIKNATIS MERKEZ AYARI (mm)",
                     font=("", 12, "bold")).pack(anchor="w", pady=(8, 0))
        self.trim_lbl = ctk.CTkLabel(left, text="", font=("", 11),
                                     text_color="#9c9")
        self.trim_lbl.pack(anchor="w")
        trow = ctk.CTkFrame(left, fg_color="transparent")
        trow.pack(fill="x", pady=1)
        ctk.CTkButton(trow, text="⬆ İleri", width=70, fg_color="#46a",
                      command=lambda: self._trim("aim_trim_fwd", 0.3)
                      ).pack(side="left", expand=True, fill="x", padx=1)
        ctk.CTkButton(trow, text="⬇ Geri", width=70, fg_color="#46a",
                      command=lambda: self._trim("aim_trim_fwd", -0.3)
                      ).pack(side="left", expand=True, fill="x", padx=1)
        ctk.CTkButton(trow, text="⬅ Sol", width=70, fg_color="#46a",
                      command=lambda: self._trim("aim_trim_lat", -0.3)
                      ).pack(side="left", expand=True, fill="x", padx=1)
        ctk.CTkButton(trow, text="➡ Sağ", width=70, fg_color="#46a",
                      command=lambda: self._trim("aim_trim_lat", 0.3)
                      ).pack(side="left", expand=True, fill="x", padx=1)
        ctk.CTkButton(left, text="↺ sıfırla", width=70, fg_color="#555",
                      command=self._trim_reset).pack(anchor="w", pady=1)
        # GRIP ACI: miknatis egik oturup kupun gerisine dusuyorsa bilek acisini
        # ayarla (1° adim). "yukarı" = miknatis ucu kalkar (efektif aci artar).
        grow = ctk.CTkFrame(left, fg_color="transparent")
        grow.pack(fill="x", pady=1)
        ctk.CTkLabel(grow, text="grip açı", width=56).pack(side="left")
        ctk.CTkButton(grow, text="⬆ yukarı", width=80, fg_color="#46a",
                      command=lambda: self._trim("grab_gamma_off", 1.0)
                      ).pack(side="left", expand=True, fill="x", padx=1)
        ctk.CTkButton(grow, text="⬇ aşağı", width=80, fg_color="#46a",
                      command=lambda: self._trim("grab_gamma_off", -1.0)
                      ).pack(side="left", expand=True, fill="x", padx=1)
        self._refresh_trim()

        self.status = ctk.CTkLabel(left, text="durum: IDLE", font=("", 12, "bold"),
                                   text_color="#ffd24a", justify="left",
                                   wraplength=300)
        self.status.pack(anchor="w", pady=(6, 2))

    def _trim(self, key: str, delta: float):
        """Mıknatıs merkez trim'i: indiği yöne bas (mıknatısı oraya kaydırır).
        Config'e yazar, çalışan kontrolöre de anında uygular, kalıcı kaydet için
        💾 gerekir değil — direkt diske yazalım ki koşular arası kalsın."""
        auto = self.pickup_cfg.setdefault("autonomous", {})
        auto[key] = round(float(auto.get(key, 0) or 0) + delta, 2)
        if self.pc is not None:
            self.pc._cfg[key] = auto[key]
        self._refresh_trim()
        self._save_cfg()
        self.log(f"🎯 {key} = {auto[key]:+g} cm")

    def _trim_reset(self):
        auto = self.pickup_cfg.setdefault("autonomous", {})
        for k in ("aim_trim_fwd", "aim_trim_lat", "grab_gamma_off"):
            auto[k] = 0.0
            if self.pc is not None:
                self.pc._cfg[k] = 0.0
        self._refresh_trim()
        self._save_cfg()
        self.log("🎯 trim + grip açısı sıfırlandı")

    def _refresh_trim(self):
        auto = self.pickup_cfg.get("autonomous", {}) or {}
        f = float(auto.get("aim_trim_fwd", 0) or 0)
        la = float(auto.get("aim_trim_lat", 0) or 0)
        ga = float(auto.get("grab_gamma_off", 0) or 0)
        self.trim_lbl.configure(
            text=f"ileri {f:+g} cm   yan {la:+g} cm (sağ +)   grip {ga:+g}°")

    def _slider(self, parent, idx, name):
        var = self.arm_servo_vars[AGV][idx]
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", pady=1)
        lbl = ctk.CTkLabel(row, text=f"{name}: {var.get()}", width=110, anchor="w")
        lbl.pack(side="left")

        def apply(v):
            # Elle slider/+/- — GUVENLI ZARF uygulanir (yasak kombinasyon engellenir)
            self._manual_servo(idx, round(v))

        ctk.CTkButton(row, text="−", width=24,
                      command=lambda: apply(s.get() - 1)).pack(side="left", padx=(0, 1))
        s = ctk.CTkSlider(row, from_=0, to=180, number_of_steps=180, command=apply)
        s.set(var.get())
        s.pack(side="left", expand=True, fill="x", padx=1)
        ctk.CTkButton(row, text="+", width=24,
                      command=lambda: apply(s.get() + 1)).pack(side="left", padx=(1, 0))
        self.servo_sliders[idx] = (s, lbl, name)

    def _sync_sliders_quiet(self):
        """Slider + var + label'i kolun gercek pozuna esitle — HTTP GONDERMEZ
        (kol zaten orada; sadece UI'nin bayat kalmasini onler)."""
        st = self.arm_state.get(AGV) or {}
        tgt = st.get("targets") or (self.pc.cmd if self.pc is not None else None)
        if not tgt:
            return
        for i, v in enumerate(tgt[:4]):
            v = max(0, min(180, int(v)))
            s, lbl, name = self.servo_sliders[i]
            s.set(v)
            self.arm_servo_vars[AGV][i].set(v)
            lbl.configure(text=f"{name}: {v}")
        self.log(f"🔄 slider'lar kol pozuna eşitlendi: {list(tgt[:4])}")

    def _set_servo(self, idx: int, val: int):
        """Programatik servo: slider + label + var + gercek kol birlikte."""
        val = max(0, min(180, int(round(val))))
        self._set_slider_widget(idx, val)
        self._http(f"/servo?id={idx}&a={val}")

    def _set_slider_widget(self, idx: int, v: int):
        """Slider + var + label gunceller — HTTP GONDERMEZ."""
        s, lbl, name = self.servo_sliders[idx]
        s.set(v)
        self.arm_servo_vars[AGV][idx].set(v)
        lbl.configure(text=f"{name}: {v}")

    # ------------------------------------------------- GUVENLI ZARF (manuel)
    def _envelope(self):
        env = (self.pickup_cfg.get("autonomous", {}) or {}).get("safe_envelope")
        if env and env.get("type") == "limits" and env.get("sh_grid"):
            return env
        return None

    def _repair_pose(self, pose):
        """elbow'u shoulder bandina, gripper'i (sh,el) bandina kistir. Band
        tanimsizsa (olculmemis/yasak bolge) None. Zarf yoksa pose aynen."""
        env = self._envelope()
        if env is None:
            return list(pose)
        m = float(env.get("margin_deg", 0))
        sh, el, gr = pose[1], pose[2], pose[3]
        eb = interp_band(env["sh_grid"], env["el_min"], env["el_max"], sh)
        if eb is None:
            return None
        lo, hi = eb[0] + m, eb[1] - m
        if lo > hi:
            return None
        el = max(lo, min(hi, el))
        gb = gr_band(env.get("gr_rows") or [], sh, el)
        if gb is None:
            return None
        glo, ghi = gb[0] + m, gb[1] - m
        if glo > ghi:
            return None
        gr = max(glo, min(ghi, gr))
        out = list(pose)
        out[2] = round(el)
        out[3] = round(gr)
        return out

    def _manual_servo(self, idx: int, v: int):
        """Elle servo komutu — GUVENLI ZARF: yeni poz yasak kombinasyona
        giriyorsa elbow/gripper banda kistirilir; band tanimsiz/yasaksa hareket
        REDDEDILIR (slider eski degerine doner). base zarfa dahil degil (serbest)."""
        v = max(0, min(180, round(v)))
        cur = [self.arm_servo_vars[AGV][i].get() for i in range(4)]
        if v == cur[idx]:
            return
        cand = list(cur)
        cand[idx] = v
        rep = self._repair_pose(cand)
        if rep is None:
            self._set_slider_widget(idx, cur[idx])   # reddet, eski degere don
            self.log(f"⛔ yasak bölge: {SERVO_NAMES[idx]} {v}° engellendi "
                     f"(ölçülmemiş/çarpışma bölgesi)")
            return
        if rep[idx] != v:   # moved servo banda kistirildi
            self.log(f"⛔ zarf sınırı: {SERVO_NAMES[idx]} {v}°→{rep[idx]}°")
        changed = [i for i in range(4) if rep[i] != cur[i]]
        for i in changed:
            self._set_slider_widget(i, rep[i])
            self._http(f"/servo?id={i}&a={rep[i]}")
        drag = [SERVO_NAMES[i] for i in changed if i != idx]
        if drag:
            self.log(f"↪ zarf kıstı: {', '.join(drag)} otomatik ayarlandı")

    # ------------------------------------------------------------------ arama pozu
    SEARCH_POSE_DEFAULT = [104, 76, 124, 21]

    def _search_pose(self) -> list:
        auto = self.pickup_cfg.get("autonomous", {}) or {}
        pose = auto.get("search_pose") or self.SEARCH_POSE_DEFAULT
        return [max(0, min(180, int(v))) for v in pose[:4]]

    def _goto_search(self):
        if self.pc is not None and self.pc.active:
            self.log("⚠ Sekans aktifken arama pozuna gidilmez — önce ⏹ DURDUR.")
            return
        pose = self._search_pose()
        # once shoulder (carpisma onleme — HOME ile ayni mantik), sonra digerleri
        self._set_servo(1, pose[1])
        for i in (0, 2, 3):
            self._set_servo(i, pose[i])
        self.log(f"🔍 ARAMA pozuna gidiliyor: {pose} (servo ramping ile)")

    def _save_search(self):
        pose = [v.get() for v in self.arm_servo_vars[AGV]]
        auto = self.pickup_cfg.setdefault("autonomous", {})
        auto["search_pose"] = pose
        self.log(f"📍 arama pozu = {pose} (💾 Config Kaydet ile kalıcı olur)")

    # ------------------------------------------------------------------ log
    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        try:
            self.logbox.configure(state="normal")
            self.logbox.insert("end", f"[{ts}] {msg}\n")
            self.logbox.see("end")
            self.logbox.configure(state="disabled")
        except Exception:
            pass

    # --- PickupController'in cagirdigi app arayuzu ---
    def _pickup_log(self, _agv: str, msg: str):
        self.log(f"🤖 {msg}")

    def _set_status(self, msg: str):
        pass   # log zaten ayni bilgiyi tasiyor

    def _pickup_ui_refresh(self, _agv: str = ""):
        c = self.pc
        # Kosu bitince slider'lari KOLUN GERCEK pozuna esitle (HTTP yok) —
        # kontrolor servolari dogrudan surdugu icin slider'lar bayat kaliyordu
        # ve bir sonraki start yanlis pozdan sayiyordu (saha 09:45).
        active = c is not None and c.active
        if getattr(self, "_was_active", False) and not active:
            self._sync_sliders_quiet()
        self._was_active = active
        if c is None:
            self.status.configure(text="durum: IDLE")
            return
        live = c.live or {}
        extra = (f"\nküp=({live.get('x', '-')}, {live.get('y', '-')}) cm  "
                 f"h={live.get('h', '-')} cm  px=({live.get('u', '-')}, "
                 f"{live.get('v', '-')})")
        self.status.configure(text=f"durum: {c.state}\n{c.message}{extra}")

    def _arm_http_get(self, _agv: str, path: str):
        self._http(path)

    # ------------------------------------------------------------------ http
    def _http(self, path: str, log: bool = False):
        """Komutu SIRALI kuyruga koy. Eski duzen her komutu ayri thread'le
        yolluyordu -> art arda iki /servo komutu YER DEGISTIREBILIYORDU
        (saha 09:36: elbow a=97 once varip a=95 onu ezdi, settle timeout).
        Tek worker thread sirayi garanti eder. URL cagri aninda (UI thread)
        cozulur — worker tkinter widget'a dokunmaz."""
        base = (self.url.get().strip() or self.cam_url).rstrip("/")
        self._http_q.put((f"{base}{path}", path, log))

    def _http_worker(self):
        while True:
            url, path, log = self._http_q.get()
            try:
                with urllib.request.urlopen(url, timeout=1.5) as r:
                    r.read()
                if log:
                    self.after(0, lambda p=path: self.log(f"→ {p} OK"))
            except Exception as e:
                self.after(0, lambda p=path, err=e: self.log(f"⚠ HTTP hata {p}: {err}"))

    # ------------------------------------------------------------------ stream
    def _toggle_stream(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
            self._poll_stop.set()
            self.stream_btn.configure(text="📡 Yayını Aç", fg_color="#46a")
            self.video.configure(text="Yayın kapalı", image=None)
            self._img_ref = None
            self.log("📡 yayın kapatıldı")
            return
        base = (self.url.get().strip() or self.cam_url).rstrip("/")
        self.cam_url = base
        host = base.split("//")[-1].split("/")[0].split(":")[0]
        self.reader = StreamReader(f"http://{host}:81/")
        self.reader.start()
        self.stream_btn.configure(text="📡 Yayını Kapat", fg_color="#a33")
        self.log(f"📡 yayın açılıyor: http://{host}:81/")
        # grip/log poll thread
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(target=self._poll_loop,
                                             args=(host,), daemon=True)
        self._poll_thread.start()

    def _toggle_detect(self):
        self.detect_on = not self.detect_on
        self.detect_btn.configure(
            text=f"🎯 Tespit: {'AÇIK' if self.detect_on else 'KAPALI'}",
            fg_color="#3fbf66" if self.detect_on else "#444")
        if not self.detect_on:
            self.detection_state.pop(AGV, None)

    # ------------------------------------------------------------------ poll
    def _poll_loop(self, host: str):
        """Grip mikroswitch olaylari + CAM loglari. Kontrolor aktifken 0.25 sn."""
        while not self._poll_stop.is_set():
            try:
                url = f"http://{host}/poll?logSince={self._last_seq}"
                with urllib.request.urlopen(url, timeout=3.0) as r:
                    data = json.loads(r.read().decode("utf-8", errors="replace"))
                arm = data.get("arm", {}) or {}
                # GERCEK SETTLE verisi: servos=anlik, targets=hedef — kontrolor
                # "hareket bitti mi" sorusunu bundan okur (tek dict atamasi).
                srv, tgt = arm.get("servos"), arm.get("targets")
                if (isinstance(srv, list) and isinstance(tgt, list)
                        and len(srv) == 4 and len(tgt) == 4):
                    self.arm_state[AGV] = {"servos": [int(x) for x in srv],
                                           "targets": [int(x) for x in tgt],
                                           "ts": time.time()}
                gseq = int(arm.get("gripEventSeq", 0))
                gtype = str(arm.get("gripEvent", "none"))
                if gseq > self._grip_last_seq and gtype in ("held", "lost"):
                    self._grip_last_seq = gseq
                    self.grip_state[AGV] = {"type": gtype, "seq": gseq,
                                            "ts": time.time()}
                    icon = "🎯 KÜP TUTULDU" if gtype == "held" else "❗ KÜP DÜŞTÜ"
                    self.after(0, lambda t=icon, s=gseq: self.log(f"{t} (seq={s})"))
                blk = data.get("log", {}) or {}
                for e in sorted(blk.get("entries", []) or [],
                                key=lambda x: int(x.get("seq", 0))):
                    si = int(e.get("seq", 0))
                    if si <= self._last_seq:
                        continue
                    self._last_seq = si
                    self.after(0, lambda m=str(e.get("msg", "")):
                               self.log(f"[CAM] {m}"))
                cur = int(blk.get("seq", 0))
                if cur > self._last_seq:
                    self._last_seq = cur
            except Exception:
                pass
            fast = self.pc is not None and self.pc.active
            self._poll_stop.wait(0.15 if fast else 1.0)   # tick_ms=150 ile hizali

    # ------------------------------------------------------------------ kontrol
    def start(self):
        if self.reader is None or not self.detect_on:
            self.log("⚠ Önce 📡 yayını ve 🎯 tespiti aç.")
            return
        # Diskten taze yukle (arm_sim bu arada yeni zarf kaydetmis olabilir);
        # UI'da degistirilen ayar anahtarlarini uzerine bindir.
        fresh = self._load_cfg()
        auto = fresh.setdefault("autonomous", {})
        ui = self.pickup_cfg.get("autonomous", {}) or {}
        for k in ("search_pose", "kinematics"):
            if k in ui:
                auto[k] = ui[k]
        self.pickup_cfg = fresh
        self.pc = PickupController(self, AGV)
        ok = self.pc.start()
        self.log("▶ BAŞLATILDI" if ok else f"❌ başlamadı: {self.pc.message}")
        if ok and self.step_mode.get():
            self.pc._cancel()
            self.log("⏯ adım modu: ⏭ TEK ADIM ile ilerle")

    def stop(self):
        if self.pc is not None:
            self.pc.stop()
            self.log("⏹ durduruldu")

    def estop(self):
        if self.pc is not None:
            self.pc.estop()
        self._http("/arm/freeze", log=True)
        self._http("/magnet?s=0", log=True)
        self.log("🛑 ACİL DUR — servolar donduruldu, mıknatıs KAPALI")

    def _on_step_mode(self):
        if self.pc is None:
            return
        if self.step_mode.get():
            self.pc._cancel()
            self.log("⏯ adım modu AÇIK — otomatik tick durdu, ⏭ ile ilerle")
        elif self.pc.active:
            self.pc._schedule()
            self.log("⏯ adım modu KAPALI — otomatik tick devam")

    def _single_tick(self):
        if self.pc is None or not self.pc.active:
            self.log("⚠ aktif sekans yok (▶ BAŞLAT)")
            return
        self.pc._cancel()
        self.pc._tick()
        if self.step_mode.get():
            self.pc._cancel()   # _tick yeniden zamanladiysa iptal — adim adim kal

    # ------------------------------------------------------------------ ana dongu
    def _loop(self):
        try:
            self._frame_tick()
        except Exception as e:
            self.log(f"❌ döngü hatası: {e}")
        self.after(80, self._loop)

    def _frame_tick(self):
        if self.reader is None:
            return
        frame, _ = self.reader.read()
        if frame is None:
            if self.reader.status in ("failed", "connecting"):
                self.video.configure(text=f"({self.reader.status}) {self.cam_url}:81",
                                     image=None)
            return
        # HAM kare (overlay'siz) — dataset cekimi bundan kaydeder
        self._raw_frame = frame.copy()
        if self.auto_shot.get() and time.time() - self._last_auto_shot >= 5.0:
            self._last_auto_shot = time.time()
            self._capture_shot()
        auto = self.pickup_cfg.get("autonomous", {}) or {}
        best = None
        if self.detect_on:
            det = get_detector()
            dets = det.detect(frame)
            best = det.best(dets)
            if best is not None:
                self.detection_state[AGV] = {
                    "cx": best.cx, "cy": best.cy, "area": best.area,
                    "conf": best.conf, "h": best.height, "w": best.x2 - best.x1,
                    "x1": best.x1, "y1": best.y1, "x2": best.x2, "y2": best.y2,
                    "frame_w": frame.shape[1], "frame_h": frame.shape[0],
                    "ts": time.time(),
                }
            else:
                self.detection_state.pop(AGV, None)
            frame = det.draw(frame, dets)
        # KINEMATIK OVERLAY — kalibrasyonu otonomi olmadan gozle dogrulama:
        #  * macenta arti = MIKNATISIN YERE IZDUSUMU (FK + projeksiyon): kol
        #    inerse miknatis buraya iner. Arti kupun ustundeyse model dogru.
        #  * tespit varsa sol ust koseye kupun HESAPLANAN dunya konumu (cm)
        #    yazilir (isin-zemin kesisimi) — cetvelle kontrol edilebilir.
        h, w = frame.shape[:2]
        model = ArmModel(auto.get("kinematics") or {})
        if not model.missing():
            servos = [self.arm_servo_vars[AGV][i].get() for i in range(4)]
            st = self.arm_state.get(AGV) or {}
            if st.get("targets"):
                servos = st["targets"]
            cube_cm = float(model.c.get("cube_cm") or 4.0)   # type: ignore[arg-type]
            mx, my, _mz = model.magnet_world(servos)
            pj = model.project_point(servos, mx, my, cube_cm)
            if pj is not None and 0 <= pj[0] < w and 0 <= pj[1] < h:
                cv2.drawMarker(frame, (int(pj[0]), int(pj[1])), (255, 0, 255),
                               cv2.MARKER_CROSS, 18, 2)
            if best is not None:
                est = model.cube_from_pixel(servos, best.cx, best.cy,
                                            z_plane=cube_cm / 2.0)
                if est is not None:
                    cv2.putText(frame, f"kup=({est[0]:.1f},{est[1]:.1f})cm",
                                (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 0, 255), 1, cv2.LINE_AA)

        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(img)
        scale = min(720 / w, 480 / h)
        pil = pil.resize((int(w * scale), int(h * scale)))
        self._img_ref = ctk.CTkImage(light_image=pil, dark_image=pil,
                                     size=pil.size)
        self.video.configure(image=self._img_ref, text="")
        self._pickup_ui_refresh()

    # ------------------------------------------------------------------ kinematik
    def _kin(self) -> dict:
        return {**KIN_DEFAULTS,
                **((self.pickup_cfg.get("autonomous", {}) or {})
                   .get("kinematics") or {})}

    def _open_wizard(self):
        if getattr(self, "_wizard", None) is not None:
            try:
                self._wizard.focus()
                return
            except Exception:
                self._wizard = None
        self._wizard = CalibWizard(self)

    # kind -> (pts anahtari, servo index, sabit aci | None=hesaplanir, aciklama)
    REF_KINDS = {
        "sh90": ("sh_pts", 1, 90.0, "L1 tam DİK yukarı"),
        "sh0":  ("sh_pts", 1, 0.0, "L1 tam YATAY ileri"),
        "el0":  ("el_pts", 2, None, "kol DÜZ (L2, L1 doğrultusunda)"),
        "elh":  ("el_pts", 2, None, "L2 tam YATAY (L1 nerede olursa)"),
        "grd":  ("gr_pts", 3, None, "mıknatıs yüzü TAM AŞAĞI"),
        "grf":  ("gr_pts", 3, None, "mıknatıs ucu TAM İLERİ (yüz dik)"),
        "b0":   ("base_pts", 0, 0.0, "base tam İLERİ"),
        "b90":  ("base_pts", 0, 90.0, "base tam SAĞA dönük (90°)"),
    }

    def _add_ref_point(self, key: str, servo: float, angle: float, desc: str):
        """[servo, aci] noktasini ekle/yenile (ayni servoda ±2 -> guncelle)."""
        kin = (self.pickup_cfg.setdefault("autonomous", {})
               .setdefault("kinematics", {}))
        pts = kin.setdefault(key, [])
        for i, p in enumerate(pts):
            if abs(float(p[0]) - servo) <= 2:
                pts[i] = [servo, angle]
                break
        else:
            pts.append([servo, angle])
        self.log(f"📐 {desc}: servo={servo:g} → açı={angle:g}° "
                 f"({key} {len(pts)} nokta; 💾 ile kalıcı)")
        self._refresh_kin_status()

    def _capture_ref(self, kind: str):
        """Referans NOKTASI yakala: kolu tarif edilen fiziksel durusa getir,
        bas — [servo, aci] cifti eklenir. Servo birimi derece OLMADIGI icin
        her eklemde EN AZ IKI nokta gerekir (egim). Hesaplanan acilar onceki
        fitlere dayanir — sira: L1 -> dirsek -> bilek. Paralelkenar kolda
        (el_abs) dirsek noktalari MUTLAK on kol acisi tutar."""
        key, idx, angle, desc = self.REF_KINDS[kind]
        val = self.arm_servo_vars[AGV][idx].get()
        m = ArmModel(self._kin())
        el_abs = bool(self._kin().get("el_abs"))
        if angle is None:
            s1 = self.arm_servo_vars[AGV][1].get()
            s2 = self.arm_servo_vars[AGV][2].get()
            if kind in ("el0", "elh"):
                # el0: beta_abs = alpha (kol duz) — elh: beta_abs = 0 (yatay)
                need_alpha = (kind == "el0") or not el_abs
                if need_alpha and m.maps["sh"] is None:
                    self.log("⚠ Önce L1'in iki referansını kaydet.")
                    return
                alpha = m.alpha(s1) if m.maps["sh"] is not None else 0.0
                babs = alpha if kind == "el0" else 0.0
                angle = round(babs if el_abs else babs - alpha, 2)
            else:   # grd / grf — beta_abs gerekli
                if m.maps["sh"] is None or m.maps["el"] is None:
                    self.log("⚠ Önce L1 ve dirsek referanslarını kaydet.")
                    return
                beta = m.beta_abs(s1, s2)
                angle = round((-90.0 if kind == "grd" else 0.0) - beta, 2)
        self._add_ref_point(key, val, angle, desc)

    def _clear_refs(self):
        kin = (self.pickup_cfg.setdefault("autonomous", {})
               .setdefault("kinematics", {}))
        for k in ("sh_pts", "el_pts", "gr_pts", "base_pts"):
            kin[k] = []
        self._refresh_kin_status()
        self.log("🗑 tüm açı referansları silindi (💾 ile kalıcı olur)")

    HEIGHT_HELP = {"sh": ("sh_pts", "DİRSEK mafsalı"),
                   "el": ("el_pts", "BİLEK mafsalı"),
                   "gr": ("gr_pts", "MIKNATIS UCU")}

    def _capture_height_val(self, joint: str, z: float):
        """YUKSEKLIKTEN referans (sihirbaz + ana panel ortak): kol su anki
        pozundayken ilgili noktanin YERDEN yuksekligi z'den aci hesaplanir
        (sin(aci)=dz/L). 'Kol tam yatay/dik olmuyor' problemini cozer.
        Donus: (basari, mesaj)."""
        key, what = self.HEIGHT_HELP[joint]
        servos = [self.arm_servo_vars[AGV][i].get() for i in range(4)]
        res = point_from_height(ArmModel(self._kin()), joint, servos, z)
        if isinstance(res, str):
            return False, res
        servo, angle = res
        self._add_ref_point(key, servo, angle, f"{what} {z:g} cm")
        return True, f"{what} {z:g} cm → açı={angle:g}° (servo={servo:g})"

    def _calib_focal_val(self, dist: float):
        """Odak kalibrasyonu (ortak): kup OLCULEN mesafede + tespitliyken
        cam_f = bh_px * mesafe / kup_kenari. Donus: (basari, mesaj)."""
        d = self.detection_state.get(AGV)
        if not d:
            return False, "Önce küp tespit edilmeli (🎯 Tespit aç)"
        kin = (self.pickup_cfg.setdefault("autonomous", {})
               .setdefault("kinematics", {}))
        cube = float(self._kin().get("cube_cm") or 4.0)
        bh = float(d.get("h") or 0)
        if bh <= 0 or dist <= 0:
            return False, "geçersiz ölçüm (bh veya mesafe 0)"
        kin["cam_f"] = round(bh * dist / cube, 1)
        self._refresh_kin_status()
        return True, (f"odak: bh={bh:.0f}px @ {dist:g}cm, küp {cube:g}cm → "
                      f"cam_f={kin['cam_f']} px")

    def _refresh_kin_status(self):
        m = ArmModel(self._kin())
        miss = m.missing()
        fits = []
        for j in ("sh", "el", "gr", "base"):
            n = len(m.pts(j))
            fit = m.maps[j]
            fits.append(f"{j}:{n}" + (f"(k={fit[0]:.2f})" if fit else ""))
        if miss:
            self.kin_status.configure(
                text="⚠ eksik: " + ", ".join(miss) + "\n" + "  ".join(fits),
                text_color="#ffd24a")
        else:
            self.kin_status.configure(
                text=("✅ hazır — " + "  ".join(fits)
                      + f"  f={float(self._kin().get('cam_f') or 0):g}px"),
                text_color="#7c7")

    def _set_led(self, val):
        """Flash LED parlakligi (0-255) — debounce: surukleme bitince SON deger
        gonderilir (slider spam'i ESP32'yi bogmasin)."""
        v = round(float(val))
        self.led_lbl.configure(text=str(v))
        prev = getattr(self, "_led_after", None)
        if prev is not None:
            try:
                self.after_cancel(prev)
            except Exception:
                pass
        self._led_after = self.after(120, lambda: self._http(f"/led?b={v}", log=True))

    # ------------------------------------------------------------------ dataset
    def _capture_shot(self):
        """Son HAM kareyi (overlay'siz) dataset2/ klasorune kaydet."""
        if self._raw_frame is None:
            self.log("⚠ Kare yok — önce 📡 yayını aç.")
            return
        try:
            os.makedirs(DATASET_DIR, exist_ok=True)
            self._shot_n += 1
            name = f"cube_{time.strftime('%Y%m%d_%H%M%S')}_{self._shot_n:03d}.jpg"
            path = os.path.join(DATASET_DIR, name)
            cv2.imwrite(path, self._raw_frame)
            self.log(f"📸 {name} kaydedildi ({self._shot_n}. kare → dataset2/)")
        except Exception as e:
            self.log(f"❌ kare kaydedilemedi: {e}")

    def _on_close(self):
        try:
            if self.pc is not None:
                self.pc.estop()
            self._poll_stop.set()
            if self.reader is not None:
                self.reader.stop()
        except Exception:
            pass
        self.destroy()


class CalibWizard(ctk.CTkToplevel):
    """Adim adim KINEMATIK kalibrasyon sihirbazi.

    NON-MODAL: ana penceredeki servo slider'lari acik kalir — kullanici kolu
    her adimda elle konumlar, sihirbaz o anki servo degerini okuyup referans
    cifti uretir. Her adim TEK is anlatir, dogrular ve ilerlemeye izin verir.

    Servo birimi DERECE DEGIL (PWM + mekanik) -> her eklem icin IKI fiziksel
    durus yakalanir, aci = k*servo + b uydurmasiyla cikar. Aci hesaplanan
    duruslar (L2 yatay, mik. asagi...) onceki eklemlerin fitine dayanir; bu
    yuzden SIRA: olculer -> kol tipi -> L1 -> dirsek -> bilek -> base -> odak."""

    def __init__(self, app: "PickupTest"):
        super().__init__(app)
        self.app = app
        self.title("🧙 Kinematik Kalibrasyon Sihirbazı")
        self.geometry("560x600")
        self.transient(app)
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.i = 0
        self.steps = [
            self._s_intro, self._s_measures, self._s_armtype,
            self._s_l1, self._s_elbow, self._s_wrist, self._s_base,
            self._s_focal, self._s_verify,
        ]
        self.meas_entries: dict = {}
        self.h_entry: ctk.CTkEntry | None = None
        self.cx_entry: ctk.CTkEntry | None = None
        self.cy_entry: ctk.CTkEntry | None = None
        # kamera ornekleri config'te KALICI -> sihirbazi tekrar acinca durur,
        # uzerine eklenip yeniden cozulebilir
        self.cam_samples: list = [list(s) for s in
                                  (self._kin().get("cam_samples") or [])]
        self.cam_rms = None

        head = ctk.CTkFrame(self, fg_color="transparent")
        head.pack(fill="x", padx=12, pady=(10, 0))
        self.step_lbl = ctk.CTkLabel(head, text="", font=("", 13, "bold"),
                                     text_color="#b89cff")
        self.step_lbl.pack(side="left")
        self.live_lbl = ctk.CTkLabel(head, text="", font=("Consolas", 11),
                                     text_color="#8c8")
        self.live_lbl.pack(side="right")

        self.body = ctk.CTkScrollableFrame(self, fg_color="#1a1a1a")
        self.body.pack(fill="both", expand=True, padx=12, pady=8)

        nav = ctk.CTkFrame(self, fg_color="transparent")
        nav.pack(fill="x", padx=12, pady=(0, 10))
        self.back_btn = ctk.CTkButton(nav, text="← Geri", width=80,
                                      fg_color="#555", command=self._back)
        self.back_btn.pack(side="left")
        # her an diske kaydet (sihirbazi bitirmeden de) — uygulama kapaninca ucmaz
        ctk.CTkButton(nav, text="💾 Kaydet", width=84, fg_color="#46a",
                      command=self._save_now).pack(side="left", padx=4)
        self.status = ctk.CTkLabel(nav, text="", font=("", 11), wraplength=260)
        self.status.pack(side="left", padx=8)
        self.next_btn = ctk.CTkButton(nav, text="İleri →", width=100,
                                      fg_color="#3fbf66", command=self._next)
        self.next_btn.pack(side="right")

        self._tick()
        self._render()

    # ---------------------------------------------------------------- yardim
    def _kin(self) -> dict:
        return (self.app.pickup_cfg.setdefault("autonomous", {})
                .setdefault("kinematics", {}))

    def _model(self) -> ArmModel:
        return ArmModel(self.app._kin())

    def _servos(self) -> list:
        return [self.app.arm_servo_vars[AGV][i].get() for i in range(4)]

    def _tick(self):
        """Canli servo okuma — kol nerede oldugunu hep gosterir."""
        try:
            s = self._servos()
            st = self.app.arm_state.get(AGV) or {}
            real = st.get("servos")
            txt = f"servo: {s}"
            if real:
                txt += f"\nkol:   {list(real)}"
            self.live_lbl.configure(text=txt)
        except Exception:
            pass
        self._after = self.after(200, self._tick)

    def _save_now(self):
        """Sihirbazi bitirmeden DİSKE kaydet — her an basilabilir."""
        self.app._save_cfg()
        self.app._refresh_kin_status()
        self.status.configure(text="💾 diske kaydedildi", text_color="#7c7")

    def _clear_body(self):
        for w in self.body.winfo_children():
            w.destroy()

    def _para(self, text, color="#ccc", size=12, pad=(0, 4)):
        ctk.CTkLabel(self.body, text=text, justify="left", wraplength=500,
                     font=("", size), text_color=color).pack(anchor="w", pady=pad)

    def _joint_ready(self, key: str) -> bool:
        pts = self._kin().get(key) or []
        if len(pts) < 2:
            return False
        sp = max(float(p[0]) for p in pts) - min(float(p[0]) for p in pts)
        return sp >= 15.0

    # ---------------------------------------------------------------- akis
    def _render(self):
        self._clear_body()
        self.step_lbl.configure(text=f"Adım {self.i + 1}/{len(self.steps)}")
        self.back_btn.configure(state="normal" if self.i > 0 else "disabled")
        ready, hint = self.steps[self.i]()
        self.status.configure(text=hint or "",
                              text_color="#7c7" if ready else "#ffd24a")
        last = self.i == len(self.steps) - 1
        self.next_btn.configure(
            text="✅ Bitir & Kaydet" if last else "İleri →",
            state="normal" if ready else "disabled",
            fg_color="#3fbf66" if ready else "#444")

    def _next(self):
        if self.i < len(self.steps) - 1:
            self.i += 1
            self._render()
        else:
            self.app._save_cfg()
            self.app._refresh_kin_status()
            self.app.log("✅ kalibrasyon sihirbazı tamamlandı + kaydedildi")
            self._close()

    def _back(self):
        if self.i > 0:
            self.i -= 1
            self._render()

    def _close(self):
        try:
            self.after_cancel(self._after)
        except Exception:
            pass
        self.app._wizard = None
        self.destroy()

    def _cap_btn(self, parent, text, cmd):
        ctk.CTkButton(parent, text=text, fg_color="#3a7", command=cmd
                      ).pack(side="left", expand=True, fill="x", padx=2)

    def _do(self, fn):
        """Yakalama sarmalayici: calistir, sonucu durum satirina yaz, yeniden ciz."""
        try:
            r = fn()
        except Exception as e:
            r = (False, str(e))
        if isinstance(r, tuple):
            ok, msg = r
            self.app.log(("✔ " if ok else "⚠ ") + msg)
        self._render()

    # ---------------------------------------------------------------- adimlar
    def _s_intro(self):
        self._para("Bu sihirbaz kolu FİZİKSEL olarak modeller: cetvelle ölçüp "
                   "birkaç duruş yakalarsın, gerisini matematik hesaplar.",
                   "#fff", 13)
        self._para("\nÖnemli: servo değeri (0–180) GERÇEK AÇI DEĞİLDİR. Bu yüzden "
                   "her eklem için İKİ bilinen duruş yakalarız; aradaki doğru "
                   "her servo komutunu gerçek açıya çevirir.")
        self._para("\nHazırlık: ana pencerede 📡 Yayın + 🎯 Tespit açık olsun "
                   "(odak adımı küpü görmen gerekir). Kolu slider'larla "
                   "konumlandıracaksın — bu pencere açık kalır.", "#8af")
        self._para("\nSüre ~5 dk. İstediğin adıma Geri/İleri ile dönebilirsin.",
                   "#999")
        return True, "Başlamak için İleri'ye bas"

    def _s_measures(self):
        self._para("1) ÖLÇÜLER — cetvelle ölç, cm gir:", "#fff", 13)
        defs = [
            ("L1", "omuz mafsalı → dirsek mafsalı"),
            ("L2", "dirsek mafsalı → bilek mafsalı"),
            ("L3", "bilek mafsalı → MIKNATIS UCU (buton yüzeyi)"),
            ("H0", "omuz mafsalı → ZEMİN (küplerin durduğu yüzey!) dikey"),
            ("R0", "omuz mafsalı → base dönme ekseni yatay (üstten bak)"),
            ("cube_cm", "küp kenarı"),
        ]
        kin = self._kin()
        self.meas_entries = {}
        for k, desc in defs:
            row = ctk.CTkFrame(self.body, fg_color="transparent")
            row.pack(fill="x", pady=2)
            ctk.CTkLabel(row, text="küp" if k == "cube_cm" else k,
                         width=42, anchor="w").pack(side="left")
            e = ctk.CTkEntry(row, width=64)
            v = kin.get(k, self.app._kin().get(k, ""))
            e.insert(0, "" if v in (None, "") else f"{float(v):g}")
            e.pack(side="left", padx=(0, 8))
            e.bind("<KeyRelease>", lambda _e: self._save_measures())
            ctk.CTkLabel(row, text=desc, anchor="w", font=("", 11),
                         text_color="#999").pack(side="left")
            self.meas_entries[k] = e
        m = self.app._kin()
        need = [k for k in ("L1", "L2", "L3", "H0")
                if not (float(m.get(k) or 0) > 0)]
        ok = not need
        return ok, ("kaydedildi ✔" if ok
                    else "gerekli (>0): " + ", ".join(need))

    def _save_measures(self):
        kin = self._kin()
        for k, e in self.meas_entries.items():
            try:
                kin[k] = float(e.get().replace(",", "."))
            except Exception:
                kin.pop(k, None)
        self.app._refresh_kin_status()
        # nav durumunu guncelle (yeniden cizmeden — entry odagi kacmasin)
        m = self.app._kin()
        ok = all(float(m.get(k) or 0) > 0 for k in ("L1", "L2", "L3", "H0"))
        self.status.configure(text="kaydedildi ✔" if ok else "L1,L2,L3,H0 > 0 olmalı",
                              text_color="#7c7" if ok else "#ffd24a")
        self.next_btn.configure(state="normal" if ok else "disabled",
                                fg_color="#3fbf66" if ok else "#444")

    def _s_armtype(self):
        self._para("2) KOL TİPİ — bu testi yap:", "#fff", 13)
        self._para("\nSADECE shoulder slider'ını oynat (dirsek/bilek'e dokunma). "
                   "Ön kola (L2) bak:")
        self._para("• Ön kolun yere göre EĞİMİ değişmiyorsa → PARALELKENAR kol\n"
                   "• Ön kol shoulder ile birlikte dönüyorsa → KLASİK kol", "#8af")
        cur = bool(self._kin().get("el_abs"))
        self._para(f"\nŞu an: {'PARALELKENAR' if cur else 'KLASİK'}", "#fc6")
        row = ctk.CTkFrame(self.body, fg_color="transparent")
        row.pack(fill="x", pady=6)

        def setp(v):
            self._kin()["el_abs"] = v
            self.app.log(f"kol tipi: {'paralelkenar' if v else 'klasik'}")
            self._render()
        ctk.CTkButton(row, text="Klasik kol", fg_color="#46a" if not cur else "#3a7",
                      command=lambda: setp(False)).pack(side="left", expand=True,
                                                        fill="x", padx=2)
        ctk.CTkButton(row, text="Paralelkenar", fg_color="#46a" if cur else "#3a7",
                      command=lambda: setp(True)).pack(side="left", expand=True,
                                                       fill="x", padx=2)
        return True, "tipi seç (emin değilsen Klasik)"

    def _joint_step(self, title, key, joint, angle_btns, height_label):
        """Ortak 2-nokta eklem adimi: aci-poz butonlari + yukseklikten yakalama."""
        self._para(title, "#fff", 13)
        self._para("Kolu tarif edilen duruşa getir (slider) ve yakala. İKİ farklı "
                   "duruş gerekli (servo değerleri ≥15 birim ayrık olmalı).")
        # aci-poz butonlari
        if angle_btns:
            self._para("\nDuruşu tutturabiliyorsan:", "#8af", 11)
            row = ctk.CTkFrame(self.body, fg_color="transparent")
            row.pack(fill="x", pady=2)
            for kind, label in angle_btns:
                self._cap_btn(row, label,
                              lambda k=kind: self._do(lambda: self._cap_ref(k)))
        # yukseklikten
        if height_label:
            self._para(f"\nVeya {height_label}: kol herhangi bir duruşta dururken "
                       "o noktanın YERDEN yüksekliğini ölç (cm), yaz, yakala — "
                       "açı hesaplanır ('yatay olmuyor' sorununu çözer):", "#8af", 11)
            row = ctk.CTkFrame(self.body, fg_color="transparent")
            row.pack(fill="x", pady=2)
            self.h_entry = ctk.CTkEntry(row, width=70)
            self.h_entry.pack(side="left", padx=(0, 6))
            self._cap_btn(row, "📏 yükseklikten yakala",
                          lambda: self._do(self._cap_height(joint)))
        # mevcut noktalar
        pts = self._kin().get(key) or []
        txt = " / ".join(f"s{p[0]:g}={p[1]:g}°" for p in pts) or "—"
        self._para(f"\nYakalanan: {len(pts)} nokta:  {txt}", "#9c9", 11)
        if pts:
            ctk.CTkButton(self.body, text="🗑 bu eklemi sıfırla", fg_color="#a33",
                          width=140,
                          command=lambda: self._do(lambda: self._clr(key))
                          ).pack(anchor="w", pady=2)
        ready = self._joint_ready(key)
        return ready, ("tamam ✔" if ready else
                       "2 ayrık nokta gerekli" if len(pts) < 2
                       else "noktalar çok yakın — daha ayrık 2. duruş")

    def _cap_ref(self, kind):
        before = len(self._kin().get(self.app.REF_KINDS[kind][0]) or [])
        self.app._capture_ref(kind)
        key = self.app.REF_KINDS[kind][0]
        after = len(self._kin().get(key) or [])
        return (after > before or True), f"{kind} yakalandı"

    def _cap_height(self, joint):
        def fn():
            try:
                z = float((self.h_entry.get() if self.h_entry else "")
                          .replace(",", "."))
            except Exception:
                return False, "yükseklik sayı değil (cm)"
            return self.app._capture_height_val(joint, z)
        return fn

    def _clr(self, key):
        self._kin()[key] = []
        self.app._refresh_kin_status()
        return True, f"{key} sıfırlandı"

    def _s_l1(self):
        return self._joint_step(
            "3) OMUZ (L1) — 2 nokta", "sh_pts", "sh",
            [("sh90", "📐 L1 DİK (90°)"), ("sh0", "📐 L1 YATAY (0°)")],
            "DİRSEK mafsalı yüksekliği")

    def _s_elbow(self):
        return self._joint_step(
            "4) DİRSEK — 2 nokta", "el_pts", "el",
            [("el0", "📐 KOL DÜZ"), ("elh", "📐 L2 YATAY")],
            "BİLEK mafsalı yüksekliği")

    def _s_wrist(self):
        return self._joint_step(
            "5) BİLEK — 2 nokta (mıknatıs yönü)", "gr_pts", "gr",
            [("grd", "📐 MIK. AŞAĞI"), ("grf", "📐 MIK. İLERİ")],
            "MIKNATIS UCU yüksekliği")

    def _s_base(self):
        self._para("6) BASE — 2 nokta (yana dönüş)", "#fff", 13)
        self._para("Base yatay döner; yükseklikten ölçülemez. İki belirgin "
                   "duruşu tuttur:")
        row = ctk.CTkFrame(self.body, fg_color="transparent")
        row.pack(fill="x", pady=4)
        self._cap_btn(row, "📐 BASE İLERİ (0°)",
                      lambda: self._do(lambda: self._cap_ref("b0")))
        self._cap_btn(row, "📐 BASE SAĞ (90°)",
                      lambda: self._do(lambda: self._cap_ref("b90")))
        pts = self._kin().get("base_pts") or []
        txt = " / ".join(f"s{p[0]:g}={p[1]:g}°" for p in pts) or "—"
        self._para(f"\nYakalanan: {len(pts)} nokta:  {txt}", "#9c9", 11)
        if pts:
            ctk.CTkButton(self.body, text="🗑 sıfırla", fg_color="#a33", width=100,
                          command=lambda: self._do(lambda: self._clr("base_pts"))
                          ).pack(anchor="w", pady=2)
        ready = self._joint_ready("base_pts")
        return ready, ("tamam ✔" if ready else "2 ayrık nokta gerekli")

    def _s_focal(self):
        self._para("7) KAMERA — açı + odak (mesafeyi DOĞRU ölçmesi için):",
                   "#fff", 13)
        self._para("\nKamera bileğe eğik bağlı; eğimi elle bilinemez. Bu yüzden "
                   "küpü BİLİNEN birkaç noktaya koyarsın, sistem geri çözer.")
        self._para("\n① Kolu küpleri iyi gören bir duruşa getir, bu adımda "
                   "oynatma. ② Küpü koy, base'den İLERİ (y) ve YANA (x, sağ +/"
                   "sol −) ölç, yaz, ➕ bas. ③ En az 4-5 örnek; ÖNEMLİ: küpleri "
                   "hem farklı MESAFELERE hem SAĞA-SOLA yay (hepsi tek çizgide "
                   "OLMASIN — yoksa odak çözülemez). ④ 🧮 Çöz.", "#8af")
        self._para("Örn: (0,12) (4,15) (-4,13) (2,18) (-3,11) gibi karışık. "
                   "Örnekler KAYDEDİLİR — sihirbazı kapatıp sonra açıp üstüne "
                   "ekleyebilirsin.", "#9c9", 11)
        row = ctk.CTkFrame(self.body, fg_color="transparent")
        row.pack(fill="x", pady=4)
        ctk.CTkLabel(row, text="x(yan)", width=44).pack(side="left")
        self.cx_entry = ctk.CTkEntry(row, width=52)
        self.cx_entry.insert(0, "0")
        self.cx_entry.pack(side="left", padx=(0, 4))
        ctk.CTkLabel(row, text="y(ileri)", width=48).pack(side="left")
        self.cy_entry = ctk.CTkEntry(row, width=52)
        self.cy_entry.pack(side="left", padx=(0, 4))
        self._cap_btn(row, "➕ örnek ekle", lambda: self._do(self._add_cam_sample))
        samples = getattr(self, "cam_samples", [])
        for i, s in enumerate(samples):
            srow = ctk.CTkFrame(self.body, fg_color="transparent")
            srow.pack(fill="x", pady=1)
            ctk.CTkLabel(srow, text=f"#{i+1}: ölçülen ({s[3]:g}, {s[4]:g}) cm  "
                         f"piksel ({s[1]:.0f}, {s[2]:.0f})", anchor="w",
                         font=("", 11), text_color="#9c9").pack(side="left")
            ctk.CTkButton(srow, text="🗑", width=30, height=22, fg_color="#a33",
                          command=lambda k=i: self._do(lambda: self._del_cam(k))
                          ).pack(side="right")
        if samples:
            r2 = ctk.CTkFrame(self.body, fg_color="transparent")
            r2.pack(fill="x", pady=2)
            self._cap_btn(r2, "🧮 Çöz (kamera açısı+odak)",
                          lambda: self._do(self._solve_cam))
            ctk.CTkButton(r2, text="🗑", fg_color="#a33", width=40,
                          command=lambda: self._do(self._clr_cam)
                          ).pack(side="left", padx=2)
        k = self.app._kin()
        det = self.app.detection_state.get(AGV)
        seen = "küp görülüyor ✔" if det else "⚠ küp tespit YOK (🎯 Tespit aç)"
        self._para(f"\npitch={float(k.get('cam_pitch') or 0):g}°  "
                   f"f={float(k.get('cam_f') or 0):g}px  "
                   f"dz={float(k.get('cam_dz') or 0):g}cm   ({seen})",
                   "#9c9" if det else "#fc6", 11)
        rms = getattr(self, "cam_rms", None)
        if rms is not None:
            self._para(f"çözüm hatası (RMS): {rms:g} cm "
                       f"{'✔ iyi' if rms < 2 else '⚠ yüksek — daha çok/ayrık örnek'}",
                       "#7c7" if rms < 2 else "#fc6", 12)
        return True, "kamerayı çöz (≥3 örnek) veya atla"

    def _add_cam_sample(self):
        d = self.app.detection_state.get(AGV)
        if not d:
            return False, "küp tespit edilmiyor (🎯 Tespit aç)"
        try:
            x = float((self.cx_entry.get() if self.cx_entry else "0").replace(",", "."))
            y = float((self.cy_entry.get() if self.cy_entry else "").replace(",", "."))
        except Exception:
            return False, "x/y sayı değil (cm)"
        servos = self._servos()
        self.cam_samples.append([servos, float(d["cx"]), float(d["cy"]), x, y])
        self._save_cam_samples()
        return True, f"örnek {len(self.cam_samples)}: ({x:g}, {y:g}) cm eklendi"

    def _save_cam_samples(self):
        """Ornekleri config'e yaz (kalici) — sihirbaz kapanip acilinca durur."""
        self._kin()["cam_samples"] = [list(s) for s in self.cam_samples]

    def _del_cam(self, idx: int):
        if 0 <= idx < len(self.cam_samples):
            s = self.cam_samples.pop(idx)
            self._save_cam_samples()
            return True, f"örnek #{idx+1} ({s[3]:g}, {s[4]:g}) silindi"
        return False, "örnek yok"

    def _clr_cam(self):
        self.cam_samples = []
        self.cam_rms = None
        self._save_cam_samples()
        return True, "tüm kamera örnekleri silindi"

    def _solve_cam(self):
        samples = getattr(self, "cam_samples", [])
        if len(samples) < 3:
            return False, f"en az 3 örnek gerekli ({len(samples)} var)"
        out, res = calibrate_camera(self.app._kin(), samples)
        if out is None:
            return False, str(res)
        kin = self._kin()
        for k in ("cam_pitch", "cam_f", "cam_dz", "cam_dx"):
            if k in out:
                kin[k] = out[k]
        self.cam_rms = res
        self.app._refresh_kin_status()
        return True, (f"çözüldü: pitch={kin.get('cam_pitch'):g}° "
                      f"f={kin.get('cam_f'):g}px, RMS={res:g} cm")

    def _s_verify(self):
        self._para("8) DOĞRULAMA", "#fff", 13)
        miss = self._model().missing()
        if miss:
            self._para("\n⚠ Hâlâ eksik:\n• " + "\n• ".join(miss), "#fc6")
            self._para("\nGeri dönüp tamamla.", "#999")
            return False, "eksik kalibrasyon var"
        self._para("\n✅ Model hazır! Şimdi GÖZLE doğrula:", "#7c7", 12)
        self._para("\n1) Ana penceredeki videoda MACENTA ARTI = mıknatısın yere "
                   "izdüşümü. Kolu gezdir — artı her zaman mıknatısın tam "
                   "altındaki noktada durmalı.", "#8af")
        self._para("2) Küp koy: sol üstte 'küp=(x,y)cm' yazar. Cetvelle ölç, "
                   "tutuyor mu bak. ±1–2 cm iyidir.", "#8af")
        self._para("\nTutmuyorsa: en çok H0 ve ilgili eklemin 2. noktasını "
                   "kontrol et (geri dön, yenile).", "#999")
        # canli kup tahmini
        det = self.app.detection_state.get(AGV)
        if det:
            est = self._model().cube_from_pixel(
                self._servos(), det["cx"], det["cy"],
                z_plane=float(self.app._kin().get("cube_cm") or 4) / 2.0)
            if est:
                self._para(f"\nşu an küp ≈ ({est[0]:.1f}, {est[1]:.1f}) cm",
                           "#9f9", 12)
        return True, "Bitir & Kaydet ile tamamla"


if __name__ == "__main__":
    PickupTest().mainloop()

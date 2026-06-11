"""
OTONOM KAPMA TEST ARACI — bagimsiz teshis UI'si.

agv_control.py'den BAGIMSIZ calisir: ESP32-CAM'e dogrudan baglanir
(WS server gerekmez). Amac: kapma sekansini adim adim izleyip sorunu bulmak.

Iceren:
  * Canli MJPEG + YOLO overlay (bbox, merkez, KILIT hedef pikseli + deadband)
  * 4 servo slider (kolu konumlamak icin — kontrolorun baslangic pozu buradan)
  * ▶ BASLAT / ⏹ DURDUR / 🛑 ACIL DUR (Esc) — acil dur: freeze + miknatis OFF
  * ⏯ ADIM MODU: kontrolor otomatik tick atmaz; ⏭ TEK ADIM ile her tick'i
    elinle ilerletirsin — her adimin karari logda
  * Detayli log ekrani (kontrolor kararlari + grip olaylari + CAM loglari);
    ayrica her kosu pc/pickup_logs/run_*.jsonl dosyasina TAM kayit yazar
    (sonradan satir satir incelenebilir — debug'in tek dogru kaynagi)
  * KILIT & DALIS kalibrasyonu (v2 mimarisinin iki ogretilen parcasi)

Kontrolor: pickup_controller.PickupController v2 (KILIT + OGRETILMIS DALIS):
  TRACK (kaba ortalama) -> GO_LOCK (kilit pozuna transit) -> FINE (kilit
  hedef pikseline ince nisan) -> PLUNGE (ogretilmis kor inis, buton sonlandirir)
  -> GRABBED (miknatis ACIK).

KALIBRASYON SIRASI (bir kez):
  1) arm_sim sinir sihirbazi (guvenli zarf)
  2) 📍 KILIT kaydet: kolu "kup tam miknatisin altinda + rahat gorunur"
     yukseklige getir, kupu miknatisin TAM altina koy, butona bas
  3) 🎬 Dalis: kilit pozundan sliderlarla kupe kadar in, yol boyunca
     "Dalis noktasi ekle" (son nokta = buton temas pozu), 💾 Config Kaydet.
     FARKLI kup mesafeleri icin "🎬 YENI DALIS" ile EK profiller ogretilebilir
     (her profil kendi baslangic pozuyla kaydedilir; kosuda FINE bitis pozuna
     EN YAKIN baslangicli profil otomatik secilir ve secim loglanir).

Calistir:  & ".venv\\Scripts\\python.exe" pc\\pickup_test.py
"""

from __future__ import annotations

import json
import os
import threading
import time
import urllib.request

import customtkinter as ctk
import cv2
from PIL import Image

from detector import get_detector
from pickup_controller import PickupController
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

        self._build()
        self.bind("<Escape>", lambda e: self.estop())
        self.bind("<space>", lambda e: self._capture_shot())
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(80, self._loop)
        auto = self.pickup_cfg.get("autonomous", {}) or {}
        env = auto.get("safe_envelope") or {}
        self.log("✅ güvenli bölge kayıtlı" if env.get("sh_grid")
                 else "⚠ güvenli bölge YOK — önce arm_sim sihirbazı")
        lock_ok = (auto.get("lock_pose") and auto.get("lock_col_frac") is not None
                   and auto.get("lock_row_frac") is not None)
        self.log(f"✅ KİLİT kayıtlı: {auto['lock_pose']}" if lock_ok
                 else "⚠ KİLİT YOK — kolu küp mıknatıs altındayken 📍 KİLİT kaydet")
        profs = [p for p in (auto.get("plunge_profiles") or []) if p.get("path")]
        if auto.get("plunge_path"):
            profs = profs + [{"path": auto["plunge_path"]}]
        self.log(f"✅ DALIŞ: {len(profs)} profil "
                 f"({', '.join(str(len(p['path'])) + ' nokta' for p in profs)})"
                 if profs else
                 "⚠ DALIŞ yolu YOK — kilitten küpe inerken 📍 Dalış noktası ekle")

    # ------------------------------------------------------------------ cfg
    def _load_cfg(self) -> dict:
        try:
            with open(PICKUP_CFG, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def _save_cfg(self):
        try:
            with open(PICKUP_CFG, "w", encoding="utf-8") as f:
                json.dump(self.pickup_cfg, f, indent=2)
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

        # SOL: ayarlar
        ctk.CTkLabel(left, text="AYAR", font=("", 13, "bold")).pack(anchor="w",
                                                                    pady=(10, 2))
        drow = ctk.CTkFrame(left, fg_color="transparent")
        drow.pack(fill="x", pady=2)
        self.bdir_btn = ctk.CTkButton(drow, text="", fg_color="#46a",
                                      command=lambda: self._flip_dir("base_dir"))
        self.bdir_btn.pack(side="left", expand=True, fill="x", padx=(0, 2))
        self.gdir_btn = ctk.CTkButton(drow, text="", fg_color="#46a",
                                      command=lambda: self._flip_dir("gripper_dir"))
        self.gdir_btn.pack(side="left", expand=True, fill="x", padx=(2, 0))
        self._refresh_dir_btns()

        # SOL: KILIT & DALIS kalibrasyonu (v2 mimarisinin ogretilen parcalari)
        ctk.CTkLabel(left, text="KİLİT & DALIŞ KALİBRASYONU",
                     font=("", 13, "bold")).pack(anchor="w", pady=(10, 2))
        self.lock_btn = ctk.CTkButton(left, text="", fg_color="#a63",
                                      command=self._capture_lock)
        self.lock_btn.pack(fill="x", pady=2)
        self._refresh_lock_btn()
        ctk.CTkButton(left, text="🎬 YENİ DALIŞ başlat (bu pozdan)", fg_color="#3a7",
                      command=self._new_plunge).pack(fill="x", pady=2)
        plrow = ctk.CTkFrame(left, fg_color="transparent")
        plrow.pack(fill="x", pady=2)
        self.plunge_btn = ctk.CTkButton(plrow, text="", fg_color="#46a",
                                        command=self._add_plunge)
        self.plunge_btn.pack(side="left", expand=True, fill="x", padx=(0, 2))
        ctk.CTkButton(plrow, text="🗑", fg_color="#a33", width=36,
                      command=self._clear_plunge).pack(side="left", padx=(2, 0))
        self._refresh_plunge_btn()
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

        self.status = ctk.CTkLabel(left, text="durum: IDLE", font=("", 12, "bold"),
                                   text_color="#ffd24a", justify="left",
                                   wraplength=300)
        self.status.pack(anchor="w", pady=(6, 2))

    def _slider(self, parent, idx, name):
        var = self.arm_servo_vars[AGV][idx]
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", pady=1)
        lbl = ctk.CTkLabel(row, text=f"{name}: {var.get()}", width=110, anchor="w")
        lbl.pack(side="left")

        def apply(v):
            v = max(0, min(180, round(v)))
            s.set(v)
            var.set(v)
            lbl.configure(text=f"{name}: {v}")
            self._http(f"/servo?id={idx}&a={v}")

        ctk.CTkButton(row, text="−", width=24,
                      command=lambda: apply(s.get() - 1)).pack(side="left", padx=(0, 1))
        s = ctk.CTkSlider(row, from_=0, to=180, number_of_steps=180, command=apply)
        s.set(var.get())
        s.pack(side="left", expand=True, fill="x", padx=1)
        ctk.CTkButton(row, text="+", width=24,
                      command=lambda: apply(s.get() + 1)).pack(side="left", padx=(1, 0))
        self.servo_sliders[idx] = (s, lbl, name)

    def _set_servo(self, idx: int, val: int):
        """Programatik servo: slider + label + var + gercek kol birlikte."""
        val = max(0, min(180, int(round(val))))
        s, lbl, name = self.servo_sliders[idx]
        s.set(val)
        self.arm_servo_vars[AGV][idx].set(val)
        lbl.configure(text=f"{name}: {val}")
        self._http(f"/servo?id={idx}&a={val}")

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
        self.pickup_cfg.setdefault("autonomous", {})["search_pose"] = pose
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
        if c is None:
            self.status.configure(text="durum: IDLE")
            return
        live = c.live or {}
        extra = (f"\ndx={live.get('dx', '-')}  dy={live.get('dy', '-')}  "
                 f"area={live.get('area', '-')}  conf={live.get('conf', '-')}")
        self.status.configure(text=f"durum: {c.state}\n{c.message}{extra}")

    def _arm_http_get(self, _agv: str, path: str):
        self._http(path)

    # ------------------------------------------------------------------ http
    def _http(self, path: str, log: bool = False):
        base = (self.url.get().strip() or self.cam_url).rstrip("/")
        url = f"{base}{path}"

        def worker():
            try:
                with urllib.request.urlopen(url, timeout=1.5) as r:
                    r.read()
                if log:
                    self.after(0, lambda: self.log(f"→ {path} OK"))
            except Exception as e:
                self.after(0, lambda: self.log(f"⚠ HTTP hata {path}: {e}"))
        threading.Thread(target=worker, daemon=True).start()

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
            self._poll_stop.wait(0.25 if fast else 1.0)

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
        for k in ("base_dir", "gripper_dir", "search_pose",
                  "lock_pose", "lock_col_frac", "lock_row_frac",
                  "plunge_profiles", "plunge_path"):
            if k in ui:
                auto[k] = ui[k]
        # legacy anahtar UI'da silindiyse (profile tasindi) diskten geri gelmesin
        if "plunge_profiles" in ui and "plunge_path" not in ui:
            auto.pop("plunge_path", None)
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
        if self.detect_on:
            det = get_detector()
            dets = det.detect(frame)
            best = det.best(dets)
            if best is not None:
                self.detection_state[AGV] = {
                    "cx": best.cx, "cy": best.cy, "area": best.area,
                    "conf": best.conf, "h": best.height, "w": best.x2 - best.x1,
                    "frame_w": frame.shape[1], "frame_h": frame.shape[0],
                    "ts": time.time(),
                }
            else:
                self.detection_state.pop(AGV, None)
            frame = det.draw(frame, dets)
        # hedef cizgiler: KILIT HEDEF PIKSELI (kup merkezi buraya oturtulur) +
        # deadband — kalibre degilse kadraj merkezi gosterilir (gri ton fark)
        h, w = frame.shape[:2]
        dbx = int(auto.get("deadband_x_px", 12))
        dby = int(auto.get("deadband_y_px", 12))
        lc, lr = auto.get("lock_col_frac"), auto.get("lock_row_frac")
        calibrated = lc is not None and lr is not None
        col = (255, 0, 255) if calibrated else (120, 120, 120)
        cx = int(w * float(lc if calibrated else 0.5))
        ty = int(h * float(lr if calibrated else 0.5))
        cv2.line(frame, (cx, 0), (cx, h), col, 1)
        cv2.line(frame, (cx - dbx, 0), (cx - dbx, h), (90, 90, 90), 1)
        cv2.line(frame, (cx + dbx, 0), (cx + dbx, h), (90, 90, 90), 1)
        cv2.line(frame, (0, ty), (w, ty), col, 1)
        cv2.line(frame, (0, ty - dby), (w, ty - dby), (90, 90, 90), 1)
        cv2.line(frame, (0, ty + dby), (w, ty + dby), (90, 90, 90), 1)
        cv2.drawMarker(frame, (cx, ty), col, cv2.MARKER_CROSS, 14, 2)

        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(img)
        scale = min(720 / w, 480 / h)
        pil = pil.resize((int(w * scale), int(h * scale)))
        self._img_ref = ctk.CTkImage(light_image=pil, dark_image=pil,
                                     size=pil.size)
        self.video.configure(image=self._img_ref, text="")
        self._pickup_ui_refresh()

    def _dir_val(self, key: str) -> int:
        auto = self.pickup_cfg.get("autonomous", {}) or {}
        return int(auto.get(key, PickupController.DEFAULTS[key]))

    def _flip_dir(self, key: str):
        """Eksen yonunu cevir — calisan kontrolore de ANINDA uygulanir."""
        val = -self._dir_val(key)
        self.pickup_cfg.setdefault("autonomous", {})[key] = val
        if self.pc is not None:
            self.pc._cfg[key] = val
        self._refresh_dir_btns()
        self.log(f"↔ {key} = {val:+d} olarak çevrildi (💾 ile kalıcı olur)")

    def _refresh_dir_btns(self):
        self.bdir_btn.configure(text=f"↔ base_dir: {self._dir_val('base_dir'):+d}")
        self.gdir_btn.configure(text=f"↕ gripper_dir: {self._dir_val('gripper_dir'):+d}")

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

    def _capture_lock(self):
        """KILIT KAYDET: kolu 'kup TAM miknatisin altinda + kup goruntude rahat
        gorunur' yukseklige getir, kupu miknatisin tam altina koy, bas —
        lock_pose=[sh,el,gr] + kup MERKEZININ o anki goruntu konumu hedef
        piksel (lock_col/row_frac) olarak kaydedilir. FINE fazi kupu bu piksele
        oturtur; PLUNGE bu pozdan ogretilen yolu tekrar eder."""
        d = self.detection_state.get(AGV)
        if not d:
            self.log("⚠ Önce küp tespit edilmeli (🎯 aç + küpü mıknatısın TAM altına koy)")
            return
        sh = self.arm_servo_vars[AGV][1].get()
        el = self.arm_servo_vars[AGV][2].get()
        gr = self.arm_servo_vars[AGV][3].get()
        auto = self.pickup_cfg.setdefault("autonomous", {})
        auto["lock_pose"] = [sh, el, gr]
        auto["lock_col_frac"] = round(d["cx"] / d["frame_w"], 3)
        auto["lock_row_frac"] = round(d["cy"] / d["frame_h"], 3)
        self._refresh_lock_btn()
        self.log(f"📍 KİLİT kaydedildi: poz sh={sh} el={el} gr={gr}, hedef piksel "
                 f"({auto['lock_col_frac']:.2f}, {auto['lock_row_frac']:.2f}) "
                 f"(💾 ile kalıcı olur). Şimdi buradan küpe kadar inip yol boyunca "
                 f"'Dalış noktası ekle' ile dalışı öğret.")

    def _refresh_lock_btn(self):
        auto = self.pickup_cfg.get("autonomous", {}) or {}
        lp = auto.get("lock_pose")
        c, r = auto.get("lock_col_frac"), auto.get("lock_row_frac")
        cur = (f"{lp} → ({c:.2f}, {r:.2f})"
               if lp and c is not None and r is not None else "kalibre DEĞİL")
        self.lock_btn.configure(
            text=f"📍 KİLİT kaydet (küp mıknatıs altında) — {cur}")

    def _pose3(self) -> list:
        return [self.arm_servo_vars[AGV][i].get() for i in (1, 2, 3)]

    def _migrate_legacy_plunge(self, auto) -> list:
        """ESKI tek-yol plunge_path kaydini profile cevir (baslangic=lock_pose).
        Donus: plunge_profiles listesi."""
        profs = auto.setdefault("plunge_profiles", [])
        legacy = auto.pop("plunge_path", None)
        if legacy and auto.get("lock_pose"):
            profs.append({"start": list(auto["lock_pose"]), "path": legacy})
            self.log(f"ℹ eski dalış kaydı profil #{len(profs)} olarak taşındı "
                     f"(başlangıç: kilit pozu)")
        return profs

    def _new_plunge(self):
        """YENI DALIS PROFILI: kolu inisin baslayacagi poza getir (kup farkli
        mesafedeyse FINE'in biteceği poza benzer bir poz), bas — mevcut
        [sh,el,gr] profil BASLANGICI olur. Sonra inerken 'Dalis noktasi ekle'.
        Kontrolor calisma sirasinda FINE bitis pozuna EN YAKIN baslangicli
        profili secer."""
        auto = self.pickup_cfg.setdefault("autonomous", {})
        if not auto.get("lock_pose"):
            self.log("⚠ Önce 📍 KİLİT kaydet — dalışlar kilit etrafından öğretilir.")
            return
        profs = self._migrate_legacy_plunge(auto)
        pose = self._pose3()
        if profs and not profs[-1]["path"]:
            profs[-1]["start"] = pose
            self.log(f"🎬 boş dalış #{len(profs)} başlangıcı güncellendi: {pose}")
        else:
            profs.append({"start": pose, "path": []})
            self.log(f"🎬 yeni dalış #{len(profs)} başladı (başlangıç {pose}) — "
                     f"şimdi inerken 📍 Dalış noktası ekle, son nokta=buton temas pozu")
        self._refresh_plunge_btn()

    def _add_plunge(self):
        """DALIS NOKTASI EKLE: inerken her ara durusta bas — [sh,el,gr] AKTIF
        (son) profile eklenir. SON nokta buton temas pozu olmali. PLUNGE secilen
        profili FINE duzeltmesini koruyarak GORELI tekrar eder."""
        auto = self.pickup_cfg.setdefault("autonomous", {})
        if not auto.get("lock_pose"):
            self.log("⚠ Önce 📍 KİLİT kaydet — dalış kilit pozundan başlar.")
            return
        profs = self._migrate_legacy_plunge(auto)
        if not profs:
            # 🎬'siz hizli kullanim: kilit pozundan baslayan tek profil ac
            profs.append({"start": list(auto["lock_pose"]), "path": []})
            self.log("ℹ dalış #1 otomatik açıldı (başlangıç: kilit pozu)")
        pose = self._pose3()
        path = profs[-1]["path"]
        path.append(pose)
        self._refresh_plunge_btn()
        g = self.grip_state.get(AGV) or {}
        tip = " (buton BASILI — son nokta olabilir ✔)" if g.get("type") == "held" else ""
        self.log(f"📍 dalış #{len(profs)} noktası {len(path)}: "
                 f"sh={pose[0]} el={pose[1]} gr={pose[2]}{tip} (💾 ile kalıcı olur)")

    def _clear_plunge(self):
        auto = self.pickup_cfg.setdefault("autonomous", {})
        auto["plunge_profiles"] = []
        auto.pop("plunge_path", None)
        self._refresh_plunge_btn()
        self.log("🗑 tüm dalış profilleri silindi (💾 ile kalıcı olur)")

    def _refresh_plunge_btn(self):
        auto = self.pickup_cfg.get("autonomous", {}) or {}
        profs = auto.get("plunge_profiles") or []
        if auto.get("plunge_path"):   # henuz tasinmamis eski kayit da sayilir
            profs = profs + [{"path": auto["plunge_path"]}]
        n = len(profs)
        last = len(profs[-1]["path"]) if n else 0
        self.plunge_btn.configure(
            text=f"📍 Dalış noktası ekle — {n} dalış, son: {last} nokta")

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


if __name__ == "__main__":
    PickupTest().mainloop()

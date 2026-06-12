"""
OTONOM KAPMA TEST ARACI — bagimsiz teshis UI'si.

agv_control.py'den BAGIMSIZ calisir: ESP32-CAM'e dogrudan baglanir
(WS server gerekmez). Amac: kapma sekansini adim adim izleyip sorunu bulmak.

Iceren:
  * Canli MJPEG + YOLO overlay + HAREKETLI paralaks hedef artisi (anlik bbox
    yuksekligiyle interpole edilen "kup miknatisin altinda olsaydi burada
    gorunurdu" noktasi — kalibrasyonu otonomi RISKI OLMADAN gozle dogrulama araci)
  * 4 servo slider (kolu konumlamak icin — kontrolorun baslangic pozu buradan)
  * ▶ BASLAT / ⏹ DURDUR / 🛑 ACIL DUR (Esc) — acil dur: freeze + miknatis OFF
  * ⏯ ADIM MODU: kontrolor otomatik tick atmaz; ⏭ TEK ADIM ile her tick'i
    elinle ilerletirsin — her adimin karari logda
  * Detayli log ekrani; ayrica her kosu pc/pickup_logs/run_*.jsonl dosyasina
    TAM kayit yazar (debug'in tek dogru kaynagi)
  * HIZA NOKTALARI + SEAT kalibrasyonu (v3'un ogretilen parcalari)

Kontrolor: pickup_controller.PickupController v3 (SONA KADAR KAMERA):
  SEAT (bilek paralel aciya) -> PROBE (eklem yon/kazanc jog olcumu) ->
  ALIGN (paralaks hedefiyle hizala + alcal dongusu) -> ENDGAME (son 2-4 cm
  kor, miknatis erken acik, buton sonlandirir) -> GRABBED.

KALIBRASYON SIRASI (bir kez):
  1) arm_sim sinir sihirbazi (guvenli zarf)
  2) SEAT noktalari (>=3): kolu farkli (sh, el) pozlara getir, bilegi miknatis
     yuzu yere TAM PARALEL olana kadar cevir, "SEAT noktasi ekle"
  3) HIZA noktalari (>=1 TEMAS + 2-4 yukseklik): kup MIKNATISIN TAM ALTINDA
     dururken — once degdir (buton basili, temas ornegi), sonra ~2/5/9 cm
     yukseklikte (once 🪑 seat butonu!) "HIZA noktasi ekle". Canli macenta
     artiyla dogrula: kup altta iken arti her yukseklikte kup merkezine
     oturmali. 💾 Config Kaydet.

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

from detector import get_detector
from pickup_controller import PickupController, interp_target, seat_deg
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
        ns = len(auto.get("seat_points") or [])
        self.log(f"✅ SEAT: {ns} nokta" if ns >= 3
                 else f"⚠ SEAT noktası {ns}/3 — bileği paralel yapıp 'SEAT noktası ekle'")
        ap = auto.get("align_points") or []
        nc = sum(1 for p in ap if p.get("contact"))
        self.log(f"✅ HİZA: {len(ap)} nokta ({nc} temas)" if ap and nc
                 else "⚠ HİZA noktası YOK/temassız — küp mıknatıs altındayken "
                      "farklı yüksekliklerde 'HİZA noktası ekle' (biri buton basılı)")
        if ap and nc == len(ap) and len(ap) > 1:
            self.log("⚠ TÜM hiza örnekleri 'temas' işaretli — kalibrasyon "
                     "muhtemelen YANLIŞ alındı. 🗑 silip yeniden yap: küp hep "
                     "YERDE durur, kol yükseltilir; buton YALNIZ en alçak "
                     "(değme) örneğinde basılı olur.")

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
        self.bdir_btn = ctk.CTkButton(left, text="", fg_color="#46a",
                                      command=lambda: self._flip_dir("base_dir"))
        self.bdir_btn.pack(fill="x", pady=2)
        self._refresh_dir_btns()

        # SOL: SEAT + HIZA NOKTALARI kalibrasyonu (v3'un ogretilen parcalari)
        ctk.CTkLabel(left, text="SEAT & HİZA NOKTALARI (paralaks)",
                     font=("", 13, "bold")).pack(anchor="w", pady=(10, 2))
        ctk.CTkButton(left, text="🪑 Bileği SEAT açısına getir", fg_color="#3a7",
                      command=self._goto_seat).pack(fill="x", pady=2)
        strow = ctk.CTkFrame(left, fg_color="transparent")
        strow.pack(fill="x", pady=2)
        self.seat_btn = ctk.CTkButton(strow, text="", fg_color="#46a",
                                      command=self._add_seat)
        self.seat_btn.pack(side="left", expand=True, fill="x", padx=(0, 2))
        ctk.CTkButton(strow, text="🗑", fg_color="#a33", width=36,
                      command=self._clear_seat).pack(side="left", padx=(2, 0))
        self._refresh_seat_btn()
        alrow = ctk.CTkFrame(left, fg_color="transparent")
        alrow.pack(fill="x", pady=2)
        self.align_btn = ctk.CTkButton(alrow, text="", fg_color="#a63",
                                       command=self._add_align)
        self.align_btn.pack(side="left", expand=True, fill="x", padx=(0, 2))
        ctk.CTkButton(alrow, text="🗑", fg_color="#a33", width=36,
                      command=self._clear_align).pack(side="left", padx=(2, 0))
        self._refresh_align_btn()
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
        auto = self.pickup_cfg.setdefault("autonomous", {})
        auto["search_pose"] = pose
        self.log(f"📍 arama pozu = {pose} (💾 Config Kaydet ile kalıcı olur)")
        # v3 bilegi hemen seat'e cevirir — arama pozu seat'ten uzaksa kup SEAT
        # gecisinde kadrajdan kacar (saha 09:37). Kaydederken uyar.
        pts = auto.get("seat_points") or []
        if len(pts) >= 3:
            seat = round(seat_deg(pts, pose[1], pose[2]))
            if abs(pose[3] - seat) > 8:
                self.log(f"⚠ bu pozun bileği ({pose[3]}°) seat açısından ({seat}°) "
                         f"uzak — önce 🪑 'Bileği SEAT açısına getir' deyip küpü "
                         f"görüntüde tutarak kaydetmen önerilir.")

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
        extra = (f"\ndx={live.get('dx', '-')}  dy={live.get('dy', '-')}  "
                 f"area={live.get('area', '-')}  conf={live.get('conf', '-')}")
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
        for k in ("base_dir", "search_pose", "align_points", "seat_points",
                  "probe", "align", "endgame"):
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
        # PARALAKS HEDEF OVERLAY: kayitli hiza orneklerinin hedefleri kucuk
        # noktalar; tespit varsa ANLIK bbox yuksekligiyle interpole edilen
        # HAREKETLI macenta arti — "kup miknatisin altinda olsaydi burada
        # gorunurdu". Kalibrasyonu otonomi olmadan gozle dogrulama araci:
        # kup miknatisin altindayken arti her yukseklikte kup merkezine oturmali.
        h, w = frame.shape[:2]
        aln = auto.get("align", {}) or {}
        dbx = int(aln.get("deadband_x_px", 12))
        dby = int(aln.get("deadband_y_px", 12))
        pts = auto.get("align_points") or []
        for p in pts:
            try:
                px = int(w * float(p["col"]))
                py = int(h * float(p["row"]))
            except Exception:
                continue
            color = (0, 215, 255) if p.get("contact") else (200, 120, 255)
            cv2.circle(frame, (px, py), 3, color, -1)
        if best is not None and pts:
            tx, ty = interp_target(pts, float(best.height))
            cx, cy = int(w * tx), int(h * ty)
            cv2.drawMarker(frame, (cx, cy), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.rectangle(frame, (cx - dbx, cy - dby), (cx + dbx, cy + dby),
                          (130, 60, 130), 1)

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

    def _pose3(self) -> list:
        return [self.arm_servo_vars[AGV][i].get() for i in (1, 2, 3)]

    # ------------------------------------------------------------------ SEAT
    def _goto_seat(self):
        """Bilegi MEVCUT (sh, el) icin SEAT acisina (miknatis yuzu yere paralel)
        getir — seat_points IDW interpolasyonu. Hiza noktasi almadan once ve
        kalibrasyonu dogrularken kullanilir."""
        if self.pc is not None and self.pc.active:
            self.log("⚠ Sekans aktifken seat'e gidilmez — önce ⏹ DURDUR.")
            return
        auto = self.pickup_cfg.get("autonomous", {}) or {}
        pts = auto.get("seat_points") or []
        if len(pts) < 3:
            self.log(f"⚠ SEAT noktası {len(pts)}/3 — önce bileği elle paralel "
                     f"yapıp 'SEAT noktası ekle' ile öğret.")
            return
        sh, el, _ = self._pose3()
        seat = max(0, min(180, round(seat_deg(pts, sh, el))))
        self._set_servo(3, seat)
        self.log(f"🪑 bilek seat açısına: sh={sh} el={el} → gripper {seat}°")

    def _add_seat(self):
        """SEAT NOKTASI EKLE: bilegi miknatis yuzu yere TAM PARALEL olana kadar
        elle cevir, bas — [sh, el, gr] eklenir. Kontrolor calisma boyunca bilegi
        bu noktalardan interpole edilen acida tutar (kamera pitch sabit =
        paralaks modelinin on kosulu). Farkli (sh, el) bolgelerinden >=3 nokta."""
        sh, el, gr = self._pose3()
        pts = self.pickup_cfg.setdefault("autonomous", {}).setdefault("seat_points", [])
        pts.append([sh, el, gr])
        self._refresh_seat_btn()
        self.log(f"📍 seat noktası eklendi: sh={sh} el={el} → gripper {gr}° "
                 f"({len(pts)} nokta; 💾 ile kalıcı olur)")

    def _clear_seat(self):
        self.pickup_cfg.setdefault("autonomous", {})["seat_points"] = []
        self._refresh_seat_btn()
        self.log("🗑 seat noktaları silindi (💾 ile kalıcı olur)")

    def _refresh_seat_btn(self):
        n = len((self.pickup_cfg.get("autonomous", {}) or {}).get("seat_points") or [])
        self.seat_btn.configure(text=f"📍 SEAT noktası ekle — {n} nokta")

    # ------------------------------------------------------------------ HIZA
    def _add_align(self):
        """HIZA NOKTASI EKLE (paralaks kalibrasyonu): kup MIKNATISIN TAM
        ALTINDA + bilek SEAT acisindayken bas — {bh, col, row, contact}
        kaydedilir. contact=true: kup miknatisa degerken (buton basili).
        Kontrolor calisma sirasinda anlik bbox yuksekligiyle bu noktalardan
        hedef pikseli interpole eder. Ayni yukseklige (±%10 bh) ikinci ornek
        eskisini DEGISTIRIR (yenileme)."""
        d = self.detection_state.get(AGV)
        if not d:
            self.log("⚠ Önce küp tespit edilmeli (🎯 aç + küp mıknatısın TAM altında)")
            return
        auto = self.pickup_cfg.setdefault("autonomous", {})
        pts = auto.setdefault("align_points", [])
        bh = float(d.get("h") or 0)
        cx, cy = float(d["cx"]), float(d["cy"])
        if bh <= 0:
            self.log("⚠ bbox yüksekliği okunamadı")
            return
        # Kirpilma duzeltmesi (kontrolorle ayni mantik): alt/ust kenarda kesik
        # kupte gercek boyut GENISLIKTEN, merkez gorunen kenardan kurulur.
        H, m = d["frame_h"], 4
        y1, y2 = float(d.get("y1", cy - bh / 2)), float(d.get("y2", cy + bh / 2))
        clipped = False
        if y2 >= H - m and y1 > m:        # alt kenar kesik
            bh = float(d.get("w") or bh)
            cy = y1 + bh / 2.0
            clipped = True
        elif y1 <= m and y2 < H - m:      # ust kenar kesik
            bh = float(d.get("w") or bh)
            cy = y2 - bh / 2.0
            clipped = True
        g = self.grip_state.get(AGV) or {}
        contact = g.get("type") == "held"
        # SAHA 09:45: kullanici 4 ornegin DORDUNU de temasta aldi (kupu
        # miknatisa yapistirip kolla kaldirarak) -> paralaks modeli coker.
        # Dogru yontem: kup hep YERDE; yalniz en alcak orneukte buton basili.
        if contact and any(p.get("contact")
                           and abs(float(p.get("bh", 0)) - bh) > 0.10 * bh
                           for p in pts):
            self.log("⚠ İKİNCİ temas örneği — normalde 1 temas yeter! Yükseklik "
                     "örnekleri küp YERDE ve buton SERBESTKEN alınmalı; küpü "
                     "mıknatısa yapıştırıp kolu kaldırarak örnek almak paralaks "
                     "kalibrasyonunu BOZAR.")
        rec = {"bh": round(bh, 1),
               "col": round(cx / d["frame_w"], 3),
               "row": round(cy / H, 3),
               "pose": self._pose3(), "contact": contact,
               "ts": time.strftime("%Y-%m-%d %H:%M")}
        if clipped:
            self.log("⚠ bbox kadraj kenarında kırpıktı — boyut genişlikten, "
                     "merkez görünen kenardan düzeltildi (mümkünse küp TAM "
                     "görünürken örnek almak daha sağlıklı)")
        for i, p in enumerate(pts):
            if abs(float(p.get("bh", 0)) - bh) <= 0.10 * bh:
                pts[i] = rec
                self._refresh_align_btn()
                self.log(f"📍 hiza noktası YENİLENDİ (bh≈{bh:.0f}px): "
                         f"({rec['col']:.2f}, {rec['row']:.2f})"
                         f"{' TEMAS ✔' if contact else ''} (💾 ile kalıcı)")
                return
        pts.append(rec)
        self._refresh_align_btn()
        tip = " TEMAS ✔ (buton basılı)" if contact else \
              " — temas örneği için küpü mıknatısa değdirip tekrar ekle"
        self.log(f"📍 hiza noktası {len(pts)}: bh={bh:.0f}px hedef "
                 f"({rec['col']:.2f}, {rec['row']:.2f}){tip} (💾 ile kalıcı)")

    def _clear_align(self):
        self.pickup_cfg.setdefault("autonomous", {})["align_points"] = []
        self._refresh_align_btn()
        self.log("🗑 hiza noktaları silindi (💾 ile kalıcı olur)")

    def _refresh_align_btn(self):
        pts = (self.pickup_cfg.get("autonomous", {}) or {}).get("align_points") or []
        nc = sum(1 for p in pts if p.get("contact"))
        rng = ""
        if pts:
            bhs = sorted(float(p.get("bh", 0)) for p in pts)
            rng = f", bh {bhs[0]:.0f}..{bhs[-1]:.0f}"
        self.align_btn.configure(
            text=f"📍 HİZA noktası ekle — {len(pts)} nokta ({nc} temas{rng})")

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

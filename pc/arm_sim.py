"""
Robot Kol — GÜVENLİ SINIR SİHİRBAZI (adım adım + canlı log).

Zarf modeli (pickup_controller ile ayni):
  * Sabit SHOULDER'da ELBOW'un guvenli bolgesi bir ARALIK -> el_min/el_max = f(sh).
    Sinir shoulder'a gore DEGISIR — bu yuzden her durakta ayri olculur.
  * GRIPPER'in araligi (SHOULDER, ELBOW) CIFTINE bagli -> gr_rows = f(sh, el).

AKIS — her adimda tek is, yonerge kutusu soyler, her olay log paneline duser:
  FAZ 1 (13 durak): sihirbaz shoulder'i duraga OTOMATIK goturur (Canli acikken;
    ilk durakta ve ⛔ sonrasi sen getirirsin). Sen yalniz ELBOW'u ayarlarsin:
    en dusuk guvenli aci -> ✓ ALT, en yuksek -> ✓ UST. Ikisi de basilinca
    otomatik sonraki durak. Guvensiz shoulder -> ⛔ Atla.
  FAZ 2 (gripper duraklari): kol her duragin (shoulder, elbow)'una otomatik
    gider (Faz 1 zarfi icinden kopruleyerek). Sen yalniz GRIPPER'i ayarlarsin:
    ✓ ALT / ✓ UST. Guvensiz kombinasyon -> ⛔ Atla.
  Bitince 💾 KAYDET -> pickup_config.json (kalici).

Kayit: autonomous.safe_envelope (type="limits").

Calistir:  & ".venv\\Scripts\\python.exe" pc\\arm_sim.py
"""

from __future__ import annotations

import json
import os
import threading
import time
import tkinter as tk
import urllib.request

import customtkinter as ctk

from pickup_controller import gr_band, interp_band

PICKUP_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pickup_config.json")

SH_STEP = 15      # FAZ 1 shoulder duraklari (0..180)
GR_SH_STEP = 30   # FAZ 2 shoulder satirlari
GR_EL_STEP = 30   # FAZ 2 satir ici elbow duraklari (bant ici)
TOL = 3           # durak toleransi (derece)
MARGIN = 2        # kontrolorun sinira yaklasma payi (derece)
MS_PER_DEG = 40   # firmware ramping ~1°/40ms — oto-konumlama bekleme hesabi

COL_BG = "#1a1a1a"
COL_SAFE = "#2e7d4f"
COL_PT = "#ffd24a"
COL_AX = "#3a3a3a"
COL_TXT = "#888888"
COL_TARGET = "#d9534f"


class ArmRecord(ctk.CTk):
    SERVO = {"base": 0, "shoulder": 1, "elbow": 2, "gripper": 3}

    def __init__(self):
        super().__init__()
        self.title("Robot Kol — Güvenli Sınır Sihirbazı (adım adım)")
        self.geometry("1060x680")
        ctk.set_appearance_mode("dark")

        self.cam_url = "http://192.168.4.50"
        self.live = False
        self.base, self.shoulder, self.elbow, self.gripper = 90, 120, 60, 110
        self._last_send: dict = {}
        self._last_err = 0.0
        self.sld: dict = {}            # isim -> (slider, label, gorunen ad)

        self.sh_grid = list(range(0, 181, SH_STEP))
        self.el_min: list = [None] * len(self.sh_grid)
        self.el_max: list = [None] * len(self.sh_grid)
        self.gr_rows: list = []        # [{"sh":int,"el":[..],"lo":[..],"hi":[..]}]
        self.stops2: list = []         # [(row_idx, el_idx)]
        self.phase = 0                 # 0=bekliyor 1=elbow 2=gripper 3=bitti
        self.step = 0
        self._manual_next = True       # ilk duraga / ⛔ sonrasi kol elle goturulur
        self._auto_afters: list = []
        self._load()

        self._build()
        self._wiz_status()
        self._redraw()
        if self.phase == 3:
            self._log("📂 Mevcut kayıt yüklendi (pickup_config.json). "
                      "Yeniden ölçmek için ▶ Sihirbazı Başlat.")

    # ----------------------------------------------------------------- IO
    def _load(self):
        try:
            with open(PICKUP_CFG, encoding="utf-8") as f:
                cfg = json.load(f)
            env = (cfg.get("autonomous", {}) or {}).get("safe_envelope", {})
            if env.get("type") == "limits" and env.get("sh_grid"):
                self.sh_grid = [int(x) for x in env["sh_grid"]]
                self.el_min = [None if v is None else int(v) for v in env["el_min"]]
                self.el_max = [None if v is None else int(v) for v in env["el_max"]]
                self.gr_rows = env.get("gr_rows", []) or []
                self.phase = 3
        except Exception:
            pass

    def _save(self):
        try:
            cfg = {}
            if os.path.exists(PICKUP_CFG):
                with open(PICKUP_CFG, encoding="utf-8") as f:
                    cfg = json.load(f)
            cfg.setdefault("autonomous", {})["safe_envelope"] = {
                "type": "limits", "margin_deg": MARGIN,
                "sh_grid": self.sh_grid, "el_min": self.el_min, "el_max": self.el_max,
                "gr_rows": self.gr_rows,
            }
            with open(PICKUP_CFG, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=2)
            n1 = sum(1 for v in self.el_min if v is not None)
            n2 = sum(1 for r in self.gr_rows for v in r["lo"] if v is not None)
            self._log(f"💾 KAYDEDİLDİ → pickup_config.json "
                      f"({n1}/{len(self.sh_grid)} shoulder durağı, "
                      f"{n2} gripper kombinasyonu)")
        except Exception as e:
            self._log(f"❌ Kayıt hatası: {e}")

    # ----------------------------------------------------------------- UI
    def _build(self):
        left = ctk.CTkScrollableFrame(self, width=345)
        left.pack(side="left", fill="y", padx=6, pady=6)
        right = ctk.CTkFrame(self, fg_color="transparent")
        right.pack(side="right", fill="both", expand=True, padx=6, pady=6)
        self.canvas = tk.Canvas(right, bg=COL_BG, highlightthickness=0)
        self.canvas.pack(side="top", fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self._redraw())
        self.logbox = ctk.CTkTextbox(right, height=150, font=("Consolas", 11),
                                     fg_color="#101010", text_color="#bbb")
        self.logbox.pack(side="bottom", fill="x", pady=(6, 0))
        self.logbox.configure(state="disabled")

        ctk.CTkLabel(left, text="SERVO (0–180) — Canlı'da gerçek kolu döndürür",
                     font=("", 13, "bold")).pack(anchor="w", pady=(2, 4))
        self._slider(left, "base", "Base")
        self._slider(left, "shoulder", "Shoulder")
        self._slider(left, "elbow", "Elbow")
        self._slider(left, "gripper", "Gripper")

        urow = ctk.CTkFrame(left, fg_color="transparent")
        urow.pack(fill="x", pady=(6, 2))
        self.url = ctk.CTkEntry(urow)
        self.url.insert(0, self.cam_url)
        self.url.pack(side="left", expand=True, fill="x", padx=2)
        self.live_btn = ctk.CTkButton(urow, text="Canlı: KAPALI", width=110,
                                      fg_color="#444", command=self._toggle_live)
        self.live_btn.pack(side="left", padx=2)

        ctk.CTkLabel(left, text="SINIR SİHİRBAZI — ADIM ADIM", font=("", 13, "bold")
                     ).pack(anchor="w", pady=(12, 2))
        self.wiz = ctk.CTkLabel(left, text="", font=("", 12), text_color="#ddd",
                                justify="left", wraplength=315)
        self.wiz.pack(anchor="w", padx=2, pady=(0, 4))

        row = ctk.CTkFrame(left, fg_color="transparent")
        row.pack(fill="x", pady=2)
        self.lo_btn = ctk.CTkButton(row, text="✓ ALT SINIR", fg_color="#3a7",
                                    command=lambda: self._wiz_mark(False))
        self.lo_btn.pack(side="left", expand=True, fill="x", padx=(0, 2))
        self.hi_btn = ctk.CTkButton(row, text="✓ ÜST SINIR", fg_color="#3a7",
                                    command=lambda: self._wiz_mark(True))
        self.hi_btn.pack(side="left", expand=True, fill="x", padx=(2, 0))

        row2 = ctk.CTkFrame(left, fg_color="transparent")
        row2.pack(fill="x", pady=2)
        ctk.CTkButton(row2, text="⛔ Güvensiz — Atla", fg_color="#a33",
                      command=self._wiz_skip).pack(side="left", expand=True, fill="x",
                                                   padx=(0, 2))
        ctk.CTkButton(row2, text="← Geri", fg_color="#555", width=80,
                      command=self._wiz_back).pack(side="left", padx=(2, 0))

        ctk.CTkButton(left, text="▶ Sihirbazı Başlat (sıfırdan)", fg_color="#46a",
                      command=self._wiz_start).pack(fill="x", pady=(8, 2))
        ctk.CTkButton(left, text="💾 KAYDET (pickup_config.json)", fg_color="#3a7",
                      command=self._save).pack(fill="x", pady=2)

        self.info = ctk.CTkLabel(left, text="", font=("", 11), text_color="#aaa",
                                 justify="left", wraplength=315)
        self.info.pack(anchor="w", pady=4)

    def _slider(self, parent, attr, name):
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", pady=1)
        lbl = ctk.CTkLabel(row, text=f"{name}: {getattr(self, attr)}",
                           width=120, anchor="w")
        lbl.pack(side="left")

        def apply(v):
            v = max(0, min(180, round(v)))
            s.set(v)
            lbl.configure(text=f"{name}: {v}")
            self._set(attr, v)

        ctk.CTkButton(row, text="−", width=26,
                      command=lambda: apply(s.get() - 1)).pack(side="left", padx=(0, 1))
        s = ctk.CTkSlider(row, from_=0, to=180, number_of_steps=180, command=apply)
        s.set(getattr(self, attr))
        s.pack(side="left", expand=True, fill="x", padx=1)
        ctk.CTkButton(row, text="+", width=26,
                      command=lambda: apply(s.get() + 1)).pack(side="left", padx=(1, 0))
        self.sld[attr] = (s, lbl, name)

    # ----------------------------------------------------------------- log
    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        try:
            self.logbox.configure(state="normal")
            self.logbox.insert("end", f"[{ts}] {msg}\n")
            self.logbox.see("end")
            self.logbox.configure(state="disabled")
        except Exception:
            pass
        self.info.configure(text=msg)

    # ----------------------------------------------------------------- kontrol
    def _set(self, attr, v):
        setattr(self, attr, int(v))
        sid = self.SERVO.get(attr)
        if sid is not None and self.live:
            self._send(sid, int(v))
        self._redraw()

    def _set_servo(self, attr, v, why=""):
        """Programatik servo komutu: slider + label + gercek kol birlikte."""
        v = max(0, min(180, int(round(v))))
        s, lbl, name = self.sld[attr]
        s.set(v)
        lbl.configure(text=f"{name}: {v}")
        setattr(self, attr, v)
        if self.live:
            self._send(self.SERVO[attr], v, force=True)
        if why:
            self._log(f"🤖 {name} → {v}° ({why})")
        self._redraw()

    def _toggle_live(self):
        self.live = not self.live
        self.cam_url = self.url.get().strip() or self.cam_url
        self.live_btn.configure(text="Canlı: AÇIK" if self.live else "Canlı: KAPALI",
                                fg_color="#3fbf66" if self.live else "#444")
        self._log("🔌 Canlı mod AÇIK — slider'lar gerçek kolu sürüyor."
                  if self.live else "🔌 Canlı mod KAPALI.")

    def _send(self, sid, angle, force=False):
        now = time.time()
        if not force and now - self._last_send.get(sid, 0.0) < 0.08:
            return
        self._last_send[sid] = now
        url = f"{self.cam_url.rstrip('/')}/servo?id={sid}&a={int(angle)}"

        def worker():
            try:
                with urllib.request.urlopen(url, timeout=1.5) as r:
                    r.read()
            except Exception:
                if time.time() - self._last_err > 2.0:
                    self._last_err = time.time()
                    self.after(0, lambda: self._log("⚠ Servo komutu ulaşmadı — "
                                                    "URL/WiFi kontrol et."))
        threading.Thread(target=worker, daemon=True).start()

    # ----------------------------------------------------------------- yardimcilar
    def _band_at(self, sh):
        """Faz 1 zarfinin interpolasyonlu elbow bandi (margin'siz)."""
        return interp_band(self.sh_grid, self.el_min, self.el_max, sh)

    def _cancel_autos(self):
        for a in self._auto_afters:
            try:
                self.after_cancel(a)
            except Exception:
                pass
        self._auto_afters = []

    def _play(self, seq):
        """[(attr, deger, neden)] dizisini ramping suresine gore sirayla oynat."""
        delay = 0
        for attr, val, why in seq:
            dist = abs(getattr(self, attr) - val)
            self._auto_afters.append(
                self.after(delay, lambda a=attr, v=val, w=why: self._set_servo(a, v, w)))
            delay += int(dist * MS_PER_DEG) + 400

    # ----------------------------------------------------------------- sihirbaz
    def _wiz_start(self):
        self._cancel_autos()
        self.el_min = [None] * len(self.sh_grid)
        self.el_max = [None] * len(self.sh_grid)
        self.gr_rows = []
        self.stops2 = []
        self.phase, self.step = 1, 0
        # Ilk duraga kullanici kendisi gider (asagidaki log); sonraki duraklara
        # gecis otomatiktir (olculen bandin ortasi uzerinden kopru).
        self._manual_next = False
        self._log(f"▶ Sihirbaz başladı. FAZ 1 — Durak 1/{len(self.sh_grid)}: "
                  f"shoulder={self.sh_grid[0]}°. Kolu bu durağa KENDİN getir "
                  f"(elbow'u güvenli tutarak).")
        self._wiz_status()
        self._redraw()

    # --- FAZ 1 ---
    def _goto_stop1(self):
        """Sonraki shoulder duragina gecis. Olculen son bandin ORTASINA elbow'u
        cekip (guvenli kopru) shoulder'i duraga otomatik surer."""
        self._cancel_autos()
        tgt = self.sh_grid[self.step]
        if not self.live or self._manual_next:
            self._log(f"👉 Durak {self.step + 1}/{len(self.sh_grid)}: shoulder'ı "
                      f"{tgt}°'ye KENDİN getir (Canlı kapalıysa aç).")
            self._wiz_status()
            return
        seq = []
        prev_band = self._band_at(self.shoulder)
        if prev_band is not None:
            mid = (prev_band[0] + prev_band[1]) / 2
            seq.append(("elbow", mid, "güvenli köprü — bandın ortası"))
        seq.append(("shoulder", tgt, f"durak {self.step + 1}/{len(self.sh_grid)}"))
        self._play(seq)
        self._wiz_status()

    # --- FAZ 2 ---
    def _build_rows(self):
        self.gr_rows = []
        for sh in range(0, 181, GR_SH_STEP):
            if sh not in self.sh_grid:
                continue
            k = self.sh_grid.index(sh)
            lo, hi = self.el_min[k], self.el_max[k]
            if lo is None or hi is None:
                continue
            els = sorted({lo, hi, *(e for e in range(0, 181, GR_EL_STEP) if lo < e < hi)})
            self.gr_rows.append({"sh": sh, "el": els,
                                 "lo": [None] * len(els), "hi": [None] * len(els)})
        self.stops2 = [(i, j) for i, r in enumerate(self.gr_rows)
                       for j in range(len(r["el"]))]

    def _stop2(self):
        ri, ei = self.stops2[self.step]
        row = self.gr_rows[ri]
        return row, ei, row["sh"], row["el"][ei]

    def _goto_stop2(self):
        """Kolu duragin (shoulder, elbow)'una otomatik gotur — once elbow'u iki
        bandin KESISIMINE (kopru), sonra shoulder, en son elbow hedefe."""
        self._cancel_autos()
        self._wiz_status()
        _, _, sh_t, el_t = self._stop2()
        if not self.live:
            self._log(f"👉 Durak {self.step + 1}/{len(self.stops2)}: kolu "
                      f"shoulder={sh_t}°, elbow={el_t}°'ye KENDİN getir "
                      f"(Canlı açıksa otomatik giderdi).")
            return
        seq = []
        b_cur = self._band_at(self.shoulder)
        b_tgt = self._band_at(sh_t)
        if b_cur is not None and b_tgt is not None:
            lo = max(b_cur[0], b_tgt[0])
            hi = min(b_cur[1], b_tgt[1])
            bridge = (lo + hi) / 2 if lo <= hi else (b_cur[0] + b_cur[1]) / 2
            seq.append(("elbow", bridge, "güvenli köprü"))
        seq.append(("shoulder", sh_t, f"durak {self.step + 1}/{len(self.stops2)}"))
        seq.append(("elbow", el_t, "durak elbow hedefi"))
        self._play(seq)

    # --- ortak isaretleme ---
    def _wiz_status(self):
        if self.phase == 0:
            self.wiz.configure(text="▶ Sihirbazı Başlat'a bas. Önce Canlı'yı açmayı "
                                    "unutma (gerçek kol hareket etsin).")
            return
        if self.phase == 1:
            tgt = self.sh_grid[self.step]
            done = "✓" if self.el_min[self.step] is not None else "…"
            done2 = "✓" if self.el_max[self.step] is not None else "…"
            self.wiz.configure(
                text=f"FAZ 1 — Durak {self.step + 1}/{len(self.sh_grid)} "
                     f"(shoulder={tgt}°)\n"
                     f"1) ELBOW'u EN DÜŞÜK güvenli açıya indir → ✓ ALT {done}\n"
                     f"2) ELBOW'u EN YÜKSEK güvenli açıya kaldır → ✓ ÜST {done2}\n"
                     f"İkisi de basılınca otomatik sonraki durak.\n"
                     f"Bu shoulder açısı tamamen güvensizse ⛔ Atla.")
            return
        if self.phase == 2:
            row, ei, sh_t, el_t = self._stop2()
            done = "✓" if row["lo"][ei] is not None else "…"
            done2 = "✓" if row["hi"][ei] is not None else "…"
            self.wiz.configure(
                text=f"FAZ 2 — Durak {self.step + 1}/{len(self.stops2)} "
                     f"(sh={sh_t}°, el={el_t}°)\n"
                     f"Kol durağa gidiyor/geldi. Şimdi sadece GRIPPER:\n"
                     f"1) EN DÜŞÜK güvenli açı → ✓ ALT {done}\n"
                     f"2) EN YÜKSEK güvenli açı → ✓ ÜST {done2}\n"
                     f"Bu kombinasyon güvensizse ⛔ Atla.")
            return
        self.wiz.configure(text="✅ BİTTİ — 💾 KAYDET'e bas (kalıcı olur).\n"
                                "Düzeltme için ← Geri ile son duraklara dönebilirsin.")

    def _wiz_mark(self, is_hi: bool):
        if self.phase == 1:
            tgt = self.sh_grid[self.step]
            if abs(self.shoulder - tgt) > TOL:
                self._log(f"⚠ Shoulder hedefte değil: hedef {tgt}°, şu an "
                          f"{self.shoulder}°. Hareket bitsin ya da elle getir.")
                return
            val = self.elbow
            lo_a, hi_a, idx = self.el_min, self.el_max, self.step
            what = f"shoulder={tgt}° elbow"
        elif self.phase == 2:
            row, ei, sh_t, el_t = self._stop2()
            if abs(self.shoulder - sh_t) > TOL or abs(self.elbow - el_t) > TOL:
                self._log(f"⚠ Kol durakta değil: hedef sh={sh_t}°/el={el_t}°, "
                          f"şu an {self.shoulder}°/{self.elbow}°.")
                return
            val = self.gripper
            lo_a, hi_a, idx = row["lo"], row["hi"], ei
            what = f"sh={sh_t}° el={el_t}° gripper"
        else:
            self._log("Önce ▶ ile sihirbazı başlat.")
            return

        if is_hi:
            hi_a[idx] = val
        else:
            lo_a[idx] = val
        self._log(f"✓ {what} {'ÜST' if is_hi else 'ALT'} sınırı = {val}°")
        lo, hi = lo_a[idx], hi_a[idx]
        if lo is not None and hi is not None:
            if lo > hi:
                lo_a[idx], hi_a[idx] = hi, lo
            self._log(f"✅ Durak tamam: {what} aralığı "
                      f"[{lo_a[idx]}–{hi_a[idx]}] — sonraki durağa geçiliyor.")
            self._wiz_next()
        self._redraw()

    def _wiz_skip(self):
        if self.phase == 1:
            self._cancel_autos()
            self.el_min[self.step] = None
            self.el_max[self.step] = None
            self._log(f"⛔ shoulder={self.sh_grid[self.step]}° GÜVENSİZ işaretlendi. "
                      f"Sonraki durağa kolu KENDİN götür (otomatik sürme kapatıldı).")
            self._manual_next = True
            self._wiz_next()
        elif self.phase == 2:
            self._cancel_autos()
            row, ei, sh_t, el_t = self._stop2()
            row["lo"][ei] = None
            row["hi"][ei] = None
            self._log(f"⛔ sh={sh_t}° el={el_t}° kombinasyonu GÜVENSİZ — atlandı.")
            self._wiz_next()

    def _wiz_next(self):
        self.step += 1
        if self.phase == 1:
            if self.step >= len(self.sh_grid):
                self._build_rows()
                if self.stops2:
                    self.phase, self.step = 2, 0
                    self._log(f"➡ FAZ 2 başladı: {len(self.stops2)} gripper durağı "
                              f"({len(self.gr_rows)} shoulder satırı).")
                    self._goto_stop2()
                else:
                    self.phase, self.step = 3, 0
                    self._log("⚠ Hiç ölçülü bölge yok — FAZ 2 atlandı.")
                    self._wiz_status()
            else:
                self._goto_stop1()
                self._manual_next = False
        elif self.phase == 2:
            if self.step >= len(self.stops2):
                self.phase, self.step = 3, 0
                self._cancel_autos()
                self._log("🏁 SİHİRBAZ BİTTİ — 💾 KAYDET'e basmayı unutma!")
                self._wiz_status()
            else:
                self._goto_stop2()
        self._redraw()

    def _wiz_back(self):
        self._cancel_autos()
        if self.phase == 3 and self.stops2:
            self.phase, self.step = 2, len(self.stops2) - 1
        elif self.phase in (1, 2) and self.step > 0:
            self.step -= 1
        elif self.phase == 2:
            self.phase, self.step = 1, len(self.sh_grid) - 1
        else:
            return
        if self.phase == 1:
            self.el_min[self.step] = None
            self.el_max[self.step] = None
            self._log(f"← Geri: shoulder={self.sh_grid[self.step]}° durağı "
                      f"yeniden ölçülecek.")
            self._goto_stop1()
        else:
            row, ei, sh_t, el_t = self._stop2()
            row["lo"][ei] = None
            row["hi"][ei] = None
            self._log(f"← Geri: sh={sh_t}° el={el_t}° durağı yeniden ölçülecek.")
            self._goto_stop2()
        self._redraw()

    # ----------------------------------------------------------------- cizim
    def _redraw(self):
        c = self.canvas
        c.delete("all")
        W = c.winfo_width() or 660
        H = c.winfo_height() or 460
        c.create_text(W / 2, 12, fill="#bbb", font=("", 11, "bold"),
                      text="Güvenli zarf — yeşil bant = izinli aralık (interpolasyonla dolu)")
        half = W / 2
        tgt_sh = tgt_el = None
        if self.phase == 1:
            tgt_sh = self.sh_grid[self.step]
        elif self.phase == 2 and self.stops2:
            _, _, tgt_sh, tgt_el = self._stop2()
        self._band_left(c, 0, 22, half, H - 22, tgt_sh)
        self._band_right(c, half, 22, half, H - 22, tgt_el)
        c.create_line(half, 22, half, H, fill="#2a2a2a")

    def _axes(self, c, x0, y0, w, h, pad, xlab, ylab):
        def px(xv, yv):
            return (x0 + pad + xv / 180 * (w - 2 * pad),
                    y0 + h - pad - yv / 180 * (h - 2 * pad))

        for g in range(0, 181, 30):
            x, _ = px(g, 0)
            c.create_line(x, y0 + pad, x, y0 + h - pad, fill=COL_AX)
            c.create_text(x, y0 + h - pad + 10, text=str(g), fill=COL_TXT, font=("", 7))
            _, yy = px(0, g)
            c.create_line(x0 + pad, yy, x0 + w - pad, yy, fill=COL_AX)
            c.create_text(x0 + pad - 14, yy, text=str(g), fill=COL_TXT, font=("", 7))
        c.create_text(x0 + w / 2, y0 + h - 8, text=xlab + " → " + ylab,
                      fill=COL_TXT, font=("", 9))
        return px

    def _band_left(self, c, x0, y0, w, h, target):
        """SHOULDER (x) → ELBOW banti (Faz 1 olcumleri)."""
        px = self._axes(c, x0, y0, w, h, 42, "SHOULDER", "ELBOW")
        grid, lo_a, hi_a = self.sh_grid, self.el_min, self.el_max

        def ok(i):
            return (lo_a[i] is not None and hi_a[i] is not None
                    and lo_a[i] <= hi_a[i])

        for i in range(len(grid) - 1):
            if not (ok(i) and ok(i + 1)):
                continue
            p1 = px(grid[i], lo_a[i])
            p2 = px(grid[i + 1], lo_a[i + 1])
            p3 = px(grid[i + 1], hi_a[i + 1])
            p4 = px(grid[i], hi_a[i])
            c.create_polygon(*p1, *p2, *p3, *p4, fill=COL_SAFE, outline="")
        for i, g in enumerate(grid):
            if lo_a[i] is not None:
                x, y = px(g, lo_a[i])
                c.create_line(x - 3, y, x + 3, y, fill="#7cf", width=2)
            if hi_a[i] is not None:
                x, y = px(g, hi_a[i])
                c.create_line(x - 3, y, x + 3, y, fill="#fc7", width=2)
        if target is not None:
            tx, _ = px(target, 0)
            c.create_line(tx, y0 + 42, tx, y0 + h - 42, fill=COL_TARGET,
                          dash=(4, 3), width=2)
        cx, cy = px(self.shoulder, self.elbow)
        c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill=COL_PT, outline="#fff")

    def _band_right(self, c, x0, y0, w, h, target_el):
        """ELBOW (x) → GRIPPER banti, SU ANKI SHOULDER icin (2B zarfin dilimi)."""
        px = self._axes(c, x0, y0, w, h, 42,
                        f"ELBOW (shoulder={self.shoulder}°)", "GRIPPER")
        prev = None
        for el in range(0, 181, 2):
            band = gr_band(self.gr_rows, self.shoulder, el)
            if band is None:
                prev = None
                continue
            x, yl = px(el, band[0])
            _, yh = px(el, band[1])
            if prev is not None:
                c.create_polygon(prev[0], prev[1], x, yl, x, yh, prev[0], prev[2],
                                 fill=COL_SAFE, outline="")
            prev = (x, yl, yh)
        if target_el is not None:
            tx, _ = px(target_el, 0)
            c.create_line(tx, y0 + 42, tx, y0 + h - 42, fill=COL_TARGET,
                          dash=(4, 3), width=2)
        cx, cy = px(self.elbow, self.gripper)
        c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill=COL_PT, outline="#fff")


if __name__ == "__main__":
    ArmRecord().mainloop()

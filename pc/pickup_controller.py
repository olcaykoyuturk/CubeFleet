"""
Otonom kup kapma — v3: SONA KADAR KAMERA (paralaks hedef modeli).

NEDEN v3: v1 tek sabit hedef pikseli kullaniyordu (eye-in-hand: kamera bilekle
donunce hedef kayar — mıknatis kupun gerisine indi); v2 son inisi EZBERLENMIS
dalisla yapiyordu (kullanici istemedi). v3 inisin SONUNA KADAR gorsel
servolama yapar; tek kor kisim kupun fiziksel olarak miknatisin altina girip
kadrajdan ciktigi son ~2-4 cm'dir (ENDGAME, buton sonlandirir).

PARALAKS HEDEF MODELI (cekirdek): kamera + miknatis ayni bilege rijit monteli.
Bilek SEAT acisinda tutulursa (miknatis yuzu yere paralel -> kamera pitch
~sabit) "kup TAM miknatisin altinda" kosulunun goruntudeki yeri YALNIZ
yuksekligin fonksiyonudur. Yukseklik proxy'si: bbox yuksekligi bh (piksel
konumu da bh de 1/Z ile orantili -> iliski DOGRUSAL). Hedef piksel:
    (tx, ty) = interp_target(align_points, bh)      [parcali-dogrusal, CLAMP]
align_points = kullanicinin "kup tam miknatisin altinda" durumda 3-5 farkli
yukseklikte kaydettigi {bh, col, row, contact} ornekleri (en az biri contact:
kup miknatisa degerken, buton basili). dx/dy boylece HER yukseklikte fiziksel
olarak dogru "miknatisin altinda mi" hatasini verir.

OTOMATIK YON OLCUMU (PROBE): hangi eklemin dy'yi duzelttigi / hangisinin
alcalttigi POZA bagli — tahmin edilmez, OLCULUR: shoulder ve elbow'a ±step°
test hareketi (jog), dy ve bh tepkisi olculur -> gain_dy, gain_bh. dy ekseni =
|gain_dy| en buyuk eklem; alcalma ekseni = digeri (bh'yi buyuten yonuyle).
Isaret kazanctan gelir -> elle yon sabiti YOK. Tutarsizsa adim 2x'e cikar;
yine olmazsa strict -> ABORT, degilse config fallback.

Durum makinesi:
    IDLE -> SEAT    : bilek seat_deg(seat_points, sh, el) acisina (IDW)
         -> PROBE   : jog olcumu (eksen/yon/kazanc)
         -> ALIGN ⟳ : olc-bekle-duzelt dongusu — |dx|>db: BASE; |dy|>db:
                      dy-eklemi (delta=-dy/gain*damping); ikisi de gate icinde:
                      ALCALMA adimi + ayni partide bilek seat duzeltmesi.
                      Hedef (tx,ty) her olcumde bh ile yeniden hesaplanir.
         -> ENDGAME : bh temas esigine ulasti (veya kup yakinda kayboldu +
                      hiza iyiydi) -> miknatis erken ACIK (magnet_early) +
                      kisa kor inis; BUTON sonlandirir.
         -> GRABBED : buton -> miknatis ACIK, dur.
         \\-> ABORT : kayip / probe sonucsuz / zarf / timeout / estop.

GERCEK SETTLE: /poll arm verisi (app.arm_state: servos=anlik, targets=hedef)
tazeyse "hareket bitti" = servos==targets==komut + 1 kare kamera payi; bayatsa
eski settle_ticks sayacina duser (tick logunda hangi yol kullanildi yazar).

GUVENLI BOLGE: arm_sim SINIR SIHIRBAZI (type="limits") — her servo komutu
_move -> _repair -> _envelope_ok zincirinden gecer (PROBE/ENDGAME dahil).

KOSU LOGU: her kosu pc/pickup_logs/run_*.jsonl (JSONL) — config, her tick
(durum/poz/tespit/dx/dy/bh/tx/ty/karar), her servo komutu, zarf engelleri,
probe ham verileri + sonucu, align hedefleri, endgame tetigi, bitis nedeni.
Kosudan sonra dosya satir satir okunarak davranis geri sarilabilir.

Kontrolor UI thread'inde `app.after(tick_ms)` ile tick'ler; `app.detection_state`
(YOLO) ve `app.arm_state` (servo poll) okur, `app._arm_http_get` ile komut yollar.
"""

from __future__ import annotations

import json
import os
import time
from typing import Optional, TextIO


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def interp_band(grid, lo_a, hi_a, x):
    """1B grid uzerinde lineer interpolasyonla (lo, hi) bandi. x grid disinda,
    durak atlanmis (None) veya komsu durak atlanmissa None -> o bolge YASAK.
    arm_sim de cizim icin ayni fonksiyonu kullanir (tek kaynak)."""
    if not grid or x < grid[0] or x > grid[-1]:
        return None
    for i, g in enumerate(grid):
        if abs(x - g) < 1e-9:
            if lo_a[i] is None or hi_a[i] is None:
                return None
            return float(lo_a[i]), float(hi_a[i])
    for i in range(len(grid) - 1):
        if grid[i] < x < grid[i + 1]:
            if None in (lo_a[i], lo_a[i + 1], hi_a[i], hi_a[i + 1]):
                return None
            t = (x - grid[i]) / (grid[i + 1] - grid[i])
            return (lo_a[i] + (lo_a[i + 1] - lo_a[i]) * t,
                    hi_a[i] + (hi_a[i + 1] - hi_a[i]) * t)
    return None


def gr_band(rows, sh, el):
    """Gripper bandi — (SHOULDER, ELBOW) ciftine bagli 2B interpolasyon.
    rows: [{"sh": int, "el": [..], "lo": [..], "hi": [..]}, ...] (sh artan).
    Iki komsu shoulder satirinin el-bantlari arasinda sh'e gore harmanlanir."""
    if not rows:
        return None
    shs = [r["sh"] for r in rows]
    if sh < shs[0] or sh > shs[-1]:
        return None
    for i, g in enumerate(shs):
        if abs(sh - g) < 1e-9:
            return interp_band(rows[i]["el"], rows[i]["lo"], rows[i]["hi"], el)
    for i in range(len(shs) - 1):
        if shs[i] < sh < shs[i + 1]:
            b1 = interp_band(rows[i]["el"], rows[i]["lo"], rows[i]["hi"], el)
            b2 = interp_band(rows[i + 1]["el"], rows[i + 1]["lo"], rows[i + 1]["hi"], el)
            if b1 is None and b2 is None:
                return None
            # Satirlardan yalniz BIRI bu elbow'u kapsiyorsa ona yaslan — yine
            # gercek olcum (saha: sh=28'de inis butona 2° kala bloklaniyordu).
            if b1 is None:
                return b2
            if b2 is None:
                return b1
            t = (sh - shs[i]) / (shs[i + 1] - shs[i])
            return (b1[0] + (b2[0] - b1[0]) * t, b1[1] + (b2[1] - b1[1]) * t)
    return None


def seat_deg(points, sh, el, fallback=90.0):
    """Bu (shoulder, elbow) durusunda 'miknatis yuzu yere paralel' bilek acisi.
    points: [[sh, el, gripper], ...] kalibrasyon noktalari; ters-mesafe agirlikli
    (IDW, p=2) interpolasyon. Kol indikce aci canli degisir — kamera pitch'i
    sabit kalir (paralaks modelinin on kosulu). Nokta yoksa fallback."""
    num = den = 0.0
    for p in points or []:
        try:
            psh, pel, pg = float(p[0]), float(p[1]), float(p[2])
        except Exception:
            continue
        d2 = (sh - psh) ** 2 + (el - pel) ** 2
        if d2 < 1e-9:
            return pg
        w = 1.0 / d2
        num += w * pg
        den += w
    return num / den if den > 0.0 else float(fallback)


def interp_target(points, bh, fb=(0.5, 0.5)):
    """PARALAKS HEDEFI: bu bbox yuksekliginde (bh) 'kup tam miknatisin altinda'
    iken kup MERKEZININ goruntude gorundugu (col, row) — align_points uzerinde
    bh'ye gore PARCALI-DOGRUSAL interpolasyon (geometri: piksel konumu ve bh
    ikisi de 1/Z ile orantili -> iliski dogrusal). Uclarin DISINDA CLAMP:
    ekstrapolasyon yasak (uclarda gurultu hedefi savurur). Nokta yoksa fb."""
    pts = []
    for p in points or []:
        try:
            pts.append((float(p["bh"]), float(p["col"]), float(p["row"])))
        except Exception:
            continue
    if not pts:
        return float(fb[0]), float(fb[1])
    pts.sort()
    if bh <= pts[0][0]:
        return pts[0][1], pts[0][2]
    if bh >= pts[-1][0]:
        return pts[-1][1], pts[-1][2]
    for i in range(len(pts) - 1):
        b0, c0, r0 = pts[i]
        b1, c1, r1 = pts[i + 1]
        if b0 <= bh <= b1:
            t = (bh - b0) / (b1 - b0) if b1 > b0 else 0.0
            return c0 + (c1 - c0) * t, r0 + (r1 - r0) * t
    return pts[-1][1], pts[-1][2]


def _median(vals):
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


class RunLogger:
    """Kosu basina bir JSONL dosyasi (pc/pickup_logs/run_*.jsonl) — debug'in
    tek dogru kaynagi; hicbir seyi atlamayan makine okunur kayit."""

    def __init__(self, agv_id: str):
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pickup_logs")
        os.makedirs(d, exist_ok=True)
        self.path = os.path.join(d, time.strftime("run_%Y%m%d_%H%M%S") + ".jsonl")
        self._t0 = time.time()
        self._f: Optional[TextIO] = None
        try:
            self._f = open(self.path, "a", encoding="utf-8")
        except Exception:
            self._f = None
        self.event("run_start", agv=agv_id,
                   wall=time.strftime("%Y-%m-%d %H:%M:%S"))

    def event(self, kind: str, **data):
        if self._f is None:
            return
        try:
            rec = {"t": round(time.time() - self._t0, 3), "kind": kind}
            rec.update(data)
            self._f.write(json.dumps(rec, ensure_ascii=False, default=str) + "\n")
            self._f.flush()
        except Exception:
            pass

    def close(self):
        f, self._f = self._f, None
        try:
            if f:
                f.close()
        except Exception:
            pass


class PickupController:
    # Servo indexleri (firmware ile ayni): 0=Base 1=Shoulder 2=Elbow 3=Gripper
    BASE, SHOULDER, ELBOW, GRIPPER = 0, 1, 2, 3
    SERVO_NAMES = ("base", "shoulder", "elbow", "gripper")
    JOINT_BY_NAME = {"shoulder": 1, "elbow": 2}

    DEFAULTS = {
        # --- PARALAKS HEDEF MODELI — ZORUNLU ---
        # align_points: [{"bh": px, "col": 0..1, "row": 0..1, "pose": [sh,el,gr],
        #                 "contact": bool, "ts": str}, ...]  (pose salt teshis)
        # En az 1 ornek ZORUNLU ve icinde contact=true olmali (temas ani —
        # ENDGAME esiginin kaynagi). Tek ornek = sabit hedef (paralaks pasif,
        # uyarilir); onerilen 4 ornek (temas, ~2, ~5, ~9 cm).
        "align_points": [],
        # SEAT: bilek "miknatis yuzu paralel" acisi — [[sh, el, gr], ...] IDW
        "seat_points": [], "seat_fallback_deg": 90,
        # --- PROBE (jog yon/kazanc olcumu) ---
        "probe": {
            # SAHA 09:36: 2° adim + 1.0 px/° taban bu kol icin fazla siki cikti
            # (YOLO zıplamasi ±5px, uzak pozda kazanc 0.5-1 px/°) -> 3°/0.6.
            "step_deg": 3, "repeats": 2,
            "min_px_per_deg": 0.6,      # dy kazanci guvenilirlik tabani
            "min_bh_px_per_deg": 0.25,  # alcalma (bh) kazanci tabani
            "drift_max_px": 10,         # gidis-donus taban kaymasi toleransi
            "strict": True,             # sonucsuz -> ABORT (False: fallback)
            "fallback": {"dy_joint": "shoulder", "dy_dir": 1,
                         "dy_gain_px_per_deg": 3.0,
                         "descend_joint": "elbow", "descend_dir": -1},
        },
        # --- ALIGN ---
        "align": {
            "deadband_x_px": 12, "deadband_y_px": 12,
            "descend_gate_px": 8,       # alcalma izni icin daha siki hiza
            "descend_step_deg": 2,
            "dy_damping": 0.7,          # delta = clamp(-dy/gain*damping, ±max)
            "diverge_max": 3,           # |dy| ust uste N kez buyudu -> re-probe
            "reprobe_max": 2,
            "stall_max": 3,             # alcalma N kez zarfta bloklandi -> abort
        },
        # --- ENDGAME (kor son cm'ler) ---
        # KOR INIS YALNIZ GORUS GIDINCE baslar (kup ekrani kaplayinca/kaybolunca
        # lost_wait_s bekle -> kor). Gorus varken gorsel inis butona kadar surer
        # (saha 09:46: bh esiginde korlesmek son cm'de yoldan cikardi — gorus
        # conf 0.96 ile hala oradaydi). bh_frac artik yalniz ERKEN MIKNATIS esigi.
        "endgame": {
            "bh_frac": 0.80,            # bh >= contact*frac + hizada -> miknatis AC
            "lost_bh_frac": 0.60,       # kor inis icin "yeterince yakindi" tabani
            "lost_wait_s": 0.5,         # gorus gidince kor inise gecmeden bekleme
            "blind_max_ticks": 12, "grace_ticks": 8,
            "magnet_early": True,
        },
        # --- olcum/stabilizasyon ---
        "kp_base": 0.09, "base_dir": 1,   # saha-kanitli
        "max_step_deg": 2, "pickup_speed_ms": 40,
        "tick_ms": 150, "det_stale_s": 0.7,
        # GERCEK SETTLE: app.arm_state (servos/targets) tazeyse ondan; bayatsa
        # settle_ticks sayaci (eski davranis). settle_timeout: targets hicbir
        # zaman komuta esitlenmezse (firmware clamp) devam + cmd senkronu.
        "arm_state_max_age_s": 0.6, "settle_timeout_ticks": 8,
        "settle_ticks": 2, "stable_frames": 2,
        "lost_max": 20, "total_timeout_ticks": 600,
        "debug": True, "debug_every_ticks": 4,
    }

    def __init__(self, app, agv_id: str):
        self.app = app
        self.agv_id = agv_id
        self.state = "IDLE"
        self.message = "hazır"
        self.live: dict = {}            # {dx,dy,bh,tx,ty,area,conf} — UI + tick logu
        self._cfg: dict = dict(self.DEFAULTS)
        self._after_id = None
        self.cmd = [93, 120, 60, 110]   # komut edilen servo acilari (lokal takip)
        self.rl: Optional[RunLogger] = None
        self._decision = ""
        self._ticks = 0
        self._lost_count = 0
        self._grip_seq0 = 0
        self._err_hist: list = []       # temiz kare (cx, cy, bh) birikimi
        self._live_tick = -1
        # settle
        self._await_settle = False
        self._settle_left = 0           # tick-fallback sayaci
        self._settle_started = 0
        self._cam_grace = 0
        self._settle_path = ""          # "arm" | "ticks" (tick loguna)
        self._move_t = 0.0
        # model
        self._clip_tick = -99           # son kirpik-duzeltmeli olcum tick'i
        self._low_seen = False          # SEAT: kup alt kenara dayanmisti
        self._magnet_on = False         # erken miknatis acildi mi
        self._contact_bh = 0.0
        self._last_stable: Optional[dict] = None
        self._gain: Optional[dict] = None   # probe sonucu
        self._pr: dict = {}                 # probe sub-makinesi
        self._reprobes = 0
        self._dy_prev = None
        self._dy_grow = 0
        self._stalls = 0
        self._blind = 0
        self._grace = 0

    # ------------------------------------------------------------------ kontrol
    def start(self) -> bool:
        """MEVCUT kol pozundan baslat. Zarf + seat + hiza (align) kalibrasyonu
        eksikse reddeder ve neyin eksik oldugunu acikca soyler."""
        auto = (getattr(self.app, "pickup_cfg", None) or {}).get("autonomous", {})
        self._cfg = {**self.DEFAULTS, **auto}
        for sec in ("probe", "align", "endgame"):   # ic ice sozlukler derin birlesir
            self._cfg[sec] = {**self.DEFAULTS[sec], **(auto.get(sec) or {})}
        if self._envelope() is None:
            self._set("IDLE", "Önce arm_sim SINIR SİHİRBAZI'nı çalıştırıp kaydet.")
            return False
        missing = []
        if len(self._cfg.get("seat_points") or []) < 3:
            missing.append("SEAT noktaları (en az 3)")
        pts = self._cfg.get("align_points") or []
        contacts = [p for p in pts if p.get("contact")]
        if not pts or not contacts:
            missing.append("HİZA noktaları (en az 1 TEMAS örneği şart)")
        if missing:
            self._set("IDLE", "Eksik kalibrasyon: " + ", ".join(missing) +
                              " — pickup_test 'HİZA NOKTALARI' bölümünden kaydet.")
            return False
        self._contact_bh = max(float(p["bh"]) for p in contacts)
        pose = self._current_pose()
        if pose is None:
            self._set("IDLE", "Kol pozu okunamadı (servo slider'ları yok).")
            return False
        if not self._envelope_ok(pose):
            self._set("IDLE", f"Mevcut poz {pose} güvenli alan DIŞINDA — "
                              f"kolu zarf içine getir.")
            return False
        self._cancel()
        self._ticks = 0
        self._lost_count = 0
        self._err_hist = []
        self._await_settle = False
        self._cam_grace = 0
        self._last_stable = None
        self._gain = None
        self._pr = {}
        self._reprobes = 0
        self._dy_prev = None
        self._dy_grow = 0
        self._stalls = 0
        self._blind = 0
        self._grace = 0
        self._magnet_on = False
        self._low_seen = False
        g = getattr(self.app, "grip_state", {}).get(self.agv_id) or {}
        self._grip_seq0 = int(g.get("seq", 0))
        self.rl = RunLogger(self.agv_id)
        self.rl.event("config",
                      cfg={k: v for k, v in self._cfg.items() if k != "safe_envelope"},
                      envelope=True, start_pose=list(pose),
                      contact_bh=self._contact_bh, grip_seq0=self._grip_seq0)
        self._http(f"/servo/speed?ms={int(self._cfg['pickup_speed_ms'])}")
        self._http("/magnet?s=0")
        self.cmd = pose
        if len(pts) < 2:
            self._log("⚠ tek HİZA örneği — paralaks pasif (sabit hedef); "
                      "farklı yüksekliklerde örnek eklemen önerilir.")
        self._log(f"📄 koşu logu: {self.rl.path}")
        self._set("SEAT", f"Mevcut pozdan başladı {pose} — bilek seat açısına…")
        self._schedule()
        return True

    def _current_pose(self):
        """Kolun su anki pozu — ONCE taze firmware hedefleri (slider'lar onceki
        kosudan BAYAT kalabiliyor: saha 09:45'te slider 93/120 derken firmware
        104/76 idi, start yanlis pozdan sayip settle timeout yedi), sonra
        slider'lar, en son bayat firmware verisi."""
        st = self._arm_fresh()
        if st is not None:
            return [self._clamp_servo(x) for x in st["targets"]]
        try:
            vars_ = self.app.arm_servo_vars.get(self.agv_id)
            if vars_ and len(vars_) == 4:
                return [self._clamp_servo(v.get()) for v in vars_]
        except Exception:
            pass
        st = self._arm_raw()
        if st is not None:
            return [self._clamp_servo(x) for x in st["targets"]]
        return None

    def stop(self):
        self._cancel()
        self._set("IDLE", "Durduruldu.")
        self._finish("stop")
        self._refresh()

    def estop(self):
        """ACİL DUR — hareketi aninda kes + miknatisi kapat."""
        self._cancel()
        self._http("/magnet?s=0")
        self._set("ABORT", "ACİL DUR — kapma iptal, mıknatıs KAPALI.")
        self._finish("estop")
        self._refresh()

    @property
    def active(self) -> bool:
        return self.state not in ("IDLE", "GRABBED", "ABORT")

    # ------------------------------------------------------------------- zarf
    def _clamp_servo(self, angle) -> int:
        return round(_clamp(angle, 0, 180))

    def _envelope(self):
        env = self._cfg.get("safe_envelope")
        if env and env.get("type") == "limits" and env.get("sh_grid"):
            return env
        return None

    def _margined(self, band):
        env = self._envelope()
        if band is None or env is None:
            return band
        m = float(env.get("margin_deg", 0))
        lo, hi = band[0] + m, band[1] - m
        return (lo, hi) if lo <= hi else None

    def _el_band(self, sh):
        env = self._envelope()
        if env is None:
            return None
        return self._margined(
            interp_band(env["sh_grid"], env["el_min"], env["el_max"], sh))

    def _gr_band(self, sh, el):
        env = self._envelope()
        if env is None:
            return None
        return self._margined(gr_band(env.get("gr_rows") or [], sh, el))

    def _envelope_ok(self, cmd) -> bool:
        if self._envelope() is None:
            return True
        eb = self._el_band(cmd[self.SHOULDER])
        if eb is None or not (eb[0] - 0.5 <= cmd[self.ELBOW] <= eb[1] + 0.5):
            return False
        gb = self._gr_band(cmd[self.SHOULDER], cmd[self.ELBOW])
        return gb is not None and (gb[0] - 0.5 <= cmd[self.GRIPPER] <= gb[1] + 0.5)

    def _repair(self, cand):
        if self._envelope() is None:
            return cand
        eb = self._el_band(cand[self.SHOULDER])
        if eb is None:
            return None
        el = _clamp(cand[self.ELBOW], eb[0], eb[1])
        gb = self._gr_band(cand[self.SHOULDER], el)
        if gb is None:
            return None
        gr = _clamp(cand[self.GRIPPER], gb[0], gb[1])
        out = list(cand)
        out[self.ELBOW] = self._clamp_servo(el)
        out[self.GRIPPER] = self._clamp_servo(gr)
        return out

    # ------------------------------------------------------------------- hareket
    def _http(self, path: str):
        try:
            self.app._arm_http_get(self.agv_id, path)
        except Exception:
            pass

    def _move(self, i: int, delta: float) -> list:
        """Bir servoyu delta kadar oynat — tick adimi + 0..180 + GUVENLI ZARF.
        Donus: gercekten degisen [(servo, eski, yeni), ...] (bos = hareket yok).
        Her sonuc (engel dahil) dosya loguna yazilir; hareket settle baslatir."""
        ms = self._cfg["max_step_deg"]
        want = delta
        delta = _clamp(delta, -ms, ms)
        new = self._clamp_servo(self.cmd[i] + delta)
        if new == self.cmd[i]:
            return []
        candidate = list(self.cmd)
        candidate[i] = new
        candidate = self._repair(candidate)
        if candidate is None or not self._envelope_ok(candidate):
            if self.rl:
                self.rl.event("block", servo=self.SERVO_NAMES[i],
                              frm=self.cmd[i], to=new, want=round(want, 2))
            if self._ticks - getattr(self, "_last_block_log", -99) >= 6:
                self._last_block_log = self._ticks
                self._log(f"⛔ zarf engelledi: {self.SERVO_NAMES[i]} "
                          f"{self.cmd[i]}→{new} (ölçülmemiş/yasak bölge)")
            return []
        changed = [(j, self.cmd[j], candidate[j])
                   for j in range(4) if candidate[j] != self.cmd[j]]
        if not changed:
            return []
        drags = [(j, o, v) for j, o, v in changed if j != i]
        if self.rl:
            self.rl.event("move", servo=self.SERVO_NAMES[i],
                          frm=self.cmd[i], to=candidate[i], want=round(want, 2),
                          drag=[[self.SERVO_NAMES[j], o, v] for j, o, v in drags] or None)
        if drags and self._ticks - getattr(self, "_last_drag_log", -99) >= 4:
            self._last_drag_log = self._ticks
            self._log("↪ zarf kıstı: " + ", ".join(
                f"{self.SERVO_NAMES[j]}→{v}°" for j, _, v in drags))
        self.cmd = candidate
        for sid, _, val in changed:
            self._http(f"/servo?id={sid}&a={val}")
        # SETTLE baslat: servo bitene + kamera oturana kadar yeni gorsel karar yok
        self._await_settle = True
        self._settle_left = int(self._cfg.get("settle_ticks", 2))
        self._settle_started = self._ticks
        self._move_t = time.time()
        self._err_hist = []
        return changed

    def _descend_step(self) -> list:
        """Bir ALCALMA adimi (probe'un sectigi eklem + yonu) + OLCULMUS
        ILERI-KAYMA TELAFISI: alcalma eklemi miknatisi ayni anda ileri/geri de
        kaydirir (probe bunu gain_dy olarak olctu) — dy ekseniyle feedforward
        sifirlanir. Boylece ENDGAME'de gorus YOKKEN bile hiza korunur.
        + bilek seat duzeltmesi. Donus: alcalma hareketi (bos = bloklandi)."""
        g = self._gain or {}
        dsj = int(g.get("descend_joint", self.ELBOW))
        ddir = int(g.get("descend_dir", -1))
        step = float(self._cfg["align"]["descend_step_deg"]) * ddir
        ch = self._move(dsj, step)
        gdy_d = g.get("descend_dy_gain")
        gdy_j = g.get("dy_gain")
        dyj = int(g.get("dy_joint", self.SHOULDER))
        if ch and gdy_d and gdy_j and dsj != dyj:
            comp = _clamp(-(float(gdy_d) * step) / float(gdy_j),
                          -self._cfg["max_step_deg"], self._cfg["max_step_deg"])
            if abs(comp) >= 0.5:
                self._move(dyj, comp)
        self._seat_correct()
        return ch

    def _seat_correct(self):
        """Bilegi mevcut (sh, el) icin seat acisina dogru duzelt (ayni partide)."""
        seat = self._clamp_servo(seat_deg(self._cfg.get("seat_points"),
                                          self.cmd[self.SHOULDER],
                                          self.cmd[self.ELBOW],
                                          self._cfg.get("seat_fallback_deg", 90)))
        if self.cmd[self.GRIPPER] != seat:
            ch = self._move(self.GRIPPER, seat - self.cmd[self.GRIPPER])
            if ch and self.rl:
                self.rl.event("seat", sh=self.cmd[self.SHOULDER],
                              el=self.cmd[self.ELBOW], gr=self.cmd[self.GRIPPER])
        return seat

    # ------------------------------------------------------------------- settle
    def _arm_raw(self) -> Optional[dict]:
        st = getattr(self.app, "arm_state", {}).get(self.agv_id)
        if not st:
            return None
        s, t = st.get("servos"), st.get("targets")
        if not (isinstance(s, list) and isinstance(t, list)
                and len(s) == 4 and len(t) == 4):
            return None
        return st

    def _arm_fresh(self) -> Optional[dict]:
        st = self._arm_raw()
        if st is None:
            return None
        if time.time() - st.get("ts", 0) > self._cfg["arm_state_max_age_s"]:
            return None
        return st

    def _settling(self) -> bool:
        """True = hareket/kamera henuz oturmadi -> bu tick olcum/karar YOK.
        Taze arm verisi varsa GERCEK durumdan (servos==targets==komut), yoksa
        settle_ticks fallback'i. Oturunca +1 tick kamera payi."""
        if self._cam_grace > 0:
            self._cam_grace -= 1
            self._decision = "kamera payı"
            self._settle_path = "grace"
            return True
        if not self._await_settle:
            return False
        st = self._arm_fresh()
        if st is not None:
            self._settle_path = "arm"
            if (st.get("ts", 0) >= self._move_t and st["targets"] == self.cmd
                    and st["servos"] == st["targets"]):
                self._await_settle = False
                self._cam_grace = 1
                return True
            # firmware targets komuta hic esitlenmiyorsa (per-servo clamp vb.)
            if self._ticks - self._settle_started > self._cfg["settle_timeout_ticks"]:
                if st["targets"] != self.cmd:
                    if self.rl:
                        self.rl.event("sync", cmd=list(self.cmd),
                                      fw_targets=list(st["targets"]))
                    self._log(f"⚠ settle timeout: cmd {self.cmd} ≠ firmware "
                              f"{st['targets']} — komut firmware'e senkronlandı")
                    self.cmd = [self._clamp_servo(x) for x in st["targets"]]
                else:
                    if self.rl:
                        self.rl.event("settle_timeout", cmd=list(self.cmd))
                self._await_settle = False
                self._cam_grace = 1
                return True
            self._decision = "servo bekleniyor (arm)"
            return True
        # arm verisi yok/bayat -> eski tick sayaci
        self._settle_path = "ticks"
        if self._settle_left > 0:
            self._settle_left -= 1
            self._decision = f"settle {self._settle_left} (tick)"
            return True
        self._await_settle = False
        return False

    # ------------------------------------------------------------------- olcum
    def _det(self) -> Optional[dict]:
        d = getattr(self.app, "detection_state", {}).get(self.agv_id)
        if not d:
            return None
        if time.time() - d.get("ts", 0) > self._cfg["det_stale_s"]:
            return None
        return d

    def _det_snap(self) -> Optional[dict]:
        raw = getattr(self.app, "detection_state", {}).get(self.agv_id)
        if not raw:
            return None
        return {"cx": raw["cx"], "cy": raw["cy"], "area": raw["area"],
                "bh": raw.get("h"), "conf": round(raw.get("conf", 0), 2),
                "age": round(time.time() - raw.get("ts", 0), 2)}

    def _eff_det(self, d):
        """Kirpilma-duzeltmeli (cx, cy, bh, clip). Kadraj kenarina tasan bbox'in
        merkezi 'cerceveye civilenir' ve kol hareketine DUYARSIZLASIR (saha:
        alt kenarda kirpik kupte probe kazanclari ~0 olctu). Duzeltme: kup KARE
        oldugu icin gercek boyut kirpilmamis eksenden alinir, merkez GORUNEN
        kenardan kurulur (alt kirpik: cy = y1 + genislik/2). Iki eksen birden
        kirpiksa None (guvenilmez -> kayip sayilir)."""
        cx, cy = float(d["cx"]), float(d["cy"])
        bh = float(d.get("h") or 0)
        x1, y1, x2, y2 = d.get("x1"), d.get("y1"), d.get("x2"), d.get("y2")
        if None in (x1, y1, x2, y2) or bh <= 0:
            return (cx, cy, bh, False) if bh > 0 else None
        W, H, m = d["frame_w"], d["frame_h"], 4
        clip_l, clip_r = x1 <= m, x2 >= W - m
        clip_t, clip_b = y1 <= m, y2 >= H - m
        if (clip_t and clip_b) or (clip_l and clip_r):
            return None
        clip_v, clip_h = clip_t or clip_b, clip_l or clip_r
        if clip_v and clip_h:
            return None
        if clip_v:
            size = float(x2 - x1)
            if size <= 0:
                return None
            cy = y1 + size / 2.0 if clip_b else y2 - size / 2.0
            return cx, cy, size, True
        if clip_h:
            size = float(y2 - y1)
            if size <= 0:
                return None
            cx = x1 + size / 2.0 if clip_r else x2 - size / 2.0
            return cx, cy, size, True
        return cx, cy, bh, False

    def _measure(self):
        """OLC-BEKLE-DUZELT olcumu: temiz karelerden stable_frames adedi toplanir;
        medyan bh ile PARALAKS HEDEFI hesaplanir, dx/dy ortalamadan.
        Donus: None = tespit yok/guvenilmez; "WAIT" = kare birikiyor;
        {"dx","dy","bh","tx","ty"}."""
        d = self._det()
        if d is None:
            return None
        eff = self._eff_det(d)
        if eff is None:
            return None     # iki eksende kirpik: merkez guvenilmez -> kayip say
        cx0, cy0, bh0, clip = eff
        if clip:
            self._clip_tick = self._ticks   # probe-sonucsuz teshisinde kullanilir
        self._err_hist.append((cx0, cy0, bh0,
                               float(d["frame_w"]), float(d["frame_h"])))
        need = max(1, int(self._cfg["stable_frames"]))
        if len(self._err_hist) < need:
            return "WAIT"
        self._err_hist = self._err_hist[-need:]
        n = len(self._err_hist)
        cx = sum(e[0] for e in self._err_hist) / n
        cy = sum(e[1] for e in self._err_hist) / n
        bh_med = _median([e[2] for e in self._err_hist])
        W, H = self._err_hist[-1][3], self._err_hist[-1][4]
        tx, ty = interp_target(self._cfg.get("align_points"), bh_med)
        dx = cx - W * tx
        dy = cy - H * ty
        m = {"dx": dx, "dy": dy, "bh": bh_med, "tx": tx, "ty": ty}
        self.live = {"dx": round(dx), "dy": round(dy), "bh": round(bh_med),
                     "tx": round(tx, 3), "ty": round(ty, 3),
                     "area": d["area"], "conf": round(d.get("conf", 0), 2)}
        self._live_tick = self._ticks
        self._last_stable = {"dx": dx, "dy": dy, "bh": bh_med, "tick": self._ticks}
        if self.rl:
            self.rl.event("align_target", bh=round(bh_med, 1),
                          tx=round(tx, 3), ty=round(ty, 3),
                          dx=round(dx, 1), dy=round(dy, 1), clip=clip)
        return m

    def _grip_held(self) -> bool:
        g = getattr(self.app, "grip_state", {}).get(self.agv_id)
        return bool(g and g.get("type") == "held"
                    and int(g.get("seq", 0)) > self._grip_seq0)

    # ------------------------------------------------------------------- yasam
    def _schedule(self):
        self._after_id = self.app.after(int(self._cfg["tick_ms"]), self._tick)

    def _cancel(self):
        if self._after_id is not None:
            try:
                self.app.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None

    def _log(self, msg: str):
        if self.rl:
            self.rl.event("log", msg=msg)
        try:
            self.app._pickup_log(self.agv_id, msg)
        except Exception:
            pass

    def _set(self, state: str, msg: str):
        if self.rl:
            self.rl.event("state", frm=self.state, to=state, msg=msg,
                          tick=self._ticks, pose=list(self.cmd))
        self.state = state
        self.message = msg
        self._log(f"▶ {state}: {msg}")
        try:
            self.app._set_status(f"[KAPMA {self.agv_id}] {state}: {msg}")
        except Exception:
            pass

    def _refresh(self):
        try:
            self.app._pickup_ui_refresh(self.agv_id)
        except Exception:
            pass

    def _finish(self, reason: str):
        if self.rl:
            self.rl.event("run_end", reason=reason, state=self.state,
                          ticks=self._ticks, pose=list(self.cmd))
            self.rl.close()
            self.rl = None

    def _abort(self, msg: str):
        self._cancel()
        self._http("/magnet?s=0")
        self._set("ABORT", msg)
        self._finish("abort: " + msg)

    def _grab(self):
        self._cancel()
        self._http("/magnet?s=1")
        self._set("GRABBED", "🎯 Küp yakalandı (buton) — MIKNATIS AÇIK.")
        self._finish("grabbed")

    def _on_lost(self, where: str, limit: Optional[int] = None):
        self._lost_count += 1
        lim = limit if limit is not None else self._cfg["lost_max"]
        self._decision = f"kayıp {self._lost_count}/{lim}"
        if self._lost_count == 1 or self._lost_count % 4 == 0:
            self._log(f"⚠ tespit yok ({self._lost_count}/{lim}) [{where}] — "
                      f"{self._det_debug()}")
        if self._lost_count > lim:
            extra = ""
            if where == "SEAT":
                # SAHA 09:37: bilek seat'e donerken kup alt kenardan tasip
                # kayboluyor = kol kupe COK YAKIN/ustunde duruyor.
                extra = (" Küp kadrajın ALTINDAN çıktı — kol küpe çok yakın: "
                         "kolu biraz GERİ çek (shoulder +) veya küpü ileri koy; "
                         "ARAMA POZU'nu bilek seat'teyken kaydet."
                         if self._low_seen else " Kolu ARAMA POZU'ndan başlat.")
            self._abort(f"Küp kayboldu ({where}) — iptal." + extra)

    def _det_debug(self) -> str:
        raw = getattr(self.app, "detection_state", {}).get(self.agv_id)
        if raw is None:
            return "YOK — YOLO küp görmüyor / 🎯 Tespit toggle kapalı"
        age = time.time() - raw.get("ts", 0)
        if age > self._cfg["det_stale_s"]:
            return f"BAYAT {age:.1f}s — stream/YOLO yavaş"
        return (f"cx={raw['cx']} cy={raw['cy']} bh={raw.get('h')} "
                f"conf={raw.get('conf', 0):.2f}")

    # ------------------------------------------------------------------- tick
    def _tick(self):
        self._ticks += 1
        self._decision = ""
        self._settle_path = ""
        try:
            if self._grip_held():
                self._decision = "buton → yakalandı"
                self._grab()
            elif self._ticks > self._cfg["total_timeout_ticks"]:
                self._abort("Zaman aşımı — iptal.")
            else:
                self._step()
        except Exception as e:
            self._abort(f"hata: {e}")
        if self.rl:
            self.rl.event("tick", n=self._ticks, state=self.state,
                          pose=list(self.cmd), det=self._det_snap(),
                          err=(self.live if self._live_tick == self._ticks else None),
                          dec=self._decision, settle=self._settle_path or None,
                          lost=self._lost_count, blind=self._blind)
        if (self._cfg.get("debug") and self.active
                and self._ticks % max(1, int(self._cfg["debug_every_ticks"])) == 0):
            self._log(f"{self.state} t={self._ticks} poz={self.cmd} "
                      f"karar[{self._decision or '-'}] tespit[{self._det_debug()}]")
        if self.active:
            self._schedule()
        self._refresh()

    # ------------------------------------------------------------------- PROBE
    def _probe_init(self):
        self._pr = {"jorder": [self.SHOULDER, self.ELBOW], "ji": 0,
                    "cycle": 0, "phase": "M0", "sdir": 0, "drag": False,
                    "step": float(self._cfg["probe"]["step_deg"]),
                    "m0": None, "m1": None, "orig": None,
                    "cycles": {self.SHOULDER: [], self.ELBOW: []},
                    "escalated": False}

    def _probe_move(self, j: int, delta: float):
        """Probe adimi — max_step'ten buyuk adimlar ayni tick'te parcalanir
        (servo zaten ramping yapar). Donus: (hareket var mi, drag var mi)."""
        target = self._clamp_servo(self.cmd[j] + delta)
        moved_any = drag = False
        guard = 0
        while self.cmd[j] != target and guard < 20:
            guard += 1
            step = _clamp(target - self.cmd[j], -self._cfg["max_step_deg"],
                          self._cfg["max_step_deg"])
            ch = self._move(j, step)
            if not ch:
                break
            moved_any = True
            drag = drag or any(cj != j for cj, _, _ in ch)
            if not any(cj == j for cj, _, _ in ch):
                break   # repair j'yi sabitledi — ilerleme yok
        return moved_any, drag

    def _probe_step(self):
        """Jog probe sub-makinesi: M0 (taban olc) -> +adim -> M1 (olc) ->
        geri don -> M2 (olc, drift kontrol) -> cycle degerlendir. Her eklem
        repeats cycle; sonra _probe_finish."""
        pr = self._pr
        prc = self._cfg["probe"]
        m = self._measure()
        if m is None:
            self._on_lost("PROBE")
            return
        self._lost_count = 0
        if m == "WAIT":
            self._decision = "kare birikiyor"
            return
        j = pr["jorder"][pr["ji"]]
        name = self.SERVO_NAMES[j]
        if pr["phase"] == "M0":
            pr["m0"] = m
            pr["orig"] = self.cmd[j]
            pr["drag"] = False
            moved, drag = self._probe_move(j, pr["step"])
            pr["sdir"] = 1
            if not moved:
                moved, drag = self._probe_move(j, -pr["step"])
                pr["sdir"] = -1
            if not moved:
                self._decision = f"probe {name}: iki yön de zarfta — ölçülemez"
                self._probe_record(j, None, None, "zarf blokladı")
                self._probe_next(j)
                return
            pr["drag"] = drag
            pr["phase"] = "M1"
            self._decision = f"probe {name} {pr['sdir'] * pr['step']:+.0f}°"
            return
        if pr["phase"] == "M1":
            pr["m1"] = m
            self._probe_move(j, pr["orig"] - self.cmd[j])   # geri don
            pr["phase"] = "M2"
            self._decision = f"probe {name} geri dönüyor"
            return
        if pr["phase"] == "M2":
            m0, m1, m2 = pr["m0"], pr["m1"], m
            step_signed = pr["sdir"] * pr["step"]
            drift = abs(m2["dy"] - m0["dy"])
            if pr["drag"]:
                self._probe_record(j, None, None, "zarf sürükledi (drag)")
            elif drift > float(prc["drift_max_px"]):
                self._probe_record(j, None, None, f"taban kaydı {drift:.0f}px")
            else:
                gdy = (m1["dy"] - m0["dy"]) / step_signed
                gbh = (m1["bh"] - m0["bh"]) / step_signed
                gdx = (m1["dx"] - m0["dx"]) / step_signed
                self._probe_record(j, gdy, gbh, "ok", gdx=gdx, drift=drift)
            pr["cycle"] += 1
            pr["phase"] = "M0"
            if pr["cycle"] >= int(prc["repeats"]):
                self._probe_next(j)
            return

    def _probe_record(self, j, gdy, gbh, reason, gdx=None, drift=None):
        rec = {"gdy": gdy, "gbh": gbh, "gdx": gdx,
               "valid": gdy is not None, "reason": reason}
        self._pr["cycles"][j].append(rec)
        if self.rl:
            self.rl.event("probe", joint=self.SERVO_NAMES[j],
                          cycle=len(self._pr["cycles"][j]),
                          step=self._pr["sdir"] * self._pr["step"],
                          gdy=None if gdy is None else round(gdy, 2),
                          gbh=None if gbh is None else round(gbh, 2),
                          gdx=None if gdx is None else round(gdx, 2),
                          drift=None if drift is None else round(drift, 1),
                          reason=reason)

    def _probe_next(self, _j):
        pr = self._pr
        pr["cycle"] = 0
        pr["phase"] = "M0"
        if pr["ji"] + 1 < len(pr["jorder"]):
            pr["ji"] += 1
            return
        self._probe_finish()

    def _probe_eval(self, j) -> Optional[dict]:
        """Eklemin cycle'larini birlestir: isaretler ayni + buyukluk orani
        [0.5, 2] -> ortalama kazanc; degilse None (guvenilmez)."""
        valid = [c for c in self._pr["cycles"][j] if c["valid"]]
        if not valid:
            return None
        gdys = [c["gdy"] for c in valid]
        gbhs = [c["gbh"] for c in valid]
        if len(gdys) >= 2:
            if any(g * gdys[0] <= 0 for g in gdys):
                return None
            mags = sorted(abs(g) for g in gdys)
            if mags[0] > 0 and mags[-1] / max(mags[0], 1e-6) > 2.0:
                return None
        return {"gdy": sum(gdys) / len(gdys), "gbh": sum(gbhs) / len(gbhs)}

    def _probe_finish(self):
        prc = self._cfg["probe"]
        res = {j: self._probe_eval(j) for j in self._pr["jorder"]}
        resv = {j: r for j, r in res.items() if r is not None}   # gecerli olcumler
        cands = [j for j, r in resv.items()
                 if abs(r["gdy"]) >= float(prc["min_px_per_deg"])]
        if not cands:
            if not self._pr["escalated"]:
                self._pr["escalated"] = True
                self._pr["step"] *= 2
                self._pr["ji"] = 0
                self._pr["cycle"] = 0
                self._pr["phase"] = "M0"
                self._pr["cycles"] = {j: [] for j in self._pr["jorder"]}
                self._log(f"⚠ probe kazançları zayıf — adım {self._pr['step']:.0f}°"
                          f"'ye çıkarıldı, tekrar ölçülüyor")
                return
            if prc.get("strict", True):
                hint = (" Ölçümler kadraj kenarında KIRPIK küple yapıldı — kolu "
                        "küpü TAM gösterecek pozdan başlat (biraz geri çek)."
                        if self._ticks - self._clip_tick <= 60 else
                        " Işık/küp konumunu kontrol et.")
                self._abort("Probe sonuçsuz: eklem hareketi görüntüde ölçülemedi."
                            + hint + " (logda 'probe' olayları)")
                return
            fb = prc["fallback"]
            dyj = self.JOINT_BY_NAME.get(str(fb.get("dy_joint")), self.SHOULDER)
            dsj = self.JOINT_BY_NAME.get(str(fb.get("descend_joint")), self.ELBOW)
            self._gain = {"dy_joint": dyj,
                          "dy_gain": float(fb.get("dy_gain_px_per_deg", 3.0))
                          * (1 if int(fb.get("dy_dir", 1)) >= 0 else -1),
                          "descend_joint": dsj,
                          "descend_dir": 1 if int(fb.get("descend_dir", -1)) >= 0 else -1,
                          "fallback": True}
            self._log("⚠ probe sonuçsuz — config fallback eksenleri kullanılıyor!")
        else:
            dyj = max(cands, key=lambda j: abs(resv[j]["gdy"]))
            other = [j for j in self._pr["jorder"] if j != dyj][0]
            min_bh = float(prc["min_bh_px_per_deg"])
            if other in resv and abs(resv[other]["gbh"]) >= min_bh:
                dsj, dsg = other, resv[other]["gbh"]
            elif abs(resv[dyj]["gbh"]) >= min_bh:
                dsj, dsg = dyj, resv[dyj]["gbh"]
            else:
                fb = prc["fallback"]
                dsj = self.JOINT_BY_NAME.get(str(fb.get("descend_joint")), self.ELBOW)
                dsg = float(1 if int(fb.get("descend_dir", -1)) >= 0 else -1)
                self._log("⚠ alçalma (bh) kazancı iki eklemde de zayıf — "
                          "fallback alçalma ekseni kullanılıyor")
            self._gain = {"dy_joint": dyj, "dy_gain": resv[dyj]["gdy"],
                          "descend_joint": dsj,
                          "descend_dir": 1 if dsg > 0 else -1,
                          "descend_gain": dsg,
                          # alcalma ekleminin dy YAN ETKISI (ileri kayma) —
                          # _descend_step bunu dy ekseniyle olcumlu telafi eder
                          "descend_dy_gain": (resv[dsj]["gdy"]
                                              if dsj in resv else None),
                          "fallback": False}
        g = self._gain
        if self.rl:
            self.rl.event("probe_result",
                          dy_joint=self.SERVO_NAMES[g["dy_joint"]],
                          dy_gain=round(g["dy_gain"], 2),
                          descend_joint=self.SERVO_NAMES[g["descend_joint"]],
                          descend_dir=g["descend_dir"],
                          fallback=g.get("fallback", False),
                          reprobe=self._reprobes,
                          raw={self.SERVO_NAMES[j]: r for j, r in res.items()})
        self._dy_prev = None
        self._dy_grow = 0
        self._set("ALIGN", f"Probe tamam: dy→{self.SERVO_NAMES[g['dy_joint']]} "
                           f"(kazanç {g['dy_gain']:+.1f}px/°), alçalma→"
                           f"{self.SERVO_NAMES[g['descend_joint']]} "
                           f"({g['descend_dir']:+d} yönü). Hizalanıyor…")

    # ------------------------------------------------------------------- ENDGAME
    def _enter_endgame(self, trigger: str):
        eg = self._cfg["endgame"]
        ls = self._last_stable or {}
        if self.rl:
            self.rl.event("endgame", trigger=trigger,
                          last_dx=round(ls.get("dx", 0), 1),
                          last_dy=round(ls.get("dy", 0), 1),
                          last_bh=round(ls.get("bh", 0), 1),
                          contact_bh=self._contact_bh,
                          magnet_on=self._magnet_on)
        self._blind = 0
        self._grace = 0
        self._stalls = 0
        if eg.get("magnet_early", True) and not self._magnet_on:
            self._magnet_on = True
            self._http("/magnet?s=1")
        self._set("ENDGAME", "Görüş gitti (küp çok yakın) — kör son iniş, "
                             "mıknatıs AÇIK, buton bekleniyor…")

    def _lost_endgame_ok(self) -> bool:
        """Kayip tetigi B: kup yakinda kayboldu (cok yakin olabilir) VE
        kaybolmadan onceki son stabil olcum hem yakin hem hizaliydi."""
        ls = self._last_stable
        if not ls:
            return False
        eg = self._cfg["endgame"]
        gate = float(self._cfg["align"]["descend_gate_px"])
        return (ls["bh"] >= self._contact_bh * float(eg["lost_bh_frac"])
                and abs(ls["dx"]) <= gate and abs(ls["dy"]) <= gate)

    # ------------------------------------------------------------------- adimlar
    def _step(self):
        st = self.state

        # --- SEAT: bilek seat acisina (paralaks modelinin on kosulu) ---
        if st == "SEAT":
            if self._settling():
                return
            d = self._det()
            if d is None:
                self._on_lost("SEAT", limit=self._cfg["lost_max"] // 2)
                return
            self._lost_count = 0
            # kup alt kenara dayaniyor mu? (kayip olursa teshis mesaji icin)
            self._low_seen = (float(d.get("y2") or d["cy"]) >= d["frame_h"] - 6
                              or d["cy"] >= 0.85 * d["frame_h"])
            seat = self._clamp_servo(seat_deg(self._cfg.get("seat_points"),
                                              self.cmd[self.SHOULDER],
                                              self.cmd[self.ELBOW],
                                              self._cfg.get("seat_fallback_deg", 90)))
            if abs(self.cmd[self.GRIPPER] - seat) <= 1:
                self._probe_init()
                self._set("PROBE", "Bilek seat'te — eklem yön/kazanç ölçümü (jog)…")
                return
            self._decision = f"seat {self.cmd[self.GRIPPER]}→{seat}"
            if not self._move(self.GRIPPER, seat - self.cmd[self.GRIPPER]):
                self._abort("Bilek seat açısına zarf izin vermiyor — seat/zarf "
                            "kalibrasyonu uyumsuz.")
            return

        # --- PROBE: jog yon/kazanc olcumu ---
        if st == "PROBE":
            if self._settling():
                return
            self._probe_step()
            return

        # --- ALIGN: olc-bekle-duzelt — hareketli paralaks hedefiyle ---
        if st == "ALIGN":
            if self._settling():
                return
            aln = self._cfg["align"]
            eg = self._cfg["endgame"]
            m = self._measure()
            if m is None:
                # Gorus gitti (kup ekrani kapladi / kadraj disi). Yeterince
                # yakin + hizadaysak lost_wait_s bekleyip KOR inise gec.
                wait_ticks = max(1, round(float(eg.get("lost_wait_s", 0.5)) * 1000.0
                                          / float(self._cfg["tick_ms"])))
                if (self._lost_count + 1 >= wait_ticks
                        and self._lost_endgame_ok()):
                    self._enter_endgame("lost")
                    return
                self._on_lost("ALIGN")
                return
            self._lost_count = 0
            if m == "WAIT":
                self._decision = "kare birikiyor"
                return
            dx, dy, bh = m["dx"], m["dy"], m["bh"]
            gate = float(aln["descend_gate_px"])
            # Temas bolgesi: KORLESME YOK — gorus varken gorsel inis butona
            # kadar surer; burada yalniz MIKNATIS erken acilir (son mm'lerde
            # kupu ceker, butonu erken tetikler).
            if (eg.get("magnet_early", True) and not self._magnet_on
                    and bh >= self._contact_bh * float(eg["bh_frac"])
                    and abs(dx) <= gate and abs(dy) <= gate):
                self._magnet_on = True
                self._http("/magnet?s=1")
                if self.rl:
                    self.rl.event("magnet_early", bh=round(bh, 1),
                                  dx=round(dx, 1), dy=round(dy, 1))
                self._log(f"🧲 temas bölgesi (bh={bh:.0f}/{self._contact_bh:.0f}) "
                          f"— mıknatıs AÇIK, görsel iniş sürüyor…")
            if abs(dx) > float(aln["deadband_x_px"]):
                self._decision = f"base dx={dx:+.0f}"
                self._move(self.BASE,
                           -self._cfg["kp_base"] * dx * self._cfg["base_dir"])
                return
            if abs(dy) > float(aln["deadband_y_px"]):
                # iraksamayi izle: dy duzeltmeleri ust uste buyuyorsa kazanc
                # yanlis/poz degisti -> yeniden probe
                if self._dy_prev is not None and abs(dy) > self._dy_prev + 1:
                    self._dy_grow += 1
                else:
                    self._dy_grow = 0
                self._dy_prev = abs(dy)
                if self._dy_grow >= int(aln["diverge_max"]):
                    if self._reprobes < int(aln["reprobe_max"]):
                        self._reprobes += 1
                        self._probe_init()
                        self._set("PROBE", f"dy ıraksıyor (|dy| {abs(dy):.0f}px) — "
                                           f"yeniden probe ({self._reprobes}.)")
                    else:
                        self._abort("Hizalama ıraksıyor (probe tekrarları tükendi) "
                                    "— logda align_target/probe olaylarına bak.")
                    return
                g = self._gain or {}
                gain = float(g.get("dy_gain", 3.0))
                delta = -dy / gain * float(aln["dy_damping"])
                delta = _clamp(delta, -self._cfg["max_step_deg"],
                               self._cfg["max_step_deg"])
                if abs(delta) < 1:
                    delta = 1.0 if delta >= 0 else -1.0
                self._decision = (f"dy={dy:+.0f} → "
                                  f"{self.SERVO_NAMES[g.get('dy_joint', 1)]} "
                                  f"{delta:+.1f}°")
                self._move(int(g.get("dy_joint", self.SHOULDER)), delta)
                return
            # hizada -> ALCALMA adimi (+olculmus ileri telafisi + seat, ayni parti)
            self._dy_grow = 0
            g = self._gain or {}
            dsj = int(g.get("descend_joint", self.ELBOW))
            ch = self._descend_step()
            if ch:
                self._stalls = 0
                self._decision = (f"alçal {self.SERVO_NAMES[dsj]} "
                                  f"(bh={bh:.0f}/{self._contact_bh:.0f})")
            else:
                self._stalls += 1
                self._decision = f"alçalma bloklandı ({self._stalls})"
                if self._stalls >= int(aln["stall_max"]):
                    self._abort("Alçalma zarf sınırında kilitli — arm_sim'de bu "
                                "bölgeyi (alçak pozlar) genişletmek gerek.")
            return

        # --- ENDGAME: kor son inis (buton _tick basinda yakalanir) ---
        if st == "ENDGAME":
            if self._settling():
                return
            eg = self._cfg["endgame"]
            # Gorus GERI geldiyse korluge gerek yok — gorsel inise don
            m = self._measure()
            if isinstance(m, dict):
                self._dy_grow = 0
                self._stalls = 0
                self._lost_count = 0
                self._set("ALIGN", "Görüş geri geldi — görsel iniş devam…")
                return
            if m == "WAIT":
                self._decision = "kare birikiyor"
                return
            if self._blind < int(eg["blind_max_ticks"]):
                self._blind += 1
                ch = self._descend_step()
                if ch:
                    self._stalls = 0
                    self._decision = f"kör iniş {self._blind}/{eg['blind_max_ticks']}"
                else:
                    self._stalls += 1
                    self._decision = f"kör iniş bloklandı ({self._stalls})"
                    if self._stalls >= 2:
                        self._blind = int(eg["blind_max_ticks"])   # bekleme fazina gec
                return
            self._grace += 1
            self._decision = f"buton bekleniyor {self._grace}/{eg['grace_ticks']}"
            if self._grace > int(eg["grace_ticks"]):
                self._abort("Kör iniş bitti, buton tetiklenmedi — küp mıknatıs "
                            "altında değil (logda 'endgame' olayındaki son "
                            "dx/dy/bh'ye bak).")
            return

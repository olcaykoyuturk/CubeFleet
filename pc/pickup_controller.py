"""
Otonom kup kapma — v2: KILIT + OGRETILMIS DALIS mimarisi.

NEDEN v2: eski tam-gorsel inis (APPROACH) son 10-15 cm'de tutarsizdi —
kamera bilekle birlikte dondugu icin "kup goruntude nerede gorunmeli" hedefi
poza gore kayiyor (eye-in-hand problemi), kup temas aninda kadrajdan cikiyor,
dikey hatayi hangi eklemin duzeltecegi pozdan poza degisiyordu. v2 gorusu
YALNIZ guvenilir oldugu bolgede kullanir; son inis kullanicinin OGRETTIGI
deterministik harekettir ve BUTON (grip mikroswitch) sonlandirir.

Durum makinesi:
    IDLE -> TRACK   : kup kadrajda kabaca hedefe (dx -> BASE, dy -> GRIPPER)
         -> GO_LOCK : sh/el/gr KILIT POZUNA adim adim tasinir; araya
                      olc-bekle-duzelt ritmiyle base yatay duzeltmesi girer
                      (fast_advance adimda bir olcum — hiz/kontrol dengesi)
         -> FINE    : kilit pozunda ince nisan — kup goruntude KILIT HEDEF
                      pikseline oturtulur (dx -> BASE, dy -> SHOULDER:
                      kup hedefin ustunde = uzak -> ileri uzan; altinda =
                      yakin -> geri cekil)
         -> PLUNGE  : ogretilmis dalis yolunun KOR tekrari. Birden fazla dalis
                      PROFILI ogretilebilir (her biri kendi baslangic pozuyla);
                      FINE bitince mevcut poza EN YAKIN baslangicli profil
                      secilir (secim + aday mesafeleri loglanir). FINE'in
                      duzeltmesi korunsun diye yol GORELI uygulanir
                      (hedef = fine_son_pozu + (yol_noktasi - profil_baslangici)).
                      Gorsel karar YOK; her tick BUTON kontrolu.
         -> GRABBED : buton tetiklendi -> MIKNATIS ACIK (/magnet?s=1), dur.
         \\-> ABORT : kayip / zaman asimi / zarf blogu / estop (miknatis OFF)

KALIBRASYON (pickup_test.py ile):
  1) arm_sim sinir sihirbazi -> safe_envelope (zorunlu — yoksa start reddeder)
  2) KILIT: kolu "kup tam miknatisin altinda + kup rahat gorunur" yukseklige
     getir, kupu miknatisin TAM altina koy -> "KILIT kaydet":
     lock_pose=[sh,el,gr] + kupun o anki goruntu MERKEZI lock_col/row_frac.
  3) DALIS: kilit pozundan sliderlarla kupe kadar IN (buton tetiklenene dek),
     yol boyunca "Dalis noktasi ekle" -> plunge_profiles=[{"start":[sh,el,gr],
     "path":[[sh,el,gr],...]}, ...] (son nokta = buton temas pozu). FARKLI kup
     mesafeleri icin birden cok profil ogretilebilir ("YENI DALIS" baslangic
     pozunu kaydeder); eski tek-yol plunge_path kaydi otomatik profil sayilir.

KOSU LOGU (debug'in tek dogru kaynagi): her kosu
pc/pickup_logs/run_YYYYmmdd_HHMMSS.jsonl dosyasina JSONL yazar — config
anlik goruntusu, HER tick (durum/poz/tespit/hata/karar/sayaclar), her servo
komutu (istenen -> kistirilan), zarf engelleri, durum gecisleri, grip
olaylari, bitis nedeni. Kosudan sonra dosya satir satir okunarak "neden
boyle davrandi" sorusu kesin cevaplanir.

Servo yon semantigi (kullanici olcumu): BASE 0=SOL/180=SAG,
SHOULDER 0=ILERI/180=GERI, ELBOW 0=ILERI/180=GERI, GRIPPER 0=GERI/180=ILERI.

GUVENLI BOLGE: arm_sim SINIR SIHIRBAZI (type="limits") — elbow bandi
shoulder'a, gripper bandi (shoulder, elbow) ciftine bagli; duraklar arasi
lineer interpolasyon, olculmemis bolge YASAK. Her servo komutu
_move -> _repair -> _envelope_ok zincirinden gecer; PLUNGE dahil.

Kontrolor UI thread'inde `app.after(tick_ms)` ile tick'ler; `app.detection_state`
(en guncel YOLO sonucu) okur, `app._arm_http_get` ile /servo yollar. Servo
acilarini LOKAL takip eder (komut edilen hedef; geri besleme kamera + buton).
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
    Iki komsu shoulder satirinin el-bantlari arasinda sh'e gore harmanlanir;
    herhangi biri tanimsizsa None -> YASAK. Ucu bag (sh, el, gr) boylece korunur."""
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
            # gercek olcum (saha: sh=28'de sh=0 satiri el<84'u kapsamiyordu,
            # inis butona 2° kala bloklaniyordu).
            if b1 is None:
                return b2
            if b2 is None:
                return b1
            t = (sh - shs[i]) / (shs[i + 1] - shs[i])
            return (b1[0] + (b2[0] - b1[0]) * t, b1[1] + (b2[1] - b1[1]) * t)
    return None


class RunLogger:
    """Kosu basina bir JSONL dosyasi (pc/pickup_logs/run_*.jsonl).

    Amac: UI logundaki ozet satirlarin aksine HICBIR SEYI ATLAMAYAN makine
    okunur kayit — kosu bittikten sonra dosya okunarak her tick'te kontrolorun
    ne gordugu, ne karar verdigi ve hangi komutu yolladigi geri sarilabilir.
    Satir formati: {"t": kosu_baslangicindan_sn, "kind": tur, ...alanlar}."""

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

    DEFAULTS = {
        # --- KILIT kalibrasyonu (pickup_test "KILIT kaydet") — ZORUNLU ---
        # lock_pose: gorusun guvenilir oldugu "kup tam miknatisin altinda"
        # yuksekligindeki [sh, el, gr]; lock_*_frac: o pozda kup MERKEZININ
        # goruntude gorundugu hedef piksel (FINE bu pikselle nisan alir).
        "lock_pose": None,
        "lock_col_frac": None, "lock_row_frac": None,
        # --- OGRETILMIS DALIS — ZORUNLU (>=1 profil) ---
        # plunge_profiles: [{"start": [sh,el,gr], "path": [[sh,el,gr],...]},...]
        # Her profil = bir baslangic pozundan kupe elle inilen yol (son nokta =
        # buton temas pozu). FINE bitince mevcut poza EN YAKIN baslangicli
        # profil secilir ve GORELI tekrar edilir. plunge_path: ESKI tek-yol
        # format (baslangici lock_pose sayilir) — profil yoksa ona dusulur.
        "plunge_profiles": [],
        "plunge_path": [],
        # --- goruntu hedef bantlari ---
        "deadband_x_px": 12, "deadband_y_px": 12,
        # --- TRACK kazanc/yon (saha kalibreli: her ikisi +1) ---
        "kp_base": 0.09, "base_dir": 1,
        "kp_gripper": 0.10, "gripper_dir": 1,
        # --- FINE: dy -> SHOULDER ileri/geri ---
        # dy>0 = kup hedef pikselin ALTINDA = miknatisa gore YAKIN -> shoulder
        # ARTAR (0=ILERI semantigi: artmak = geri cekilmek). Ters cikarsa
        # fine_dir=-1 (config'ten).
        "fine_step_deg": 2, "fine_dir": 1, "fine_timeout_ticks": 200,
        # --- GO_LOCK: olcumler arasi ardisik poz adimi sayisi ---
        # Poz adimi gorsel karar GEREKTIRMEZ (hedef sabit kilit pozu) ->
        # fast_advance adim pes pese atilir, sonra durup olculur (base duzeltme).
        "fast_advance": 3,
        # --- PLUNGE: yol bitince butonsuz beklenecek tick (sonra iptal) ---
        "plunge_grace_ticks": 8,
        # pickup_speed_ms 40 = ~25°/sn servo hizi. max_step_deg=2 (13°/s komut)
        # bilerek servo hizinin ALTINDA -> surekli ama sakin, sallantisiz surus.
        "max_step_deg": 2, "pickup_speed_ms": 40,
        "tick_ms": 150, "det_stale_s": 0.7,
        # STABILIZASYON (OLC-BEKLE-DUZELT): her hareketten sonra settle_ticks
        # bekle (servo bitsin + kamera otursun), sonra stable_frames temiz kare
        # ORTALANIP tek duzeltme yapilir — sallanan kameradan karar YOK.
        "settle_ticks": 2, "stable_frames": 2,
        "lost_max": 20, "total_timeout_ticks": 600,
        "debug": True, "debug_every_ticks": 4,   # ~0.6 sn'de bir UI durum satiri
    }

    def __init__(self, app, agv_id: str):
        self.app = app
        self.agv_id = agv_id
        self.state = "IDLE"
        self.message = "hazır"
        self.live: dict = {}            # {dx, dy, area, conf} — UI gosterimi
        self._cfg: dict = dict(self.DEFAULTS)
        self._after_id = None
        self.cmd = [93, 120, 60, 110]   # komut edilen servo acilari (lokal takip)
        self.rl: Optional[RunLogger] = None
        self._decision = ""             # bu tick'in karari (dosya loguna gider)
        self._lost_count = 0
        self._ticks = 0
        self._grip_seq0 = 0             # start aninda grip seq (eski olaylar sayilmaz)
        self._cooldown = 0              # hareket sonrasi bekleme (servo + kamera otursun)
        self._err_hist: list = []       # temiz kare (dx, dy) birikimi — ortalanir
        self._live_tick = -1            # self.live en son hangi tick'te hesaplandi
        self._fast = 0                  # GO_LOCK: kalan olcumsuz hizli adim
        self._fine_ticks = 0
        self._profiles: list = []       # normalize dalis profilleri (start'ta dolar)
        self._plunge: Optional[dict] = None        # secilen profil {"start","path"}
        self._plunge_base: Optional[list] = None   # FINE bitis pozu (goreli dalis tabani)
        self._plunge_i = 0
        self._grace = 0

    # ------------------------------------------------------------------ kontrol
    def start(self) -> bool:
        """MEVCUT kol pozundan sekansi baslat (kullanici kolu konumlar).
        Zarf + KILIT + DALIS kalibrasyonlarindan biri eksikse reddeder ve
        hangisinin eksik oldugunu acikca soyler."""
        auto = (getattr(self.app, "pickup_cfg", None) or {}).get("autonomous", {})
        self._cfg = {**self.DEFAULTS, **auto}
        if self._envelope() is None:
            self._set("IDLE", "Önce arm_sim SINIR SİHİRBAZI'nı çalıştırıp kaydet.")
            return False
        missing = []
        lock = self._cfg.get("lock_pose")
        if not (isinstance(lock, (list, tuple)) and len(lock) == 3):
            missing.append("KİLİT pozu")
        if self._cfg.get("lock_col_frac") is None or self._cfg.get("lock_row_frac") is None:
            missing.append("KİLİT hedef pikseli")
        self._profiles = self._norm_profiles()
        if not self._profiles:
            missing.append("DALIŞ yolu")
        if missing:
            self._set("IDLE", "Eksik kalibrasyon: " + ", ".join(missing) +
                              " — pickup_test 'KİLİT & DALIŞ' bölümünden kaydet.")
            return False
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
        self._cooldown = 0
        self._err_hist = []
        self._fast = 0
        self._fine_ticks = 0
        self._plunge = None
        self._plunge_base = None
        self._plunge_i = 0
        self._grace = 0
        # Eski grip olaylari sayilmasin — yalniz BU sekans sirasinda basilan buton
        g = getattr(self.app, "grip_state", {}).get(self.agv_id) or {}
        self._grip_seq0 = int(g.get("seq", 0))
        # KOSU LOGU — config anlik goruntusu (zarf haric: buyuk + zaten diskte)
        self.rl = RunLogger(self.agv_id)
        self.rl.event("config",
                      cfg={k: v for k, v in self._cfg.items() if k != "safe_envelope"},
                      envelope=True, start_pose=list(pose), grip_seq0=self._grip_seq0)
        # Yavas hiz — yumusak hareket; miknatis temiz baslangic icin KAPALI
        # (kalibrasyondan acik kalmissa 30 sn termal timeout'a takiliyordu)
        self._http(f"/servo/speed?ms={int(self._cfg['pickup_speed_ms'])}")
        self._http("/magnet?s=0")
        self.cmd = pose
        self._log(f"📄 koşu logu: {self.rl.path}")
        self._set("TRACK", f"Mevcut pozdan başladı {pose} — küp hedefe ortalanıyor…")
        self._schedule()
        return True

    def _current_pose(self):
        """Kolun su anki pozu: Arm slider degerleri (kullanici kolu oradan
        konumlar); yoksa /poll arm state hedefleri."""
        try:
            vars_ = self.app.arm_servo_vars.get(self.agv_id)
            if vars_ and len(vars_) == 4:
                return [self._clamp_servo(v.get()) for v in vars_]
        except Exception:
            pass
        try:
            t = (getattr(self.app, "_cam_arm_state", {}) or {}).get("targets") or []
            if len(t) == 4:
                return [self._clamp_servo(x) for x in t]
        except Exception:
            pass
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

    # ------------------------------------------------------------------- dahili
    def _clamp_servo(self, angle) -> int:
        """Firmware mekanik araligi icin 0..180 sanity clamp."""
        return round(_clamp(angle, 0, 180))

    # --- GUVENLI ALAN (arm_sim sinir sihirbazi: limit bantlari + interpolasyon) ---
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
        """Bu shoulder acisinda elbow'un izinli (lo, hi) bandi (margin uygulanmis)."""
        env = self._envelope()
        if env is None:
            return None
        return self._margined(
            interp_band(env["sh_grid"], env["el_min"], env["el_max"], sh))

    def _gr_band(self, sh, el):
        """(SHOULDER, ELBOW) ciftine bagli gripper bandi (margin uygulanmis)."""
        env = self._envelope()
        if env is None:
            return None
        return self._margined(gr_band(env.get("gr_rows") or [], sh, el))

    def _envelope_ok(self, cmd) -> bool:
        """cmd guvenli zarf icinde mi? elbow shoulder'a bagli bantta; gripper
        (shoulder, elbow) CIFTINE bagli bantta olmali. Zarf yoksa True."""
        if self._envelope() is None:
            return True
        eb = self._el_band(cmd[self.SHOULDER])
        if eb is None or not (eb[0] - 0.5 <= cmd[self.ELBOW] <= eb[1] + 0.5):
            return False
        gb = self._gr_band(cmd[self.SHOULDER], cmd[self.ELBOW])
        return gb is not None and (gb[0] - 0.5 <= cmd[self.GRIPPER] <= gb[1] + 0.5)

    def _repair(self, cand):
        """Aday pozu zarfa kistir: elbow'u shoulder bandina, gripper'i (shoulder,
        elbow) bandina clamp et. Band tanimsizsa None -> hareket iptal."""
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

    def _http(self, path: str):
        try:
            self.app._arm_http_get(self.agv_id, path)
        except Exception:
            pass

    def _move(self, i: int, delta: float) -> bool:
        """Bir servoyu delta kadar oynat — tick adimi + 0..180 + GUVENLI ZARF.
        Aday poz _repair ile banda kistirilir; band tanimsizsa hareket YAPILMAZ.
        Donus: gercekten hareket gonderildi mi? Her sonuc dosya loguna yazilir."""
        ms = self._cfg["max_step_deg"]
        want = delta
        delta = _clamp(delta, -ms, ms)
        new = self._clamp_servo(self.cmd[i] + delta)
        if new == self.cmd[i]:
            return False
        candidate = list(self.cmd)
        candidate[i] = new
        candidate = self._repair(candidate)
        if candidate is None or not self._envelope_ok(candidate):
            # ZARF ENGELLEDI — dosyaya her sefer, UI'a throttle'li
            if self.rl:
                self.rl.event("block", servo=self.SERVO_NAMES[i],
                              frm=self.cmd[i], to=new, want=round(want, 2))
            if self._ticks - getattr(self, "_last_block_log", -99) >= 6:
                self._last_block_log = self._ticks
                self._log(f"⛔ zarf engelledi: {self.SERVO_NAMES[i]} "
                          f"{self.cmd[i]}→{new} (ölçülmemiş/yasak bölge)")
            return False
        changed = [(j, self.cmd[j], candidate[j])
                   for j in range(4) if candidate[j] != self.cmd[j]]
        if not changed:
            return False
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
        # STABILIZASYON: hareket gonderildi -> servo bitene + kamera oturana
        # kadar yeni gorsel karar yok; eski (bulanik) olcumler gecersiz
        self._cooldown = max(self._cooldown, int(self._cfg.get("settle_ticks", 2)))
        self._err_hist = []
        return True

    def _grip_held(self) -> bool:
        """Grip mikroswitch BU sekans icinde tetiklendi mi? (/poll -> app.grip_state;
        start anindaki seq'ten yeni 'held' olayi = kup yakalandi.)"""
        g = getattr(self.app, "grip_state", {}).get(self.agv_id)
        return bool(g and g.get("type") == "held"
                    and int(g.get("seq", 0)) > self._grip_seq0)

    def _det(self) -> Optional[dict]:
        d = getattr(self.app, "detection_state", {}).get(self.agv_id)
        if not d:
            return None
        if time.time() - d.get("ts", 0) > self._cfg["det_stale_s"]:
            return None
        return d

    def _det_snap(self) -> Optional[dict]:
        """Dosya logu icin HAM tespit anlik goruntusu (bayatlik filtresiz —
        'o anda ekranda ne vardi' sorusunun cevabi)."""
        raw = getattr(self.app, "detection_state", {}).get(self.agv_id)
        if not raw:
            return None
        return {"cx": raw["cx"], "cy": raw["cy"], "area": raw["area"],
                "conf": round(raw.get("conf", 0), 2),
                "age": round(time.time() - raw.get("ts", 0), 2)}

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
        """Olay logu -> uygulamanin log paneli + dosya."""
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
        """Kosu logunu sonlandir (bitis nedeni + ozet) ve dosyayi kapat."""
        if self.rl:
            self.rl.event("run_end", reason=reason, state=self.state,
                          ticks=self._ticks, pose=list(self.cmd))
            self.rl.close()
            self.rl = None

    def _abort(self, msg: str):
        self._cancel()
        self._http("/magnet?s=0")   # iptalde miknatis ACIK kalmasin
        self._set("ABORT", msg)
        self._finish("abort: " + msg)

    def _grab(self):
        self._cancel()
        self._http("/magnet?s=1")
        self._set("GRABBED", "🎯 Küp yakalandı (buton) — MIKNATIS AÇIK.")
        self._finish("grabbed")

    def _on_lost(self, where: str):
        self._lost_count += 1
        self._decision = f"kayıp {self._lost_count}/{self._cfg['lost_max']}"
        if self._lost_count == 1 or self._lost_count % 4 == 0:
            self._log(f"⚠ tespit yok ({self._lost_count}/{self._cfg['lost_max']}) "
                      f"[{where}] — {self._det_debug()}")
        if self._lost_count > self._cfg["lost_max"]:
            self._abort(f"Küp kayboldu ({where}) — iptal.")

    # ------------------------------------------------------------------- tick
    def _tick(self):
        self._ticks += 1
        self._decision = ""
        try:
            # BUTON her durumda, her tick'te ilk kontrol — temas = yakalama.
            if self._grip_held():
                self._decision = "buton → yakalandı"
                self._grab()
            elif self._ticks > self._cfg["total_timeout_ticks"]:
                self._abort("Zaman aşımı — iptal.")
            else:
                self._step()
        except Exception as e:
            self._abort(f"hata: {e}")
        # DOSYA LOGU: her tick'in tam anlik goruntusu (durum ne olursa olsun)
        if self.rl:
            self.rl.event("tick", n=self._ticks, state=self.state,
                          pose=list(self.cmd), det=self._det_snap(),
                          err=(self.live if self._live_tick == self._ticks else None),
                          dec=self._decision, cd=self._cooldown,
                          lost=self._lost_count, fast=self._fast,
                          fine_t=self._fine_ticks, wp=self._plunge_i)
        if (self._cfg.get("debug") and self.active
                and self._ticks % max(1, int(self._cfg["debug_every_ticks"])) == 0):
            self._log(f"{self.state} t={self._ticks} poz={self.cmd} "
                      f"karar[{self._decision or '-'}] tespit[{self._det_debug()}]")
        if self.active:
            self._schedule()
        self._refresh()

    def _det_debug(self) -> str:
        """Tespit durumunun UI ozeti: 'neden takip etmiyor' sorusuna cevap."""
        raw = getattr(self.app, "detection_state", {}).get(self.agv_id)
        if raw is None:
            return "YOK — YOLO küp görmüyor / 🎯 Tespit toggle kapalı"
        age = time.time() - raw.get("ts", 0)
        if age > self._cfg["det_stale_s"]:
            return f"BAYAT {age:.1f}s — stream/YOLO yavaş"
        return (f"cx={raw['cx']} cy={raw['cy']} area={raw['area']} "
                f"conf={raw.get('conf', 0):.2f}")

    def _errors(self, d):
        """Goruntu hatalari — hedef: KILIT HEDEF PIKSELI (kilit pozunda kup tam
        miknatisin altindayken kup merkezinin gorundugu yer).
          dx = kup_merkez_x - hedef_x   (+ = kup hedefin SAGINDA)
          dy = kup_merkez_y - hedef_y   (+ = kup hedefin ALTINDA = yakin)"""
        W, H = d["frame_w"], d["frame_h"]
        dx = d["cx"] - W * float(self._cfg["lock_col_frac"])
        dy = d["cy"] - H * float(self._cfg["lock_row_frac"])
        self.live = {"dx": round(dx), "dy": round(dy),
                     "area": d["area"], "conf": round(d.get("conf", 0), 2)}
        self._live_tick = self._ticks
        return dx, dy

    def _stable_errors(self):
        """STABILIZASYON: hareket sonrasi (cooldown bitti) gelen TEMIZ karelerden
        stable_frames adedi ORTALANIR — tek karelik YOLO zipirtisi karari bozamaz.
        Donus: None = tespit yok; "WAIT" = temiz kare birikiyor; (dx, dy)."""
        d = self._det()
        if d is None:
            return None
        dx, dy = self._errors(d)
        self._err_hist.append((dx, dy))
        need = max(1, int(self._cfg["stable_frames"]))
        if len(self._err_hist) < need:
            return "WAIT"
        self._err_hist = self._err_hist[-need:]
        n = len(self._err_hist)
        return (sum(e[0] for e in self._err_hist) / n,
                sum(e[1] for e in self._err_hist) / n)

    # ------------------------------------------------------------------- dalis profilleri
    def _norm_profiles(self) -> list:
        """Config'teki dalis kayitlarini tek bicime getir:
        [{"start": [sh,el,gr], "path": [[sh,el,gr],...]}, ...].
        Bozuk/eksik girdiler atlanir. Hic profil yoksa ESKI tek-yol format
        (plunge_path, baslangici lock_pose) tek profil olarak kabul edilir."""
        out = []
        for p in self._cfg.get("plunge_profiles") or []:
            try:
                st = [self._clamp_servo(v) for v in (p.get("start") or [])]
                path = [[self._clamp_servo(x) for x in wp] for wp in (p.get("path") or [])]
            except Exception:
                continue
            if len(st) == 3 and path and all(len(wp) == 3 for wp in path):
                out.append({"start": st, "path": path})
        if not out:
            lock = self._cfg.get("lock_pose")
            legacy = self._cfg.get("plunge_path") or []
            if (isinstance(lock, (list, tuple)) and len(lock) == 3 and legacy
                    and all(len(wp) == 3 for wp in legacy)):
                out.append({"start": [self._clamp_servo(v) for v in lock],
                            "path": [[self._clamp_servo(x) for x in wp] for wp in legacy]})
        return out

    def _select_profile(self) -> dict:
        """FINE bitis pozuna (sh, el, gr) EN YAKIN baslangicli dalis profilini
        sec — kup yakin/uzakken FINE shoulder'i kaydirir, farkli mesafeler icin
        ogretilen profillerden dogru olani boyle bulunur. Secim ve tum aday
        mesafeleri dosya loguna yazilir (yanlis profil secildiyse logdan gorulur)."""
        pose = (self.cmd[self.SHOULDER], self.cmd[self.ELBOW], self.cmd[self.GRIPPER])
        dists = []
        for p in self._profiles:
            st = p["start"]
            dists.append(round(((pose[0] - st[0]) ** 2 + (pose[1] - st[1]) ** 2
                                + (pose[2] - st[2]) ** 2) ** 0.5, 1))
        best = min(range(len(self._profiles)), key=lambda i: dists[i])
        prof = self._profiles[best]
        if self.rl:
            self.rl.event("plunge_select", chosen=best, dist=dists[best],
                          starts=[p["start"] for p in self._profiles],
                          dists=dists, pose=list(pose))
        if len(self._profiles) > 1:
            self._log(f"🎯 dalış profili #{best + 1}/{len(self._profiles)} seçildi "
                      f"(başlangıç {prof['start']}, mesafe {dists[best]}°; "
                      f"adaylar {dists})")
        return prof

    # ------------------------------------------------------------------- adimlar
    def _lock_step(self) -> str:
        """sh/el/gr'yi kilit pozuna dogru birer adim (≤max_step) tasi.
        Donus: "arrived" (±1° icinde) / "moved" / "stuck" (zarf blokladi)."""
        lock = self._cfg["lock_pose"]
        targets = ((self.SHOULDER, lock[0]), (self.ELBOW, lock[1]),
                   (self.GRIPPER, lock[2]))
        if all(abs(self.cmd[i] - t) <= 1 for i, t in targets):
            return "arrived"
        moved = False
        for i, t in targets:
            if self.cmd[i] != t:
                moved = self._move(i, t - self.cmd[i]) or moved
        if moved:
            return "moved"
        return "arrived" if all(abs(self.cmd[i] - t) <= 1 for i, t in targets) \
            else "stuck"

    def _step(self):
        st = self.state

        # --- TRACK: kup kadrajda hedef piksele kabaca (dx->BASE, dy->GRIPPER) ---
        if st == "TRACK":
            if self._cooldown > 0:
                self._cooldown -= 1
                self._decision = f"cooldown {self._cooldown}"
                return
            res = self._stable_errors()
            if res is None:
                self._on_lost("TRACK")
                return
            self._lost_count = 0
            if res == "WAIT":
                self._decision = "kare birikiyor"
                return
            dx, dy = res
            if (abs(dx) <= self._cfg["deadband_x_px"]
                    and abs(dy) <= self._cfg["deadband_y_px"]):
                self._fast = 0
                self._set("GO_LOCK", "Hedefte — kilit pozuna gidiliyor…")
                return
            self._decision = f"ortala dx={dx:+.0f} dy={dy:+.0f}"
            if abs(dx) > self._cfg["deadband_x_px"]:
                self._move(self.BASE,
                           -self._cfg["kp_base"] * dx * self._cfg["base_dir"])
            if abs(dy) > self._cfg["deadband_y_px"]:
                self._move(self.GRIPPER,
                           -self._cfg["kp_gripper"] * dy * self._cfg["gripper_dir"])
            return

        # --- GO_LOCK: kilit pozuna deterministik transit + araliklarla base ---
        if st == "GO_LOCK":
            # Hizli adim fazi: poz adimi gorsel karar gerektirmez (hedef sabit)
            if self._fast > 0:
                self._fast -= 1
                r = self._lock_step()
                self._decision = f"kilide hızlı adım ({r})"
                if r == "arrived":
                    self._enter_fine()
                elif r == "stuck":
                    self._abort("Kilit pozuna zarf izin vermiyor — kilidi "
                                "zarf içinde yeniden kaydet.")
                return
            if self._cooldown > 0:
                self._cooldown -= 1
                self._decision = f"cooldown {self._cooldown}"
                return
            res = self._stable_errors()
            if res is None:
                # Transit sirasinda kup gecici kadraj disina cikabilir (kamera
                # egimi degisiyor) — kilide ilerlemeye DEVAM, kilitte gorunmeli.
                self._lost_count += 1
                if self._lost_count > self._cfg["lost_max"]:
                    self._abort("Küp kayboldu (GO_LOCK) — iptal.")
                    return
                r = self._lock_step()
                self._decision = f"kayıp ama kilide ilerliyor ({r})"
                if r == "arrived":
                    self._enter_fine()
                elif r == "stuck":
                    self._abort("Kilit pozuna zarf izin vermiyor.")
                return
            if res == "WAIT":
                self._decision = "kare birikiyor"
                return
            self._lost_count = 0
            dx, _dy = res
            if abs(dx) > self._cfg["deadband_x_px"]:
                self._decision = f"base düzelt dx={dx:+.0f}"
                self._move(self.BASE,
                           -self._cfg["kp_base"] * dx * self._cfg["base_dir"])
                return
            # yatay hizada -> bir sonraki olcume kadar hizli adim burst'u
            self._fast = max(0, int(self._cfg["fast_advance"]) - 1)
            r = self._lock_step()
            self._decision = f"kilide adım ({r})"
            if r == "arrived":
                self._enter_fine()
            elif r == "stuck":
                self._abort("Kilit pozuna zarf izin vermiyor.")
            return

        # --- FINE: kilit pozunda ince nisan (dx->BASE, dy->SHOULDER) ---
        if st == "FINE":
            self._fine_ticks += 1
            if self._fine_ticks > self._cfg["fine_timeout_ticks"]:
                self._abort("İnce nişan zaman aşımı — küp hedefe oturmadı.")
                return
            if self._cooldown > 0:
                self._cooldown -= 1
                self._decision = f"cooldown {self._cooldown}"
                return
            res = self._stable_errors()
            if res is None:
                self._on_lost("FINE")
                return
            self._lost_count = 0
            if res == "WAIT":
                self._decision = "kare birikiyor"
                return
            dx, dy = res
            if abs(dx) > self._cfg["deadband_x_px"]:
                self._decision = f"fine base dx={dx:+.0f}"
                self._move(self.BASE,
                           -self._cfg["kp_base"] * dx * self._cfg["base_dir"])
                return
            if abs(dy) > self._cfg["deadband_y_px"]:
                # dy>0 = kup hedefin ALTINDA = yakin -> shoulder GERI (artar)
                step = (self._cfg["fine_step_deg"]
                        * (1 if dy > 0 else -1) * self._cfg["fine_dir"])
                self._decision = (f"fine shoulder dy={dy:+.0f} → "
                                  f"{'geri' if step > 0 else 'ileri'}")
                if not self._move(self.SHOULDER, step):
                    self._abort("İnce nişan: shoulder zarf sınırında — "
                                "küp erişilebilir mesafede değil.")
                return
            # Iki eksen de hedefte -> profil sec, dalis tabanini sabitle, KOR inise gec
            self._plunge = self._select_profile()
            self._plunge_base = list(self.cmd)
            self._plunge_i = 0
            self._grace = 0
            self._set("PLUNGE", f"Nişan tamam (dx={dx:+.0f} dy={dy:+.0f}) — "
                                f"öğretilmiş dalış başlıyor (görüş kapalı, buton bekleniyor)…")
            return

        # --- PLUNGE: secilen profilin GORELI kor tekrari; buton sonlandirir ---
        if st == "PLUNGE":
            prof = self._plunge or self._profiles[0]
            path = prof["path"]
            anchor = prof["start"]
            base = self._plunge_base or list(self.cmd)
            if self._plunge_i >= len(path):
                self._grace += 1
                self._decision = f"dalış bitti — buton bekleniyor {self._grace}/{self._cfg['plunge_grace_ticks']}"
                if self._grace > self._cfg["plunge_grace_ticks"]:
                    self._abort("Dalış tamamlandı ama buton tetiklenmedi — "
                                "küp mıknatıs altında değil (logda FINE dx/dy'ye bak).")
                return
            wp = path[self._plunge_i]
            # GORELI hedef: FINE'in duzeltmesi korunur (anchor = profil baslangici)
            tgt = {self.SHOULDER: base[self.SHOULDER] + (wp[0] - anchor[0]),
                   self.ELBOW:    base[self.ELBOW] + (wp[1] - anchor[1]),
                   self.GRIPPER:  base[self.GRIPPER] + (wp[2] - anchor[2])}
            tgt = {i: self._clamp_servo(v) for i, v in tgt.items()}
            if all(self.cmd[i] == v for i, v in tgt.items()):
                self._plunge_i += 1
                self._decision = f"dalış wp {self._plunge_i}/{len(path)} tamam"
                return
            moved = False
            for i, v in tgt.items():
                if self.cmd[i] != v:
                    moved = self._move(i, v - self.cmd[i]) or moved
            self._decision = f"dalış wp {self._plunge_i + 1}/{len(path)} → {list(tgt.values())}"
            if not moved and not all(self.cmd[i] == v for i, v in tgt.items()):
                self._abort("Dalış zarf tarafından bloklandı — dalış yolunu "
                            "zarf içinde yeniden öğret.")
            return

    def _enter_fine(self):
        self._fine_ticks = 0
        self._fast = 0
        self._set("FINE", "Kilit pozunda — ince nişan (küp → kilit hedef pikseli)…")

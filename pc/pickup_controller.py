"""
Otonom kup kapma — v4: GERCEK KINEMATIK (aci hesabiyla konum).

NEDEN v4: v1-v3 goruntu-uzayi kalibrasyonlariyla ugrasti (sabit hedef piksel,
paralaks ornekleri, jog probe) — kullanici "aci hesabi yapip GERCEKTEN
hesaplayan bir sey" istedi. v4 kolu fiziksel modeller (arm_kinematics.ArmModel):

  1) Kameradan KUPUN DUNYA KONUMU (cm) hesaplanir — isin-zemin kesisimi
     (bbox boyutuna gerek yok; piksel + kamera pozu + zemin duzlemi yeter).
  2) Ters kinematikle "miknatis kupun TAM USTUNDE, TAM ASAGI bakar" servo
     acilari HESAPLANIR ve kol oraya surulur (NISAN/AIM).
  3) Inis = hedef yuksekligi cm cm dusurup IK'yi yeniden cozmek (DESCEND).
     Gorus varsa kup konumu yumusakca guncellenir; gorus gitse de matematik
     ayni dikey cizgide inmeye devam eder. BUTON (grip switch) sonlandirir.

Durum makinesi:
    IDLE -> AIM     : kup konumu olculur (ardisik iki tahmin uyusunca kilit),
                      IK ile hover pozu hesaplanir, kol surulur, varista
                      yeniden olcum -> sapma buyukse yeniden nisan
         -> DESCEND : tip_z adim adim iner (IK her adimda yeniden cozulur,
                      bilek hep tam asagi); magnet_on_cm'de miknatis ACIK;
                      zemine bastirmaya kadar (floor_cm) surer
         -> GRABBED : buton -> miknatis ACIK, dur
         \\-> ABORT : kayip (nisan asamasinda) / erisim disi / zarf / timeout

KALIBRASYON (pickup_test 'KINEMATIK' bolumu — cetvel + 4 referans + odak):
  olculer: L1 L2 L3 H0 R0 kup boyutu (cm)
  referanslar: L1 tam DIK, kol DUZ, miknatis TAM ASAGI, base TAM ILERI
  odak: kup bilinen mesafede -> cam_f = bh_px * D / kup_cm

GUVENLI BOLGE: arm_sim zarfi her servo komutunda korunur (_move -> _repair).
SETTLE: /poll arm verisi (servos==targets==komut) + 1 kare; bayatsa tick sayaci.
KOSU LOGU: pc/pickup_logs/run_*.jsonl — config, her tick (poz/tespit/dunya
tahmini/hedef/karar), her servo komutu, IK/zarf engelleri, bitis nedeni.
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Optional, TextIO

from arm_kinematics import ArmModel


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def interp_band(grid, lo_a, hi_a, x):
    """1B grid uzerinde lineer interpolasyonla (lo, hi) bandi — guvenli zarf.
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
    """Gripper bandi — (SHOULDER, ELBOW) ciftine bagli 2B interpolasyon."""
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
            if b1 is None:
                return b2
            if b2 is None:
                return b1
            t = (sh - shs[i]) / (shs[i + 1] - shs[i])
            return (b1[0] + (b2[0] - b1[0]) * t, b1[1] + (b2[1] - b1[1]) * t)
    return None


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

    DEFAULTS = {
        # --- KINEMATIK MODEL (arm_kinematics.DEFAULTS + kullanici olcumleri) ---
        "kinematics": {},
        # --- nisan/inis (cm) ---
        "hover_cm": 6.0,          # nisan yuksekligi (kup USTUNDEN)
        "descend_step_cm": 0.7,   # inis adimi
        "magnet_on_cm": 2.0,      # bu yukseklikte miknatis ACIK
        "est_agree_cm": 1.0,      # ardisik iki konum tahmini uyusma esigi
        "re_aim_cm": 1.5,         # hover varisinde sapma esigi -> yeniden nisan
        "re_aim_max": 6,
        "floor_cm": -1.5,         # tip_z = kup_ustu + h; h buraya kadar (bastirma)
        "track_clamp_cm": 0.5,    # DESCEND'de tahmin guncelleme adimi siniri
        "grace_ticks": 8,         # zemine inildi, butonsuz bekleme
        # INCE AYAR: miknatis ucu ile kinematik model arasindaki kucuk kayma
        # (montaj/L3 toleransi). IK hedefi kup yonunde ileri/yana kaydirilir ->
        # miknatis kupun TAM ortasina (buton kupe basacak) iner. pickup_test
        # nudge butonlariyla 0.3'er ayarlanir.
        "aim_trim_fwd": 0.0, "aim_trim_lat": 0.0,
        # --- hareket/olcum ---
        "max_step_deg": 2, "pickup_speed_ms": 40,
        "tick_ms": 150, "det_stale_s": 0.7,
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
        self.live: dict = {}            # {x, y, h, u, v} — UI gosterimi
        self._cfg: dict = dict(self.DEFAULTS)
        self._after_id = None
        self.cmd = [93, 120, 60, 110]   # komut edilen servo acilari (lokal takip)
        self.model: Optional[ArmModel] = None
        self.rl: Optional[RunLogger] = None
        self._decision = ""
        self._ticks = 0
        self._lost_count = 0
        self._grip_seq0 = 0
        self._px_hist: list = []        # temiz kare (u, v) birikimi
        self._live_tick = -1
        # settle
        self._await_settle = False
        self._settle_left = 0
        self._settle_started = 0
        self._cam_grace = 0
        self._settle_path = ""
        self._move_t = 0.0
        # nisan/inis durumu
        self._cube: Optional[tuple] = None   # kilitli kup dunya konumu (x, y)
        self._prev_est: Optional[tuple] = None
        self._tgt: Optional[list] = None     # hedef servolar
        self._h = 0.0                        # mevcut hedef yukseklik (kup ustunden)
        self._re_aims = 0
        self._magnet_on = False
        self._stalls = 0
        self._grace = 0

    # ------------------------------------------------------------------ kontrol
    def start(self) -> bool:
        """MEVCUT kol pozundan baslat. Zarf + kinematik kalibrasyon (olculer +
        4 referans) eksikse reddeder ve neyin eksik oldugunu soyler."""
        auto = (getattr(self.app, "pickup_cfg", None) or {}).get("autonomous", {})
        self._cfg = {**self.DEFAULTS, **auto}
        if self._envelope() is None:
            self._set("IDLE", "Önce arm_sim SINIR SİHİRBAZI'nı çalıştırıp kaydet.")
            return False
        self.model = ArmModel(self._cfg.get("kinematics") or {})
        missing = self.model.missing()
        if missing:
            self._set("IDLE", "Eksik KİNEMATİK kalibrasyon: " + ", ".join(missing)
                              + " — pickup_test 'KİNEMATİK' bölümünden tamamla.")
            return False
        pose = self._current_pose()
        if pose is None:
            self._set("IDLE", "Kol pozu okunamadı.")
            return False
        if not self._envelope_ok(pose):
            self._set("IDLE", f"Mevcut poz {pose} güvenli alan DIŞINDA — "
                              f"kolu zarf içine getir.")
            return False
        self._cancel()
        self._ticks = 0
        self._lost_count = 0
        self._px_hist = []
        self._await_settle = False
        self._cam_grace = 0
        self._cube = None
        self._prev_est = None
        self._tgt = None
        self._h = float(self._cfg["hover_cm"])
        self._re_aims = 0
        self._magnet_on = False
        self._stalls = 0
        self._grace = 0
        g = getattr(self.app, "grip_state", {}).get(self.agv_id) or {}
        self._grip_seq0 = int(g.get("seq", 0))
        self.rl = RunLogger(self.agv_id)
        self.rl.event("config",
                      cfg={k: v for k, v in self._cfg.items() if k != "safe_envelope"},
                      start_pose=list(pose), grip_seq0=self._grip_seq0)
        self._http(f"/servo/speed?ms={int(self._cfg['pickup_speed_ms'])}")
        self._http("/magnet?s=0")
        self.cmd = pose
        self._log(f"📄 koşu logu: {self.rl.path}")
        self._set("AIM", f"Mevcut pozdan başladı {pose} — küp konumu ölçülüyor…")
        self._schedule()
        return True

    def _current_pose(self):
        """Kol pozu — ONCE taze firmware hedefleri (slider'lar bayat kalabilir),
        sonra slider'lar, en son bayat firmware verisi."""
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
        Donus: gercekten degisen [(servo, eski, yeni), ...]."""
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
                          f"{self.cmd[i]}→{new}")
            return []
        changed = [(j, self.cmd[j], candidate[j])
                   for j in range(4) if candidate[j] != self.cmd[j]]
        if not changed:
            return []
        if self.rl:
            self.rl.event("move", servo=self.SERVO_NAMES[i],
                          frm=self.cmd[i], to=candidate[i], want=round(want, 2))
        self.cmd = candidate
        for sid, _, val in changed:
            self._http(f"/servo?id={sid}&a={val}")
        self._await_settle = True
        self._settle_left = int(self._cfg.get("settle_ticks", 2))
        self._settle_started = self._ticks
        self._move_t = time.time()
        self._px_hist = []
        return changed

    def _step_towards(self, tgt) -> str:
        """Tum servolari hedefe dogru birer adim. "arrived"/"moved"/"stuck"."""
        if all(self.cmd[i] == tgt[i] for i in range(4)):
            return "arrived"
        moved = False
        for i in range(4):
            if self.cmd[i] != tgt[i]:
                moved = bool(self._move(i, tgt[i] - self.cmd[i])) or moved
        if all(self.cmd[i] == tgt[i] for i in range(4)):
            return "arrived"
        return "moved" if moved else "stuck"

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
        """True = hareket/kamera oturmadi -> olcum/karar yok. Taze arm verisi
        varsa gercek durumdan; yoksa settle_ticks fallback'i. +1 kare payi."""
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
            if self._ticks - self._settle_started > self._cfg["settle_timeout_ticks"]:
                if st["targets"] != self.cmd:
                    if self.rl:
                        self.rl.event("sync", cmd=list(self.cmd),
                                      fw_targets=list(st["targets"]))
                    self._log(f"⚠ settle timeout: cmd {self.cmd} ≠ firmware "
                              f"{st['targets']} — komut firmware'e senkronlandı")
                    self.cmd = [self._clamp_servo(x) for x in st["targets"]]
                self._await_settle = False
                self._cam_grace = 1
                return True
            self._decision = "servo bekleniyor (arm)"
            return True
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
        return {"cx": raw["cx"], "cy": raw["cy"], "bh": raw.get("h"),
                "conf": round(raw.get("conf", 0), 2),
                "age": round(time.time() - raw.get("ts", 0), 2)}

    def _eff_center(self, d) -> Optional[tuple]:
        """Kirpilma-duzeltmeli kup merkez pikseli (u, v). Kadraj kenarinda kesik
        bbox'ta gercek boyut kirpilmamis eksenden, merkez gorunen kenardan
        (kup kare). Iki eksende kirpiksa None (guvenilmez)."""
        u, v = float(d["cx"]), float(d["cy"])
        x1, y1, x2, y2 = d.get("x1"), d.get("y1"), d.get("x2"), d.get("y2")
        if None in (x1, y1, x2, y2):
            return u, v
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
            v = y1 + size / 2.0 if clip_b else y2 - size / 2.0
        elif clip_h:
            size = float(y2 - y1)
            if size <= 0:
                return None
            u = x1 + size / 2.0 if clip_r else x2 - size / 2.0
        return u, v

    def _measure_px(self):
        """Temiz karelerden stable_frames adedi ortalanmis kup merkez pikseli.
        None = tespit yok/guvenilmez; "WAIT" = birikiyor; (u, v)."""
        d = self._det()
        if d is None:
            return None
        c = self._eff_center(d)
        if c is None:
            return None
        self._px_hist.append(c)
        need = max(1, int(self._cfg["stable_frames"]))
        if len(self._px_hist) < need:
            return "WAIT"
        self._px_hist = self._px_hist[-need:]
        n = len(self._px_hist)
        return (sum(p[0] for p in self._px_hist) / n,
                sum(p[1] for p in self._px_hist) / n)

    def _estimate(self):
        """Kupun dunya konumu (x, y) cm — piksel olcumu + isin-zemin kesisimi.
        None / "WAIT" / (x, y). Basarili tahmin live + dosya loguna yazilir."""
        model = self.model
        if model is None:
            return None
        px = self._measure_px()
        if px is None or px == "WAIT":
            return px
        u, v = px
        cube = float(model.c.get("cube_cm") or 4.0)
        est = model.cube_from_pixel(self.cmd, u, v, z_plane=cube / 2.0)
        if est is None:
            self._decision = "ışın zemini kesmiyor (kamera yukarı bakıyor?)"
            return None
        x, y = est
        self.live = {"x": round(x, 1), "y": round(y, 1),
                     "h": round(self._h, 1), "u": round(u), "v": round(v)}
        self._live_tick = self._ticks
        if self.rl:
            self.rl.event("estimate", x=round(x, 2), y=round(y, 2),
                          u=round(u, 1), v=round(v, 1), pose=list(self.cmd))
        return est

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
                          ticks=self._ticks, pose=list(self.cmd),
                          cube=(list(self._cube) if self._cube else None),
                          h=round(self._h, 2))
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

    def _on_lost(self, where: str):
        self._lost_count += 1
        lim = self._cfg["lost_max"]
        self._decision = f"kayıp {self._lost_count}/{lim}"
        if self._lost_count == 1 or self._lost_count % 4 == 0:
            self._log(f"⚠ tespit yok ({self._lost_count}/{lim}) [{where}] — "
                      f"{self._det_debug()}")
        if self._lost_count > lim:
            self._abort(f"Küp kayboldu ({where}) — iptal.")

    def _det_debug(self) -> str:
        raw = getattr(self.app, "detection_state", {}).get(self.agv_id)
        if raw is None:
            return "YOK — YOLO küp görmüyor / 🎯 Tespit toggle kapalı"
        age = time.time() - raw.get("ts", 0)
        if age > self._cfg["det_stale_s"]:
            return f"BAYAT {age:.1f}s"
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
                          cube=(list(self._cube) if self._cube else None),
                          h=round(self._h, 2), tgt=self._tgt,
                          dec=self._decision, settle=self._settle_path or None,
                          lost=self._lost_count)
        if (self._cfg.get("debug") and self.active
                and self._ticks % max(1, int(self._cfg["debug_every_ticks"])) == 0):
            cube = (f"küp=({self._cube[0]:.1f},{self._cube[1]:.1f})cm"
                    if self._cube else "küp=?")
            self._log(f"{self.state} t={self._ticks} poz={self.cmd} {cube} "
                      f"h={self._h:.1f}cm karar[{self._decision or '-'}] "
                      f"tespit[{self._det_debug()}]")
        if self.active:
            self._schedule()
        self._refresh()

    # ------------------------------------------------------------------- adimlar
    def _aimed_xy(self, x: float, y: float):
        """IK hedefini INCE AYAR trim'iyle kaydir: miknatis ucu kinematik
        modelden biraz kaymis olabilir; kup yonunde ileri (+fwd) ve yana (+lat
        sag) kaydirarak miknatis kupun TAM ortasina iner (buton basar)."""
        tf = float(self._cfg.get("aim_trim_fwd", 0) or 0)
        tl = float(self._cfg.get("aim_trim_lat", 0) or 0)
        if not tf and not tl:
            return x, y
        yaw = math.atan2(x, y)
        fx, fy = math.sin(yaw), math.cos(yaw)      # kupe dogru (ileri)
        lx, ly = math.cos(yaw), -math.sin(yaw)     # saga dik (lateral)
        return x + tf * fx + tl * lx, y + tf * fy + tl * ly

    def _aim_target(self) -> bool:
        """Kilitli kup konumu icin hover pozunu coz -> self._tgt. Hem IK hem
        GUVENLI ZARF gecen ILK (en yuksek) hover yuksekligini secer: yakin kup
        icin tam hover_cm katlanma zarf disinda kalabilir, ama biraz alcagi
        iceride olabilir (saha: (2.6,10.6) cm'de [85,127,36] zarf disiydi).
        Hicbiri gecmezse False (gercekten erisilemez)."""
        if self.model is None or self._cube is None:
            return False
        x, y = self._aimed_xy(*self._cube)
        cube = float(self.model.c.get("cube_cm") or 4.0)
        hov = float(self._cfg["hover_cm"])
        # yuksekten alcaga: hover_cm, 0.66x, 0.33x, sonra dogrudan temas
        for frac in (1.0, 0.66, 0.33, 0.0):
            h = hov * frac
            tgt = self.model.ik(x, y, cube + h, current=self.cmd)
            if tgt is not None and self._envelope_ok(tgt):
                self._tgt = tgt
                self._h = h
                if self.rl:
                    self.rl.event("ik", x=round(x, 2), y=round(y, 2),
                                  tip_z=round(cube + h, 2), hover=round(h, 2),
                                  servos=list(tgt))
                if frac < 1.0:
                    self._log(f"ℹ hover {hov:g} cm zarf dışı — {h:.1f} cm'den "
                              f"başlanıyor (yakın küp, kol katlanması sınırlı)")
                return True
        return False

    def _step(self):
        st = self.state

        # --- AIM: kup konumunu olc, kilitle, hover pozuna sur, dogrula ---
        if st == "AIM":
            if self._settling():
                return
            # hedef belirlendiyse once oraya var
            if self._tgt is not None and self._cube is not None:
                r = self._step_towards(self._tgt)
                if r == "moved":
                    self._decision = f"hover'a sürülüyor → {self._tgt}"
                    return
                if r == "stuck":
                    self._stalls += 1
                    self._decision = f"hover yolu zarfta bloklandı ({self._stalls})"
                    if self._stalls >= 3:
                        self._abort("Hover pozuna zarf izin vermiyor — küp zarf "
                                    "içinde erişilebilir bölgede değil.")
                    return
                # varildi -> dogrulama olcumu. Kup GORUNMUYORSA iptal ETME:
                # hover'da kamera ofseti yuzunden kup kadraj disi kalabilir;
                # v4'un amaci gorus olmadan da inmek -> kinematik tahmine guven.
                est = self._estimate()
                if est == "WAIT":
                    self._decision = "kare birikiyor (doğrulama)"
                    return
                if est is None:
                    self._lost_count = 0
                    self._log("hover'da küp görüş dışı (kamera ofseti) — "
                              "kinematik tahminle iniliyor")
                    self._set("DESCEND", f"Hover'da (görüşsüz) — iniş başlıyor "
                                         f"(h={self._h:.1f} cm)…")
                    return
                self._lost_count = 0
                ex, ey = est
                dx, dy = ex - self._cube[0], ey - self._cube[1]
                err = (dx * dx + dy * dy) ** 0.5
                if err > float(self._cfg["re_aim_cm"]):
                    self._re_aims += 1
                    if self._re_aims > int(self._cfg["re_aim_max"]):
                        self._abort(f"Nişan oturmuyor (sapma {err:.1f} cm) — "
                                    "kinematik ölçüler/referanslar hatalı olabilir; "
                                    "logda 'estimate'/'ik' olaylarına bak.")
                        return
                    self._cube = (ex, ey)
                    if not self._aim_target():
                        self._abort(f"Küp ({ex:.1f}, {ey:.1f}) cm erişim dışı.")
                        return
                    self._decision = f"yeniden nişan #{self._re_aims} (sapma {err:.1f}cm)"
                    self._log(f"🎯 yeniden nişan: küp ({ex:.1f}, {ey:.1f}) cm, "
                              f"sapma {err:.1f} cm")
                    return
                self._stalls = 0
                self._set("DESCEND", f"Mıknatıs küpün üstünde (sapma {err:.1f} cm) "
                                     f"— iniş başlıyor (h={self._h:.1f} cm)…")
                return
            # henuz kilit yok -> tahmin topla
            est = self._estimate()
            if est is None:
                self._on_lost("AIM")
                return
            if est == "WAIT":
                self._decision = "kare birikiyor"
                return
            self._lost_count = 0
            ex, ey = est
            if self._prev_est is not None:
                px, py = self._prev_est
                if ((ex - px) ** 2 + (ey - py) ** 2) ** 0.5 <= float(
                        self._cfg["est_agree_cm"]):
                    self._cube = ((ex + px) / 2.0, (ey + py) / 2.0)
                    if not self._aim_target():
                        self._abort(f"Küp ({self._cube[0]:.1f}, "
                                    f"{self._cube[1]:.1f}) cm erişim dışı — "
                                    f"küpü kola yaklaştır.")
                        return
                    self._log(f"🔒 küp kilitlendi: ({self._cube[0]:.1f}, "
                              f"{self._cube[1]:.1f}) cm → hover {self._tgt}")
                    self._decision = "kilitlendi"
                    return
            self._prev_est = (ex, ey)
            self._decision = f"tahmin ({ex:.1f}, {ey:.1f}) cm — teyit bekleniyor"
            return

        # --- DESCEND: tip_z'yi cm cm dusur, IK'yi yeniden coz, butona kadar ---
        if st == "DESCEND":
            if self._settling():
                return
            if self.model is None or self._cube is None:
                self._abort("İç durum eksik (model/küp) — yeniden başlat.")
                return
            model = self.model
            cube_cm = float(model.c.get("cube_cm") or 4.0)
            # miknatis erken acma
            if not self._magnet_on and self._h <= float(self._cfg["magnet_on_cm"]):
                self._magnet_on = True
                self._http("/magnet?s=1")
                if self.rl:
                    self.rl.event("magnet_early", h=round(self._h, 2))
                self._log(f"🧲 h={self._h:.1f} cm — mıknatıs AÇIK")
            # gorus varsa kup konumunu yumusakca guncelle (matematik inis bozulmaz)
            est = self._estimate()
            if isinstance(est, tuple) and self._cube is not None:
                cl = float(self._cfg["track_clamp_cm"])
                nx = self._cube[0] + _clamp(est[0] - self._cube[0], -cl, cl)
                ny = self._cube[1] + _clamp(est[1] - self._cube[1], -cl, cl)
                if (abs(nx - self._cube[0]) > 0.05 or abs(ny - self._cube[1]) > 0.05):
                    self._cube = (nx, ny)
            # hedefe sur; varildiysa bir adim daha in
            if self._tgt is not None:
                r = self._step_towards(self._tgt)
                if r == "moved":
                    self._decision = f"iniş h={self._h:.1f}cm → {self._tgt}"
                    return
                if r == "stuck":
                    # transit bloklandi: en alt nokta -> miknatis ac + butonu bekle
                    if not self._magnet_on:
                        self._magnet_on = True
                        self._http("/magnet?s=1")
                        self._log(f"🧲 iniş bloklandı (h={self._h:.1f} cm) — mıknatıs AÇIK")
                    self._grace += 1
                    self._decision = (f"iniş bloklandı h={self._h:.1f}cm — buton "
                                      f"bekleniyor {self._grace}/{self._cfg['grace_ticks']}")
                    if self._grace > int(self._cfg["grace_ticks"]):
                        self._abort(f"İniş zarfta kilitli (h={self._h:.1f} cm), "
                                    "buton yok — arm_sim'de alçak pozları genişlet "
                                    "veya küpü biraz uzağa koy.")
                    return
                self._stalls = 0
            if self._h <= float(self._cfg["floor_cm"]) + 1e-6:
                self._grace += 1
                self._decision = f"zeminde — buton bekleniyor {self._grace}/{self._cfg['grace_ticks']}"
                if self._grace > int(self._cfg["grace_ticks"]):
                    self._abort("Zemine kadar inildi, buton tetiklenmedi — "
                                "logda 'estimate'/'ik' olaylarına bak (küp konumu "
                                "veya ölçüler hatalı olabilir).")
                return
            next_h = max(float(self._cfg["floor_cm"]),
                         self._h - float(self._cfg["descend_step_cm"]))
            x, y = self._aimed_xy(self._cube[0], self._cube[1])
            tgt = self.model.ik(x, y, cube_cm + next_h, current=self.cmd)
            # Daha alcak hedef IK/zarf disindaysa: ABA ETME — burasi inebildigimiz
            # EN ALT nokta; miknatis aciksa kup cekilir, butonu bekle (grace).
            if tgt is None or not self._envelope_ok(tgt):
                if not self._magnet_on:   # son carede miknatis ac
                    self._magnet_on = True
                    self._http("/magnet?s=1")
                    self._log(f"🧲 iniş sınırı (h={self._h:.1f} cm) — mıknatıs AÇIK")
                self._grace += 1
                self._decision = (f"iniş sınırı h={self._h:.1f}cm — buton "
                                  f"bekleniyor {self._grace}/{self._cfg['grace_ticks']}")
                if self._grace > int(self._cfg["grace_ticks"]):
                    # IK None = kol UZANMA sinirinda (kup cok uzak) ->
                    # YAKLASTIR; zarf blogu = katlanma sinirinda -> zarfi genislet
                    why = ("küp ÇOK UZAK, kol tam uzandı yetişemedi — küpü "
                           "KOLA YAKLAŞTIR (~13-16 cm)" if tgt is None else
                           "kol katlanma (zarf) sınırında — arm_sim'de alçak "
                           "pozları genişlet veya küpü biraz UZAĞA koy")
                    self._abort(f"En alt noktaya inildi (h={self._h:.1f} cm), "
                                f"mıknatıs küpe değmedi — {why}.")
                return
            self._h = next_h
            self._tgt = tgt
            if self.rl:
                self.rl.event("ik", x=round(x, 2), y=round(y, 2),
                              tip_z=round(cube_cm + self._h, 2), servos=list(tgt))
            self._decision = f"yeni iniş hedefi h={self._h:.1f}cm → {tgt}"
            return

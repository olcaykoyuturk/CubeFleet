"""
Kapma v3 (sona kadar kamera / paralaks hedef modeli) cevrimdisi testleri.

SENTETIK KAMERA MODELI: donanim olmadan tam kapali dongu — eklem acilarindan
fiziksel olarak tutarli (cx, cy, bbox_h) ureten oyuncak projeksiyon:
    yanal   l = (93 - base) * 0.4 cm
    ileri   f = 5 + 0.30*(120-sh) + 0.18*(120-el) cm
    yukselt h = 0.05*sh + 0.15*el - 16 cm  (alcalmak = el/sh azaltmak)
    Z = h + 2.5 ;  bh = 300/Z ;  K = 110 px*cm
    cx = 240 + K*(X_c - l)/Z
    cy = 160 + K*(f + o - F_c)/Z + 6*(gr - seat_true)   [o = 2 cm ofset]
    seat_true = 0.5*sh + 0.25*el + 5  (bilek bundan saparsa goruntu kayar)
Kup miknatisin TAM altindayken cy = 160 + K*o/Z -> bh'ye gore DOGRUSAL kayan
hedef (paralaks modelinin birebir karsiligi). Buton: h<=0.6 ve |X_c-l|,|F_c-f|<=1.5.

Kontroller: seat_deg IDW; interp_target clamp/dogrusallik; PROBE dogru eksen/
yon bulur; tam SEAT->...->GRABBED akisi; ENDGAME-lost (yakin korluk) yolu;
butonsuz ENDGAME abort; probe-strict abort; SEAT kayip abort; zarf saygisi +
alcalma blok abort; arm_state settle ("arm") ve bayat fallback ("ticks");
start dogrulama (eksik temas/seat reddi).

Calistir: python pc/test_pickup_v3.py   (run_tests.py otomatik calistirir)
"""

from __future__ import annotations

import json
import os
import sys
import time

for _stream in (sys.stdout, sys.stderr):
    _reconf = getattr(_stream, "reconfigure", None)
    if callable(_reconf):
        _reconf(encoding="utf-8")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pickup_controller import PickupController, interp_target, seat_deg

_passed = 0
_failed = 0
_failures: list[str] = []
_run_logs: set = set()


def assert_true(label: str, cond, detail: object = ""):
    global _passed, _failed
    if cond:
        _passed += 1
        print(f"  ✓ {label}")
    else:
        _failed += 1
        msg = f"  ✗ {label}" + (f": {detail}" if detail else "")
        _failures.append(msg)
        print(msg)


def assert_close(label: str, actual, expected, tol):
    assert_true(label, abs(actual - expected) <= tol,
                f"got {actual!r}, expected {expected!r} ±{tol}")


# ---------------------------------------------------------------- sentetik model
W, H, K, OFS, S = 480, 320, 110.0, 2.0, 300.0
CUBE_X, CUBE_F = 1.0, 12.0


def seat_true(sh, el):
    return max(0.0, min(180.0, 0.5 * sh + 0.25 * el + 5.0))


def model_pixels(servos):
    """Eklem acilarindan (cx, cy, bh, h_cm)."""
    base, sh, el, gr = servos
    l = (93 - base) * 0.4
    f = 5.0 + 0.30 * (120 - sh) + 0.18 * (120 - el)
    h = max(0.0, 0.05 * sh + 0.15 * el - 16.0)
    z = h + 2.5
    bh = S / z
    cx = 240.0 + K * (CUBE_X - l) / z
    cy = 160.0 + K * (f + OFS - CUBE_F) / z + 6.0 * (gr - seat_true(sh, el))
    return cx, cy, bh, h, l, f


def gen_align_points():
    """Modelden 'kup tam miknatisin altinda' ornekleri (4 yukseklik; Z=3 temas)."""
    pts = []
    for z, contact in ((3.0, True), (4.5, False), (7.0, False), (11.0, False)):
        bh = S / z
        row = (160.0 + K * OFS / z) / H
        pts.append({"bh": round(bh, 1), "col": 0.5, "row": round(row, 4),
                    "contact": contact})
    return pts


def gen_seat_points():
    pts = []
    for sh in (40, 80, 120, 160):
        for el in (40, 90, 140):
            pts.append([sh, el, round(seat_true(sh, el))])
    return pts


def make_env(el_min=0):
    return {"type": "limits", "margin_deg": 0,
            "sh_grid": [0, 180], "el_min": [el_min, el_min],
            "el_max": [180, 180],
            "gr_rows": [{"sh": 0, "el": [0, 180], "lo": [0, 0], "hi": [180, 180]},
                        {"sh": 180, "el": [0, 180], "lo": [0, 0], "hi": [180, 180]}]}


class Var:
    def __init__(self, v):
        self.v = v

    def get(self):
        return self.v


class FakeApp:
    """PickupController'in app arayuzu + sentetik fizik. step(): servolar
    hedefe 5°/adim yurur, arm_state + tespit + buton guncellenir."""

    def __init__(self, cfg, start_pose, arm_feedback=True, vis_min_z=0.0,
                 button=True, frozen_cam=False, blind_cam=False):
        self.pickup_cfg = cfg
        self.detection_state: dict = {}
        self.grip_state: dict = {}
        self.arm_state: dict = {}
        self.arm_servo_vars = {"T": [Var(v) for v in start_pose]}
        self.http_calls: list = []
        self.servos = list(start_pose)      # anlik (fiziksel) acilar
        self.targets = list(start_pose)     # firmware hedefleri
        self.arm_feedback = arm_feedback    # False -> arm_state hic dolmaz
        self.vis_min_z = vis_min_z          # Z bunun altinda -> kup gorunmez
        self.button = button
        self.frozen_cam = frozen_cam        # tespit poza DUYARSIZ (probe testi)
        self.blind_cam = blind_cam          # hic tespit yok
        self._seq = 0

    def after(self, _ms, _cb=None):
        return 1

    def after_cancel(self, _i):
        pass

    def _arm_http_get(self, _agv, path):
        self.http_calls.append(path)
        if path.startswith("/servo?id="):
            q = path.split("?", 1)[1]
            kv = dict(p.split("=") for p in q.split("&"))
            self.targets[int(kv["id"])] = int(kv["a"])

    def _pickup_log(self, _agv, msg):
        pass

    def _set_status(self, msg):
        pass

    def _pickup_ui_refresh(self, _agv=""):
        pass

    def step(self):
        # servolar hedefe yurur (5°/adim — kontrol adimi 2° oldugundan 1 adimda oturur)
        for i in range(4):
            d = self.targets[i] - self.servos[i]
            if d:
                self.servos[i] += max(-5, min(5, d))
        if self.arm_feedback:
            self.arm_state["T"] = {"servos": list(self.servos),
                                   "targets": list(self.targets),
                                   "ts": time.time()}
        cx, cy, bh, h, l, f = model_pixels(self.servos)
        z = h + 2.5
        visible = (not self.blind_cam and 0 <= cx <= W and 0 <= cy <= H
                   and z >= self.vis_min_z)
        if self.frozen_cam:
            cx, cy, bh = 240.0, 200.0, 40.0   # poza duyarsiz sabit tespit
            visible = True
        if visible:
            half = bh / 2.0
            self.detection_state["T"] = {
                "cx": cx, "cy": cy, "area": int(bh * bh), "conf": 0.95,
                "h": bh, "w": bh,
                "x1": cx - half, "y1": cy - half, "x2": cx + half, "y2": cy + half,
                "frame_w": W, "frame_h": H, "ts": time.time(),
            }
        else:
            self.detection_state.pop("T", None)
        if (self.button and h <= 0.6 and abs(CUBE_X - l) <= 1.5
                and abs(CUBE_F - f) <= 1.5):
            g = self.grip_state.get("T") or {}
            if g.get("type") != "held":
                self._seq += 1
                self.grip_state["T"] = {"type": "held", "seq": self._seq,
                                        "ts": time.time()}


def make_cfg(el_min=0, **over):
    auto = {"safe_envelope": make_env(el_min),
            "seat_points": gen_seat_points(),
            "align_points": gen_align_points(),
            "total_timeout_ticks": 4000}
    auto.update(over)
    return {"autonomous": auto}


def run(pc, app, max_ticks=4000):
    states = []
    while pc.active and pc._ticks < max_ticks:
        app.step()
        pc._cancel()
        pc._tick()
        if not states or states[-1] != pc.state:
            states.append(pc.state)
        assert pc._envelope_ok(pc.cmd), f"zarf ihlali: {pc.cmd}"
    return states


def start_pc(app):
    pc = PickupController(app, "T")
    app.step()
    ok = pc.start()
    if pc.rl is not None:
        _run_logs.add(pc.rl.path)
    return pc, ok


START = [97, 90, 110, 60]

# ---------------------------------------------------------------- 1. saf fonksiyonlar
print("\n[1] seat_deg / interp_target")
sp = gen_seat_points()
assert_close("seat_deg kalibre noktada birebir", seat_deg(sp, 80, 90),
             round(seat_true(80, 90)), 0.01)   # noktalar tamsayi kaydedilir
assert_close("seat_deg ara noktada makul", seat_deg(sp, 100, 100),
             seat_true(100, 100), 6.0)
assert_true("seat_deg nokta yoksa fallback", seat_deg([], 80, 90, 77) == 77)
ap = gen_align_points()
lo_bh = min(p["bh"] for p in ap)
hi_bh = max(p["bh"] for p in ap)
assert_true("interp alt clamp", interp_target(ap, lo_bh - 20)
            == interp_target(ap, lo_bh))
assert_true("interp ust clamp", interp_target(ap, hi_bh + 50)
            == interp_target(ap, hi_bh))
mid_bh = (S / 4.5 + S / 7.0) / 2.0
_, row_mid = interp_target(ap, mid_bh)
row_a = (160.0 + K * OFS / 4.5) / H
row_b = (160.0 + K * OFS / 7.0) / H
assert_close("interp ara deger dogrusal", row_mid, (row_a + row_b) / 2.0, 0.01)
tek = [{"bh": 50, "col": 0.4, "row": 0.6, "contact": True}]
assert_true("tek ornek sabit hedef", interp_target(tek, 20) == (0.4, 0.6)
            and interp_target(tek, 90) == (0.4, 0.6))

# kirpilma duzeltmesi: alt kenarda kesik kupun merkezi gorunen ust kenar +
# genislik/2'den kurulur (saha 09:36: civilenmis bbox probe'u korletmisti)
_pc_tmp = PickupController(FakeApp(make_cfg(), START), "T")
_d = {"cx": 240, "cy": 295, "h": 46, "w": 80, "x1": 200, "y1": 272,
      "x2": 280, "y2": 318, "frame_w": W, "frame_h": H}
_eff = _pc_tmp._eff_det(_d)
assert_true("alt-kirpik: boyut genişlikten", _eff is not None and _eff[2] == 80, _eff)
assert_close("alt-kirpik: merkez y1+w/2", _eff[1], 272 + 40, 0.01)
assert_true("alt-kirpik: clip işaretli", _eff[3] is True)
_d2 = dict(_d, x1=2, x2=478)   # iki eksen birden kirpik -> guvenilmez
assert_true("çift-kirpik güvenilmez (None)", _pc_tmp._eff_det(_d2) is None)
_d3 = {"cx": 240, "cy": 160, "h": 60, "w": 60, "x1": 210, "y1": 130,
       "x2": 270, "y2": 190, "frame_w": W, "frame_h": H}
_e3 = _pc_tmp._eff_det(_d3)
assert_true("kirpiksiz: ham değerler", _e3 == (240.0, 160.0, 60.0, False), _e3)

# ---------------------------------------------------------------- 2. start dogrulama
print("\n[2] start doğrulama")
cfg = make_cfg()
cfg["autonomous"]["align_points"] = [dict(p, contact=False) for p in ap]
pc, ok = start_pc(FakeApp(cfg, START))
assert_true("temassız hiza seti reddedilir", not ok and "HİZA" in pc.message,
            pc.message)
cfg = make_cfg(seat_points=[[80, 90, 60]])
pc, ok = start_pc(FakeApp(cfg, START))
assert_true("3'ten az seat noktası reddedilir", not ok and "SEAT" in pc.message,
            pc.message)
cfg = make_cfg()
pc, ok = start_pc(FakeApp(cfg, [97, 0, 5, 60]))
ok2 = ok  # zarf permissive: bu poz icinde -> kabul edilebilir; sadece calistigini gor
assert_true("start zarf-içi pozu kabul eder", ok2, pc.message)
pc.stop()

# ---------------------------------------------------------------- 3. tam akis
print("\n[3] tam akış: SEAT → PROBE → ALIGN → GRABBED (görüş sona kadar)")
cfg = make_cfg()
app = FakeApp(cfg, START)
pc, ok = start_pc(app)
assert_true("start kabul", ok, pc.message)
logpath = pc.rl.path if pc.rl else ""
states = run(pc, app)
assert_true("GRABBED'e ulaşıldı", pc.state == "GRABBED",
            f"durumlar={states} son={pc.state} msg={pc.message}")
# kup sona kadar gorunur -> KOR faza HIC girilmemeli (gorsel inis butona kadar)
assert_true("durum sırası doğru (ENDGAME yok)",
            states == ["SEAT", "PROBE", "ALIGN", "GRABBED"], states)
assert_true("mıknatıs erken açıldı (ALIGN içinde)", "/magnet?s=1" in app.http_calls)
g = pc._gain or {}
assert_true("probe fallback'e düşmedi", g.get("fallback") is False, g)
assert_true("probe dy ekseni shoulder'ı buldu",
            g.get("dy_joint") == PickupController.SHOULDER, g)
# alcalma yonu modelde bh'yi BUYUTMELI
dsj, ddir = int(g["descend_joint"]), int(g["descend_dir"])
sv = list(app.servos)
_, _, bh0, _, _, _ = model_pixels(sv)
sv[dsj] += 5 * ddir
_, _, bh1, _, _, _ = model_pixels(sv)
assert_true("alçalma yönü bh'yi büyütüyor", bh1 > bh0,
            f"{PickupController.SERVO_NAMES[dsj]} {ddir:+d}: bh {bh0:.0f}→{bh1:.0f}")
# kosu logunda arm-settle yolu kullanilmis olmali
ticks = []
if logpath and os.path.exists(logpath):
    with open(logpath, encoding="utf-8") as fh:
        ticks = [json.loads(ln) for ln in fh if '"kind": "tick"' in ln]
assert_true("settle 'arm' yolu kullanıldı",
            any(t.get("settle") == "arm" for t in ticks),
            f"{len(ticks)} tick incelendi")

# ---------------------------------------------------------------- 4. ENDGAME-lost
print("\n[4] yakın körlük: kayıp → ENDGAME-B → GRABBED")
cfg = make_cfg()
app = FakeApp(cfg, START, vis_min_z=4.6)   # Z<4.6 -> kup gorunmez (bh~65'te kor)
pc, ok = start_pc(app)
assert_true("start kabul", ok, pc.message)
states = run(pc, app)
assert_true("körlükte de GRABBED", pc.state == "GRABBED",
            f"durumlar={states} msg={pc.message}")

# ---------------------------------------------------------------- 5. abort yollari
print("\n[5] abort yolları")
cfg = make_cfg()
app = FakeApp(cfg, START, button=False, vis_min_z=4.6)   # kor inise girsin
pc, ok = start_pc(app)
states = run(pc, app)
assert_true("butonsuz ENDGAME → ABORT", pc.state == "ABORT"
            and "buton" in pc.message, f"{states} {pc.message}")
assert_true("abort mıknatısı kapattı", app.http_calls
            and app.http_calls[-1] == "/magnet?s=0" or "/magnet?s=0" in app.http_calls)

cfg = make_cfg()
app = FakeApp(cfg, START, frozen_cam=True)
pc, ok = start_pc(app)
states = run(pc, app)
assert_true("donuk kamera: probe strict ABORT", pc.state == "ABORT"
            and "robe" in pc.message, f"{states} {pc.message}")

cfg = make_cfg()
app = FakeApp(cfg, START, blind_cam=True)
pc, ok = start_pc(app)
states = run(pc, app)
assert_true("tespitsiz SEAT → ABORT + ARAMA POZU yönlendirmesi",
            pc.state == "ABORT" and "ARAMA POZU" in pc.message,
            f"{states} {pc.message}")

# ---------------------------------------------------------------- 6. zarf blogu
print("\n[6] kısıtlı zarf: alçalma bloklanır → ABORT (ihlal yok)")
cfg = make_cfg(el_min=92)   # buton el≈76 ister -> floor 92 asla ulastirmaz
app = FakeApp(cfg, [97, 90, 110, 60])
pc, ok = start_pc(app)
assert_true("kısıtlı zarfta start kabul", ok, pc.message)
states = run(pc, app)
assert_true("alçalma bloklanınca ABORT", pc.state == "ABORT"
            and ("zarf" in pc.message or "buton" in pc.message),
            f"{states} {pc.message}")

# ---------------------------------------------------------------- 7. settle fallback
print("\n[7] arm_state yok → settle_ticks fallback ile yine GRABBED")
cfg = make_cfg()
app = FakeApp(cfg, START, arm_feedback=False)
pc, ok = start_pc(app)
logpath = pc.rl.path if pc.rl else ""
states = run(pc, app)
assert_true("feedback'siz de GRABBED", pc.state == "GRABBED",
            f"{states} {pc.message}")
ticks = []
if logpath and os.path.exists(logpath):
    with open(logpath, encoding="utf-8") as fh:
        ticks = [json.loads(ln) for ln in fh if '"kind": "tick"' in ln]
assert_true("settle 'ticks' fallback yolu kullanıldı",
            any(t.get("settle") == "ticks" for t in ticks))

# ---------------------------------------------------------------- temizlik + sonuc
for p in _run_logs:
    try:
        os.remove(p)
    except OSError:
        pass

print()
print(f"SONUC: {_passed} OK / {_failed} HATA")
if _failures:
    for m in _failures:
        print(m)
sys.exit(0 if _failed == 0 else 1)

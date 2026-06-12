"""
Kapma v4 (gercek kinematik) cevrimdisi testleri.

SENTETIK DUNYA: ayni ArmModel hem kontrolorde hem similasyonda — kup SABIT bir
dunya konumunda durur; kamera goruntusu model.project_point ile, bbox boyutu
pinhole'dan (f*kup/mesafe) uretilir; buton, FK'ya gore miknatis ucu kupun
ustune inince tetiklenir. Boylece kontrol mantigi + IK/FK/isin matematigi
kapali dongude dogrulanir (kalibrasyon dogrulugu sahada cetvelle saglanir).

Calistir: python pc/test_pickup_v4.py   (run_tests.py otomatik calistirir)
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

for _stream in (sys.stdout, sys.stderr):
    _reconf = getattr(_stream, "reconfigure", None)
    if callable(_reconf):
        _reconf(encoding="utf-8")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_kinematics import ArmModel, point_from_height, calibrate_camera
from pickup_controller import PickupController

_passed = 0
_failed = 0
_failures: list = []
_run_logs: set = set()


def assert_true(label: str, cond, detail: object = ""):
    global _passed, _failed
    if cond:
        _passed += 1
        print(f"  ✓ {label}")
    else:
        _failed += 1
        msg = f"  ✗ {label}" + (f": {detail}" if detail != "" else "")
        _failures.append(msg)
        print(msg)


def assert_close(label: str, actual, expected, tol):
    assert_true(label, abs(actual - expected) <= tol,
                f"got {actual!r}, expected {expected!r} ±{tol}")


# ---------------------------------------------------------------- model + dunya
KIN = {
    "cube_cm": 4.0, "L1": 10.0, "L2": 10.0, "L3": 6.0, "H0": 7.0, "R0": 2.0,
    # IKI-NOKTALI referanslar, BILEREK 1:1 OLMAYAN egimlerle (servo birimi ≠
    # derece — kullanicinin uyardigi durum): sh 0.9°/birim, el 0.682°/birim.
    # Ofsetler, dirsek-asagi IK cozumleri 0..180 servo bandina dussun diye
    # secildi (gercek kolda mekanik montajdan gelir, kalibrasyonla olculur).
    "sh_pts": [[0, 0], [100, 90]],          # alpha = 0.9*s
    "el_pts": [[170, -40], [60, -115]],     # beta_rel = 0.682*s - 155.9
    "gr_pts": [[90, -90], [150, -30]],      # gamma_rel = s - 180
    "base_pts": [[93, 0], [3, -90]],        # yaw = s - 93
    "cam_f": 370.0, "cam_cx": 240.0, "cam_cy": 160.0,
    "cam_dx": 2.0, "cam_dz": 2.5, "cam_pitch": 0.0,
}
MODEL = ArmModel(KIN)
W, H = 480, 320
CUBE = (1.0, 13.0)          # kup dunya (x, y) cm; ust z=4, govde merkezi z=2


def cam_world(servos):
    f = MODEL.fk(servos)
    yw = f["yaw"] * math.pi / 180.0
    r, z = f["C"]
    return (r * math.sin(yw), r * math.cos(yw), z)


def make_env(el_min=0):
    return {"type": "limits", "margin_deg": 0,
            "sh_grid": [0, 180], "el_min": [el_min, el_min],
            "el_max": [180, 180],
            "gr_rows": [{"sh": 0, "el": [0, 180], "lo": [0, 0], "hi": [180, 180]},
                        {"sh": 180, "el": [0, 180], "lo": [0, 0], "hi": [180, 180]}]}


def make_cfg(el_min=0, **over):
    auto: dict = {"safe_envelope": make_env(el_min), "kinematics": dict(KIN),
                  "total_timeout_ticks": 4000}
    auto.update(over)
    return {"autonomous": auto}


class Var:
    def __init__(self, v):
        self.v = v

    def get(self):
        return self.v


class FakeApp:
    """Sentetik fizik: servolar hedefe 5°/adim yurur; tespit modelden uretilir;
    buton FK'ya gore tetiklenir."""

    def __init__(self, cfg, start_pose, arm_feedback=True, button=True,
                 blind_cam=False):
        self.pickup_cfg = cfg
        self.detection_state: dict = {}
        self.grip_state: dict = {}
        self.arm_state: dict = {}
        self.arm_servo_vars = {"T": [Var(v) for v in start_pose]}
        self.http_calls: list = []
        self.servos = list(start_pose)
        self.targets = list(start_pose)
        self.arm_feedback = arm_feedback
        self.button = button
        self.blind_cam = blind_cam
        self._seq = 0

    def after(self, _ms, _cb=None):
        return 1

    def after_cancel(self, _i):
        pass

    def _arm_http_get(self, _agv, path):
        self.http_calls.append(path)
        if path.startswith("/servo?id="):
            kv = dict(p.split("=") for p in path.split("?", 1)[1].split("&"))
            self.targets[int(kv["id"])] = int(kv["a"])

    def _pickup_log(self, _agv, msg):
        pass

    def _set_status(self, msg):
        pass

    def _pickup_ui_refresh(self, _agv=""):
        pass

    def step(self):
        for i in range(4):
            d = self.targets[i] - self.servos[i]
            if d:
                self.servos[i] += max(-5, min(5, d))
        if self.arm_feedback:
            self.arm_state["T"] = {"servos": list(self.servos),
                                   "targets": list(self.targets),
                                   "ts": time.time()}
        # tespit: kup govde merkezi (z=2) modelle projeksiyonlanir
        if not self.blind_cam:
            pj = MODEL.project_point(self.servos, CUBE[0], CUBE[1], 2.0)
        else:
            pj = None
        if pj is not None and -50 < pj[0] < W + 50 and -50 < pj[1] < H + 50:
            cx, cy = pj
            c = cam_world(self.servos)
            dist = math.dist(c, (CUBE[0], CUBE[1], 2.0))
            bh = KIN["cam_f"] * KIN["cube_cm"] / max(dist, 0.1)
            x1, y1 = max(0.0, cx - bh / 2), max(0.0, cy - bh / 2)
            x2, y2 = min(float(W), cx + bh / 2), min(float(H), cy + bh / 2)
            if x2 - x1 >= 6 and y2 - y1 >= 6 and 0 <= cx <= W and 0 <= cy <= H:
                self.detection_state["T"] = {
                    "cx": cx, "cy": cy, "area": int(bh * bh), "conf": 0.95,
                    "h": y2 - y1, "w": x2 - x1,
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                    "frame_w": W, "frame_h": H, "ts": time.time()}
            else:
                self.detection_state.pop("T", None)
        else:
            self.detection_state.pop("T", None)
        # buton: miknatis ucu kupun ustunde + yeterince alcak
        mx, my, mz = MODEL.magnet_world(self.servos)
        if (self.button and mz <= 4.45
                and math.hypot(mx - CUBE[0], my - CUBE[1]) <= 1.3):
            g = self.grip_state.get("T") or {}
            if g.get("type") != "held":
                self._seq += 1
                self.grip_state["T"] = {"type": "held", "seq": self._seq,
                                        "ts": time.time()}


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


# baslangic: kupun ~12 cm ustunde hover (kullanici kolu kabaca konumlamis gibi)
START = MODEL.ik(CUBE[0], CUBE[1], 4.0 + 12.0)
assert START is not None, "test kurulumu: baslangic IK cozulemedi"

# ---------------------------------------------------------------- 1. saf matematik
print("\n[1] FK/IK/isin matematigi")
tgt = MODEL.ik(2.0, 14.0, 8.0, current=START)
assert_true("IK çözüm buldu", tgt is not None)
if tgt:
    mx, my, mz = MODEL.magnet_world(tgt)
    assert_close("IK→FK roundtrip x", mx, 2.0, 0.6)
    assert_close("IK→FK roundtrip y", my, 14.0, 0.6)
    assert_close("IK→FK roundtrip z", mz, 8.0, 0.6)
    g = MODEL.fk(tgt)["gamma"]
    assert_close("mıknatıs tam aşağı (γ=-90)", g, -90.0, 4.0)
ok_round = True
for (x, y) in ((0.0, 11.0), (3.0, 14.0), (-4.0, 12.0)):
    pj = MODEL.project_point(START, x, y, 2.0)
    if pj is None:
        ok_round = False
        continue
    est = MODEL.cube_from_pixel(START, pj[0], pj[1], z_plane=2.0)
    if est is None or math.hypot(est[0] - x, est[1] - y) > 0.15:
        ok_round = False
assert_true("ışın-zemin roundtrip (3 nokta, <1.5mm)", ok_round)
assert_true("erişim dışı IK → None", MODEL.ik(0.0, 40.0, 8.0) is None)
m2 = ArmModel({**KIN, "sh_pts": []})
assert_true("eksik referans missing() listesinde",
            any("L1" in s for s in m2.missing()), m2.missing())
m3 = ArmModel({**KIN, "el_pts": [[60, -115]]})   # tek nokta = egim bilinmiyor
assert_true("tek nokta '2. referans' ister",
            any("2. referans" in s for s in m3.missing()), m3.missing())
k_sh = MODEL.maps["sh"]
assert_true("sh fit mevcut", k_sh is not None)
if k_sh is not None:
    assert_close("uydurulan eğim 0.9 (servo≠derece)", k_sh[0], 0.9, 0.01)
# YUKSEKLIKTEN referans: FK'nin verdigi gercek yuksekligi point_from_height'a
# geri besle -> ayni aciyi bulmali ("kol yatay olmuyor" cozumu)
fkd = MODEL.fk(START)
pfh = point_from_height(MODEL, "sh", START, fkd["E"][1])
assert_true("📏 dirsek yüks. → alpha geri çıkıyor",
            not isinstance(pfh, str) and abs(pfh[1] - fkd["alpha"]) < 0.5,
            (pfh, fkd["alpha"]))
pfh = point_from_height(MODEL, "el", START, fkd["W"][1])
assert_true("📏 bilek yüks. → beta_rel geri çıkıyor",
            not isinstance(pfh, str)
            and abs(pfh[1] - (fkd["beta"] - fkd["alpha"])) < 0.5,
            (pfh, fkd["beta"] - fkd["alpha"]))
pfh = point_from_height(MODEL, "gr", START, fkd["M"][1])
assert_true("📏 uç yüks. → gamma_rel geri çıkıyor",
            not isinstance(pfh, str)
            and abs(pfh[1] - (fkd["gamma"] - fkd["beta"])) < 1.0,
            (pfh, fkd["gamma"] - fkd["beta"]))
assert_true("📏 H0 yokken net hata",
            isinstance(point_from_height(ArmModel({**KIN, "H0": 0}), "sh",
                                         START, 15.0), str))

# KAMERA KALIBRASYONU: bilerek BOZUK kamera (egim+odak) kur, bilinen dunya
# noktalarindan geri coz -> dogru pitch/f bulunmali, RMS kucuk olmali.
true_cam = {**KIN, "cam_pitch": -12.0, "cam_f": 410.0, "cam_dz": 3.5}
TM = ArmModel(true_cam)
def cam_samples_at(model, pts, hover=10.0):
    out = []
    for (cx, cy) in pts:
        pose = model.ik(cx, cy, 4.0 + hover)
        if pose is None:
            continue
        pj = model.project_point(pose, cx, cy, 2.0)
        if pj is not None and 0 <= pj[0] <= W and 0 <= pj[1] <= H:
            out.append((pose, pj[0], pj[1], cx, cy))
    return out


cam_samples = cam_samples_at(
    TM, [(1.0, 13.0), (3.0, 14.0), (-2.0, 12.0), (0.0, 15.0), (2.0, 11.0)])
assert_true("kamera test örnekleri üretildi (≥4)", len(cam_samples) >= 4,
            len(cam_samples))
# bozuk baslangic (pitch=0, f=370, dz=2.5) -> coz
guess = {**KIN, "cam_pitch": 0.0, "cam_f": 370.0, "cam_dz": 2.5}
out, rms = calibrate_camera(guess, cam_samples)
assert_true("kamera çözümü RMS < 0.5 cm",
            out is not None and isinstance(rms, (int, float)) and rms < 0.5,
            f"rms={rms}")
# ASIL TEST: kalibre model TUTULAN-DIŞI yeni noktalarda küpü doğru bulmalı
# (param degerleri degil, TAHMIN dogrulugu onemli — pitch/dz dejenerasyonu)
if out:
    CM = ArmModel(out)
    held = cam_samples_at(TM, [(0.0, 13.5), (2.5, 12.5), (-1.5, 14.0)])
    worst = 0.0
    for (pose, u, v, x, y) in held:
        est = CM.cube_from_pixel(pose, u, v, 2.0)
        if est is None:
            worst = 99.0
        else:
            worst = max(worst, ((est[0] - x) ** 2 + (est[1] - y) ** 2) ** 0.5)
    assert_true("kalibre model held-out küpü <1 cm bulur", worst < 1.0,
                f"en kötü {worst:.2f} cm")
    # bozuk model (kalibrasyonsuz) AYNI noktalarda KÖTÜ olmali (anlamli fark)
    BM = ArmModel(guess)
    bad = 0.0
    for (pose, u, v, x, y) in held:
        est = BM.cube_from_pixel(pose, u, v, 2.0)
        if est:
            bad = max(bad, ((est[0] - x) ** 2 + (est[1] - y) ** 2) ** 0.5)
    assert_true("kalibrasyon belirgin düzeltme yaptı", bad > worst + 1.0,
                f"bozuk {bad:.1f} vs kalibre {worst:.1f}")
assert_true("az örnekte net hata",
            calibrate_camera(KIN, cam_samples[:2])[0] is None)
# YANAL yayilim YOK (hepsi x=0 cizgisinde) -> f cozulmemeli, KACMAMALI
line_samples = cam_samples_at(TM, [(0.0, 11.0), (0.0, 14.0), (0.0, 17.0),
                                   (0.0, 12.5), (0.0, 15.5)])
lo, lr = calibrate_camera(guess, line_samples)
assert_true("tek çizgi: f makul sınırda kalır (kaçmaz)",
            lo is not None and 250 <= float(lo.get("cam_f") or 0) <= 950,
            lo.get("cam_f") if lo else None)
# saglamlik: gurultulu olcumle bile RMS makul + f sinirda
import random as _r
_r.seed(1)
noisy = [(p, u + _r.uniform(-3, 3), v + _r.uniform(-3, 3), x, y)
         for (p, u, v, x, y) in cam_samples]
no, nr = calibrate_camera(guess, noisy)
assert_true("gürültülü veri: f sınırda + RMS sonlu",
            no is not None and 250 <= float(no.get("cam_f") or 0) <= 950
            and isinstance(nr, (int, float)) and nr < 5.0, (no.get("cam_f") if no else None, nr))

# PARALELKENAR KOL (el_abs): dirsek servosu on kolun MUTLAK acisini kontrol
# eder -> shoulder oynayinca on kol egimi DEGISMEZ.
PKIN = {**KIN, "el_abs": True,
        "el_pts": [[90, 0], [0, -90]]}   # beta_abs = s2 - 90 (L1'den bagimsiz)
PM = ArmModel(PKIN)
b1 = PM.beta_abs(40, 120)
b2 = PM.beta_abs(140, 120)   # ayni dirsek servosu, FARKLI shoulder
assert_close("paralelkenar: ön kol açısı shoulder'dan bağımsız", b1, b2, 0.01)
assert_close("paralelkenar beta_abs = s2-90", b1, 30.0, 0.01)
# IK->FK roundtrip paralelkenar modelde de tutmali
pj = PM.ik(1.0, 13.0, 8.0)
assert_true("paralelkenar IK çözüm", pj is not None)
if pj:
    mx, my, mz = PM.magnet_world(pj)
    assert_true("paralelkenar IK→FK roundtrip",
                abs(mx - 1.0) < 0.6 and abs(my - 13.0) < 0.6 and abs(mz - 8.0) < 0.6,
                (mx, my, mz))

# ---------------------------------------------------------------- 2. start dogrulama
print("\n[2] start doğrulama")
cfg = make_cfg()
cfg["autonomous"]["kinematics"] = {**KIN, "gr_pts": []}
pc, ok = start_pc(FakeApp(cfg, START))
assert_true("eksik kinematik reddedilir", not ok and "KİNEMATİK" in pc.message,
            pc.message)

# ---------------------------------------------------------------- 3. tam akis
print("\n[3] tam akış: AIM → DESCEND → GRABBED")
cfg = make_cfg()
app = FakeApp(cfg, START)
pc, ok = start_pc(app)
assert_true("start kabul", ok, pc.message)
logpath = pc.rl.path if pc.rl else ""
states = run(pc, app)
assert_true("GRABBED'e ulaşıldı", pc.state == "GRABBED",
            f"durumlar={states} msg={pc.message}")
assert_true("durum sırası AIM→DESCEND→GRABBED",
            states == ["AIM", "DESCEND", "GRABBED"], states)
assert_true("mıknatıs açıldı", "/magnet?s=1" in app.http_calls)
if pc._cube:
    err = math.hypot(pc._cube[0] - CUBE[0], pc._cube[1] - CUBE[1])
    assert_true("küp konumu <1 cm hatayla bulundu",
                err < 1.0, f"tahmin={pc._cube} gerçek={CUBE} hata={err:.2f}cm")
ticks = []
if logpath and os.path.exists(logpath):
    with open(logpath, encoding="utf-8") as fh:
        ticks = [json.loads(ln) for ln in fh if '"kind": "tick"' in ln]
assert_true("settle 'arm' yolu kullanıldı",
            any(t.get("settle") == "arm" for t in ticks))

# ---------------------------------------------------------------- 3b. aim trim
print("\n[3b] mıknatıs merkez trim")
cfg = make_cfg()
cfg["autonomous"]["aim_trim_fwd"] = 1.0
cfg["autonomous"]["aim_trim_lat"] = 0.5
pcx = PickupController(FakeApp(cfg, START), "T")
pcx._cfg = {**PickupController.DEFAULTS, **cfg["autonomous"]}
pcx.model = ArmModel(KIN)
# kup tam ileride (x=0): ileri trim y'yi artirmali, yan trim x'i artirmali
ax, ay = pcx._aimed_xy(0.0, 14.0)
assert_close("trim: ileri hedefi uzaklaştırır", ay, 15.0, 0.01)
assert_close("trim: yan hedefi sağa kaydırır", ax, 0.5, 0.01)
# trim yokken degismez
pcx._cfg["aim_trim_fwd"] = 0.0
pcx._cfg["aim_trim_lat"] = 0.0
assert_true("trim 0 → değişmez", pcx._aimed_xy(2.0, 13.0) == (2.0, 13.0))

# ---------------------------------------------------------------- 4. abort yollari
print("\n[4] abort yolları")
cfg = make_cfg()
app = FakeApp(cfg, START, button=False)
pc, ok = start_pc(app)
states = run(pc, app)
assert_true("butonsuz: zemine inip ABORT", pc.state == "ABORT"
            and "buton" in pc.message, f"{states} {pc.message}")
assert_true("abort mıknatısı kapattı", "/magnet?s=0" in app.http_calls)

cfg = make_cfg()
app = FakeApp(cfg, START, blind_cam=True)
pc, ok = start_pc(app)
states = run(pc, app)
assert_true("tespitsiz AIM → ABORT", pc.state == "ABORT"
            and "kayboldu" in pc.message, f"{states} {pc.message}")

# ---------------------------------------------------------------- 5. zarf blogu
print("\n[5] kısıtlı zarf: iniş bloklanır → ABORT (ihlal yok)")
cfg = make_cfg(el_min=int(START[2]) + 4)   # elbow tabani inise izin vermesin
app = FakeApp(cfg, [START[0], START[1], max(START[2], int(START[2]) + 4), START[3]])
pc, ok = start_pc(app)
if ok:
    states = run(pc, app)
    assert_true("zarf blokunda ABORT", pc.state == "ABORT", f"{states} {pc.message}")
else:
    assert_true("kısıtlı zarf: start poz reddi de kabul", "DIŞINDA" in pc.message,
                pc.message)

# ---------------------------------------------------------------- 6. settle fallback
print("\n[6] arm_state yok → settle_ticks fallback ile yine GRABBED")
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
for m in _failures:
    print(m)
sys.exit(0 if _failed == 0 else 1)

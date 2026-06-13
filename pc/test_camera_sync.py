"""
firmware/camera ↔ firmware/camera2 senkron guard'i.

camera2/ AGV_2'nin kamerasidir ve camera/ ile KIMLIK DISINDA birebir ayni
olmalidir (agv/agv2 ile ayni elle-senkron modeli). Desync SESSIZDIR: camera'ya
yapilan bir bug fix camera2'ye tasinmazsa AGV_2'nin kolu farkli davranir ve
bunu fark etmek cok zordur. Bu test her dosyayi karsilastirir:

  * camera.ino ↔ camera2.ino                       → birebir ayni
  * config.h                                       → yalniz CAM_ID ve
    CAM_IP_OCT_4 define satirlari farkli olabilir (CAM_2 / 51 olmali)
  * diger tum ortak dosyalar (arm.ino, mjpeg_server.ino, cam_log.ino,
    wifi_setup.ino, pin.h, types.h)                → birebir ayni
  * iki klasorde fazladan/eksik dosya olmamali

Calistir: python pc/test_camera_sync.py   (run_tests.py otomatik calistirir)
"""

from __future__ import annotations

import os
import sys
from typing import List

for _stream in (sys.stdout, sys.stderr):
    _reconf = getattr(_stream, "reconfigure", None)
    if callable(_reconf):
        _reconf(encoding="utf-8")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAM1 = os.path.join(ROOT, "firmware", "camera")
CAM2 = os.path.join(ROOT, "firmware", "camera2")

_passed = 0
_failed = 0
_failures: List[str] = []


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


def read(path: str) -> str:
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


def norm_lines(text: str) -> List[str]:
    """Satir sonu farklarini (CRLF/LF) yok say."""
    return text.replace("\r\n", "\n").split("\n")


def main() -> int:
    print("\n[camera ↔ camera2 senkron]")
    ok1 = os.path.isdir(CAM1)
    ok2 = os.path.isdir(CAM2)
    assert_true("iki klasor de mevcut", ok1 and ok2, (CAM1, CAM2))
    if not (ok1 and ok2):
        return 1

    f1 = {n for n in os.listdir(CAM1) if not n.startswith(".")}
    f2 = {n for n in os.listdir(CAM2) if not n.startswith(".")}
    # Ana sketch'ler klasor adiyla eslesir (Arduino kurali)
    expect_map = {("camera.ino" if n == "camera2.ino" else n) for n in f2}
    assert_true("dosya kumeleri esit (ana sketch adı haric)",
                f1 == expect_map,
                f"sadece camera: {sorted(f1 - expect_map)}, "
                f"sadece camera2: {sorted(expect_map - f1)}")

    # Ana sketch birebir ayni
    a = norm_lines(read(os.path.join(CAM1, "camera.ino")))
    b = norm_lines(read(os.path.join(CAM2, "camera2.ino")))
    assert_true("camera.ino ↔ camera2.ino birebir aynı", a == b,
                f"ilk fark: satır "
                f"{next((i + 1 for i, (x, y) in enumerate(zip(a, b)) if x != y), '?')}")

    # config.h: yalniz kimlik satirlari farkli olabilir
    c1 = norm_lines(read(os.path.join(CAM1, "config.h")))
    c2 = norm_lines(read(os.path.join(CAM2, "config.h")))
    assert_true("config.h satır sayısı aynı", len(c1) == len(c2),
                (len(c1), len(c2)))
    ident_keys = ("#define CAM_ID", "#define CAM_IP_OCT_4")
    # AGV_2'nin kolu fiziksel olarak FARKLI — bu mekanik kalibrasyon define'lari
    # camera2'de kasten farkli olabilir (per-kol kimlik, CAM_ID gibi). Diger her
    # sey birebir ayni kalmali. (pickup_config_<AGV>.json'daki PC tarafi ayrim
    # ile ayni mantik.)
    arm_calib_keys = (
        "#define SERVO_BASE_HOME", "#define SERVO_SHOULDER_HOME",
        "#define SERVO_ELBOW_HOME", "#define SERVO_GRIPPER_HOME",
        "#define SERVO_ELBOW_CLEAR",
        "#define SERVO_BASE_BOOT", "#define SERVO_SHOULDER_BOOT",
        "#define SERVO_ELBOW_BOOT", "#define SERVO_GRIPPER_BOOT",
    )
    bad = []
    cam2_id_ok = cam2_ip_ok = False
    for i, (l1, l2) in enumerate(zip(c1, c2), start=1):
        if l1 == l2:
            continue
        if any(k in l1 and k in l2 for k in ident_keys):
            if "#define CAM_ID" in l2 and '"CAM_2"' in l2:
                cam2_id_ok = True
            if "#define CAM_IP_OCT_4" in l2 and "51" in l2.split("//")[0]:
                cam2_ip_ok = True
            continue
        # Kol mekanik kalibrasyonu (HOME/CLEAR/BOOT): AGV_2'de farkli olabilir
        if any(k in l1 and k in l2 for k in arm_calib_keys):
            continue
        bad.append(f"satır {i}: {l1!r} ↔ {l2!r}")
    assert_true("config.h yalnız kimlik + kol kalibrasyon satırlarında farklı",
                not bad, bad[:3])
    assert_true("camera2 CAM_ID = CAM_2", cam2_id_ok)
    assert_true("camera2 CAM_IP_OCT_4 = 51", cam2_ip_ok)

    # Diger ortak dosyalar birebir ayni
    for name in sorted(f1 - {"camera.ino", "config.h"}):
        p1, p2 = os.path.join(CAM1, name), os.path.join(CAM2, name)
        if not os.path.exists(p2):
            assert_true(f"{name} camera2'de var", False)
            continue
        l1, l2 = norm_lines(read(p1)), norm_lines(read(p2))
        first = next((i + 1 for i, (x, y) in enumerate(zip(l1, l2)) if x != y),
                     None if len(l1) == len(l2) else min(len(l1), len(l2)) + 1)
        assert_true(f"{name} birebir aynı", l1 == l2,
                    f"ilk fark satır {first}")

    print()
    print(f"SONUC: {_passed} OK / {_failed} HATA")
    for msg in _failures:
        print(msg)
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

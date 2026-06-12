"""
Tum PC testlerini siralı calistir + ozet rapor.

Kullanim:
    python run_tests.py

Donus kodu: tum testler gectiyse 0, herhangi biri basarisizsa 1.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time


HERE = os.path.dirname(os.path.abspath(__file__))
PYTHON = sys.executable

TEST_FILES = [
    ("graph",         "test_graph.py"),
    ("fleet_planner", "test_fleet_planner.py"),
    ("integration",   "test_integration.py"),
    ("waypoint_sync", "test_waypoint_sync.py"),   # PC<->firmware graf senkron
    ("wire_protocol", "test_wire_protocol.py"),   # setHop govde sozlesmesi
    ("pickup_v3",     "test_pickup_v3.py"),       # kapma v3 sentetik kapali dongu
]


def run_one(label: str, path: str) -> tuple[bool, float, str]:
    t0 = time.time()
    proc = subprocess.run(
        [PYTHON, os.path.join(HERE, path)],
        capture_output=True, text=True, encoding="utf-8",
    )
    dt = time.time() - t0
    # "SONUC: N OK / M HATA" satirini bul
    summary = ""
    for line in (proc.stdout + proc.stderr).splitlines():
        if "SONUC:" in line:
            summary = line.strip()
            break
    ok = proc.returncode == 0
    return ok, dt, summary or f"exit {proc.returncode}"


def main() -> int:
    print("=" * 60)
    print(" PC Test Suite")
    print("=" * 60)
    results = []
    for label, path in TEST_FILES:
        if not os.path.exists(os.path.join(HERE, path)):
            print(f"  SKIP {label}: {path} bulunamadi")
            continue
        ok, dt, summary = run_one(label, path)
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {label:<14} {dt:5.2f}s  {summary}")
        results.append((label, ok))

    failed = [lbl for lbl, ok in results if not ok]
    print("=" * 60)
    if failed:
        print(f" {len(failed)}/{len(results)} HATA: {', '.join(failed)}")
        return 1
    print(f" Tum testler gecti ({len(results)} dosya)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

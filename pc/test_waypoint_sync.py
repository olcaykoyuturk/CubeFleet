"""
Graf topolojisi senkron testi — PC <-> firmware uyumluluk guvencesi.

NEDEN: Graf topolojisi BIRDEN COK elle bakimi yapilan kopyada tanimli:
  - pc/waypoints.json                 (PC A* — cost + koordinat)
  - firmware/agv/waypoint_map.h        (AGV_1 donus lookup — heading)
  - firmware/agv2/waypoint_map.h       (AGV_2 donus lookup — heading)
Bunlari senkron tutacak codegen yok. Desync SESSIZDIR: PC json'a kenar eklerse
firmware derlenmez/uyarmaz; getDirection() o junction'da false doner ve AGV
gorev ortasinda sessizce NAV_IDLE'a duser. Bu test desync'i CI-tarzi yakalar.

Kontroller:
  1. firmware/agv WAYPOINT_MAP undirected kenar seti == waypoints.json kenar seti
  2. firmware/agv2 WAYPOINT_MAP undirected kenar seti == waypoints.json kenar seti
  3. agv ve agv2 node->(komsu, yon) haritasi BIREBIR ayni (copy-paste drift yok)
  4. firmware haritasi bidirectional tutarli (her kenar iki yonde de tanimli)

Calistir: python pc/test_waypoint_sync.py   (run_tests.py otomatik calistirir)
Donus kodu: tum kontroller gecerse 0, aksi halde 1.
"""

from __future__ import annotations
import json
import os
import re
import sys

for _stream in (sys.stdout, sys.stderr):
    _reconf = getattr(_stream, "reconfigure", None)
    if callable(_reconf):
        _reconf(encoding="utf-8")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WAYPOINTS_JSON = os.path.join(HERE, "waypoints.json")
AGV_MAP  = os.path.join(ROOT, "firmware", "agv",  "waypoint_map.h")
AGV2_MAP = os.path.join(ROOT, "firmware", "agv2", "waypoint_map.h")

_passed = 0
_failed = 0
_failures: list[str] = []


def check(label: str, ok: bool, detail: str = "") -> None:
    global _passed, _failed
    if ok:
        _passed += 1
        print(f"  ✓ {label}")
    else:
        _failed += 1
        msg = f"  ✗ {label}" + (f": {detail}" if detail else "")
        _failures.append(msg)
        print(msg)


# ----- waypoints.json -----------------------------------------------------

def json_edges(path: str) -> set:
    """Undirected kenar seti: {(min,max), ...}."""
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    edges = set()
    for e in doc["edges"]:
        a, b = e["from"], e["to"]
        edges.add((a, b) if a <= b else (b, a))
    return edges


# ----- waypoint_map.h parse ----------------------------------------------

# Her satir: {'X', {{'Y', DIR}, {'Z', DIR}}, N},
_ENTRY_RE = re.compile(r"\{\s*'([A-Z])'\s*,\s*\{(.*)\}\s*,\s*\d+\s*\}")
_NEIGH_RE = re.compile(r"\{\s*'([A-Z])'\s*,\s*([A-Z_]+)\s*\}")


def parse_map(path: str) -> dict:
    """Node -> [(komsu, yon), ...] sozlugu dondur."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    # WAYPOINT_MAP bloku
    start = text.find("WAYPOINT_MAP")
    block = text[start:] if start >= 0 else text
    out: dict = {}
    for m in _ENTRY_RE.finditer(block):
        node = m.group(1)
        neighbors = [(g[0], g[1]) for g in _NEIGH_RE.findall(m.group(2))]
        out[node] = sorted(neighbors)
    return out


def map_edges(node_map: dict) -> set:
    """Node->komsu haritasindan undirected kenar seti."""
    edges = set()
    for node, neighbors in node_map.items():
        for nb, _dir in neighbors:
            edges.add((node, nb) if node <= nb else (nb, node))
    return edges


# ----- Testler ------------------------------------------------------------

def main() -> int:
    print("=" * 60)
    print(" Waypoint Graf Senkron Testi (PC <-> firmware)")
    print("=" * 60)

    je = json_edges(WAYPOINTS_JSON)
    agv_map  = parse_map(AGV_MAP)
    agv2_map = parse_map(AGV2_MAP)
    agv_e  = map_edges(agv_map)
    agv2_e = map_edges(agv2_map)
    print(f"\n  waypoints.json: {len(je)} kenar | "
          f"agv: {len(agv_e)} kenar | agv2: {len(agv2_e)} kenar")

    # 1) PC <-> AGV_1 kenar seti
    check("waypoints.json == firmware/agv WAYPOINT_MAP (kenar seti)",
          je == agv_e,
          f"PC-only={sorted(je - agv_e)}, fw-only={sorted(agv_e - je)}")

    # 2) PC <-> AGV_2 kenar seti
    check("waypoints.json == firmware/agv2 WAYPOINT_MAP (kenar seti)",
          je == agv2_e,
          f"PC-only={sorted(je - agv2_e)}, fw-only={sorted(agv2_e - je)}")

    # 3) agv ve agv2 node->(komsu,yon) BIREBIR ayni (copy-paste drift)
    drift = [n for n in set(agv_map) | set(agv2_map)
             if agv_map.get(n) != agv2_map.get(n)]
    check("firmware/agv ve firmware/agv2 WAYPOINT_MAP birebir ayni",
          not drift, f"farkli node'lar: {drift}")

    # 4) Bidirectional tutarlilik: her A->B icin B->A da tanimli
    asym = []
    for node, neighbors in agv_map.items():
        for nb, _dir in neighbors:
            back = [x for x, _ in agv_map.get(nb, [])]
            if node not in back:
                asym.append((node, nb))
    check("firmware haritasi bidirectional (her kenar iki yonde)",
          not asym, f"tek-yon kenarlar: {asym}")

    print("\n" + "=" * 60)
    print(f" SONUC: {_passed} OK / {_failed} HATA")
    print("=" * 60)
    if _failures:
        print("\nBasarisiz kontroller:")
        for f in _failures:
            print(" ", f)
        print("\n  → Graf topolojisini degistirdiysen TUM kopyalari guncelle: "
              "waypoints.json, firmware/agv/waypoint_map.h, "
              "firmware/agv2/waypoint_map.h, pc/waypoint_canvas.py")
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

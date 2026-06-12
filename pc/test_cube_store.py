"""
Kup deposu + yon konvansiyonu testi.

NEDEN: Kup yerlestirme "node'un cizgisiz kenari" kavramina dayanir; bu kenarlar
PC'de koordinattan (heading_between) turetilir, AGV ise faceDir ile firmware
WAYPOINT_MAP'in yon semantigine gore doner. Iki taraf FARKLI kuzey anlarsa kup
yanlis kenara aranir. Bu test PC turetimini firmware haritasina KILITLER:
waypoints.json'daki HER kenar icin heading_between == WAYPOINT_MAP yonu.

Kontroller:
  1. heading_between: tum kenarlarda firmware WAYPOINT_MAP yonuyle birebir
  2. free_sides: kenarli yonler dislanir (F→E/W, B→N, H→S, A→N/S/W ornekleri)
  3. CubeStore: 4 kup default null + place/pick_up/drop/clear yasam dongusu
  4. JSON round-trip (atomik save → load) + bozuk dosya toleransi
  5. Sorgular: cube_at / carried_by / placed_cubes / unplaced_cubes

Calistir: python pc/test_cube_store.py   (run_tests.py otomatik calistirir)
"""

from __future__ import annotations
import os
import re
import sys
import tempfile

for _stream in (sys.stdout, sys.stderr):
    _reconf = getattr(_stream, "reconfigure", None)
    if callable(_reconf):
        _reconf(encoding="utf-8")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from graph import Graph
from cube_store import (
    CubeStore, CUBE_COUNT, SIDES, free_sides, heading_between,
)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WAYPOINTS_JSON = os.path.join(HERE, "waypoints.json")
AGV_MAP = os.path.join(ROOT, "firmware", "agv", "waypoint_map.h")

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


# ----- firmware WAYPOINT_MAP parse (test_waypoint_sync ile ayni desen) ------
_ENTRY_RE = re.compile(r"\{\s*'([A-Z])'\s*,\s*\{(.*)\}\s*,\s*\d+\s*\}")
_NEIGH_RE = re.compile(r"\{\s*'([A-Z])'\s*,\s*([A-Z_]+)\s*\}")
_DIR_TO_SIDE = {"NORTH": "N", "EAST": "E", "SOUTH": "S", "WEST": "W"}


def parse_fw_dirs(path: str) -> dict:
    """{(node, komsu): 'N'/'E'/'S'/'W'} — firmware'in yon semantigi."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    start = text.find("WAYPOINT_MAP")
    block = text[start:] if start >= 0 else text
    out: dict = {}
    for m in _ENTRY_RE.finditer(block):
        node = m.group(1)
        for nb, d in _NEIGH_RE.findall(m.group(2)):
            out[(node, nb)] = _DIR_TO_SIDE.get(d, "?")
    return out


# ----- Testler ---------------------------------------------------------------

def test_heading_convention(g: Graph):
    print("\n[1] heading_between == firmware WAYPOINT_MAP (tum kenarlar)")
    fw = parse_fw_dirs(AGV_MAP)
    check("firmware haritasi parse edildi", len(fw) >= 20,
          f"sadece {len(fw)} yonlu kenar")
    bad = []
    for (a, b), fw_side in fw.items():
        pc_side = heading_between(g, a, b)
        if pc_side != fw_side:
            bad.append(f"{a}->{b}: pc={pc_side} fw={fw_side}")
    check("her kenarda PC yonu == firmware yonu", not bad, "; ".join(bad))


def test_free_sides(g: Graph):
    print("\n[2] free_sides — cizgili kenarlar dislanir")
    # Kullanicinin ornegi: F'nin N/S komsusu var (C/I), kup yalniz E/W'ye konur
    check("F → {E, W}", set(free_sides(g, "F")) == {"E", "W"},
          f"got {free_sides(g, 'F')}")
    check("B → {N}", set(free_sides(g, "B")) == {"N"},
          f"got {free_sides(g, 'B')}")
    check("E → {E}", set(free_sides(g, "E")) == {"E"},
          f"got {free_sides(g, 'E')}")
    check("H → {S}", set(free_sides(g, "H")) == {"S"},
          f"got {free_sides(g, 'H')}")
    check("A → {N, S, W}", set(free_sides(g, "A")) == {"N", "S", "W"},
          f"got {free_sides(g, 'A')}")
    check("bilinmeyen node → []", free_sides(g, "Z") == [])
    # Tum node'larda: free + kenarli = 4 yon, kesisim bos
    for n in g.nodes:
        used = {heading_between(g, n, nb) or "?" for nb, _ in g.neighbors(n)}
        fs = set(free_sides(g, n))
        check(f"{n}: free ∪ kenarli = 4 yon, kesisim bos",
              fs | used == set(SIDES) and not (fs & used),
              f"free={sorted(fs)} used={sorted(used)}")


def test_store_lifecycle(tmp: str):
    print("\n[3] CubeStore yasam dongusu (place/pick_up/drop/clear)")
    path = os.path.join(tmp, "cubes.json")
    st = CubeStore(path)
    check("4 kup olusur", len(st.cubes) == CUBE_COUNT)
    check("baslangicta hepsi null",
          all(not c.placed and not c.carried for c in st.cubes.values()))

    st.place(1, "F", "E")
    check("place: yerlesti", st.cubes[1].node == "F" and st.cubes[1].side == "E")
    check("place: tasinmiyor", not st.cubes[1].carried)

    st.pick_up(1, "AGV_1")
    c = st.cubes[1]
    check("pick_up: node/side null + carrier dolu",
          c.node is None and c.side is None and c.carrier == "AGV_1")

    st.drop(1, "H", "S")
    c = st.cubes[1]
    check("drop: yeni konum + carrier null",
          c.node == "H" and c.side == "S" and c.carrier is None)

    st.clear(1)
    c = st.cubes[1]
    check("clear: tamamen null",
          c.node is None and c.side is None and c.carrier is None)


def test_store_roundtrip(tmp: str):
    print("\n[4] JSON round-trip + bozuk dosya toleransi")
    path = os.path.join(tmp, "cubes_rt.json")
    st = CubeStore(path)
    st.place(2, "F", "W")
    st.pick_up(3, "AGV_2")
    check("dosya yazildi", os.path.exists(path))

    st2 = CubeStore(path)   # diskten yeniden yukle
    check("round-trip: K2 konumu korunur",
          st2.cubes[2].node == "F" and st2.cubes[2].side == "W")
    check("round-trip: K3 carrier korunur", st2.cubes[3].carrier == "AGV_2")
    check("round-trip: K1/K4 null",
          not st2.cubes[1].placed and not st2.cubes[4].placed)

    # Bozuk dosya → sessizce default (4 null kup), crash yok
    bad = os.path.join(tmp, "cubes_bad.json")
    with open(bad, "w", encoding="utf-8") as f:
        f.write("{bozuk json!!")
    st3 = CubeStore(bad)
    check("bozuk dosya → default 4 null kup",
          len(st3.cubes) == CUBE_COUNT
          and all(not c.placed for c in st3.cubes.values()))


def test_queries(tmp: str):
    print("\n[5] Sorgular: cube_at / carried_by / placed / unplaced")
    path = os.path.join(tmp, "cubes_q.json")
    st = CubeStore(path)
    st.place(1, "F", "E")
    st.place(2, "F", "W")
    st.pick_up(3, "AGV_1")

    c = st.cube_at("F", "E")
    check("cube_at(F,E) = K1", c is not None and c.cube_id == 1)
    c = st.cube_at("F", "W")
    check("cube_at(F,W) = K2", c is not None and c.cube_id == 2)
    check("cube_at(F) herhangi biri", st.cube_at("F") is not None)
    check("cube_at(H) bos", st.cube_at("H") is None)
    c = st.carried_by("AGV_1")
    check("carried_by(AGV_1) = K3", c is not None and c.cube_id == 3)
    check("carried_by(AGV_2) bos", st.carried_by("AGV_2") is None)
    check("placed_cubes = {1,2}",
          {c.cube_id for c in st.placed_cubes()} == {1, 2})
    check("unplaced_cubes = {4} (K3 tasiniyor, sayilmaz)",
          {c.cube_id for c in st.unplaced_cubes()} == {4})


def main() -> int:
    print("=" * 60)
    print(" Kup Deposu + Yon Konvansiyonu Testi")
    print("=" * 60)
    g = Graph.from_json(WAYPOINTS_JSON)
    with tempfile.TemporaryDirectory() as tmp:
        test_heading_convention(g)
        test_free_sides(g)
        test_store_lifecycle(tmp)
        test_store_roundtrip(tmp)
        test_queries(tmp)
    print("\n" + "=" * 60)
    print(f" SONUC: {_passed} OK / {_failed} HATA")
    print("=" * 60)
    if _failures:
        print("\nBasarisiz kontroller:")
        for f in _failures:
            print(" ", f)
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

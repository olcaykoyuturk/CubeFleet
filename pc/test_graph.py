"""
Graph + A* senaryo testleri.

Calistir: python -m pc.test_graph
Veya:     python pc/test_graph.py

Bu testler P1 baseline'i icin. Multi-agent collision avoidance (P2-P3) onayli
test seti degil — sadece tek AGV pathfinding gercekligi.

Test scenarios bilinen yapilara odaklanir:
  - en kisa yol dogrulamasi
  - alternatif rota (kenar bloklu)
  - ulasilamaz hedefler
  - degisken kenar yonu (undirected dogrulama)
  - bocek/edge case'ler
"""

from __future__ import annotations
import os
import sys

# Windows konsolu cp1254/cp857 default — Unicode glyph'leri (✓ → ⚠) icin UTF-8'e zorla.
# sys.stdout aslinda TextIOWrapper (reconfigure var) ama tip TextIO oldugu icin
# pyrefly uyari verir — getattr ile bypass.
for _stream in (sys.stdout, sys.stderr):
    _reconf = getattr(_stream, "reconfigure", None)
    if callable(_reconf):
        _reconf(encoding="utf-8")

# Eger modul olarak değil dogrudan calistirildiysa pc/ icini sys.path'e ekle
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from graph import Graph


def load() -> Graph:
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "waypoints.json")
    return Graph.from_json(path)


# ----- Test harness -----------------------------------------------------------

_passed = 0
_failed = 0
_failures = []

def expect_path(label, actual, expected_path, expected_cost, g: Graph):
    global _passed, _failed
    cost = g.path_cost(actual) if actual else None
    ok_path = actual == expected_path
    ok_cost = (cost is not None and abs(cost - expected_cost) < 0.01)
    if ok_path and ok_cost:
        _passed += 1
        print(f"  ✓ {label}: {' → '.join(actual)} ({cost:.0f} cm)")
    else:
        _failed += 1
        msg = (f"  ✗ {label}: "
               f"got {actual} cost={cost}, "
               f"expected {expected_path} cost={expected_cost}")
        _failures.append(msg)
        print(msg)


def expect_no_path(label, actual):
    global _passed, _failed
    if actual is None:
        _passed += 1
        print(f"  ✓ {label}: no path (as expected)")
    else:
        _failed += 1
        msg = f"  ✗ {label}: expected None, got {actual}"
        _failures.append(msg)
        print(msg)


# ----- Scenarios --------------------------------------------------------------

def scenario_1_basic_shortest_paths(g: Graph):
    """En temel: A* en kisa yolu buluyor mu? (4×3 A-L duzeni)"""
    print("\n[Senaryo 1] En kisa yol doğrulamaları")
    # A → I: A-E-F-J-I = 38+20+34+20 = 112 cm (alt sira sadece F-J ile bagli)
    expect_path("A→I",  g.astar("A", "I"), ["A","E","F","J","I"], 112, g)
    # G → I: G-F-J-I = 20+34+20 = 74 cm
    expect_path("G→I",  g.astar("G", "I"), ["G","F","J","I"], 74, g)
    # D → A: D-C-B-A = 20+20+20 = 60 cm (ust sira)
    expect_path("D→A",  g.astar("D", "A"), ["D","C","B","A"], 60, g)
    # L → E: L-K-J-F-E = 20+20+34+20 = 94 cm
    expect_path("L→E",  g.astar("L", "E"), ["L","K","J","F","E"], 94, g)


def scenario_2_alternative_routes_when_blocked(g: Graph):
    """Bir kenar veya node bloklanırsa A* alternatif buluyor mu?"""
    print("\n[Senaryo 2] Alternatif rotalar (bloklu kenar/node)")

    # Edge (A,E) bloklu: A → I alternatifi A-B-C-G-F-J-I
    # = 20+20+38+20+34+20 = 152 cm
    expect_path("A→I via (A,E) bloklu",
                g.astar("A", "I", blocked_edges=[("A","E")]),
                ["A","B","C","G","F","J","I"], 152, g)

    # Edge (C,G) bloklu: D → I alternatifi (ust sira soluna dolan)
    # D-C-B-A-E-F-J-I = 20+20+20+38+20+34+20 = 172 cm
    expect_path("D→I via (C,G) bloklu",
                g.astar("D", "I", blocked_edges=[("C","G")]),
                ["D","C","B","A","E","F","J","I"], 172, g)


def scenario_3_unreachable(g: Graph):
    """Ulasilamaz hedefler None dondursun (yeni grafta dar koridorlar/dead-end)"""
    print("\n[Senaryo 3] Ulasilamaz hedefler")

    # F-J alt sirayi orta/ust grid'e baglayan TEK kenar. Bloklanirsa I/J/K/L
    # erisilmez → A → I imkansiz.
    expect_no_path("A→I via (F,J) bloklu (tek koridor)",
                   g.astar("A", "I", blocked_edges=[("F","J")]))

    # J alt sira kavsagi bloklu → I izole (I sadece J'ye bagli)
    expect_no_path("A→I via J node bloklu",
                   g.astar("A", "I", blocked_nodes=["J"]))

    # H degree-1 (sadece G-H). G bloklu → H'ye ulasilamaz.
    expect_no_path("A→H via G node bloklu (dead-end)",
                   g.astar("A", "H", blocked_nodes=["G"]))

    # Hedef node'un kendisi bloklu
    expect_no_path("A→I goal I bloklu",
                   g.astar("A", "I", blocked_nodes=["I"]))


def scenario_4_edge_cases(g: Graph):
    """Edge case'ler"""
    print("\n[Senaryo 4] Edge case'ler")

    # start == goal → [start]
    expect_path("A→A (kendi)", g.astar("A", "A"), ["A"], 0, g)

    # Gecersiz node → None
    expect_no_path("Z→A (gecersiz start)", g.astar("Z", "A"))
    expect_no_path("A→Z (gecersiz goal)", g.astar("A", "Z"))

    # Bos engel listesi
    expect_path("A→I (bos engel)",
                g.astar("A", "I", blocked_nodes=[], blocked_edges=[]),
                ["A","E","F","J","I"], 112, g)

    # Edge yon-bagimsizligi: blocked_edge ("A","E") ile ("E","A") ayni
    p1 = g.astar("A","I", blocked_edges=[("A","E")])
    p2 = g.astar("A","I", blocked_edges=[("E","A")])
    if p1 == p2:
        global _passed
        _passed += 1
        print(f"  ✓ blocked_edge yön bağımsız: {' → '.join(p1)}")
    else:
        global _failed
        _failed += 1
        msg = f"  ✗ blocked_edge yön bağımlı: (A,E)={p1}, (E,A)={p2}"
        _failures.append(msg)
        print(msg)


def scenario_5_real_multiagent_situations(g: Graph):
    """Multi-agent gercek durumlar — manuel rezervasyon simulasyonu.
    Yeni grafta F-J TEK koridor (alt sira ↔ orta grid) → head-on'da kritik."""
    print("\n[Senaryo 5] Multi-agent simulasyon (manuel rezervasyon)")

    # ===== Durum A: F-J tek koridorunda head-on =====
    # AGV_1: I → A (I-J-F-E-A), AGV_2: A → I aynı koridoru ters yönde ister.
    print("  Senaryo A: head-on F-J koridoru (I→A vs A→I), AGV_1 öncelikli")
    agv1 = g.astar("I", "A") or []
    print(f"    AGV_1 (I→A): {' → '.join(agv1)} ({g.path_cost(agv1):.0f} cm)")
    blocked = [(agv1[i], agv1[i+1]) for i in range(len(agv1) - 1)]
    agv2 = g.astar("A", "I", blocked_edges=blocked)
    if agv2:
        print(f"    AGV_2 alternatif: {' → '.join(agv2)} "
              f"({g.path_cost(agv2):.0f} cm)")
    else:
        print("    AGV_2 alternatif YOK — F-J tek koridor, yield/wait gerekli")

    # ===== Durum B: degree-1 dead-end head-on (D, H, L) =====
    # D yalniz C'ye bagli. AGV_1 C→D, AGV_2 D→C: edge (C,D) tek yol.
    print("\n  Senaryo B: D dead-end head-on (C↔D tek kenar)")
    agv1_to_d = g.astar("C", "D")
    print(f"    AGV_1 (C→D): {' → '.join(agv1_to_d)} ({g.path_cost(agv1_to_d):.0f} cm)")
    agv2_from_d = g.astar("D", "C", blocked_edges=[("C","D")])
    if agv2_from_d is None:
        print("    AGV_2 (D→C) edge (C,D) bloklu = YOL YOK")
        print("    → D degree-1 dead-end; yan park imkansiz, AGV_2 bekler")

    # ===== Durum C: alt sira tek girisli (F-J) maliyet farki =====
    print("\n  Senaryo C: A→L vs L→A (ikisi de F-J'den gecer)")
    agv1c = g.astar("A", "L") or []
    print(f"    AGV_1 (A→L): {' → '.join(agv1c)} ({g.path_cost(agv1c):.0f} cm)")
    blocked_c = [(agv1c[i], agv1c[i+1]) for i in range(len(agv1c) - 1)]
    agv2c = g.astar("L", "A", blocked_edges=blocked_c)
    if agv2c:
        print(f"    AGV_2 alternatif: {' → '.join(agv2c)} "
              f"({g.path_cost(agv2c):.0f} cm)")
    else:
        print("    AGV_2 alternatif YOK — F-J tek koridor → yield zorunlu")


def scenario_6_short_edge_warning(g: Graph):
    """AGV uzunlugu 18cm; kisa kenarlarda 2 AGV adjacent durmamali"""
    print("\n[Senaryo 6] AGV uzunluğu 18cm — kısa kenar listesi")
    print("  Asagidaki kenarlarda 2 AGV adjacent node'larda DURAMAZ:")
    print("  (planner soft-reserve ile bunu engelleyecek)")
    cms = []
    for n in g.nodes:
        for m, c in g.neighbors(n):
            if m > n:   # her kenari bir kez
                cms.append((c, n, m))
    cms.sort()
    for c, n, m in cms:
        warn = "⚠ TEHLİKELİ" if c < 36 else "✓ güvenli"
        print(f"    {n}-{m}: {c:.0f} cm  {warn}")


# ----- Main -------------------------------------------------------------------

def main():
    print("=" * 60)
    print(" AGV Graph + A* — Senaryo Testleri (P1)")
    print("=" * 60)

    g = load()
    print(f"\n{len(g.nodes)} node, {len(g._ec)} kenar yuklendi")

    scenario_1_basic_shortest_paths(g)
    scenario_2_alternative_routes_when_blocked(g)
    scenario_3_unreachable(g)
    scenario_4_edge_cases(g)
    scenario_5_real_multiagent_situations(g)
    scenario_6_short_edge_warning(g)

    print("\n" + "=" * 60)
    print(f" SONUC: {_passed} OK / {_failed} HATA")
    print("=" * 60)
    if _failures:
        print("\nBasarisiz testler:")
        for f in _failures:
            print(" ", f)
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

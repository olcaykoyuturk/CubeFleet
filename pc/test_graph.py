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
from graph import Graph, edge_key


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


def expect_any_of(label, actual, expected_paths, g: Graph):
    """Birden cok eşit-maliyetli path olabilir; herhangi biri kabul."""
    global _passed, _failed
    if actual in expected_paths:
        cost = g.path_cost(actual)
        _passed += 1
        print(f"  ✓ {label}: {' → '.join(actual)} ({cost:.0f} cm)")
    else:
        _failed += 1
        msg = f"  ✗ {label}: got {actual}, expected one of {expected_paths}"
        _failures.append(msg)
        print(msg)


# ----- Scenarios --------------------------------------------------------------

def scenario_1_basic_shortest_paths(g: Graph):
    """En temel: A* en kisa yolu buluyor mu?"""
    print("\n[Senaryo 1] En kisa yol doğrulamaları")
    # A → I: A-B-E-H-I = 20+38+34+18 = 110 cm
    expect_path("A→I",  g.astar("A", "I"), ["A","B","E","H","I"], 110, g)
    # G → F: G-H-I-F = 20+18+36 = 74 cm
    expect_path("G→F",  g.astar("G", "F"), ["G","H","I","F"], 74, g)
    # F → A: F-C-B-A = 38+19+20 = 77 cm
    expect_path("F→A",  g.astar("F", "A"), ["F","C","B","A"], 77, g)
    # G → C: G-D-E-B-C = 33+20+38+19 = 110 cm
    expect_path("G→C",  g.astar("G", "C"), ["G","D","E","B","C"], 110, g)


def scenario_2_alternative_routes_when_blocked(g: Graph):
    """Bir kenar veya node bloklanırsa A* alternatif buluyor mu?"""
    print("\n[Senaryo 2] Alternatif rotalar (bloklu kenar/node)")

    # Edge (B,E) bloklu: A → I alternatifi A-B-C-F-I
    # = 20+19+38+36 = 113 cm
    expect_path("A→I via (B,E) bloklu",
                g.astar("A", "I", blocked_edges=[("B","E")]),
                ["A","B","C","F","I"], 113, g)

    # Edge (H,I) bloklu: G → I alternatifi G-D-E-B-C-F-I
    # = 33+20+38+19+38+36 = 184 cm
    # veya G-H-E-B-C-F-I = 20+34+38+19+38+36 = 185 cm (1 cm fark)
    # A* hangisini seçer? G-D-E-B-C-F-I daha kısa
    expect_path("G→I via (H,I) bloklu",
                g.astar("G", "I", blocked_edges=[("H","I")]),
                ["G","D","E","B","C","F","I"], 184, g)

    # Node H bloklu: G → I sadece sol+yukari uzerinden = G-D-E-B-C-F-I = 184 cm
    expect_path("G→I via H node bloklu",
                g.astar("G", "I", blocked_nodes=["H"]),
                ["G","D","E","B","C","F","I"], 184, g)

    # Multi-block: Edge (E,H) ve edge (C,F) bloklu, A → I
    # Geri kalan: A-B(-C) ve A-B-E-D-G-H-I
    # A-B-E-D-G-H-I = 20+38+20+33+20+18 = 149 cm
    # A-B-C dead-end (C-F bloklu, B-C sadece tek yön)
    expect_path("A→I (E,H) + (C,F) bloklu",
                g.astar("A", "I",
                        blocked_edges=[("E","H"), ("C","F")]),
                ["A","B","E","D","G","H","I"], 149, g)


def scenario_3_unreachable(g: Graph):
    """Ulasilamaz hedefler None dondursun"""
    print("\n[Senaryo 3] Ulasilamaz hedefler")

    # A → I but B is blocked (A's only neighbor)
    # A icin tek komsu B → engellenirse hicbir node'a gidemez
    expect_no_path("A→I via B bloklu",
                   g.astar("A", "I", blocked_nodes=["B"]))

    # G → I ama D, H, ikisi de bloklu — G'nin tum komsulari
    expect_no_path("G→I via D ve H bloklu",
                   g.astar("G", "I", blocked_nodes=["D", "H"]))

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
                ["A","B","E","H","I"], 110, g)

    # Edge yon-bagimsizligi: blocked_edge ("B","E") ile ("E","B") ayni
    p1 = g.astar("A","I", blocked_edges=[("B","E")])
    p2 = g.astar("A","I", blocked_edges=[("E","B")])
    if p1 == p2:
        global _passed
        _passed += 1
        print(f"  ✓ blocked_edge yön bağımsız: {' → '.join(p1)}")
    else:
        global _failed
        _failed += 1
        msg = f"  ✗ blocked_edge yön bağımlı: (B,E)={p1}, (E,B)={p2}"
        _failures.append(msg)
        print(msg)


def scenario_5_real_multiagent_situations(g: Graph):
    """Multi-agent gercek durumlar — manuel rezervasyon simulasyonu.
    Bu testler P2'nin (ConflictResolver) tasarımına ışık tutar."""
    print("\n[Senaryo 5] Multi-agent simulasyon (manuel rezervasyon)")

    # ===== Durum A: Head-on, AGV_1 yuksek oncelik =====
    # AGV_1: I → C (yuk var, direkt yol kullanır)
    # AGV_2: C → I (yol vermesi gerek)
    # AGV_1 plan: I-F-C (74cm) — edge (I,F) ve (F,C) rezerve, node F rezerve
    # AGV_2 plan: same direction blocked → alternatif arar
    print("  Senaryo A: head-on (I→C vs C→I), AGV_1 öncelikli")
    agv1_path = g.astar("I", "C")  # AGV_1'in tutacağı direkt yol
    print(f"    AGV_1 plani: {' → '.join(agv1_path)} ({g.path_cost(agv1_path):.0f} cm)")
    # AGV_2 perspektifi: AGV_1'in edge (I,F) ve (F,C) bloklu, node F bloklu (AGV_1 yolunda)
    agv2_path = g.astar("C", "I",
                         blocked_edges=[("I","F"), ("F","C")],
                         blocked_nodes=["F"])
    if agv2_path:
        print(f"    AGV_2 alternatif: {' → '.join(agv2_path)} "
              f"({g.path_cost(agv2_path):.0f} cm) — yol var, geç beklemeden!")
    else:
        print("    AGV_2 alternatif YOK — wait/yield gerekli")

    # ===== Durum B: Dar koridorda head-on (cycle olmayan koridor) =====
    # Yok aslinda — graf cyclic. Ama edge (A,B) zorunlu kalsa: A dead-end
    # A→C ile B→A: head-on at edge (A,B). A'ya gitmenin baska yolu YOK.
    print("\n  Senaryo B: A dead-end head-on")
    agv1_to_a = g.astar("C", "A")  # AGV_1: C → A direkt
    print(f"    AGV_1 (C→A): {' → '.join(agv1_to_a)} ({g.path_cost(agv1_to_a):.0f} cm)")
    # AGV_2 at A wants C — only way is through B
    agv2_from_a = g.astar("A", "C",
                          blocked_edges=[("A","B")])
    if agv2_from_a is None:
        print("    AGV_2 (A→C) edge (A,B) bloklu = YOL YOK")
        print("    → ConflictResolver kararı: AGV_2 A'da bekler "
              "(A dead-end olduğu icin yan park bile imkansız!)")

    # ===== Durum C: Hangi alternatif daha ucuz? =====
    # AGV_1: A → I (direkt 110 cm)
    # AGV_2: I → A
    # Eger ikisi de B-E koridorunu kullanırsa head-on
    # AGV_1 priority → AGV_2 alternatif arar
    print("\n  Senaryo C: A→I vs I→A head-on (en kısa yollar çakışıyor)")
    agv1 = g.astar("A", "I")
    print(f"    AGV_1 (A→I): {' → '.join(agv1)} ({g.path_cost(agv1):.0f} cm)")
    # AGV_1'in yolundaki kenarlari blockla, AGV_2 alternatif yolu bulsun
    # AGV_1 yolu: A-B-E-H-I; kenarlari (A,B), (B,E), (E,H), (H,I)
    # AGV_2 alternatif: I-F-C-B-A = 36+38+19+20 = 113 cm
    blocked = [(agv1[i], agv1[i+1]) for i in range(len(agv1)-1)]
    agv2 = g.astar("I", "A", blocked_edges=blocked)
    if agv2:
        print(f"    AGV_2 alternatif: {' → '.join(agv2)} "
              f"({g.path_cost(agv2):.0f} cm)")
        diff = g.path_cost(agv2) - g.path_cost(g.astar("I","A"))
        print(f"    → {diff:.0f} cm fazladan ama wait yok = anlık geçer")
    else:
        # KRITIK BULGU: A degree-1 dead-end. AGV_1 (A,B)'yi kullaniyorsa
        # AGV_2 A'ya hicbir alternatif yoldan ulasamaz. Planner: dusuk
        # oncelikli AGV mission bitene kadar bekleyecek.
        print("    AGV_2 alternatif YOK — A dead-end (degree=1)")
        print("    → Planner: dusuk oncelikli AGV C civarinda bekler "
              "(adjacency icin C-F'de degil, F-I civarinda)")


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

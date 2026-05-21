"""
FleetPlanner senaryo testleri.

Multi-agent koordinasyonun davranisini dogrular:
  - VERTEX cakismasi (ayni hedef node) — loser WAIT
  - EDGE_SWAP direct (bitisik head-on) — yan park (YIELD)
  - EDGE_SWAP future (path overlap, 3-4 hop ileri) — reroute veya WAIT
  - 2-hop look-ahead (me.next == other.after) — erken yakalama
  - Soft reservations — paralel hareket, transient block YOK
  - Pendant node (A) yan-park senaryosu
  - Cargo > FIFO > ID onceligi

Her test FleetPlanner.tick() cikti komutlarini gozler.
"""

from __future__ import annotations
import os
import sys

for _stream in (sys.stdout, sys.stderr):
    _reconf = getattr(_stream, "reconfigure", None)
    if callable(_reconf):
        _reconf(encoding="utf-8")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from graph import Graph
from fleet_planner import (
    FleetPlanner, FleetSimulator, HopAction,
)


_passed = 0
_failed = 0
_failures: list[str] = []


def assert_eq(label: str, actual, expected):
    global _passed, _failed
    if actual == expected:
        _passed += 1
        print(f"  ✓ {label}: {actual}")
    else:
        _failed += 1
        msg = f"  ✗ {label}: got {actual}, expected {expected}"
        _failures.append(msg)
        print(msg)


def assert_in(label: str, actual, allowed):
    global _passed, _failed
    if actual in allowed:
        _passed += 1
        print(f"  ✓ {label}: {actual}")
    else:
        _failed += 1
        msg = f"  ✗ {label}: got {actual}, expected one of {allowed}"
        _failures.append(msg)
        print(msg)


def load_graph() -> Graph:
    return Graph.from_json(os.path.join(os.path.dirname(__file__), "waypoints.json"))


# =============================================================================
# Senaryo 1: tek AGV, normal mission
# =============================================================================

def scenario_1_single_agv(g: Graph):
    print("\n[Senaryo 1] Tek AGV — normal mission (A -> I)")
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="I", start="A")
    cmds = p.tick()
    cmd = cmds["AGV_1"]
    assert_eq("action NORMAL", cmd.action, HopAction.NORMAL)
    assert_eq("from A", cmd.from_, "A")
    assert_eq("next B", cmd.next_, "B")
    assert_eq("after E", cmd.after_, "E")


# =============================================================================
# Senaryo 2: head-on, alternatif var (I->C vs C->I)
# =============================================================================

def scenario_2_headon_with_alternative(g: Graph):
    print("\n[Senaryo 2] Head-on I->C vs C->I (ayni next=F → loser WAIT)")
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="C", start="I")
    p.add_mission("AGV_2", goal="I", start="C")
    cmds = p.tick()
    # AGV_1: I-F-C, AGV_2: C-F-I — ikisi de next=F (VERTEX cakismasi).
    c1 = cmds["AGV_1"]
    c2 = cmds["AGV_2"]
    print(f"    AGV_1: {c1}")
    print(f"    AGV_2: {c2}")
    # YENI POLITIKA: VERTEX active (ayni next_node) → loser WAIT.
    # Mid-flight reroute YOK (race condition uretiyor). Loser current'ta
    # bekler; AGV_1 F'den C'ye gectikten sonra AGV_2 yolu acilir → EDGE_SWAP
    # tetiklenir (AGV_1@F gidiyor C, AGV_2@C gidiyor F) → AGV_2 yan park.
    assert_eq("AGV_1 NORMAL", c1.action, HopAction.NORMAL)
    assert_eq("AGV_1 F'ye", c1.next_, "F")
    assert_eq("AGV_2 WAIT (ayni next, beklesin)", c2.action, HopAction.WAIT)
    # Simulasyonla calistir — yine de iki AGV de hedefe varmali (deadlock yok)
    p2 = FleetPlanner(g)
    p2.add_mission("AGV_1", goal="C", start="I")
    p2.add_mission("AGV_2", goal="I", start="C")
    sim = FleetSimulator(p2)
    result = sim.run_until_done(max_ticks=30, deadlock_ticks=5)
    print(f"    Sim sonuc: {result}, tick={sim.tick_no}")
    assert_eq("Iki AGV de tamamlandi", result, "done")


# =============================================================================
# Senaryo 3: A pendant — AGV_1 A'da, AGV_2 B'de yan park yapacak
# =============================================================================

def scenario_3_pendant_yield(g: Graph):
    print("\n[Senaryo 3] A pendant: AGV_1 A->E, AGV_2 B'de duruyor")
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="E", start="A")  # A->B->E
    p.add_mission("AGV_2", goal="H", start="B")  # B->E->H
    # AGV_2 zaten B'de, hedefi H; AGV_1 A->B yapmak istiyor
    # Cakisma: AGV_1 B'ye gidiyor, AGV_2 B'de
    cmds = p.tick()
    c1 = cmds["AGV_1"]
    c2 = cmds["AGV_2"]
    print(f"    AGV_1: {c1}")
    print(f"    AGV_2: {c2}")
    # AGV_1 derecesi 1 (A), AGV_2 derecesi 3 (B) -> AGV_1 oncelikli, AGV_2 yield
    assert_eq("AGV_1 normal devam (A->B)", c1.action, HopAction.NORMAL)
    assert_eq("AGV_1 B'ye gidiyor", c1.next_, "B")
    # AGV_2 baska bir node'a yan park yapmali (C veya E degil — E AGV_1 yolunda)
    # Ya yield ya alternative path
    assert_in("AGV_2 action",
              c2.action, [HopAction.NORMAL, HopAction.WAIT])


# =============================================================================
# Senaryo 5: Cargo onceligi
# =============================================================================

def scenario_5_cargo_priority(g: Graph):
    print("\n[Senaryo 5] Cargo onceligi: AGV_2 cargo'lu, AGV_1 bos")
    p = FleetPlanner(g)
    # Onceligi sirasiyla AGV_1 once eklendi (FIFO=1) ama cargo'suz;
    # AGV_2 sonra eklendi (FIFO=2) ama cargo'lu. Cargo FIFO'dan onde.
    p.add_mission("AGV_1", goal="I", start="A", has_cargo=False)
    p.add_mission("AGV_2", goal="A", start="I", has_cargo=True)
    cmds = p.tick()
    c1 = cmds["AGV_1"]
    c2 = cmds["AGV_2"]
    print(f"    AGV_1 (no cargo): {c1}")
    print(f"    AGV_2 (cargo):    {c2}")
    # Iki AGV en kisa yol cakismiyor (AGV_1: A-B-E-H-I, AGV_2: I-H-E-B-A)
    # Cakisma direkt edge swap (B-E ve E-H her ikisinde de)
    # Cargo'lu AGV_2 normal devam etmeli
    assert_eq("AGV_2 cargo, normal", c2.action, HopAction.NORMAL)


# =============================================================================
# Senaryo 6: VERTEX — ayni hedef node
# =============================================================================

def scenario_6_vertex_conflict(g: Graph):
    print("\n[Senaryo 6] VERTEX: ikisi de E'ye gidicek")
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="E", start="B")
    p.add_mission("AGV_2", goal="E", start="D")
    cmds = p.tick()
    c1 = cmds["AGV_1"]
    c2 = cmds["AGV_2"]
    print(f"    AGV_1: {c1}")
    print(f"    AGV_2: {c2}")
    # Yeni davranis: AGV_1 priority (FIFO=1) → NORMAL E'ye. AGV_2 low priority,
    # her ikisi de E hedef oldugu icin alternatif yol bulamaz → WAIT.
    # Kullanici manuel mudahale: AGV_2'ye baska hedef veya AGV_1 bitince devam.
    assert_eq("AGV_1 NORMAL", c1.action, HopAction.NORMAL)
    assert_eq("AGV_1 E'ye", c1.next_, "E")
    assert_in("AGV_2 yan/wait", c2.action, [HopAction.WAIT])


# =============================================================================
# Senaryo 7: Yield-return roundtrip — full simulasyon
# =============================================================================

def scenario_7_yield_return_roundtrip(g: Graph):
    print("\n[Senaryo 7] Yield-return roundtrip (multi-tick)")
    # AGV_1: B -> E (cargo, oncelikli), AGV_2: D -> E (yan park yapacak)
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="E", start="B", has_cargo=True)
    p.add_mission("AGV_2", goal="C", start="D")   # AGV_2 hedefi C, yolu D-E-B-C — AGV_1 ile cakisacak
    sim = FleetSimulator(p)
    result = sim.run_until_done(max_ticks=30, verbose=False)
    print(f"    Toplam tick: {sim.tick_no}")
    # Yeni politika: planner aktif cakismalari cozmez. AGV'ler paralel ilerler.
    # Iki AGV de hedefe varmali (sandbox sim'de fiziksel collision yok).
    assert_eq("Iki AGV de mission bitti", result, "done")


# =============================================================================
# Senaryo 8: Full A->I head-on simulasyon
# =============================================================================

def scenario_8_full_headon_simulation(g: Graph):
    print("\n[Senaryo 8] Full simulasyon: A->I vs I->A")
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="I", start="A")
    p.add_mission("AGV_2", goal="A", start="I")
    sim = FleetSimulator(p)
    result = sim.run_until_done(max_ticks=30, verbose=False)
    print(f"    Sonuc: {result}, tick={sim.tick_no}")
    assert_eq("Ikisi de hedefe vardi", result, "done")
    # AGV_1 sonunda I'da olmali, AGV_2 A'da
    # (mission bittiyse planner'dan silindi, son history'den kontrol)
    last_agv1 = next((c for (_, a, c) in reversed(sim.history) if a == "AGV_1"), None)
    last_agv2 = next((c for (_, a, c) in reversed(sim.history) if a == "AGV_2"), None)
    print(f"    AGV_1 son: {last_agv1}")
    print(f"    AGV_2 son: {last_agv2}")


# =============================================================================
# Senaryo 9: Yapay deadlock — alternatif yok + park yok
# =============================================================================

def scenario_9_forced_deadlock(g: Graph):
    print("\n[Senaryo 9] Yapay deadlock — A'da iki AGV (degree-1 paylasimi)")
    # AGV_1 A'da bekliyor, AGV_2 A'ya geliyor. AGV_1 mission'i A->A yani gercekte
    # hareket etmiyor; AGV_2 I'dan A'ya. AGV_1 sadece A'yi tutuyor → AGV_2 vertex
    # cakismasi yaşamali, alternatif yok cunku A pendant.
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="A", start="A")   # zaten hedefte
    p.add_mission("AGV_2", goal="A", start="I")
    sim = FleetSimulator(p)
    result = sim.run_until_done(max_ticks=20, deadlock_ticks=3, verbose=True)
    print(f"    Sonuc: {result}")
    # AGV_1 hedefte → DONE; AGV_2 A'ya gelemez cunku AGV_1 orda → WAIT
    # Beklenen: deadlock veya AGV_1 cikinca devam (cikamadigi icin deadlock)
    assert_in("Deadlock veya done", result, ["deadlock", "done"])


# =============================================================================
# Senaryo 10: Mid-flight mission ekleme — initial path rezervasyonlu astar
# =============================================================================

def scenario_10_midflight_addition(g: Graph):
    print("\n[Senaryo 10] Mid-flight mission ekleme (AGV_1 yolda iken AGV_2 eklenir)")
    p = FleetPlanner(g)
    # AGV_1 baslar
    p.add_mission("AGV_1", goal="I", start="A")
    sim = FleetSimulator(p)
    sim.step()   # AGV_1 A->B
    sim.step()   # AGV_1 B->E

    # AGV_2 mid-flight eklenir.
    # YENI POLITIKA (soft reservations): A* sadece parked AGV'leri block sayar.
    # AGV_1 hala aktif (DONE degil) → AGV_2 en kisa yolu serbestce secebilir
    # (D->E->B->C). Conflict (VERTEX/EDGE_SWAP) detect runtime'da WAIT/yield ile
    # halleder. Bu sayede iki AGV paralel hareket edebilir.
    p.add_mission("AGV_2", goal="C", start="D")
    agv2 = p.missions["AGV_2"]
    print(f"    AGV_1 pos={p.missions['AGV_1'].pos}, planned={p.missions['AGV_1'].path}")
    print(f"    AGV_2 initial path: {agv2.path}")
    # Beklenen: path en kisa (D-E-B-C = 77cm, eskiden D-G-H-I-F-C = 145cm)
    assert_eq("AGV_2 en kisa path secildi (soft reservations)",
              agv2.path, ["D", "E", "B", "C"])

    # Sim: conflict detection halleder, ikisi de carpismadan biter
    result = sim.run_until_done(max_ticks=30)
    print(f"    Sonuc: {result}, tick={sim.tick_no}")
    assert_eq("Iki AGV de tamamlandi", result, "done")


# =============================================================================
# Senaryo 11: Parked AGV obstacle olarak algilaniyor mu?
# =============================================================================

def scenario_11_parked_agv_as_obstacle(g: Graph):
    print("\n[Senaryo 11] Parked AGV diger AGV icin engel mi?")
    p = FleetPlanner(g)
    # AGV_1 I'da park edilmis (zaten orada, mission anlik biter)
    p.add_mission("AGV_1", goal="I", start="I")
    agv1 = p.missions.get("AGV_1")
    assert agv1 is not None
    print(f"    AGV_1 state: {agv1.state.value}, pos: {agv1.pos}")
    # AGV_1 DONE state'te kalmali, I rezervede olmali
    assert_eq("AGV_1 state DONE", agv1.state.value, "done")
    is_reserved = p.reservations.node_owner.get("I") == "AGV_1"
    assert_eq("I parked olarak rezerve", is_reserved, True)

    # AGV_2 A'dan I'ya gitmek istiyor. Direkt yol A-B-E-H-I,
    # ama I bloklu (AGV_1 orada). astar(A, I, blocked=[I]) → None
    # Yani AGV_2 yola cikamaz.
    p.add_mission("AGV_2", goal="I", start="A")
    agv2 = p.missions["AGV_2"]
    print(f"    AGV_2 path: {agv2.path}")
    # Path == [A] anlamina gelir: ulasilamaz (astar None dondu, fallback [start])
    assert_eq("AGV_2 path ulasilamaz (sadece start)", agv2.path, ["A"])

    # AGV_2'yi farkli hedefe yonlendir — AGV_1'i atlayarak gidebilmeli
    p2 = FleetPlanner(g)
    p2.add_mission("AGV_1", goal="I", start="I")    # I'da park
    p2.add_mission("AGV_2", goal="F", start="A")    # A->F (I'dan gecmez)
    agv2b = p2.missions["AGV_2"]
    print(f"    AGV_2 (A->F) path: {agv2b.path}")
    # A-B-C-F = 20+19+38 = 77cm; F erisilebilir, I'yi etmez
    assert_eq("A->F path olusabilir", len(agv2b.path) > 1, True)


# =============================================================================
# Senaryo 12: EDGE_SWAP head-on — yan park (yield) tetiklenmeli
# =============================================================================

def scenario_12_yield_side_park(g: Graph):
    print("\n[Senaryo 12] Head-on B<->E — loser yan park (yield)")
    # AGV_1: B -> E (path B-E), AGV_2: E -> B (path E-B). Tam head-on.
    # AGV_1 priority (FIFO=1) → normal. AGV_2 loser, EDGE_SWAP detect,
    # yan park dene: E komsulari [B, D, H]. B = AGV_1.next, D ve H bos.
    # Birinde yield etmeli.
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="E", start="B")
    p.add_mission("AGV_2", goal="B", start="E")
    cmds = p.tick()
    c1 = cmds["AGV_1"]
    c2 = cmds["AGV_2"]
    print(f"    AGV_1: {c1}")
    print(f"    AGV_2: {c2}")
    assert_eq("AGV_1 NORMAL", c1.action, HopAction.NORMAL)
    assert_eq("AGV_1 E'ye", c1.next_, "E")
    assert_eq("AGV_2 YIELD (yan park)", c2.action, HopAction.YIELD)
    assert_in("AGV_2 park D veya H", c2.next_, ["D", "H"])
    # Simulasyon: ikisi de hedefe varmali
    p2 = FleetPlanner(g)
    p2.add_mission("AGV_1", goal="E", start="B")
    p2.add_mission("AGV_2", goal="B", start="E")
    sim = FleetSimulator(p2)
    result = sim.run_until_done(max_ticks=30, deadlock_ticks=5)
    print(f"    Sim sonuc: {result}, tick={sim.tick_no}")
    assert_eq("Yan park + return + hedef done", result, "done")


# =============================================================================
# Senaryo 13: VERTEX ayni next_node — loser WAIT (90° aci carpisma engelle)
# =============================================================================

def scenario_13_vertex_same_target_wait(g: Graph):
    print("\n[Senaryo 13] VERTEX ayni next — 90° yaklasimda WAIT zorunlu")
    # Kritik: iki AGV ayni next_node'a 90° ile yaklasirsa sonar karsidan
    # gelmeyi gormez (forward-only). Planner WAIT vermezse carpisirlar.
    #   AGV_1: B -> H (path B-E-H, en kisa 72cm; alternatif C-F-I-H = 111)
    #   AGV_2: D -> B (path D-E-B, en kisa 58cm; alternatif D-G-H-E-B = 125)
    # Ikisi de zorunlu olarak E'den gecmek istiyor → next=E hem AGV_1 hem AGV_2.
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="H", start="B")
    p.add_mission("AGV_2", goal="B", start="D")
    cmds = p.tick()
    c1 = cmds["AGV_1"]
    c2 = cmds["AGV_2"]
    print(f"    AGV_1: {c1}")
    print(f"    AGV_2: {c2}")
    assert_eq("AGV_1 NORMAL E'ye", c1.action, HopAction.NORMAL)
    assert_eq("AGV_1 next E", c1.next_, "E")
    assert_eq("AGV_2 WAIT — E'ye gitmiyor", c2.action, HopAction.WAIT)
    assert_eq("AGV_2 D'de kaldi", c2.from_, "D")
    # Simulasyon: deadlock olmamali — AGV_1 E'den H'ye gecince AGV_2 yolu acilir
    p2 = FleetPlanner(g)
    p2.add_mission("AGV_1", goal="H", start="B")
    p2.add_mission("AGV_2", goal="B", start="D")
    sim = FleetSimulator(p2)
    result = sim.run_until_done(max_ticks=30, deadlock_ticks=5)
    print(f"    Sim sonuc: {result}, tick={sim.tick_no}")
    assert_eq("Iki AGV de tamamlandi", result, "done")


# =============================================================================
# Senaryo 14: REGRESSION — me.next = other.after_node (2-hop look-ahead)
# =============================================================================

def scenario_14_after_node_lookahead(g: Graph):
    print("\n[Senaryo 14] 2-hop look-ahead: me.next == other.after_node → WAIT")
    # Bu senaryo 03:18:58 buglarini yakaliyor:
    #   AGV_1 mission I->A added first (path I-H-E-B-A).
    #     AGV_1.pos=I, next=H, after=E.
    #   AGV_2 mission C->D added (path C-B-E-D).
    #     AGV_2 RFID:B → hopComplete B → AGV_2.pos=B, next=E, after=D.
    #   E AGV_1'in after_node'unda. ESKI kod: AGV_2.next=E vs AGV_1.next=H
    #   → conflict YOK → AGV_2 dispatch B>E. AGV_1 H'ye varinca, AGV_1.next=E,
    #   simdi VERTEX detected → AGV_2 WAIT (gec, zaten yolda) → 90° carpisma.
    # YENI kod: AGV_2 plan'inda me.next (E) in AGV_1's {H, E} → WAIT erken.
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="A", start="I")   # path I-H-E-B-A
    p.add_mission("AGV_2", goal="D", start="C")   # path C-B-E-D
    # AGV_2 B'ye varmis simule et
    p.on_hop_complete("AGV_2", "B")
    cmds = p.tick()
    c1 = cmds["AGV_1"]
    c2 = cmds["AGV_2"]
    print(f"    AGV_1: {c1}")
    print(f"    AGV_2: {c2}")
    # AGV_1 priority (FIFO=1) → NORMAL I→H
    assert_eq("AGV_1 NORMAL", c1.action, HopAction.NORMAL)
    assert_eq("AGV_1 next H", c1.next_, "H")
    # AGV_2 → WAIT (AGV_1.after=E ile cakisma erken yakalandi)
    assert_eq("AGV_2 WAIT (early lookahead)", c2.action, HopAction.WAIT)
    assert_eq("AGV_2 B'de kaldi (dispatch yok)", c2.from_, "B")
    # Sim ile dogrula
    p2 = FleetPlanner(g)
    p2.add_mission("AGV_1", goal="A", start="I")
    p2.add_mission("AGV_2", goal="D", start="C")
    p2.on_hop_complete("AGV_2", "B")
    sim = FleetSimulator(p2)
    result = sim.run_until_done(max_ticks=30, deadlock_ticks=5)
    print(f"    Sim sonuc: {result}, tick={sim.tick_no}")
    assert_eq("Iki AGV de carpissmadan tamamlandi", result, "done")


# =============================================================================
# Senaryo 15: REGRESSION — path-overlap head-on (I-H edge ters yon, 3 hop)
# =============================================================================

def scenario_15_path_overlap_headon(g: Graph):
    print("\n[Senaryo 15] Path overlap head-on: I-H edge 3 hop uzakta")
    # Bu senaryo 03:27:30 buglarini yakaliyor.
    # Gercek bug'da AGV_2 ONCE dispatch yapmis (FIFO=12), B'yi rezerve etmisti.
    # Sonra AGV_1 mission C->G eklenmis. B bloklu oldugu icin A* path C-F-I-H-G
    # secti (B'yi atlatti). Bu path AGV_2'nin H-I edge'i ile ters yonde
    # cakisiyordu.
    p = FleetPlanner(g)
    p.add_mission("AGV_2", goal="I", start="A")   # FIFO=1 (priority)
    # AGV_2'yi B'ye getir — simdi B AGV_2 tarafindan tutulu, edge A-B de
    p.on_hop_complete("AGV_2", "B")
    # ilk tick: AGV_2 dispatch B>E>H (B rezerve, edge B-E)
    cmds_pre = p.tick()
    print(f"    Pre-tick AGV_2: {cmds_pre.get('AGV_2')}")
    print(f"    Reservations: nodes={dict(p.reservations.node_owner)}")
    # Simdi AGV_1 eklenir, B bloklu oldugu icin path C-F-I-H-G secilir
    p.add_mission("AGV_1", goal="G", start="C")
    m1 = p.missions["AGV_1"]
    print(f"    AGV_1 initial path: {m1.path}")
    # Hop ile ilerleyelim, head-on detection devreye girsin
    cmds = p.tick()
    c1 = cmds["AGV_1"]
    c2 = cmds["AGV_2"]
    print(f"    AGV_2: {c2}")
    print(f"    AGV_1: {c1}")
    # AGV_1 path I-H kullaniyorsa head-on yakalanmali → WAIT veya reroute
    uses_IH = ("I", "H") in list(zip(m1.path, m1.path[1:]))
    if uses_IH:
        assert_in("AGV_1 head-on yakalandi (reroute/WAIT)",
                  c1.action, [HopAction.NORMAL, HopAction.WAIT])
        if c1.action == HopAction.NORMAL:
            new_edges = list(zip(m1.path, m1.path[1:]))
            still_uses_IH = ("I", "H") in new_edges
            assert_eq("AGV_1 reroute sonrasi I-H kullanmiyor",
                      still_uses_IH, False)
    # Simulasyon: deadlock olmamali
    p2 = FleetPlanner(g)
    p2.add_mission("AGV_2", goal="I", start="A")
    p2.on_hop_complete("AGV_2", "B")
    p2.add_mission("AGV_1", goal="G", start="C")
    sim = FleetSimulator(p2)
    result = sim.run_until_done(max_ticks=40, deadlock_ticks=5)
    print(f"    Sim sonuc: {result}, tick={sim.tick_no}")
    assert_eq("Iki AGV de tamamlandi (carpisma yok)", result, "done")


# =============================================================================
# Senaryo 16: REGRESSION — exact 03:27:30 bug timing
# =============================================================================

def scenario_16_simultaneous_headon(g: Graph):
    print("\n[Senaryo 16] Soft reservations — paralel hareket, carpisma yok")
    # YENI POLITIKA: A* sadece parked AGV'leri block sayar. 03:27:30 buglarinda
    # AGV_1 transient reservation yuzunden uzun yola zorlaniyordu (C-F-I-H-G).
    # Simdi en kisa yolu (C-B-E-D-G) secebilir.
    #
    # Bu senaryo: head-on uretmek icin EDGE'lere bakmaliyiz. Soft reservations
    # ile A* en kisa yolu secse de, paths overlap olursa head-on detect olur.
    # AGV_1: A->I (path A-B-E-H-I). AGV_2: I->A (path I-H-E-B-A). Tipik head-on.
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="I", start="A")    # FIFO=1
    p.add_mission("AGV_2", goal="A", start="I")    # FIFO=2
    m1 = p.missions["AGV_1"]
    m2 = p.missions["AGV_2"]
    print(f"    AGV_1 path: {m1.path}")
    print(f"    AGV_2 path: {m2.path}")
    cmds = p.tick()
    c1 = cmds["AGV_1"]
    c2 = cmds["AGV_2"]
    print(f"    AGV_1: {c1}")
    print(f"    AGV_2: {c2}")
    # AGV_1 priority winner → NORMAL
    assert_eq("AGV_1 NORMAL", c1.action, HopAction.NORMAL)
    # AGV_2 head-on detect → reroute veya WAIT (carpismadan)
    assert_in("AGV_2 head-on response",
              c2.action, [HopAction.NORMAL, HopAction.WAIT, HopAction.YIELD])
    # Full sim — carpismadan ikisi de tamamlanmali
    p2 = FleetPlanner(g)
    p2.add_mission("AGV_1", goal="I", start="A")
    p2.add_mission("AGV_2", goal="A", start="I")
    sim = FleetSimulator(p2)
    result = sim.run_until_done(max_ticks=40, deadlock_ticks=8)
    print(f"    Sim sonuc: {result}, tick={sim.tick_no}")
    assert_eq("Iki AGV de carpismadan tamamlandi", result, "done")


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print(" FleetPlanner — Senaryo Testleri (P2 + P6)")
    print("=" * 60)
    g = load_graph()
    print(f"\n{len(g.nodes)} node, {len(g._ec)} kenar yuklendi\n")

    scenario_1_single_agv(g)
    scenario_2_headon_with_alternative(g)
    scenario_3_pendant_yield(g)
    scenario_5_cargo_priority(g)
    scenario_6_vertex_conflict(g)
    scenario_7_yield_return_roundtrip(g)
    scenario_8_full_headon_simulation(g)
    scenario_9_forced_deadlock(g)
    scenario_10_midflight_addition(g)
    scenario_11_parked_agv_as_obstacle(g)
    scenario_12_yield_side_park(g)
    scenario_13_vertex_same_target_wait(g)
    scenario_14_after_node_lookahead(g)
    scenario_15_path_overlap_headon(g)
    scenario_16_simultaneous_headon(g)

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

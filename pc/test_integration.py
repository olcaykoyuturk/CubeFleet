"""
FleetPlanner entegrasyon testleri.

test_fleet_planner.py: tek-tick davranisi + kucuk simulasyonlar.
Bu dosya:  uctan uca mission lifecycle, disconnect/reconnect persistence,
           drift correction, queue handling, edge cases.

Hepsi planner + FleetSimulator ile sandbox; gercek WS yok. Asagidaki
sahte _FakeFlow yardimcisi PC akisini taklit eder: hopComplete event,
status broadcast (drift), disconnect/reconnect.
"""

from __future__ import annotations

import os
import sys
from typing import List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from graph import Graph
from fleet_planner import (
    FleetPlanner, FleetSimulator, HopAction, MissionState,
)


# =============================================================================
# Test runner
# =============================================================================

_passed = 0
_failed = 0
_failures: List[str] = []


def _ok(label: str, ok: bool, info: str = "") -> None:
    global _passed, _failed
    if ok:
        _passed += 1
        print(f"  PASS {label}" + (f"  ({info})" if info else ""))
    else:
        _failed += 1
        msg = f"  FAIL {label}" + (f"  ({info})" if info else "")
        _failures.append(msg)
        print(msg)


def eq(label: str, got, want) -> None:
    _ok(label, got == want, f"got={got!r} want={want!r}")


def truthy(label: str, val, hint: str = "") -> None:
    _ok(label, bool(val), hint)


def load_graph() -> Graph:
    return Graph.from_json(os.path.join(os.path.dirname(__file__), "waypoints.json"))


# =============================================================================
# PC akis simulatoru — agv_control._tick() davranisini taklit eder.
# Gercek WS yerine: status broadcast'i dogrudan planner'a aktar.
# =============================================================================

class FlowSim:
    """Planner + simulator + bag/kop event'lerini birlestiren mini akis."""

    def __init__(self, graph: Graph):
        self.planner = FleetPlanner(graph)
        self.sim     = FleetSimulator(self.planner)
        # Gercek system: AGV WS register sonrasi connected=True olur.
        # Test: assign once connect cagrilir.
        self._connected: dict[str, bool] = {}

    def connect(self, agv_id: str, pos: str) -> None:
        self._connected[agv_id] = True
        self.planner.set_agv_connected(agv_id, True, pos)

    def disconnect(self, agv_id: str) -> None:
        self._connected[agv_id] = False
        self.planner.set_agv_connected(agv_id, False)

    def status_broadcast(self, agv_id: str, current_pos: str) -> None:
        """AGV status broadcast'i geldi gibi davran (PC's _planner_on_agv_position)."""
        if self._connected.get(agv_id):
            self.planner.set_agv_position(agv_id, current_pos)
            self.planner.last_known_pos[agv_id] = current_pos

    def step_until_idle(self, max_ticks: int = 40,
                        deadlock_ticks: int = 5) -> str:
        """Tum aktif mission'lar bitene/deadlock olana kadar tick at."""
        return self.sim.run_until_done(
            max_ticks=max_ticks, deadlock_ticks=deadlock_ticks,
        )


# =============================================================================
# Senaryolar
# =============================================================================

def scenario_basic_lifecycle(g: Graph) -> None:
    """Tek AGV, tek mission, tek hop. Mission DONE olur, pos = goal."""
    print("\n[01] Tek AGV mission lifecycle (A->I)")
    f = FlowSim(g)
    f.connect("AGV_1", "A")
    f.planner.add_mission("AGV_1", goal="I", start="A")
    res = f.step_until_idle()
    eq("Sim done", res, "done")
    m = f.planner.missions.get("AGV_1")
    truthy("Mission var", m is not None)
    if m:
        eq("Mission state DONE", m.state, MissionState.DONE)
        eq("Mission pos = goal", m.pos, "I")


def scenario_queue_chain(g: Graph) -> None:
    """Aktif AGV'ye 2. mission verilirse queued; ilki bitince otomatik aktive."""
    print("\n[02] Mission queue zinciri")
    f = FlowSim(g)
    f.connect("AGV_1", "A")
    m1 = f.planner.add_mission("AGV_1", goal="I", start="A")
    m2 = f.planner.add_mission("AGV_1", goal="A", start="I")
    eq("m1 active", m1.state, MissionState.QUEUED)  # ilk tick'te ACTIVE olur
    truthy("m2 queued list'te", m2 in f.planner.queued)
    res = f.step_until_idle()
    eq("Sim done", res, "done")
    m = f.planner.missions.get("AGV_1")
    truthy("Aktif mission var", m is not None)
    if m:
        eq("Son aktif mission goal=A", m.goal, "A")
        eq("Son aktif mission DONE", m.state, MissionState.DONE)


def scenario_disconnect_blocks_others(g: Graph) -> None:
    """AGV kopukken pos diger AGV'ler icin engel olmali — add_mission sirasinda
    A* bu blocked node'u atlatmali."""
    print("\n[03] Disconnect persistence: kopuk pos engel")
    f = FlowSim(g)
    f.connect("AGV_1", "E")
    f.status_broadcast("AGV_1", "E")
    # AGV_1 once disconnect olsun ki AGV_2'nin add_mission'i E'yi block görsün
    f.disconnect("AGV_1")

    blocked = f.planner._parked_blocked_nodes_for("AGV_2")
    truthy("E AGV_2 icin blocked", "E" in blocked, f"blocked={blocked}")

    f.connect("AGV_2", "A")
    f.planner.add_mission("AGV_2", goal="H", start="A")
    m2 = f.planner.missions["AGV_2"]
    eq("AGV_2 path E icermez", "E" in m2.path, False)


def scenario_disconnect_no_dispatch(g: Graph) -> None:
    """Kopuk AGV'ye tick'te dispatch edilmemeli."""
    print("\n[04] Disconnect: tick'te dispatch skip")
    f = FlowSim(g)
    f.connect("AGV_1", "A")
    f.planner.add_mission("AGV_1", goal="I", start="A")

    cmds_before = f.planner.tick()
    truthy("Once dispatch var", "AGV_1" in cmds_before)

    f.disconnect("AGV_1")
    cmds_after = f.planner.tick()
    eq("Disconnect sonrasi dispatch yok", "AGV_1" in cmds_after, False)

    # Reconnect: dispatch yeniden baslar
    f.connect("AGV_1", "A")
    cmds_reconnect = f.planner.tick()
    truthy("Reconnect sonrasi dispatch geri geldi",
           "AGV_1" in cmds_reconnect)


def scenario_drift_correction(g: Graph) -> None:
    """AGV path disinda bir node'da → planner pos guncelle + replan."""
    print("\n[05] Drift correction: off-path pos -> replan")
    f = FlowSim(g)
    f.connect("AGV_1", "A")
    f.planner.add_mission("AGV_1", goal="I", start="A")  # A-B-E-H-I
    # Beklenmedik: AGV C'de oldugunu rapor etti
    f.status_broadcast("AGV_1", "C")
    m = f.planner.missions["AGV_1"]
    eq("Pos guncellendi", m.pos, "C")
    truthy("C path'te (replan yapildi)", "C" in m.path)
    eq("Goal hala I", m.goal, "I")
    # Devam et — bitirebilmeli
    res = f.step_until_idle()
    eq("Sim done", res, "done")


def scenario_parallel_no_conflict(g: Graph) -> None:
    """Ayri yollarda 2 AGV — bekleme/yield olmamali."""
    print("\n[06] Paralel (cakismasiz) hareket")
    f = FlowSim(g)
    f.connect("AGV_1", "A")
    f.connect("AGV_2", "G")
    f.planner.add_mission("AGV_1", goal="C", start="A")  # A-B-C
    f.planner.add_mission("AGV_2", goal="I", start="G")  # G-H-I
    cmds = f.planner.tick()
    eq("AGV_1 NORMAL", cmds["AGV_1"].action, HopAction.NORMAL)
    eq("AGV_2 NORMAL", cmds["AGV_2"].action, HopAction.NORMAL)
    res = f.step_until_idle()
    eq("Sim done", res, "done")


def scenario_vertex_loser_wait(g: Graph) -> None:
    """Iki AGV ayni next-target (gecici) -> loser WAIT, sonra devam."""
    print("\n[07] VERTEX cakisma: loser WAIT, sonra devam")
    f = FlowSim(g)
    f.connect("AGV_1", "B")
    f.connect("AGV_2", "D")
    # Farkli goal'lar: ikisi de E'den GECECEK ama orada parklamayacak
    f.planner.add_mission("AGV_1", goal="H", start="B")  # B->E->H
    f.planner.add_mission("AGV_2", goal="G", start="D")  # D->G
    # ilk tick — dispatch cmds direkt sim.step ile
    cmds = f.sim.step()
    eq("AGV_1 (FIFO=1) NORMAL", cmds["AGV_1"].action, HopAction.NORMAL)
    # AGV_2 D-G path'i E'siz; AGV_1 onceki dispatch'inden sonra serbest
    truthy("AGV_2 dispatch verildi",
           cmds["AGV_2"].action in (HopAction.NORMAL, HopAction.WAIT))
    res = f.step_until_idle()
    eq("Sim done", res, "done")


def scenario_edgeswap_yield(g: Graph) -> None:
    """Head-on bitisik -> loser YIELD (yan park). 4×3 grafta G↔F kenari:
    G ve F degree-3 (yan park icin 3. komsu var), AGV_2 F'den yan park eder."""
    print("\n[08] EDGE_SWAP direct: YIELD (yan park)")
    f = FlowSim(g)
    f.connect("AGV_1", "G")
    f.connect("AGV_2", "F")
    f.planner.add_mission("AGV_1", goal="F", start="G")
    f.planner.add_mission("AGV_2", goal="G", start="F")
    # sim.step ile ilk tick + assertion (tick'i sim ile yapmak side-effect tutarli)
    cmds = f.sim.step()
    eq("AGV_1 NORMAL", cmds["AGV_1"].action, HopAction.NORMAL)
    eq("AGV_2 YIELD", cmds["AGV_2"].action, HopAction.YIELD)
    res = f.step_until_idle()
    eq("Sim done", res, "done")


def scenario_future_headon_reroute(g: Graph) -> None:
    """Path overlap 3-4 hop ileri -> reroute."""
    print("\n[09] EDGE_SWAP future: reroute")
    f = FlowSim(g)
    f.connect("AGV_1", "A")
    f.connect("AGV_2", "I")
    f.planner.add_mission("AGV_1", goal="I", start="A")  # A-B-E-H-I
    f.planner.add_mission("AGV_2", goal="A", start="I")  # I-H-E-B-A — head-on
    cmds = f.sim.step()
    c2 = cmds["AGV_2"]
    truthy("AGV_2 dispatch verildi",
           c2.action in (HopAction.NORMAL, HopAction.WAIT, HopAction.YIELD))
    res = f.step_until_idle()
    eq("Sim done", res, "done")


def scenario_parked_obstacle_reroute(g: Graph) -> None:
    """Parked AGV path'i bloklarsa diger AGV reroute eder."""
    print("\n[10] Parked AGV path bloklar -> reroute")
    f = FlowSim(g)
    # AGV_1 H'de parked
    f.connect("AGV_1", "H")
    f.planner.add_mission("AGV_1", goal="H", start="H")
    # AGV_2 A'dan I'ya — normal path A-B-E-H-I, H bloklu
    f.connect("AGV_2", "A")
    f.planner.add_mission("AGV_2", goal="I", start="A")
    m2 = f.planner.missions["AGV_2"]
    truthy("AGV_2 yolu ulasilamaz veya H'siz",
           m2.path == ["A"] or "H" not in m2.path)


def scenario_same_pos_goal_noop(g: Graph) -> None:
    """start==goal anlik DONE olmali, hareket yok."""
    print("\n[11] start==goal: instant DONE")
    f = FlowSim(g)
    f.connect("AGV_1", "B")
    m = f.planner.add_mission("AGV_1", goal="B", start="B")
    truthy("Mission anlik done", m.is_done)
    eq("State DONE", m.state, MissionState.DONE)


def scenario_unreachable_then_unblock(g: Graph) -> None:
    """Goal parked AGV ile bloklu -> path=[start]. Parked kaldirilirsa devam."""
    print("\n[12] Unreachable -> parked kalkar -> devam")
    f = FlowSim(g)
    f.connect("AGV_1", "I")
    f.planner.add_mission("AGV_1", goal="I", start="I")   # parked at I
    f.connect("AGV_2", "A")
    m2 = f.planner.add_mission("AGV_2", goal="I", start="A")
    eq("AGV_2 ulasilamaz (path=[A])", m2.path, ["A"])

    # AGV_1 yeni mission al (I terk eder)
    f.planner.add_mission("AGV_1", goal="A", start="I")
    # tick: AGV_1 hareket eder, I serbest kalir, AGV_2 replan
    f.sim.step()
    f.sim.step()
    m2 = f.planner.missions["AGV_2"]
    truthy("AGV_2 path uzunlugu > 1 (replanned)", len(m2.path) > 1)


def scenario_reconnect_resume(g: Graph) -> None:
    """Mission ortasinda disconnect/reconnect: mission devam etmeli."""
    print("\n[13] Reconnect: mission yarida kalmistan devam")
    f = FlowSim(g)
    f.connect("AGV_1", "A")
    f.planner.add_mission("AGV_1", goal="I", start="A")
    # 1-2 hop ilerle
    f.sim.step()
    f.sim.step()
    m = f.planner.missions["AGV_1"]
    pos_before = m.pos
    truthy("AGV_1 ilerledi", pos_before != "A")

    # Disconnect
    f.disconnect("AGV_1")
    cmds = f.planner.tick()
    eq("Disconnect: dispatch yok", "AGV_1" in cmds, False)

    # Mission hala duruyor mu?
    truthy("Mission silinmedi (frozen)", "AGV_1" in f.planner.missions)
    eq("Pos korundu", f.planner.missions["AGV_1"].pos, pos_before)

    # Reconnect — devam etmeli
    f.connect("AGV_1", pos_before)
    res = f.step_until_idle()
    eq("Sim done", res, "done")


def scenario_pos_outside_path(g: Graph) -> None:
    """AGV path disinda gozukurse planner anlik replan yapmali."""
    print("\n[14] Pos path disinda -> instant replan")
    f = FlowSim(g)
    f.connect("AGV_1", "A")
    f.planner.add_mission("AGV_1", goal="I", start="A")
    # Saka: AGV F'de oldugunu rapor etti (path: A-B-E-H-I, F yok)
    f.status_broadcast("AGV_1", "F")
    m = f.planner.missions["AGV_1"]
    eq("Pos = F", m.pos, "F")
    truthy("F yeni path'te", "F" in m.path)
    eq("Goal degismedi", m.goal, "I")


# =============================================================================
# Main
# =============================================================================

def main() -> int:
    print("=" * 60)
    print(" FleetPlanner Integration Testleri")
    print("=" * 60)
    g = load_graph()
    print(f"\n{len(g.nodes)} node, {len(g._ec)} kenar")

    scenario_basic_lifecycle(g)
    scenario_queue_chain(g)
    scenario_disconnect_blocks_others(g)
    scenario_disconnect_no_dispatch(g)
    scenario_drift_correction(g)
    scenario_parallel_no_conflict(g)
    scenario_vertex_loser_wait(g)
    scenario_edgeswap_yield(g)
    scenario_future_headon_reroute(g)
    scenario_parked_obstacle_reroute(g)
    scenario_same_pos_goal_noop(g)
    scenario_unreachable_then_unblock(g)
    scenario_reconnect_resume(g)
    scenario_pos_outside_path(g)

    print("\n" + "=" * 60)
    print(f" SONUC: {_passed} OK / {_failed} HATA")
    print("=" * 60)
    if _failures:
        for f in _failures:
            print(" ", f)
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

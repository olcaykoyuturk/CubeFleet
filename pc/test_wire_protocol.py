"""
Wire-protokolu (PC -> firmware) golden testi — COMPAT-009.

NEDEN: 102 mevcut testin tamami FleetSimulator sandbox'i ile PLANNER mantigini
test eder; HICBIRI gercek WS mesaj govdesini (setHop JSON alan adlari) uretmez.
Oysa firmware tam olarak `from`/`next`/`after`/`goal` alanlarini parse eder
([firmware/agv/websocket.ino]); PC tarafinda biri bu adlardan birini degistirirse
(or. `next` -> `to`) AGV emri SESSIZCE yok sayar. Bu test PC'nin urettigi govdeyi
firmware sozlesmesine kilitler.

Kontroller:
  1. set_hop golden govde: tam alan adlari {type, agvId, from, next, after, goal}
  2. after/goal None ise govdeden CIKARILIR (firmware opsiyonel okur)
  3. 'to' ve 'action' alanlari ASLA gonderilmez (yanlis sozlesme regresyonu)
  4. set_position / clear_mission govdeleri
  5. uctan-uca: FleetPlanner HopCommand.from_/next_/after_ -> setHop govdesi

Calistir: python pc/test_wire_protocol.py   (run_tests.py otomatik calistirir)
"""

from __future__ import annotations
import os
import sys

for _stream in (sys.stdout, sys.stderr):
    _reconf = getattr(_stream, "reconfigure", None)
    if callable(_reconf):
        _reconf(encoding="utf-8")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from agv_ws_client import AGVClient
from graph import Graph
from fleet_planner import FleetPlanner, HopAction

_passed = 0
_failed = 0
_failures: list[str] = []


def assert_eq(label: str, actual, expected):
    global _passed, _failed
    if actual == expected:
        _passed += 1
        print(f"  ✓ {label}")
    else:
        _failed += 1
        msg = f"  ✗ {label}: got {actual!r}, expected {expected!r}"
        _failures.append(msg)
        print(msg)


def assert_true(label: str, cond: bool, detail: str = ""):
    global _passed, _failed
    if cond:
        _passed += 1
        print(f"  ✓ {label}")
    else:
        _failed += 1
        msg = f"  ✗ {label}" + (f": {detail}" if detail else "")
        _failures.append(msg)
        print(msg)


def make_capturing_client() -> tuple:
    """Socket acmadan AGVClient kur; send_raw'i yakalayiciyla degistir."""
    client = AGVClient("ws://test.invalid:1/ws")
    captured: list = []
    client.send_raw = lambda payload: (captured.append(payload), True)[1]  # type: ignore
    return client, captured


def test_set_hop_golden():
    print("\n[1] set_hop golden govde")
    client, cap = make_capturing_client()
    client.set_hop("AGV_1", "A", "B", "E", goal="I")
    p = cap[-1]
    assert_eq("tam govde",
              p,
              {"type": "setHop", "agvId": "AGV_1",
               "from": "A", "next": "B", "after": "E", "goal": "I"})
    # Firmware'in parse ettigi alan adlari (websocket.ino doc["from"]/["next"]/...)
    assert_eq("alan adlari kume",
              set(p.keys()),
              {"type", "agvId", "from", "next", "after", "goal"})
    assert_true("'to' alani YOK (yanlis sozlesme)", "to" not in p)
    assert_true("'action' alani YOK (yanlis sozlesme)", "action" not in p)


def test_set_hop_optional_fields():
    print("\n[2] set_hop opsiyonel alanlar (after/goal None -> cikarilir)")
    client, cap = make_capturing_client()
    client.set_hop("AGV_2", "H", "I")           # after=None, goal=None
    p = cap[-1]
    assert_eq("sadece zorunlu alanlar",
              p, {"type": "setHop", "agvId": "AGV_2", "from": "H", "next": "I"})
    assert_true("after None -> yok", "after" not in p)
    assert_true("goal None -> yok",  "goal"  not in p)
    # Sadece goal verili
    client.set_hop("AGV_2", "H", "I", goal="F")
    p2 = cap[-1]
    assert_true("goal var, after yok", "goal" in p2 and "after" not in p2)


def test_other_messages():
    print("\n[3] set_position / clear_mission govdeleri")
    client, cap = make_capturing_client()
    client.set_position("AGV_1", "C")
    assert_eq("setPosition govde",
              cap[-1], {"type": "setPosition", "agvId": "AGV_1", "waypoint": "C"})
    client.clear_mission("AGV_1")
    assert_eq("clearMission govde",
              cap[-1], {"type": "clearMission", "agvId": "AGV_1"})


def test_face_dir():
    print("\n[5] faceDir govdesi (firmware doc[\"dir\"] parse'ina kilitli)")
    client, cap = make_capturing_client()
    ok = client.face_dir("AGV_1", "E")
    assert_true("gonderim True", ok)
    assert_eq("faceDir govde",
              cap[-1], {"type": "faceDir", "agvId": "AGV_1", "dir": "E"})
    # Normalizasyon: kucuk harf + bosluk kabul, tek harfe indirgenir
    client.face_dir("AGV_1", " w ")
    assert_eq("kucuk harf normalize", cap[-1]["dir"], "W")
    # Gecersiz yon gonderilmez
    n_before = len(cap)
    assert_true("gecersiz yon reddedilir", not client.face_dir("AGV_1", "X"))
    assert_eq("gecersiz yon govde uretmez", len(cap), n_before)
    for d in ("N", "E", "S", "W"):
        client.face_dir("AGV_2", d)
        assert_eq(f"dir={d} aynen gider", cap[-1]["dir"], d)


def test_face_complete_rx():
    print("\n[6] faceComplete alimi (AGV -> PC event)")
    client, _ = make_capturing_client()
    client._on_message(None, '{"type":"faceComplete","agvId":"AGV_1",'
                             '"node":"F","heading":"DOGU","time":1234}')
    evs = client.pop_events()
    face = [(k, p) for k, p in evs if k == "face_complete"]
    assert_eq("face_complete event sayisi", len(face), 1)
    if face:
        ev = face[0][1]
        assert_eq("agvId", ev.agvId, "AGV_1")
        assert_eq("node", ev.node, "F")
        assert_eq("heading", ev.heading, "DOGU")
        assert_eq("time_ms", ev.time_ms, 1234)


def test_end_to_end_hopcommand():
    print("\n[4] uctan-uca: FleetPlanner HopCommand -> setHop govde")
    g = Graph.from_json(os.path.join(os.path.dirname(__file__), "waypoints.json"))
    p = FleetPlanner(g)
    p.add_mission("AGV_1", goal="I", start="A")
    cmds = p.tick()
    c = cmds["AGV_1"]
    assert_eq("planner NORMAL hop", c.action, HopAction.NORMAL)
    # agv_control._planner_dispatch_hop'un yaptigini birebir yansit:
    client, cap = make_capturing_client()
    client.set_hop(c.agv_id, c.from_, c.next_, c.after_, goal="I")
    p_msg = cap[-1]
    assert_eq("HopCommand alanlari setHop'a dogru maplendi",
              (p_msg["from"], p_msg["next"], p_msg["after"]),
              (c.from_, c.next_, c.after_))
    assert_eq("from A", p_msg["from"], "A")
    assert_eq("next B", p_msg["next"], "B")
    assert_eq("after E", p_msg["after"], "E")


def main() -> int:
    print("=" * 60)
    print(" Wire-Protokolu (PC -> firmware) Golden Testi")
    print("=" * 60)
    test_set_hop_golden()
    test_set_hop_optional_fields()
    test_other_messages()
    test_end_to_end_hopcommand()
    test_face_dir()
    test_face_complete_rx()
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

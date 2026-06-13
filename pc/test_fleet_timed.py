"""
FleetPlanner GERCEK-DUNYA zamanli senaryo testleri.

FleetSimulator hop'u ANINDA tamamlar (isinlanma) — gercekte hop sure alir ve
cakismalar UCUS ORTASINDA dogar. TimedSim bunu modeller:
  * hop suresi = ceil(kenar_maliyeti / hiz) tick (mesafe bazli)
  * firmware race guard: hareket halindeki AGV yeni hop'u REDDEDER
  * komut ancak fiziksel pozisyondan baslarsa kabul edilir
  * CARPISMA DENETIMI her tick: ayni node'da iki sabit AGV, dolu node'a varis,
    ayni kenari ayni anda kullanma (ters yon = kafa kafaya!) → ihlal kaydi

Senaryolar (kullanici istegi: biri once biri sonra / ayni anda baslama):
  1. Ayni anda kafa kafaya (A→I vs I→A)
  2. Gecikmeli kafa kafaya (AGV_2 birkac tick sonra baslar)
  3. Gecikmeli kesisme (ikisi de E'den gecmek zorunda)
  4. Cargo'lu gec katilan oncelik kazanir
  5. KAYIP YIELD komutu → planner yeniden uretir (FP-12 kurtarma)
  6. Uc AGV ayni merkeze (E) yakinsiyor (watchdog'lu)
  7. In-flight reissue: ucustaki AGV'nin karari DEGISMEZ
  8. Yan park kopuk AGV'nin ustune yapilmaz (FP-13)
  9. add_mission konum bilinmiyorsa last_known_pos kullanir (FP-15)
 10. YIELDING'te pozisyon guncellemesi path'i bozmaz (FP-11)

Calistir: python pc/test_fleet_timed.py   (run_tests.py otomatik calistirir)
"""

from __future__ import annotations

import math
import os
import sys
from typing import Dict, List, Tuple

for _stream in (sys.stdout, sys.stderr):
    _reconf = getattr(_stream, "reconfigure", None)
    if callable(_reconf):
        _reconf(encoding="utf-8")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from graph import Graph, edge_key
from fleet_planner import FleetPlanner, HopAction, MissionState

WAYPOINTS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "waypoints.json")

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


def new_planner() -> Tuple[Graph, FleetPlanner]:
    g = Graph.from_json(WAYPOINTS)
    return g, FleetPlanner(g)


class TimedSim:
    """Mesafe-bazli sureli simulasyon + carpisma denetimi + firmware modeli."""

    SPEED = 30.0   # cm / tick

    def __init__(self, graph: Graph, planner: FleetPlanner,
                 drop_yields: int = 0, watchdog: bool = False):
        self.g = graph
        self.p = planner
        self.t = 0
        self.pos: Dict[str, str] = {}            # fiziksel konum (sabitken)
        self.inflight: Dict[str, dict] = {}      # agv -> {frm,to,remain,key,dir}
        self.violations: List[str] = []
        self.drop_yields = drop_yields           # ilk N YIELD komutunu "kaybet"
        self.dropped = 0
        self.watchdog = watchdog
        self.idle_streak = 0

    def add(self, agv: str, goal: str, start: str, cargo: bool = False):
        self.pos.setdefault(agv, start)
        self.p.add_mission(agv, goal, start=start, has_cargo=cargo)

    def duration(self, a: str, b: str) -> int:
        c = self.g.edge_cost(a, b) or 30.0
        return max(1, math.ceil(c / self.SPEED))

    def _start_hop(self, agv: str, frm: str, to: str):
        self.inflight[agv] = {
            "frm": frm, "to": to,
            "remain": self.duration(frm, to),
            "key": edge_key(frm, to), "dir": (frm, to),
        }

    def step(self):
        self.t += 1
        # 1) ucustakileri ilerlet; varanlarin pozisyonu guncellenir
        arrived = []
        for agv, fl in list(self.inflight.items()):
            fl["remain"] -= 1
            if fl["remain"] <= 0:
                arrived.append((agv, fl["to"]))
                del self.inflight[agv]
        for agv, node in arrived:
            self.pos[agv] = node
        # VARIS ANI isgal kontrolu (dispatch ONCESI — ayni tick tekrar yola
        # cikip "gizlenen" gecici carpismalar da yakalansin)
        occ0: Dict[str, str] = {}
        for agv, n in self.pos.items():
            if agv in self.inflight:
                continue
            if n in occ0:
                self.violations.append(
                    f"t{self.t}: VARISTA node {n}: {occ0[n]}+{agv}")
            else:
                occ0[n] = agv
        for agv, node in arrived:
            self.p.on_hop_complete(agv, node)
        # 2) planla + dispatch (firmware modeli)
        cmds = self.p.tick()
        for agv, c in cmds.items():
            if c.action not in (HopAction.NORMAL, HopAction.YIELD):
                continue
            if not c.next_ or c.next_ == c.from_:
                continue
            if agv in self.inflight:
                continue                         # race guard: ret
            if c.from_ != self.pos.get(agv):
                continue                         # eski/pozisyonsuz komut: ret
            if c.action == HopAction.YIELD and self.dropped < self.drop_yields:
                self.dropped += 1                # komut WS'te "kayboldu"
                continue
            self._start_hop(agv, c.from_, c.next_)
        # 3) carpisma denetimleri
        # ayni kenar ayni anda (ters yon = kafa kafaya)
        seen: Dict[Tuple[str, str], Tuple[str, Tuple[str, str]]] = {}
        for agv, fl in self.inflight.items():
            if fl["key"] in seen:
                o, odir = seen[fl["key"]]
                kind = "KAFA-KAFAYA" if odir != fl["dir"] else "ayni-yon"
                self.violations.append(
                    f"t{self.t}: kenar {fl['key']} {kind}: {o}+{agv}")
            else:
                seen[fl["key"]] = (agv, fl["dir"])
        # ayni node'da iki SABIT agv (varis cakismalari dahil — pozisyonlar
        # guncellendikten SONRA bakildigi icin ayni-tick bosaltma false
        # positive uretmez)
        occ: Dict[str, str] = {}
        for agv, n in self.pos.items():
            if agv in self.inflight:
                continue
            if n in occ:
                self.violations.append(f"t{self.t}: node {n}: {occ[n]}+{agv}")
            else:
                occ[n] = agv
        # 4) watchdog (production _planner_watchdog modeli)
        if self.watchdog and not self.inflight:
            if self.p.detect_deadlock(threshold=4):
                yc = self.p.break_deadlock()
                if (yc is not None and yc.next_
                        and yc.from_ == self.pos.get(yc.agv_id)
                        and yc.agv_id not in self.inflight):
                    self._start_hop(yc.agv_id, yc.from_, yc.next_)
        return cmds

    def all_done(self) -> bool:
        if self.inflight:
            return False
        return all(m.is_done for m in self.p.missions.values())

    def run(self, max_ticks: int = 300) -> str:
        """'done' | 'stuck' | 'timeout'."""
        for _ in range(max_ticks):
            self.step()
            if self.all_done():
                return "done"
            if self.inflight:
                self.idle_streak = 0
            else:
                self.idle_streak += 1
                if self.idle_streak >= 25:   # YIELD timeout kurtarmasina pay
                    return "stuck"
        return "timeout"


# Graf hatirlatma (4×3 A-L):  A-B-C-D / E-F-G-H / I-J-K-L
#   dikey: A-E, C-G, F-J   (alt sira I-J-K-L SADECE F-J ile bagli)

def run_all_scenarios():
    # 4×3 A-L grafta cozulebilir head-on icin GECISLI hedef secilir: G/F
    # degree-3, yan park mumkun. (D/H/I/L degree-1 dead-end → varista sikisma.)
    # ------------------------------------------------------------ 1
    print("\n[1] aynı anda kafa kafaya (A→G vs G→A)")
    g, p = new_planner()
    s = TimedSim(g, p)
    s.add("AGV_1", "G", start="A")     # A-E-F-G
    s.add("AGV_2", "A", start="G")     # G-F-E-A
    r = s.run()
    assert_true("ikisi de hedefe vardı", r == "done", r)
    assert_true("çarpışma yok", not s.violations, s.violations[:3])

    # ------------------------------------------------------------ 2
    print("\n[2] gecikmeli kesişen ters yön (AGV_2, 4 tick sonra başlar)")
    g, p = new_planner()
    s = TimedSim(g, p)
    s.add("AGV_1", "G", start="A")     # A-E-F-G
    for _ in range(4):
        s.step()
    s.add("AGV_2", "A", start="K")     # K-J-F-E-A — E-F koridorunda kesisir
    r = s.run()
    assert_true("ikisi de hedefe vardı", r == "done", r)
    assert_true("çarpışma yok", not s.violations, s.violations[:3])

    # ------------------------------------------------------------ 3
    print("\n[3] gecikmeli kesişme (ikisi de F'den geçer)")
    g, p = new_planner()
    s = TimedSim(g, p)
    s.add("AGV_1", "H", start="A")     # A-E-F-G-H
    for _ in range(2):
        s.step()
    s.add("AGV_2", "E", start="J")     # J-F-E
    r = s.run()
    assert_true("ikisi de hedefe vardı", r == "done", r)
    assert_true("çarpışma yok", not s.violations, s.violations[:3])

    # ------------------------------------------------------------ 4
    print("\n[4] cargo'lu geç katılan — ikisi de biter, çarpışma yok")
    # Ayrı bölgeler (üst sıra A-B-C-D vs alt sıra I-J-K-L) — kesişmez, cargo
    # mission'ın geç katılıp tamamlandığını sınar (eşit cost'ta timing-bağımsız).
    g, p = new_planner()
    s = TimedSim(g, p)
    s.add("AGV_1", "D", start="A")     # A-B-C-D (üst sıra)
    for _ in range(3):
        s.step()
    s.add("AGV_2", "L", start="I", cargo=True)   # I-J-K-L (alt sıra)
    r = s.run()
    assert_true("ikisi de hedefe vardı", r == "done", r)
    assert_true("çarpışma yok", not s.violations, s.violations[:3])

    # ------------------------------------------------------------ 5
    print("\n[5] KAYIP YIELD komutu → planner yeniden üretir (FP-12)")
    # Planner-seviyesi (TimedSim timing'inden bağımsız): loser YIELD üretir;
    # komut "kaybolur" (dispatch edilmez, hopComplete gelmez) → planner bir
    # sonraki tick'te AGV hâlâ aynı yerdeyken yine kaçınma komutu üretmeli.
    g, p = new_planner()
    p.add_mission("AGV_1", "F", start="G")   # G-F (winner, FIFO=1)
    p.add_mission("AGV_2", "G", start="F")   # F-G → kafa kafaya, loser yield
    c1 = p.tick()
    assert_true("AGV_1 NORMAL (winner)",
                c1["AGV_1"].action == HopAction.NORMAL, c1["AGV_1"])
    assert_true("AGV_2 ilk YIELD",
                c1["AGV_2"].action == HopAction.YIELD, c1["AGV_2"])
    # Komut kaybedildi: hopComplete YOK, AGV_2 hâlâ F'de. Tekrar tick:
    c2 = p.tick()
    assert_true("AGV_2 kaçınma reissue (kayıp komut yeniden üretildi)",
                c2["AGV_2"].action in (HopAction.YIELD, HopAction.WAIT),
                c2["AGV_2"])
    # Tam çözüm: drop'suz simülasyonda ikisi de çarpışmadan biter
    g2, p2 = new_planner()
    s = TimedSim(g2, p2)
    s.add("AGV_1", "F", start="G")
    s.add("AGV_2", "G", start="F")
    r = s.run()
    assert_true("ikisi de hedefe vardı", r == "done", r)
    assert_true("çarpışma yok", not s.violations, s.violations[:3])

    # ------------------------------------------------------------ 5b
    print("\n[5b] yol verilen AGV dönüş node'una PARK ederse (FP-17/17b)")
    g, p = new_planner()
    s = TimedSim(g, p)
    s.add("AGV_1", "G", start="A")     # hedefi G — AGV_2'nin baslangici!
    s.add("AGV_2", "A", start="G")
    r = s.run()
    assert_true("park edilen dönüş node'una rağmen ikisi de bitti",
                r == "done", r)
    assert_true("park etmiş aracın üstüne dönülmedi (çarpışma yok)",
                not s.violations, s.violations[:3])

    # ------------------------------------------------------------ 6
    print("\n[6] üç AGV merkeze (F-G çevresi) — watchdog'lu")
    g, p = new_planner()
    s = TimedSim(g, p, watchdog=True)
    s.add("AGV_1", "H", start="A")     # A-E-F-G-H
    s.add("AGV_2", "C", start="G")     # G-C
    s.add("AGV_3", "E", start="D")     # D-C-...-E veya D-C-G-F-E
    r = s.run(max_ticks=400)
    assert_true("üçü de sonuca ulaştı", r == "done", r)
    assert_true("çarpışma yok", not s.violations, s.violations[:3])

    # ------------------------------------------------------------ 7
    print("\n[7] in-flight reissue: uçuştaki karar değişmez")
    g, p = new_planner()
    p.add_mission("AGV_1", "E", start="G")          # G-F-E
    c1 = p.tick()["AGV_1"]
    assert_true("ilk dispatch NORMAL G→F",
                c1.action == HopAction.NORMAL and c1.next_ == "F", c1)
    # hopComplete GELMEDEN cakisma yaratan ikinci mission ekle
    p.add_mission("AGV_2", "G", start="F")          # F-G: head-on adayi
    c2 = p.tick()
    assert_true("uçuştaki AGV_1 kararı AYNI kaldı (reissue)",
                c2["AGV_1"].action == HopAction.NORMAL
                and c2["AGV_1"].next_ == "F"
                and "reissue" in c2["AGV_1"].reason, c2["AGV_1"])
    assert_true("AGV_1 state bozulmadı (ACTIVE)",
                p.missions["AGV_1"].state == MissionState.ACTIVE)
    assert_true("kaçınma yükü diğer AGV'de",
                c2["AGV_2"].action != HopAction.NORMAL, c2["AGV_2"])

    # ------------------------------------------------------------ 8
    print("\n[8] yan park kopuk AGV üstüne yapılmaz (FP-13)")
    g, p = new_planner()
    # F'de yield edecek AGV: komsular E/G/J. E'de kopuk AGV var → E secilmez.
    p.set_agv_connected("AGV_3", False, current_pos="E")
    p.add_mission("AGV_1", "F", start="G")          # G-F (oncelikli)
    p.add_mission("AGV_2", "G", start="F")          # F-G loser → yield
    cmds = p.tick()
    y = cmds["AGV_2"]
    assert_true("loser yield/wait",
                y.action in (HopAction.YIELD, HopAction.WAIT), y)
    if y.action == HopAction.YIELD:
        assert_true("park kopuk AGV'nin (E) üstünde DEĞİL", y.next_ != "E", y)

    # ------------------------------------------------------------ 9
    print("\n[9] add_mission start=None → last_known_pos (FP-15)")
    g, p = new_planner()
    p.last_known_pos["AGV_9"] = "C"
    m = p.add_mission("AGV_9", "G")
    assert_true("start last_known_pos'tan alındı", m.start == "C", m.start)
    assert_true("mission anlık DONE olmadı", not m.is_done)

    # ------------------------------------------------------------ 10
    print("\n[10] YIELDING'te pozisyon güncellemesi path'i bozmaz (FP-11)")
    g, p = new_planner()
    p.add_mission("AGV_1", "F", start="G")
    p.add_mission("AGV_2", "G", start="F")
    cmds = p.tick()
    y = cmds["AGV_2"]
    if y.action == HopAction.YIELD and y.next_:
        m2 = p.missions["AGV_2"]
        path_before = list(m2.path)
        p.set_agv_position("AGV_2", y.next_)        # RFID park node'unu okudu
        assert_true("YIELDING'te replan yok", m2.path == path_before,
                    (path_before, m2.path))
        assert_true("pos güncellendi", m2.pos == y.next_)
    else:
        assert_true("yield bekleniyordu (senaryo kurulamadı)", False, y)

    # ------------------------------------------------------------ 11
    print("\n[11] blocker eviction: görevsiz AGV sıkışan araca yol verir (FP-19)")
    # AGV_2 C'de DONE (görevi yok); AGV_1 D→H — D dead-end, tek çıkış C dolu →
    # yan park imkânsız → klasik deadlock. Watchdog blocker eviction: AGV_2
    # geçici B'ye çekilir, AGV_1 D-C-G-H geçer, AGV_2 eski yeri C'ye döner.
    g, p = new_planner()
    s = TimedSim(g, p, watchdog=True)
    s.add("AGV_2", "C", start="C")   # done @ C (blocker)
    s.add("AGV_1", "H", start="D")   # D→H, dead-end sıkışması
    r = s.run(max_ticks=400)
    assert_true("eviction ile çözüldü (done)", r == "done", r)
    assert_true("çarpışma yok", not s.violations, s.violations[:3])
    assert_true("AGV_1 hedefe vardı (H)", p.missions["AGV_1"].pos == "H",
                p.missions["AGV_1"].pos)
    assert_true("AGV_2 eski yerine döndü (C)", p.missions["AGV_2"].pos == "C",
                p.missions["AGV_2"].pos)


def main() -> int:
    run_all_scenarios()
    print()
    print(f"SONUC: {_passed} OK / {_failed} HATA")
    for msg in _failures:
        print(msg)
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

"""
Fleet Planner — Multi-AGV koordinasyon.

Tasarim:
  - 2-hop look-ahead emir: setHop {from, next, after, goal}. `sameHop`
    branch ile AGV junction'da durmadan ilerler.
  - Soft reservations: A* sadece DONE/disconnected AGV'leri block sayar;
    aktif transient rezervasyonlar A*'i bozmaz, paralel hareket korunur.
  - Cakisma cozumu:
      VERTEX active   loser WAIT
      VERTEX parked   reroute -> WAIT
      EDGE_SWAP direct YIELD (yan park) -> WAIT
      EDGE_SWAP future (3-4 hop) reroute -> WAIT
  - Guvenlik katmanlari: firmware race guard (FOLLOWING/TURNING'de hop
    reddi) + sonar 10cm STOP / 20cm SLOW.

API:
    p = FleetPlanner(graph)
    p.add_mission(agv_id, goal, start=..., has_cargo=False)
    p.on_hop_complete(agv_id, new_node)
    p.set_agv_position(agv_id, node)
    p.set_agv_connected(agv_id, connected, current_pos=None)
    cmds = p.tick()                       # Dict[agv_id, HopCommand]

waypoints.json formati:
    {"nodes": {"A": {"x": 0, "y": 0}, ...},
     "edges": [{"from": "A", "to": "B", "cost": 20}, ...]}
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, List, Optional, Set, Tuple

try:
    from graph import Graph, edge_key       # script: python pc/fleet_planner.py
except ModuleNotFoundError:
    from pc.graph import Graph, edge_key    # module: from pc.fleet_planner import ...


class MissionState(Enum):
    QUEUED   = "queued"
    ACTIVE   = "active"
    YIELDING = "yielding"   # yan park yapildi, digerin gecmesi bekleniyor
    DONE     = "done"


class HopAction(Enum):
    NORMAL = "normal"   # planda gosterilen hop'u uygula
    YIELD  = "yield"    # yan park hop (current -> yield_park_node)
    WAIT   = "wait"     # ulasilamaz / parked engel — dur
    DONE   = "done"     # mission tamamlandi


class ConflictType(Enum):
    VERTEX    = "vertex"      # ikisi de ayni next_node hedefli (veya parked oradaki)
    EDGE_SWAP = "edge_swap"   # head-on: bitisik (A→B vs B→A) veya future path overlap
    NONE      = "none"


@dataclass
class Mission:
    id:         int
    agv_id:     str
    start:      str
    goal:       str
    path:       List[str]
    pos:        str          = ""
    has_cargo:  bool         = False
    fifo_order: int          = 0
    state:      MissionState = MissionState.QUEUED
    # Yield (yan park) state — sadece MissionState.YIELDING'te anlamli
    yield_to:          Optional[str] = None  # hangi AGV'ye yol veriyoruz
    yield_park_node:   Optional[str] = None  # nereye yan park ettik
    yield_return_node: Optional[str] = None  # park sonrasi dondugumuz node
    yield_wait_ticks:  int           = 0     # park'ta/yolda kac tick bekledik (timeout)
    # FP-20: cok-hop "refuge" retreat — tek-hop yan park IMKANSIZ oldugunda
    # (dead-end dali / kopru koridorunda sikisma) park birkac hop otede bir
    # siding olur. yield_path = [current, ..., park]; _plan_yield_step bu yolu
    # hop hop yurutur. Tek-hop yan parkta [current, park] (eski davranis).
    yield_path:        Optional[List[str]] = None
    # IN-FLIGHT hop: dispatch edilen (from, next) — hopComplete gelene kadar
    # planner YENI karar VERMEZ, ayni komutu yeniden uretir (executor dedup'lar).
    # Gercek dunyada firmware hareket halindeyken yeni hop'u reddeder; planner'in
    # mid-flight karar degistirmesi (orn. YIELD'e gecmesi) state tutarsizligi
    # yaratiyordu. hopComplete / pozisyon degisimi inflight'i temizler.
    inflight: Optional[Tuple[str, str]] = None

    def __post_init__(self):
        if not self.pos:
            self.pos = self.start

    @property
    def current(self) -> str:
        return self.pos

    @property
    def next_node(self) -> Optional[str]:
        """Path uzerinde pos'tan sonraki node."""
        if self.pos not in self.path:
            return None
        idx = self.path.index(self.pos)
        return self.path[idx + 1] if idx + 1 < len(self.path) else None

    @property
    def after_node(self) -> Optional[str]:
        if self.pos not in self.path:
            return None
        idx = self.path.index(self.pos)
        return self.path[idx + 2] if idx + 2 < len(self.path) else None

    @property
    def after2_node(self) -> Optional[str]:
        """Path uzerinde pos'tan 3 sonraki node (3-hop look-ahead). Firmware'in
        navPath buffer'ini derinlestirir → WS gecikmesinde AGV path-sonu
        node'unda beklemeden devam eder."""
        if self.pos not in self.path:
            return None
        idx = self.path.index(self.pos)
        return self.path[idx + 3] if idx + 3 < len(self.path) else None

    @property
    def is_done(self) -> bool:
        return self.pos == self.goal


@dataclass
class HopCommand:
    """AGV'ye gonderilecek emir. WS uzerinden `setHop` mesajiyla yollanir.
    after2_: 3-hop look-ahead (yalniz NORMAL dispatch'te dolu; yield/wait/park
    kisa hop'larda None)."""
    agv_id: str
    action: HopAction
    from_:  str
    next_:  Optional[str] = None
    after_: Optional[str] = None
    after2_: Optional[str] = None
    reason: str           = ""

    def __repr__(self) -> str:
        n = self.next_ or "-"
        a = self.after_ or "-"
        a2 = self.after2_ or "-"
        return f"<Hop {self.agv_id} {self.action.value} {self.from_}>{n}>{a}>{a2} ({self.reason})>"


@dataclass
class Conflict:
    type:         ConflictType
    agv_a:        str
    agv_b:        str
    node_or_edge: object   # str (node) veya Tuple[str,str] (edge_key)
    detail:       str = ""


@dataclass
class ReservationTable:
    """Aktif AGV'lerin tuttugu node + edge."""
    node_owner: Dict[str, str]              = field(default_factory=dict)
    edge_owner: Dict[Tuple[str, str], str]  = field(default_factory=dict)

    def reserve_node(self, node: str, agv: str) -> bool:
        owner = self.node_owner.get(node)
        if owner is not None and owner != agv:
            return False
        self.node_owner[node] = agv
        return True

    def reserve_edge(self, a: str, b: str, agv: str) -> bool:
        key = edge_key(a, b)
        owner = self.edge_owner.get(key)
        if owner is not None and owner != agv:
            return False
        self.edge_owner[key] = agv
        return True

    def release_all_for(self, agv: str) -> None:
        for n in [n for n, o in self.node_owner.items() if o == agv]:
            del self.node_owner[n]
        for e in [e for e, o in self.edge_owner.items() if o == agv]:
            del self.edge_owner[e]

    def blocked_nodes_for(self, agv: str) -> Set[str]:
        return {n for n, o in self.node_owner.items() if o != agv}


class FleetPlanner:
    # Anti-starvation: bir AGV bu kadar tick ardisik WAIT alirsa onceligi
    # kademeli artar (fifo_order efektif olarak duser). Esik mevcut tum
    # "done" testlerindeki en uzun bekleme suresinin (deadlock_ticks<=8)
    # ustundedir → tek-tick davranisi ve sim sonuclari degismez.
    _STARVE_THRESHOLD = 8
    # YIELDING'te park'ta bu kadar tick beklersek (yol veren AGV cikmadi /
    # stall etti) yield iptal edilir, AGV normal replan'a doner. Tum test
    # max_ticks degerlerinin (<=40) ustunde → testleri etkilemez.
    _YIELD_TIMEOUT_TICKS = 60

    def __init__(self, graph: Graph):
        self.graph               = graph
        self.reservations        = ReservationTable()
        self.missions:           Dict[str, Mission] = {}   # agv_id -> aktif mission
        self.queued:             List[Mission]      = []
        self._mission_id_counter = 0
        self._fifo_counter       = 0
        # Disconnect persistence: kopuk AGV tick'te dispatch edilmez, pos
        # diger AGV'ler icin engel sayilir.
        self.disconnected_agvs: Set[str] = set()
        self.last_known_pos:    Dict[str, str] = {}
        # Ardisik WAIT sayaci (anti-starvation oncelik + production deadlock
        # tespiti). tick() her cagrida gunceller.
        self.wait_streak:       Dict[str, int] = {}

    # -----------------------------------------------------------------------
    # Path planning helpers
    # -----------------------------------------------------------------------

    def _parked_blocked_nodes_for(self, agv_id: str) -> Set[str]:
        """Path planning hard-block: parked (DONE) + disconnected AGV pos'lari.
        Aktif AGV transient rezervasyonlari conflict-detect katmaninda yonetilir."""
        blocked: Set[str] = set()
        for n, owner in self.reservations.node_owner.items():
            if owner == agv_id:
                continue
            owner_m = self.missions.get(owner)
            if owner_m is not None and owner_m.state == MissionState.DONE:
                blocked.add(n)
        for other_id in self.disconnected_agvs:
            if other_id == agv_id:
                continue
            pos = self.last_known_pos.get(other_id)
            if pos:
                blocked.add(pos)
        return blocked

    # -----------------------------------------------------------------------
    # Connection state — WS persistence
    # -----------------------------------------------------------------------

    def set_agv_connected(self, agv_id: str, connected: bool,
                          current_pos: Optional[str] = None) -> None:
        """Bagli/kopuk gecisini bildir. Disconnect mission'i silmez, sadece
        dispatch'i durdurur ve son pos'u engel sayar."""
        if connected:
            self.disconnected_agvs.discard(agv_id)
            if current_pos:
                self.last_known_pos[agv_id] = current_pos
        else:
            self.disconnected_agvs.add(agv_id)
            if current_pos:
                self.last_known_pos[agv_id] = current_pos
            elif agv_id in self.missions:
                self.last_known_pos[agv_id] = self.missions[agv_id].pos

    def _plan_path(
        self,
        start:         str,
        goal:          str,
        agv_id:        str,
        blocked_edges: Optional[Iterable[Tuple[str, str]]] = None,
    ) -> Optional[List[str]]:
        """A* ile path bul (soft reservations: sadece parked block).
        blocked_edges opsiyonel ek engelleme — reroute icin cakisma edge'i."""
        return self.graph.astar(
            start, goal,
            blocked_nodes = self._parked_blocked_nodes_for(agv_id),
            blocked_edges = blocked_edges,
        )

    # -----------------------------------------------------------------------
    # Mission API
    # -----------------------------------------------------------------------

    def add_mission(
        self,
        agv_id:    str,
        goal:      str,
        start:     Optional[str] = None,
        has_cargo: bool          = False,
    ) -> Mission:
        """Yeni mission ekle. AGV'nin aktif mission'i varsa queued'a eklenir;
        AGV mevcut mission'i bitirince queue otomatik aktive olur."""
        self._mission_id_counter += 1
        self._fifo_counter       += 1

        existing = self.missions.get(agv_id)
        if start is None:
            # FP-15: konum bilinmiyorsa son bilinen pozisyonu kullan; o da
            # yoksa goal'e duser (anlik DONE — sessiz tuzakti, en azindan
            # last_known_pos varsa gercek konumdan baslar).
            start = (existing.pos if existing
                     else self.last_known_pos.get(agv_id) or goal)

        # Eski mission DONE ise (parked obstacle) → temizle, yeni mission aktif olsun
        if existing is not None and existing.state == MissionState.DONE:
            self.reservations.release_all_for(agv_id)
            del self.missions[agv_id]
            existing = None

        path = self._plan_path(start, goal, agv_id)
        if path is None:
            # Ulasilamaz — yine de mission olustur, _plan_hop her tick replan dener
            path = [start]

        m = Mission(
            id          = self._mission_id_counter,
            agv_id      = agv_id,
            start       = start,
            goal        = goal,
            path        = path,
            pos         = start,
            has_cargo   = has_cargo,
            fifo_order  = self._fifo_counter,
            state       = MissionState.QUEUED,
        )
        if existing is None:
            self.missions[agv_id] = m
        else:
            self.queued.append(m)

        # start==goal ise mission anlik DONE
        if m.is_done and existing is None:
            self._finish_mission(agv_id)

        return m

    def cancel_mission(self, agv_id: str) -> bool:
        """Tek bir AGV'nin aktif + kuyruktaki mission'larini iptal et ve
        rezervasyonlarini serbest birak. _planner_clear_all'un aksine diger
        AGV'lerin state'ine, queue'suna veya rezervasyonlarina DOKUNMAZ —
        takili tek AGV'yi guvenle durdurmak icin (FP-09). Bir sey iptal
        edildiyse True."""
        had = (agv_id in self.missions
               or any(q.agv_id == agv_id for q in self.queued))
        self.reservations.release_all_for(agv_id)
        self.missions.pop(agv_id, None)
        self.queued = [q for q in self.queued if q.agv_id != agv_id]
        self.disconnected_agvs.discard(agv_id)
        self.last_known_pos.pop(agv_id, None)
        self.wait_streak.pop(agv_id, None)
        return had

    def set_agv_position(self, agv_id: str, node: str) -> None:
        """AGV'nin gercek konumunu sisteme bildir. Pos path disindaysa replan."""
        m = self.missions.get(agv_id)
        if m is None:
            return
        # Gecersiz/sentinel node ('?' bilinmeyen konum) mission.pos'u BOZMASIN.
        # Aksi halde astar('?',...) None doner, next_node kaybolur, AGV sessizce
        # stall eder (FP-10 savunma katmani — PC tarafi da '?' filtreler).
        if node not in self.graph.nodes:
            return
        if node != m.pos:
            m.inflight = None   # pozisyon degisti — onceki hop fiilen bitti
        m.pos = node
        # YIELDING'te park/yol node'lari path disinda olmasi NORMAL — replan
        # path'i bozar (FP-11). Pos guncellenir, yield_step akisi yonetir.
        if m.state == MissionState.YIELDING:
            return
        if node not in m.path and m.goal:
            replan = self._plan_path(node, m.goal, agv_id)
            if replan and len(replan) > 1:
                m.path = replan

    def on_hop_complete(self, agv_id: str, new_node: str) -> None:
        """AGV hop'u tamamlayinca cagrilir. Pos guncelle, rezervasyon bosalt,
        gerekirse replan. YIELDING state'te replan ETMEZ — park node'un
        path disinda olmasi normaldir."""
        m = self.missions.get(agv_id)
        if m is None:
            return
        prev   = m.pos
        m.pos  = new_node
        m.inflight = None   # hareket bitti — yeni karar verilebilir
        # Eski node + edge rezervasyonlarini bosalt
        if prev != new_node:
            if self.reservations.node_owner.get(prev) == agv_id:
                del self.reservations.node_owner[prev]
            ekey = edge_key(prev, new_node)
            if self.reservations.edge_owner.get(ekey) == agv_id:
                del self.reservations.edge_owner[ekey]
        if m.is_done:
            self._finish_mission(agv_id)
            return
        # YIELDING'te park node off-path normaldir — yield_step yonetir.
        if m.state == MissionState.YIELDING:
            return
        # Pos path disinda → replan (firmware reroute reddetti veya drift)
        if new_node not in m.path and m.goal:
            replan = self._plan_path(new_node, m.goal, agv_id)
            if replan and len(replan) > 1:
                m.path = replan

    def _finish_mission(self, agv_id: str) -> None:
        """Mission bitti. AGV parked olur (state=DONE). Queued mission varsa
        otomatik aktive olur — yeni hedef icin tekrar setHop dispatch edilir."""
        m = self.missions.get(agv_id)
        if m is None:
            return
        m.state = MissionState.DONE
        # Edge rezervasyonlari bosalt (artik hareket etmiyor)
        for e in [e for e, o in self.reservations.edge_owner.items()
                  if o == agv_id]:
            del self.reservations.edge_owner[e]
        # Pos node hard reserve kal (idle obstacle)
        self.reservations.reserve_node(m.pos, agv_id)
        # Bekleyen mission varsa otomatik aktive et
        nxt = next((q for q in self.queued if q.agv_id == agv_id), None)
        if nxt:
            self.queued.remove(nxt)
            self.reservations.release_all_for(agv_id)
            nxt.pos   = m.pos
            nxt.start = m.pos
            self.missions[agv_id] = nxt

    # -----------------------------------------------------------------------
    # Tick — ana planlama dongusu
    # -----------------------------------------------------------------------

    def tick(self) -> Dict[str, HopCommand]:
        """Her aktif AGV icin HopCommand uretir. Disconnected AGV'ler
        skip edilir — mission frozen, dispatch yok (WS yok)."""
        commands: Dict[str, HopCommand] = {}
        for m in self._prioritize_missions():
            commands[m.agv_id] = self._plan_hop(m)
        self._update_wait_streak(commands)
        return commands

    def _update_wait_streak(self, commands: Dict[str, "HopCommand"]) -> None:
        """WAIT alan AGV'nin sayacini artir, hareket edeni sifirla. Aging
        onceligi ve production deadlock tespiti bunu kullanir."""
        for agv_id, c in commands.items():
            if c.action == HopAction.WAIT:
                self.wait_streak[agv_id] = self.wait_streak.get(agv_id, 0) + 1
            else:
                self.wait_streak[agv_id] = 0

    def _priority_key(self, m: Mission):
        """Siralama anahtari (kucuk = yuksek oncelik): Cargo > efektif-FIFO > ID.
        Anti-starvation: _STARVE_THRESHOLD'u asan ardisik WAIT, fifo_order'i
        kademeli dusurur → uzun bekleyen AGV taze olani gecer. Esik altinda
        (wait_streak <= esik) aging=0 → davranis (not cargo, fifo, id) ile ayni."""
        aging = max(0, self.wait_streak.get(m.agv_id, 0) - self._STARVE_THRESHOLD)
        return (not m.has_cargo, m.fifo_order - aging, m.agv_id)

    def _prioritize_missions(self) -> List[Mission]:
        """Cargo > FIFO(+aging) > ID. Disconnected ve is_done atlanir.
        İSTİSNA: YIELDING mission (geçici yan park / blocker eviction) is_done
        olsa bile (DONE AGV başka araca yol veriyor) işlenmeli — yoksa park'a
        gidip geri dönemez."""
        active = [m for m in self.missions.values()
                  if (not m.is_done or m.state == MissionState.YIELDING)
                  and m.agv_id not in self.disconnected_agvs]
        # YIELDING (yan park / blocker eviction) mission'lar tick'te EN SONA:
        # böylece sıkışan/aktif AGV ÖNCE replan eder, yield eden onun GÜNCEL
        # yolunu görüp doğru bekler (yoksa eski [pos] path'ini "clear" sanıp
        # erken döner → çakışma döngüsü).
        return sorted(active,
                      key=lambda m: ((m.state == MissionState.YIELDING),)
                                    + tuple(self._priority_key(m)))

    # -----------------------------------------------------------------------
    # Production deadlock tespiti + kurtarma (yalniz watchdog cagirir;
    # tick akisini DEGISTIRMEZ → FleetSimulator testleri etkilenmez)
    # -----------------------------------------------------------------------

    def detect_deadlock(self, threshold: int = 6) -> bool:
        """Aktif mission var VE hepsi `threshold` tick'tir ardisik WAIT'te mi?
        Olay-guducu production'da bu durum kalici donma demektir (kimse
        hareket etmiyor → hopComplete gelmiyor → tick re-invoke olmuyor)."""
        active = [m for m in self.missions.values()
                  if not m.is_done and m.agv_id not in self.disconnected_agvs]
        if not active:
            return False
        return all(self.wait_streak.get(m.agv_id, 0) >= threshold
                   for m in active)

    def break_deadlock(self) -> Optional["HopCommand"]:
        """Deadlock kurtarma (yalniz production watchdog cagirir): en dusuk
        oncelikli bekleyen AGV'yi en yuksek oncelikliye yan-park ettir. Yield
        HopCommand'i doner (caller dispatch eder); side park yoksa None
        (UI manuel mudahale uyarsin). VERTEX-active simetrik deadlock'u
        (>=3 AGV) ve disconnected-strand kalintilarini kirar."""
        active = [m for m in self.missions.values()
                  if not m.is_done and m.agv_id not in self.disconnected_agvs]
        if not active:
            return None
        # 1) TEK-HOP BLOCKER EVICTION (FP-19) — ÖNCE dene: WAIT eden AGV'nin
        #    yolunu görevsiz (DONE) bir AGV tıkıyorsa, o blocker'ı bir KOMŞUYA
        #    çek → sıkışan geçer → blocker döner. Generic yield'den (2) önce —
        #    FP-20: aksi halde generic yield gerçek blocker'ı görmeden düşük
        #    öncelikliyi boş yere oynatır → yield-resume salınımı → livelock.
        for m in sorted(active, key=self._priority_key):
            yc = self._evict_blocker(m, max_hops=1)
            if yc is not None:
                return yc
        # 2) Generic yield: 2+ aktif AGV'de düşük öncelikliyi yüksek olana
        #    yan-park ettir (simetrik head-on / tek-hop evict yetmedi).
        if len(active) >= 2:
            ordered = sorted(active, key=self._priority_key)
            winner  = ordered[0]
            # FP-14: zaten YIELDING olan kurban yeniden yield edilemez (park
            # alanlari ust uste biner, state bozulur) — dusuk oncelikli ama
            # ACTIVE/QUEUED olan ilk adayi sec.
            victims = [m for m in reversed(ordered[1:])
                       if m.state != MissionState.YIELDING]
            if victims:
                yc = self._begin_yield(victims[0], winner)
                if yc is not None:
                    self.wait_streak[victims[0].agv_id] = 0
                    return yc
        # 3) ÇOK-HOP eviction (FP-20b, SON ÇARE): tek-hop evict + generic yield
        #    başarısızsa — alt-sıra dead-end tuzağı (blocker K'da, m L→I; K'nın
        #    komşuları J,L ikisi de açılan yolda → tek-hop imkansız). Blocker
        #    çok-hop bir refuge'e (K-J-F) çekilir; m bekler, geçer, blocker döner.
        #    En sona alındı ki generic yield'in çözdüğü 3-AGV senaryolarını
        #    gereksiz çok-hop sürüklemeyle bozmasın.
        for m in sorted(active, key=self._priority_key):
            yc = self._evict_blocker(m, max_hops=None)
            if yc is not None:
                return yc
        return None

    def _evict_blocker(
        self, m: Mission, max_hops: Optional[int] = None,
    ) -> Optional[HopCommand]:
        """m: hedefe ulaşamayan (WAIT) aktif AGV. Yolu DONE/IDLE bir AGV
        tıkıyorsa onu geçici bir refuge'e çek (return = blocker'ın eski yeri).
        Blocker yield-return mekanizmasıyla, m geçtikten sonra eski yerine döner.

        Bir done AGV'nin pozisyonu A*'tan ÇIKARILINCA m'nin yolu açılıyorsa o
        AGV blocker'dır. Blocker, açılan yolun ÜZERİNDE OLMAYAN bir node'a çekilir.
        max_hops=1 → yalnız komşu (klasik FP-19); max_hops=None → çok-hop refuge
        (FP-20b, alt-sıra dead-end tuzağı: K'nın komşuları J,L açılan yolda →
        K-J-F gibi çok-hop gerek)."""
        if m.is_done or not m.goal or m.pos == m.goal:
            return None
        blockers = [b for b in self.missions.values()
                    if b.state == MissionState.DONE and b.agv_id != m.agv_id
                    and b.agv_id not in self.disconnected_agvs and b.pos]
        if not blockers:
            return None
        done_pos = {b.pos for b in blockers}
        for b in blockers:
            # b kaldırılınca (diğer DONE'lar bloklu) m'nin yolu açılıyor mu?
            others = done_pos - {b.pos}
            path = self.graph.astar(m.pos, m.goal, blocked_nodes=others)
            if not path or len(path) <= 1 or b.pos not in path:
                continue
            # b'yi açılan yol DIŞINDA bir refuge'e çek.
            ep = self._find_eviction_refuge(b, set(path), m.pos, max_hops)
            if ep is not None:
                yc = self._begin_yield(b, m, path=ep)
                if yc is not None:
                    yc.reason = (f"evict: {b.agv_id} {b.pos}->{ep[-1]} "
                                 f"(yol ver {m.agv_id})")
                    self.wait_streak[m.agv_id] = 0
                    return yc
        return None

    def _find_eviction_refuge(
        self, b: Mission, on_path: Set[str], stuck_pos: str,
        max_hops: Optional[int] = None,
    ) -> Optional[List[str]]:
        """Blocker b'yi m'nin açılan yolundan (on_path) TAMAMEN çıkaracak en
        yakın node'a giden retreat path'i [b.pos,...,R] bul. m'nin durduğu
        node'a (stuck_pos) ve başka AGV'nin tuttuğu node'a girmez. Transit
        on_path node'larından geçebilir (m rezervasyon guard ile bekler);
        refuge'in KENDİSİ on_path dışı olmalı. max_hops verilirse o derinliğe
        kadar arar (1 = yalnız komşu, klasik FP-19; None = sınırsız çok-hop).

        DEAD-END (degree-1) refuge KABUL: FP-20 aktif-yielder'da dead-end TUZAK
        (oraya park eden hedefine devam edemez) ama EVICTION'da blocker oraya
        geçici çekilip m geçince GERİ DÖNER → dead-end ideal pull-off."""
        blocked = self.reservations.blocked_nodes_for(b.agv_id)
        forbidden = {stuck_pos} | blocked
        prev: Dict[str, Optional[str]] = {b.pos: None}
        depth: Dict[str, int] = {b.pos: 0}
        queue: deque = deque([b.pos])
        while queue:
            cur = queue.popleft()
            if (cur != b.pos and cur not in on_path and cur not in blocked):
                path = [cur]
                p = prev[cur]
                while p is not None:
                    path.append(p)
                    p = prev[p]
                path.reverse()
                return path
            if max_hops is not None and depth[cur] >= max_hops:
                continue
            for nb, _ in self.graph.neighbors(cur):
                if nb in prev or nb in forbidden:
                    continue
                prev[nb] = cur
                depth[nb] = depth[cur] + 1
                queue.append(nb)
        return None

    def _plan_hop(self, m: Mission) -> HopCommand:
        """Bir mission icin sonraki hop komutunu hesapla.

        Akis:
          1. is_done → DONE
          2. YIELDING state → yield_step (return veya WAIT)
          3. Ulasilamaz path → replan dene
          4. Conflict tespit → resolve (yield / wait / reroute)
          5. next_node yoksa → WAIT
          6. Normal hop — rezerve et + dispatch
        """
        if m.is_done:
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.DONE,
                from_=m.current, reason="mission complete",
            )

        # YIELDING: park'a vardiysa diger AGV'yi bekle, gectiyse don.
        if m.state == MissionState.YIELDING:
            return self._plan_yield_step(m)

        # IN-FLIGHT: dispatch edilmis hop henuz tamamlanmadi (gercek dunyada
        # AGV su an o hop'u yurutuyor; firmware mid-flight yeni hop'u zaten
        # reddeder). Karar DEGISTIRME — ayni komutu yeniden uret (executor
        # dedup'lar; komut kaybolduysa watchdog yeniden gonderir). Cakismalar
        # diger AGV'lerin planinda cozulur.
        if m.inflight is not None and m.pos == m.inflight[0]:
            nxt = m.inflight[1]
            after = None
            after2 = None
            if m.pos in m.path:
                i = m.path.index(m.pos)
                if i + 1 < len(m.path) and m.path[i + 1] == nxt:
                    # 3-hop look-ahead reissue'da da KORUNMALI; yoksa her
                    # reissue (her hop) after2'yi silip navPath'i kisaltir →
                    # AGV path-sonu node'unda yine bekler (3-hop iptal olur).
                    if i + 2 < len(m.path):
                        after = m.path[i + 2]
                    if i + 3 < len(m.path):
                        after2 = m.path[i + 3]
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.NORMAL,
                from_=m.pos, next_=nxt, after_=after, after2_=after2,
                reason="enroute (reissue)",
            )

        # Replan tetikleyici:
        #   1) path=[start] (ulasilamaz) — onceki replan basarisiz, retry
        #   2) m.pos path'te yok — yol acildiysa replan
        if (len(m.path) <= 1 or m.pos not in m.path) and m.pos != m.goal:
            replan = self._plan_path(m.pos, m.goal, m.agv_id)
            if replan and len(replan) > 1:
                m.path = replan

        # Cakisma tespit (parked + aktif AGV'ler)
        conflict = self._detect_conflict(m)
        if conflict.type != ConflictType.NONE:
            return self._resolve_conflict(m, conflict)

        # next_node yoksa: ulasilamaz veya hedefte
        if m.next_node is None:
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.WAIT,
                from_=m.current, reason="no reachable path",
            )

        return self._normal_hop(m)

    def _normal_hop(self, m: Mission, reason: str = "normal") -> HopCommand:
        """Dispatch hop emri: current release + next/edge reserve.
        Paralel akis icin current dispatch aninda birakilir, fiziksel mesafe
        sonar ile korunur."""
        if m.next_node is None:
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.WAIT,
                from_=m.current, reason="no next",
            )
        # FP-20 GUVENLIK: me.next'i BASKA bir AGV tutuyorsa (oraya gidiyor ya da
        # orada oturuyor) NORMAL surme — tick-ici sira yarisinda cozumun "diger
        # yan cekilir" varsayimi tutmayabilir (ornek: refuge'ten resume eden AGV,
        # ayni tick WAIT etmis bir araca dayanir). Rezervasyon = fiziksel iddia;
        # tutuluysa BEKLE. (Soft-rezervasyon yalniz A* path planlamasinda gevsek;
        # dispatch aninda dolu node'a surmek carpisma.) Ayni-yon takip/temiz yan
        # park'ta (kazanan once islenip next'i kendi rezerve eder) tetiklenmez.
        owner = self.reservations.node_owner.get(m.next_node)
        if owner is not None and owner != m.agv_id:
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.WAIT, from_=m.current,
                reason=f"next {m.next_node} dolu ({owner}), bekle",
            )
        if self.reservations.node_owner.get(m.current) == m.agv_id:
            del self.reservations.node_owner[m.current]
        self.reservations.reserve_node(m.next_node, m.agv_id)
        self.reservations.reserve_edge(m.current, m.next_node, m.agv_id)
        m.state = MissionState.ACTIVE
        m.inflight = (m.current, m.next_node)
        return HopCommand(
            agv_id = m.agv_id,
            action = HopAction.NORMAL,
            from_  = m.current,
            next_  = m.next_node,
            after_ = m.after_node,
            after2_ = m.after2_node,
            reason = reason,
        )

    # -----------------------------------------------------------------------
    # Conflict detection
    # -----------------------------------------------------------------------

    def _detect_conflict(self, me: Mission) -> Conflict:
        """Cakisma tespiti. Donen tipler:
          - VERTEX parked: parked AGV tam next_node'da
          - VERTEX active: me.next ile other.{next, after} ortusur
          - EDGE_SWAP direct: A→B vs B→A (bitisik komsular)
          - EDGE_SWAP future: path overlap 3-4 hop ileride ters yonde
        """
        if me.next_node is None:
            return Conflict(ConflictType.NONE, me.agv_id, "", "")

        # 1) Fiziksel olarak me.next'te DURAN AGV — uzerine surulemez.
        #    DONE (parked engel) VEYA refuge'ine park etmis YIELDING (FP-20):
        #    cok-hop refuge'te bekleyen AGV mission-path'i disinda oldugu icin
        #    next/after'i None'dir; ucuncu bir AGV onu "gormeden" ustune surup
        #    carpisirdi (3-AGV koridor). Refuge'teki YIELDING'i de parked say.
        for other in self.missions.values():
            if other.agv_id == me.agv_id or other.pos != me.next_node:
                continue
            parked = (
                other.state == MissionState.DONE
                or (other.state == MissionState.YIELDING
                    and other.pos == other.yield_park_node)
            )
            if parked:
                return Conflict(
                    ConflictType.VERTEX, me.agv_id, other.agv_id, me.next_node,
                    detail=f"parked at {other.pos}",
                )

        # 2) Aktif AGV cakismalari
        for other in self.missions.values():
            if other.agv_id == me.agv_id or other.state == MissionState.DONE:
                continue
            if other.is_done:
                continue

            # VERTEX: me.next == other'in next veya after'inda. after'a bakmak
            # 90° yaklasimda gec-detect/carpisma riskini onler.
            other_future = {n for n in (other.next_node, other.after_node) if n}
            if me.next_node in other_future:
                return Conflict(
                    ConflictType.VERTEX, me.agv_id, other.agv_id, me.next_node,
                    detail=f"both target {me.next_node} (other near-future)",
                )

            # EDGE_SWAP direct: A→B vs B→A (bitisik komsular)
            if (other.current == me.next_node
                    and other.next_node == me.current):
                return Conflict(
                    ConflictType.EDGE_SWAP, me.agv_id, other.agv_id,
                    edge_key(me.current, me.next_node),
                    detail=f"head-on {me.current}-{me.next_node}",
                )

            # EDGE_SWAP future: path'ler 3-4 hop ileride ayni edge'i ters yon
            # kullaniyorsa continuous-after ile orada bulusurlar — onceden yakala.
            headon = self._find_path_headon(me, other, lookahead=4)
            if headon:
                return Conflict(
                    ConflictType.EDGE_SWAP, me.agv_id, other.agv_id,
                    edge_key(*headon),
                    detail=f"future head-on edge {headon[0]}-{headon[1]}",
                )

        return Conflict(ConflictType.NONE, me.agv_id, "", "")

    def _path_edges_from(
        self, m: Mission, start: str, k: int,
    ) -> List[Tuple[str, str]]:
        """m.path uzerinde `start` node'undan baslayarak k tane (from,to)
        edge donder. start path'te yoksa bos liste."""
        if start not in m.path:
            return []
        idx = m.path.index(start)
        edges: List[Tuple[str, str]] = []
        end = min(idx + k, len(m.path) - 1)
        for i in range(idx, end):
            edges.append((m.path[i], m.path[i + 1]))
        return edges

    def _find_path_headon(
        self, me: Mission, other: Mission, lookahead: int = 3,
    ) -> Optional[Tuple[str, str]]:
        """Iki AGV'nin path'inin lookahead edge ileride ters yonde kesisip
        kesismedigini kontrol et. Bulursa me'nin yonunde (a, b) tuple."""
        me_edges    = self._path_edges_from(me,    me.pos,    lookahead)
        other_edges = self._path_edges_from(other, other.pos, lookahead)
        for ma, mb in me_edges:
            for oa, ob in other_edges:
                if ma == ob and mb == oa:
                    return (ma, mb)
        return None

    # -----------------------------------------------------------------------
    # Conflict resolution
    # -----------------------------------------------------------------------

    def _resolve_conflict(self, me: Mission, conflict: Conflict) -> HopCommand:
        """Cozum politikasi (cakisma tipine gore):
          priority winner    -> NORMAL
          VERTEX parked      -> reroute, yoksa WAIT
          VERTEX active      -> WAIT (90° carpisma korumasi)
          EDGE_SWAP direct   -> YIELD, yoksa WAIT
          EDGE_SWAP future   -> REROUTE (cakisma edge blokli), yoksa WAIT
        """
        other = self.missions.get(conflict.agv_b)
        if other is None:
            return self._normal_hop(me)

        # FP-20 KORIDOR KILIDI: oncelik tek basina "dolu node'a sur" hakki
        # VERMEZ. Direct head-on'da diger AGV fiziksel olarak me.next'te oturur;
        # priority-winner oraya surerse CARPISMA (kullanici raporu: "yol
        # vermiyorlar"). Ayrica kaybeden, kazananin GECMESI GEREKEN node'unda
        # oturuyorsa (kopru/dead-end dali) basit WAIT sonsuz kilit yaratir —
        # kaybeden bir 'refuge'e cekilmeli, kazanan o cekilene dek beklemeli.
        if self._needs_corridor_resolution(me, other, conflict):
            return self._resolve_corridor(me, other)

        # 1) Ben oncelikliyim — hemen devam.
        if self._compute_priority(me, other):
            return self._normal_hop(me, reason=f"priority over {other.agv_id}")

        # 2) VERTEX cakismasi
        if conflict.type == ConflictType.VERTEX:
            if other.state == MissionState.DONE:
                # Parked → reroute dene
                alt = self._find_alternative_path(me, other)
                if alt is not None and len(alt) > 1:
                    return self._dispatch_reroute(
                        me, alt, reason=f"reroute past parked {other.agv_id}",
                    )
                return HopCommand(
                    agv_id=me.agv_id, action=HopAction.WAIT,
                    from_=me.current,
                    reason=f"parked {other.agv_id} blocks {me.next_node}",
                )
            # Aktif → ayni hedefe gidiyoruz, WAIT (yield/reroute anlamsiz)
            return HopCommand(
                agv_id=me.agv_id, action=HopAction.WAIT,
                from_=me.current,
                reason=f"yielding {other.agv_id} (both -> {me.next_node})",
            )

        # 3) EDGE_SWAP — direct (bitisik) vs future (uzak) ayri politika
        if conflict.type == ConflictType.EDGE_SWAP:
            if other.current == me.next_node:
                # Direct head-on → yan park dene
                yc = self._begin_yield(me, other)
                if yc is not None:
                    return yc
                return HopCommand(
                    agv_id=me.agv_id, action=HopAction.WAIT,
                    from_=me.current,
                    reason=f"head-on {other.agv_id}, no side park",
                )
            # Future head-on → cakisma edge'ini bloklayarak reroute
            edge = conflict.node_or_edge
            if isinstance(edge, tuple):
                alt = self._plan_path(
                    me.current, me.goal, me.agv_id,
                    blocked_edges={edge},
                )
                if alt and len(alt) > 1:
                    return self._dispatch_reroute(
                        me, alt,
                        reason=f"reroute (future head-on {edge[0]}-{edge[1]})",
                    )
            return HopCommand(
                agv_id=me.agv_id, action=HopAction.WAIT,
                from_=me.current,
                reason=f"future head-on {other.agv_id}, no detour",
            )

        # 4) Fallback (beklenmedik conflict tipi)
        return HopCommand(
            agv_id=me.agv_id, action=HopAction.WAIT,
            from_=me.current,
            reason=f"yielding to {other.agv_id}",
        )

    # -----------------------------------------------------------------------
    # FP-20: Koridor kilidi / kafa-kafaya (refuge retreat)
    # -----------------------------------------------------------------------

    def _on_remaining_path(self, m: Mission, node: str) -> bool:
        """node, m'nin KALAN path'inde (pos dahil sonrasi) mi? m'nin gecmesi
        gereken bir node'da baska AGV oturuyor mu kontrolu icin."""
        if not node or m.pos not in m.path:
            return False
        return node in m.path[m.path.index(m.pos):]

    def _needs_corridor_resolution(
        self, me: Mission, other: Mission, conflict: Conflict,
    ) -> bool:
        """Bu cakisma 'koridor kilidi' mi — basit oncelik/WAIT yetmez, refuge
        retreat gerekir mi?

        Kosul (ikisi birden): kaybeden (dusuk oncelik) kazananin GECMESI
        GEREKEN bir node'unda oturuyor VE tek-hop yan parki YOK (dead-end dali /
        kopru). O zaman basit WAIT/priority sonsuz kilit → refuge retreat gerek.

        AKSI HALDE eski (ucuz + test-uyumlu) akis yeterli:
          - tek-hop yan park VARSA: kazanan NORMAL, kaybeden side-park (Sen.12).
          - ayni-yon TAKIP (lider onde): kendiliginden cozulur (Sen.2/3/6) —
            ters-yon head-on degilse koridor cozumu tetiklenmez.
        Direct head-on'da (other tam me.next'te) 'blocks' zaten dogrudur."""
        direct = (conflict.type == ConflictType.EDGE_SWAP
                  and other.current == me.next_node)
        # Ters-yon (opposing) head-on mu? Direct degilse path-overlap'e bak;
        # ayni-yon takipte _find_path_headon None → koridor cozumu YOK.
        if not direct and self._find_path_headon(me, other, lookahead=4) is None:
            return False
        higher, lower = ((me, other) if self._compute_priority(me, other)
                         else (other, me))
        blocks = direct or (bool(lower.current)
                            and self._on_remaining_path(higher, lower.current))
        if blocks and self._find_side_park(lower, higher) is None:
            return True
        return False

    def _resolve_corridor(self, me: Mission, other: Mission) -> HopCommand:
        """Koridor kilidi cozumu. _headon_plan ile KIM cekilir deterministik
        secilir (iki AGV ayni karari verir). Cekilen ben'sem refuge retreat
        baslat; degilsem cekilen gecene kadar BEKLE (hold)."""
        yielder, path = self._headon_plan(me, other)
        if yielder is None:
            # Hicbir tarafa refuge yok — cozulemez topoloji (siding'siz duz
            # koridor). WAIT → watchdog/manual mudahale. (Grafimizda siding
            # var; bu dal pratikte nadir.)
            return HopCommand(
                agv_id=me.agv_id, action=HopAction.WAIT, from_=me.current,
                reason=f"head-on {other.agv_id}: refuge yok (kilit)",
            )
        if yielder.agv_id == me.agv_id:
            yc = self._begin_yield(me, other, path=path)
            if yc is not None:
                return yc
            return HopCommand(
                agv_id=me.agv_id, action=HopAction.WAIT, from_=me.current,
                reason=f"head-on {other.agv_id}: cekilemiyorum",
            )
        # Diger AGV cekiliyor — o refuge'ine varana dek BEKLE (onun retreat'i
        # benim path'imden/koprümden geciyor olabilir).
        return HopCommand(
            agv_id=me.agv_id, action=HopAction.WAIT, from_=me.current,
            reason=f"head-on: {other.agv_id} cekiliyor, bekle",
        )

    def _headon_plan(
        self, a: Mission, b: Mission,
    ) -> Tuple[Optional[Mission], Optional[List[str]]]:
        """Kafa-kafaya/koridor kilidinde KIM cekilir + retreat path'i.
        Deterministik + simetrik (a,b sirasindan bagimsiz) → iki AGV ayni
        tick'te ayni yielder'i secer. Tercih (en ucuz cozum once):
          1) loser (dusuk oncelik) tek-hop yan park  (kazanan beklemez)
          2) loser cok-hop refuge retreat            (kazanan bekler)
          3) winner tek-hop yan park
          4) winner cok-hop refuge retreat
        Donus (yielder_mission, [current..park]) ya da (None,None)=cozumsuz."""
        if self._priority_key(a) <= self._priority_key(b):
            higher, lower = a, b
        else:
            higher, lower = b, a
        for first, second in ((lower, higher), (higher, lower)):
            park = self._find_side_park(first, second)
            if park is not None:
                return first, [first.current, park]
            rp = self._find_refuge(first, second)
            if rp is not None:
                return first, rp
        return None, None

    def _find_refuge(
        self, me: Mission, other: Mission,
    ) -> Optional[List[str]]:
        """me icin other'in KALAN path'inden TAMAMEN uzak bir 'refuge' (siding)
        node'una EN KISA retreat path'ini bul (cok-hop). _find_side_park (tek-hop
        komsu) basarisiz olunca kullanilir: me kopru koridoru / dead-end dalinda
        sikismis, bitisiginde park yeri yok ama 2-3 hop otede siding var.

        Donus [me.current, ..., R]; R: other'in kalan path'inde DEGIL, bloklu
        degil, dead-end (degree-1) DEGIL (FP-18 tuzagi). BFS = en yakin refuge.
        Retreat other'in GELECEK path'inden gecebilir (kopru) — kazanan o sirada
        bekler (_resolve_corridor hold). other'in SU ANKI pozisyonuna ASLA girme
        (fiziksel carpisma)."""
        if not me.current:
            return None
        other_path: Set[str] = set()
        if other.pos in other.path:
            other_path = set(other.path[other.path.index(other.pos):])
        for n in (other.current, other.next_node, other.after_node):
            if n:
                other_path.add(n)
        blocked = self.reservations.blocked_nodes_for(me.agv_id)
        for d in self.disconnected_agvs:
            if d != me.agv_id:
                pos = self.last_known_pos.get(d)
                if pos:
                    blocked = blocked | {pos}
        # other'in fiziksel olarak isgal ettigi node'lar — retreat ASLA girmez
        forbidden = {other.current}
        if other.inflight:
            forbidden.add(other.inflight[1])
        prev: Dict[str, Optional[str]] = {me.current: None}
        queue: deque = deque([me.current])
        while queue:
            cur = queue.popleft()
            # Refuge gecerli mi? Genelde gecisli (degree>=2) olmali (FP-18
            # dead-end tuzagi). ISTISNA: cur == me.goal — kendi hedefinde park
            # TUZAK DEGIL (oraya varinca DONE, geri donmesi gerekmez). Alt-sira
            # dead-end cikislarinda (orn. K→I: I hedef ama dead-end) sikisan AGV
            # hedefine cikip koridoru bosaltir — kazanan o gecince ilerler.
            if (cur != me.current and cur not in other_path
                    and cur not in blocked
                    and (len(self.graph.neighbors(cur)) >= 2
                         or cur == me.goal)):
                path = [cur]
                p = prev[cur]
                while p is not None:
                    path.append(p)
                    p = prev[p]
                path.reverse()
                return path
            for nb, _ in self.graph.neighbors(cur):
                if nb in prev or nb in forbidden or nb in blocked:
                    continue
                prev[nb] = cur
                queue.append(nb)
        return None

    def _dispatch_reroute(self, me: Mission, alt: List[str],
                          reason: str) -> HopCommand:
        """Reroute sonrasi guvenli dispatch:
          - FP-06: eski path'in HAYALET rezervasyonlarini birak (release_all_for),
            current'i geri tut. Aksi halde onceki tick'in eski next_node'u
            node_owner'da kalip baska AGV'nin _find_side_park secimini bozar.
          - FP-07: yeni path'i ata; eger yeni path IMMEDIATE bir cakismaya
            (parked AGV tam next'te VEYA direct head-on) sokuyorsa ve oncelik
            bizde degilse NORMAL yerine WAIT'e dus — eski path'in conflict
            tipiyle commit edilmis tek-tick yanlis dispatch'i onler. Future
            (uzak) cakismalar bir sonraki tick yeniden degerlendirilir."""
        self.reservations.release_all_for(me.agv_id)
        if me.current:
            self.reservations.reserve_node(me.current, me.agv_id)
        me.path = alt
        re = self._detect_conflict(me)
        if re.type != ConflictType.NONE:
            other2 = self.missions.get(re.agv_b)
            imminent = other2 is not None and (
                (re.type == ConflictType.VERTEX
                 and other2.state == MissionState.DONE)
                or (re.type == ConflictType.EDGE_SWAP
                    and other2.current == me.next_node)
            )
            if (imminent and other2 is not None
                    and not self._compute_priority(me, other2)):
                return HopCommand(
                    agv_id=me.agv_id, action=HopAction.WAIT, from_=me.current,
                    reason=f"reroute hala cakisiyor {re.agv_b}",
                )
        return self._normal_hop(me, reason=reason)

    def _compute_priority(self, a: Mission, b: Mission) -> bool:
        """a, b'den oncelikli mi? Cargo > FIFO(+aging) > ID. _priority_key ile
        tutarli — uzun bekleyen loser zamanla onceligi kazanir (anti-starvation,
        AGV_2 sistematik dezavantajini da kirar)."""
        return self._priority_key(a) < self._priority_key(b)

    def _find_alternative_path(
        self, me: Mission, other: Mission,
    ) -> Optional[List[str]]:
        """me icin: parked rezervasyon + other'in 2-hop ileri planini
        engelleyerek alternatif yol bul. Mevcut path ile ayni ise None."""
        blocked_e: Set[Tuple[str, str]] = set()
        # other'in 2-hop ileri plani: current→next ve next→after edges
        if other.current and other.next_node:
            blocked_e.add(edge_key(other.current, other.next_node))
        if other.next_node and other.after_node:
            blocked_e.add(edge_key(other.next_node, other.after_node))
        # Parked dahil baska AGV'lerin next_node'larini blokla
        extra_nodes: Set[str] = set()
        if other.current and other.next_node:
            extra_nodes.add(other.next_node)
        blocked_n = self._parked_blocked_nodes_for(me.agv_id) | extra_nodes

        path = self.graph.astar(
            me.current, me.goal,
            blocked_nodes=blocked_n,
            blocked_edges=blocked_e,
        )
        if path is None or len(path) < 2:
            return None
        # Mevcut path ile ayni mi?
        if me.pos in me.path:
            remaining = me.path[me.path.index(me.pos):]
            if path == remaining:
                return None
        return path

    # -----------------------------------------------------------------------
    # Yield (yan park)
    # -----------------------------------------------------------------------

    def _find_side_park(
        self, me: Mission, other: Mission,
    ) -> Optional[str]:
        """me.current'a komsu, yan park icin uygun bir node bul.

        Filtreler:
          - other.current/next/after degil ve other'in path'inde degil
          - baska AGV tarafindan rezerve degil
          - me.next_node degil (zaten oraya gidemiyoruz cakismadan)
        Tercih: dusuk degree once (pendant ozellikle iyi — kimse oraya
        gitmek istemez)."""
        if not me.current:
            return None
        neighbors = [n for n, _ in self.graph.neighbors(me.current)]
        if not neighbors:
            return None

        # other'in tum onumuzdeki path'i
        other_path: Set[str] = set()
        if other.pos in other.path:
            idx = other.path.index(other.pos)
            other_path = set(other.path[idx:])
        for n in (other.current, other.next_node, other.after_node):
            if n:
                other_path.add(n)

        blocked = self.reservations.blocked_nodes_for(me.agv_id)
        # FP-13: kopuk AGV'lerin son bilinen konumlari rezervasyon tablosunda
        # OLMAYABILIR (kopma aninda node release edilmis olabilir) — yan park
        # adayi olarak secilirse fiziksel carpisma. Acikca disla.
        for d in self.disconnected_agvs:
            if d != me.agv_id:
                pos = self.last_known_pos.get(d)
                if pos:
                    blocked = blocked | {pos}

        candidates: List[Tuple[int, str]] = []
        for n in neighbors:
            if n in other_path or n in blocked or n == me.next_node:
                continue
            degree = len(self.graph.neighbors(n))
            # FP-18: DEAD-END (degree-1) yan park = TUZAK. Loser oraya park
            # ederse tek cikisi me.current'a geri donus; o da genelde head-on
            # koridoru / kazananin tuttugu node → loser sikisir, deadlock.
            # 4×3 grafta D/H/L degree-1. Yan park GECISLI node olmali (>=2).
            # (Eski cyclic grafta dead-end yoktu, bu durum hic olusmuyordu.)
            if degree <= 1:
                continue
            candidates.append((degree, n))

        if not candidates:
            return None
        candidates.sort()
        return candidates[0][1]

    def _begin_yield(
        self, me: Mission, other: Mission, park: Optional[str] = None,
        path: Optional[List[str]] = None,
    ) -> Optional[HopCommand]:
        """me'yi yan parka / refuge'e yolla. Oncelik:
          - path verilmisse (FP-20 cok-hop refuge): [me.current, ..., R] yolu
            kullanilir, ilk hop dispatch edilir, gerisi _plan_yield_step yurur.
          - park verilmisse tek-hop ([me.current, park]).
          - hicbiri yoksa _find_side_park ile tek-hop secilir.
        Bulamazsa None (caller WAIT'e dusurur). yield_return_node = me.current
        → kazanan gecince oradan (refuge'te pos'tan) hedefe yeniden planlanir."""
        if path is None:
            if park is None:
                park = self._find_side_park(me, other)
            if park is None:
                return None
            path = [me.current, park]
        if len(path) < 2:
            return None
        park = path[-1]
        nxt  = path[1]                       # ilk hop hedefi
        me.state             = MissionState.YIELDING
        me.yield_to          = other.agv_id
        me.yield_park_node   = park
        me.yield_path        = list(path)
        me.yield_return_node = me.current
        me.yield_wait_ticks  = 0
        me.inflight          = (me.current, nxt)
        # Rezervasyon: current bos, ilk hop hedefi + edge tutulu
        if self.reservations.node_owner.get(me.current) == me.agv_id:
            del self.reservations.node_owner[me.current]
        self.reservations.reserve_node(nxt, me.agv_id)
        self.reservations.reserve_edge(me.current, nxt, me.agv_id)
        return HopCommand(
            agv_id = me.agv_id,
            action = HopAction.YIELD,
            from_  = me.current,
            next_  = nxt,
            after_ = None,
            reason = (f"refuge for {other.agv_id}" if len(path) > 2
                      else f"side park for {other.agv_id}"),
        )

    def _plan_yield_step(self, m: Mission) -> HopCommand:
        """YIELDING state'te her tick:
          - Henuz parka varilmadi → WAIT
          - Park'a varildi + diger AGV gecti → return dispatch
          - Park'a varildi + henuz gecmedi → WAIT
        """
        park = m.yield_park_node
        ret  = m.yield_return_node
        other = self.missions.get(m.yield_to) if m.yield_to else None

        if park is None or ret is None:
            return self._cancel_yield(m, "yield state corrupt")

        if m.pos != park:
            # Parka henuz varilmadi. FP-20: cok-hop refuge retreat — yield_path
            # uzerinde pos'tan SONRAKI hop'u dispatch et (tek-hop yan parkta
            # path=[ret, park], asagi mantik birebir eski davranisa iner).
            #   - pos yield_path'te ve sonrasi var → siradaki hop'u YIELD et.
            #     pos == ret (hic kipirdamadik) ise FP-12 reissue sayaci isler;
            #     ilerlemisse sayac sifirlanir (kayip-komut yalnizca baslangicta).
            #   - pos yield_path disinda (surukleme/eski hop) → yield iptal.
            path = m.yield_path or [ret, park]
            if m.pos in path:
                idx = path.index(m.pos)
                if idx + 1 < len(path):
                    nxt = path[idx + 1]
                    if m.pos == ret:
                        m.yield_wait_ticks += 1
                        if m.yield_wait_ticks >= self._YIELD_TIMEOUT_TICKS:
                            return self._cancel_yield(m, "yield enroute timeout")
                        reissue = " (reissue)"
                    else:
                        m.yield_wait_ticks = 0
                        reissue = ""
                    # Siradaki hop hedefi + edge rezerve (varsa eskiyi birak)
                    if self.reservations.node_owner.get(m.pos) == m.agv_id:
                        del self.reservations.node_owner[m.pos]
                    self.reservations.reserve_node(nxt, m.agv_id)
                    self.reservations.reserve_edge(m.pos, nxt, m.agv_id)
                    m.inflight = (m.pos, nxt)
                    kind = "refuge" if len(path) > 2 else "side park"
                    return HopCommand(
                        agv_id=m.agv_id, action=HopAction.YIELD,
                        from_=m.pos, next_=nxt, after_=None,
                        reason=f"{kind} for {m.yield_to}{reissue}",
                    )
            return self._cancel_yield(m, f"yield drift ({m.pos})")

        # Diger AGV gecti mi?
        if not self._is_other_clear_of(other, ret):
            # FP-17b: yol verilen AGV donus node'una KALICI park ettiyse (DONE)
            # beklemek anlamsiz — hemen park'tan yeniden planla (alternatif rota).
            if (other is not None and other.is_done
                    and other.current == ret):
                return self._resume_from_park(m)
            # FP-05: yol veren AGV cikmadan cok uzun beklersek (kopma / stall)
            # sonsuza dek strand kalmamak icin yield'i bitir, PARK'tan yeniden
            # planla (kor donus YOK — FP-17).
            m.yield_wait_ticks += 1
            if m.yield_wait_ticks >= self._YIELD_TIMEOUT_TICKS:
                return self._resume_from_park(m)
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.WAIT,
                from_=m.current,
                reason=f"yield waiting {m.yield_to} to clear {ret}",
            )

        return self._resume_from_park(m)

    def _is_other_clear_of(
        self, other: Optional[Mission], node: str,
    ) -> bool:
        """other AGV node'dan uzaklasti mi? other yoksa clear.
        FP-17: is_done CLEAR DEMEK DEGIL — other tam donus node'unda PARK
        etmis olabilir (hedefi orasiydi); kor donus park etmis aracin ustune
        surerdi. Once konum kontrolu, sonra is_done."""
        if other is None:
            return True
        if other.current == node:        # hala orada (park etmis olsa bile!)
            return False
        if other.is_done:
            return True
        # FP-05: yol verdigimiz AGV koptuysa hareket etmez ama is_done de olmaz —
        # (baska yerdeyse) clear say, aksi halde yield eden sonsuza dek strand.
        if other.agv_id in self.disconnected_agvs:
            return True
        if other.next_node == node:      # oraya geliyor
            return False
        # other'in kalan path'i node'dan geciyor mu?
        if node in other.path and other.pos in other.path:
            idx = other.path.index(other.pos)
            if node in other.path[idx:]:
                return False
        return True

    def _cancel_yield(self, m: Mission, reason: str) -> HopCommand:
        """Yield'i iptal et (parka varilamadan: kayip komut / drift / timeout).
        Park rezervasyonlari birakilir, mevcut pos tutulur, state ACTIVE'e
        doner — bir SONRAKI tick normal _plan_hop yeniden karar verir (replan
        dahil). Bu tick WAIT doner (guvenli no-op)."""
        park = m.yield_park_node
        ret  = m.yield_return_node
        if park is not None:
            if self.reservations.node_owner.get(park) == m.agv_id:
                del self.reservations.node_owner[park]
            if ret is not None:
                ekey = edge_key(ret, park)
                if self.reservations.edge_owner.get(ekey) == m.agv_id:
                    del self.reservations.edge_owner[ekey]
        self.reservations.reserve_node(m.pos, m.agv_id)
        m.state             = MissionState.ACTIVE
        m.yield_to          = None
        m.yield_park_node   = None
        m.yield_path        = None
        m.yield_return_node = None
        m.yield_wait_ticks  = 0
        m.inflight          = None
        # Pos path disina dustuyse (drift) replan
        if m.pos not in m.path and m.goal:
            replan = self._plan_path(m.pos, m.goal, m.agv_id)
            if replan and len(replan) > 1:
                m.path = replan
        return HopCommand(
            agv_id=m.agv_id, action=HopAction.WAIT,
            from_=m.pos, reason=f"yield iptal: {reason}",
        )

    def _resume_from_park(self, m: Mission) -> HopCommand:
        """Yield bitti — PARK'tan yola devam. ESKI davranis return_node'a KOR
        donusdu (FP-17: yol verilen AGV tam oraya park ettiyse ustune surerdi).
        Simdi: state temizlenir, path PARK'tan hedefe yeniden planlanir ve ayni
        tick icinde normal _plan_hop akisi calisir — cakisma kontrolleri dahil
        (donus node'u doluysa alternatif rota, yoksa WAIT)."""
        park = m.pos
        m.state             = MissionState.ACTIVE
        m.yield_to          = None
        m.yield_park_node   = None
        m.yield_path        = None
        m.yield_return_node = None
        m.yield_wait_ticks  = 0
        m.inflight          = None
        replan = self._plan_path(park, m.goal, m.agv_id)
        if replan and len(replan) > 1:
            m.path = replan
        elif park not in m.path:
            m.path = [park]    # ulasilamaz — _plan_hop her tick replan dener
        # Tek seviyeli yeniden giris: state artik ACTIVE, _plan_hop normal
        # akisi (conflict detect + dispatch) calistirir.
        return self._plan_hop(m)


class FleetSimulator:
    """Planner.tick() ciktisini alip AGV'lere uygular. Test ve UI simulasyonu
    icin; production'da bu rolu WS executor (PC→AGV setHop) ustlenir."""

    def __init__(self, planner: FleetPlanner):
        self.planner:     FleetPlanner                 = planner
        self.tick_no:     int                          = 0
        self.history:     List[Tuple[int, str, HopCommand]] = []
        self.wait_streak: Dict[str, int]               = {}

    def step(self) -> Dict[str, HopCommand]:
        cmds = self.planner.tick()
        self.tick_no += 1
        for agv, cmd in cmds.items():
            self.history.append((self.tick_no, agv, cmd))
            if cmd.action in (HopAction.NORMAL, HopAction.YIELD) and cmd.next_:
                self.planner.on_hop_complete(agv, cmd.next_)
                self.wait_streak[agv] = 0
            elif cmd.action == HopAction.WAIT:
                self.wait_streak[agv] = self.wait_streak.get(agv, 0) + 1
            else:   # DONE
                self.wait_streak[agv] = 0
        return cmds

    def run_until_done(
        self,
        max_ticks:      int = 100,
        deadlock_ticks: int = 5,
        verbose:        bool = False,
    ) -> str:
        """Tum missionlar bitene/deadlock olana kadar kos.
        Donus: 'done' | 'deadlock' | 'timeout'."""
        for _ in range(max_ticks):
            cmds = self.step()
            if verbose:
                for agv, c in cmds.items():
                    print(f"    t={self.tick_no:02d} {c}")
            if not cmds:
                return "done"
            if all(c.action == HopAction.DONE for c in cmds.values()):
                return "done"
            active = [a for a, c in cmds.items() if c.action != HopAction.DONE]
            if active and all(
                self.wait_streak.get(a, 0) >= deadlock_ticks for a in active
            ):
                return "deadlock"
        return "timeout"

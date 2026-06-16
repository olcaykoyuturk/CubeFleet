"""
Fleet Planner — çoklu AGV koordinasyonu.

A* + 2-hop look-ahead (setHop) + node rezervasyonu. Çakışma: VERTEX → kaybeden
bekler/reroute; EDGE_SWAP → yan park ya da reroute. add_mission/on_hop_complete/
set_agv_position/set_agv_connected ile beslenir, tick() HopCommand üretir.
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
    YIELDING = "yielding"   # yan park yapildi, digerinin gecmesi bekleniyor
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
    # Yan park state (yalnız YIELDING'te)
    yield_to:          Optional[str] = None
    yield_park_node:   Optional[str] = None
    yield_return_node: Optional[str] = None
    yield_wait_ticks:  int           = 0
    yield_path:        Optional[List[str]] = None   # [current, ..., park]
    # Dispatch edilen (from, next); hopComplete'e kadar tutulur.
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
        """pos'tan 3 sonraki node. Kullanılmıyor (2-hop)."""
        if self.pos not in self.path:
            return None
        idx = self.path.index(self.pos)
        return self.path[idx + 3] if idx + 3 < len(self.path) else None

    @property
    def is_done(self) -> bool:
        return self.pos == self.goal


@dataclass
class HopCommand:
    """AGV'ye gönderilecek emir (setHop). after2_ kullanılmıyor (2-hop)."""
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
    # Üst üste bu kadar WAIT'ten sonra öncelik artar (aç kalmasın).
    _STARVE_THRESHOLD = 8
    # Yan parkta bu kadar bekleyince yield iptal, replan.
    _YIELD_TIMEOUT_TICKS = 60

    def __init__(self, graph: Graph):
        self.graph               = graph
        self.reservations        = ReservationTable()
        self.missions:           Dict[str, Mission] = {}   # agv_id -> aktif mission
        self.queued:             List[Mission]      = []
        self._mission_id_counter = 0
        self._fifo_counter       = 0
        # Kopuk araç dispatch edilmez ama konumu diğerleri için engel sayılır.
        self.disconnected_agvs: Set[str] = set()
        self.last_known_pos:    Dict[str, str] = {}
        # Üst üste kaç tick WAIT aldı (öncelik + deadlock tespiti için).
        self.wait_streak:       Dict[str, int] = {}
        # Görevsiz duran araçların konumu — engel (rezerve + A* bloklu).
        self.idle_positions:    Dict[str, str] = {}

    # -----------------------------------------------------------------------
    # Path planning helpers
    # -----------------------------------------------------------------------

    def _parked_blocked_nodes_for(self, agv_id: str) -> Set[str]:
        """A* engelleri: parked (DONE) + kopuk + idle araç konumları."""
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
        for other_id, pos in self.idle_positions.items():   # idle de engel
            if other_id != agv_id and pos:
                blocked.add(pos)
        return blocked

    # -----------------------------------------------------------------------
    # Connection state — WS persistence
    # -----------------------------------------------------------------------

    def set_agv_connected(self, agv_id: str, connected: bool,
                          current_pos: Optional[str] = None) -> None:
        """Bagli/kopuk gecisini bildir. Disconnect mission'i silmez; sadece
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
        """A* ile path bul (sadece parked araçlar engel). blocked_edges reroute'ta
        çakışan edge'i engellemek için."""
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
            # Konum yoksa son bilinen, o da yoksa goal.
            start = (existing.pos if existing
                     else self.last_known_pos.get(agv_id) or goal)

        if existing is not None and existing.state == MissionState.DONE:
            del self.missions[agv_id]
            existing = None
        # Fresh görev: durduğu node'u rezerve et (çarpışma koruması).
        if existing is None:
            self.reservations.release_all_for(agv_id)
            self.reservations.reserve_node(start, agv_id)
            self.idle_positions.pop(agv_id, None)

        path = self._plan_path(start, goal, agv_id)
        if path is None:
            path = [start]   # ulaşılamaz; her tick replan denenir

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
        """Tek bir AGV'nin görevini (aktif + kuyruk) iptal eder. Diğer araçlara
        dokunmaz — takılan tek aracı güvenle durdurmak için. İptal olduysa True."""
        had = (agv_id in self.missions
               or any(q.agv_id == agv_id for q in self.queued))
        # İptal edilse de araç sahada; durduğu node'u engel olarak tut.
        m = self.missions.get(agv_id)
        parked = m.pos if m is not None else self.last_known_pos.get(agv_id)
        self.reservations.release_all_for(agv_id)
        self.missions.pop(agv_id, None)
        self.queued = [q for q in self.queued if q.agv_id != agv_id]
        self.disconnected_agvs.discard(agv_id)
        self.wait_streak.pop(agv_id, None)
        if parked and parked in self.graph.nodes:
            self.reservations.reserve_node(parked, agv_id)
            self.last_known_pos[agv_id] = parked
            self.idle_positions[agv_id] = parked
        else:
            self.last_known_pos.pop(agv_id, None)
            self.idle_positions.pop(agv_id, None)
        return had

    def _set_idle_position(self, agv_id: str, node: str) -> None:
        """Görevsiz aracın konumunu engel yap (rezerve + A* bloklu)."""
        old = self.idle_positions.get(agv_id)
        if old is not None and old != node:
            if self.reservations.node_owner.get(old) == agv_id:
                del self.reservations.node_owner[old]
        self.idle_positions[agv_id] = node
        self.reservations.reserve_node(node, agv_id)
        self.last_known_pos[agv_id] = node

    def set_agv_position(self, agv_id: str, node: str) -> None:
        """AGV'nin gerçek konumunu bildir. Path dışındaysa replan."""
        if node not in self.graph.nodes:   # '?' = bilinmeyen konum, atla
            return
        m = self.missions.get(agv_id)
        if m is None:
            self._set_idle_position(agv_id, node)   # görevsiz: konumu engel tut
            return
        self.idle_positions.pop(agv_id, None)
        if node != m.pos:
            m.inflight = None
            # Rezervasyonu yeni node'a taşı (yoksa çarpışma riski).
            if self.reservations.node_owner.get(m.pos) == agv_id:
                del self.reservations.node_owner[m.pos]
            self.reservations.reserve_node(node, agv_id)
        m.pos = node
        # Yan parkta park node off-path normal; yield_step yönetir.
        if m.state == MissionState.YIELDING:
            return
        if node not in m.path and m.goal:
            replan = self._plan_path(node, m.goal, agv_id)
            if replan and len(replan) > 1:
                m.path = replan

    def on_hop_complete(self, agv_id: str, new_node: str) -> None:
        """Hop bitince: pos güncelle, eski rezervasyonu bırak, gerekirse replan."""
        m = self.missions.get(agv_id)
        if m is None:
            return
        prev   = m.pos
        m.pos  = new_node
        m.inflight = None
        if prev != new_node:
            if self.reservations.node_owner.get(prev) == agv_id:
                del self.reservations.node_owner[prev]
            ekey = edge_key(prev, new_node)
            if self.reservations.edge_owner.get(ekey) == agv_id:
                del self.reservations.edge_owner[ekey]
            # Varılan node'u garantiye al: normalde dispatch'te rezerveydi ama
            # drift olduysa olmayabilir. Araç burada, kimse üstüne sürülmesin.
            self.reservations.reserve_node(new_node, agv_id)
        if m.is_done:
            self._finish_mission(agv_id)
            return
        # YIELDING'te park node off-path normaldir; yield_step yonetir.
        if m.state == MissionState.YIELDING:
            return
        # Pos path disinda → replan (firmware reroute reddetti veya drift)
        if new_node not in m.path and m.goal:
            replan = self._plan_path(new_node, m.goal, agv_id)
            if replan and len(replan) > 1:
                m.path = replan

    def _finish_mission(self, agv_id: str) -> None:
        """Görev bitti: parked (DONE). Kuyrukta görev varsa aktive et."""
        m = self.missions.get(agv_id)
        if m is None:
            return
        m.state = MissionState.DONE
        for e in [e for e, o in self.reservations.edge_owner.items()
                  if o == agv_id]:
            del self.reservations.edge_owner[e]
        self.reservations.reserve_node(m.pos, agv_id)   # parked engel
        nxt = next((q for q in self.queued if q.agv_id == agv_id), None)
        if nxt:
            self.queued.remove(nxt)
            self.reservations.release_all_for(agv_id)
            self.reservations.reserve_node(m.pos, agv_id)   # durduğu node'u koru
            nxt.pos   = m.pos
            nxt.start = m.pos
            self.missions[agv_id] = nxt

    # -----------------------------------------------------------------------
    # Tick — ana planlama dongusu
    # -----------------------------------------------------------------------

    def tick(self) -> Dict[str, HopCommand]:
        """Her aktif AGV için HopCommand üretir. Kopuklar atlanır."""
        commands: Dict[str, HopCommand] = {}
        for m in self._prioritize_missions():
            commands[m.agv_id] = self._plan_hop(m)
        self._update_wait_streak(commands)
        return commands

    def _update_wait_streak(self, commands: Dict[str, "HopCommand"]) -> None:
        """WAIT sayacını güncelle (öncelik + deadlock tespiti için)."""
        for agv_id, c in commands.items():
            if c.action == HopAction.WAIT:
                self.wait_streak[agv_id] = self.wait_streak.get(agv_id, 0) + 1
            else:
                self.wait_streak[agv_id] = 0

    def _priority_key(self, m: Mission):
        """Sıralama (küçük = yüksek öncelik): kargo > FIFO > ID. Uzun bekleyen
        öncelik kazanır."""
        aging = max(0, self.wait_streak.get(m.agv_id, 0) - self._STARVE_THRESHOLD)
        return (not m.has_cargo, m.fifo_order - aging, m.agv_id)

    def _prioritize_missions(self) -> List[Mission]:
        """Kargo > FIFO > ID. Kopuk/biten atlanır (yan park eden hariç). Yan park
        edenler tick'te en sona — önce sıkışan replan etsin, yield eden doğru beklesin."""
        active = [m for m in self.missions.values()
                  if (not m.is_done or m.state == MissionState.YIELDING)
                  and m.agv_id not in self.disconnected_agvs]
        return sorted(active,
                      key=lambda m: ((m.state == MissionState.YIELDING),)
                                    + tuple(self._priority_key(m)))

    # -----------------------------------------------------------------------
    # Deadlock tespiti + kurtarma (sadece watchdog çağırır, normal tick'i bozmaz)
    # -----------------------------------------------------------------------

    def detect_deadlock(self, threshold: int = 6) -> bool:
        """Aktif görevlerin hepsi threshold tick'tir bekliyorsa deadlock."""
        active = [m for m in self.missions.values()
                  if not m.is_done and m.agv_id not in self.disconnected_agvs]
        if not active:
            return False
        return all(self.wait_streak.get(m.agv_id, 0) >= threshold
                   for m in active)

    def break_deadlock(self) -> Optional["HopCommand"]:
        """Düşük öncelikliyi yana çektirip yol aç. Yer yoksa None (manuel)."""
        active = [m for m in self.missions.values()
                  if not m.is_done and m.agv_id not in self.disconnected_agvs]
        if not active:
            return None
        # 1) Tek-hop eviction: DONE blocker'ı komşuya çek (generic yield'den önce).
        for m in sorted(active, key=self._priority_key):
            yc = self._evict_blocker(m, max_hops=1)
            if yc is not None:
                return yc
        # 2) Generic yield: düşük öncelikliyi kazanana çek.
        if len(active) >= 2:
            ordered = sorted(active, key=self._priority_key)
            winner  = ordered[0]
            victims = [m for m in reversed(ordered[1:])
                       if m.state != MissionState.YIELDING]
            if victims:
                yc = self._begin_yield(victims[0], winner)
                if yc is not None:
                    self.wait_streak[victims[0].agv_id] = 0
                    return yc
        # 3) Son çare: çok-hop eviction (blocker'ı birkaç hop öteye).
        for m in sorted(active, key=self._priority_key):
            yc = self._evict_blocker(m, max_hops=None)
            if yc is not None:
                return yc
        return None

    def _evict_blocker(
        self, m: Mission, max_hops: Optional[int] = None,
    ) -> Optional[HopCommand]:
        """DONE bir araç yolu tıkıyorsa onu geçici kenara çek (sıkışan geçince
        döner). max_hops=1 sadece komşu, None birkaç hop öteye."""
        if m.is_done or not m.goal or m.pos == m.goal:
            return None
        blockers = [b for b in self.missions.values()
                    if b.state == MissionState.DONE and b.agv_id != m.agv_id
                    and b.agv_id not in self.disconnected_agvs and b.pos]
        if not blockers:
            return None
        done_pos = {b.pos for b in blockers}
        for b in blockers:
            # b kaldirilinca (diger DONE'lar bloklu) m'nin yolu aciliyor mu?
            others = done_pos - {b.pos}
            path = self.graph.astar(m.pos, m.goal, blocked_nodes=others)
            if not path or len(path) <= 1 or b.pos not in path:
                continue
            # b'yi acilan yolun disindaki bir node'a cek.
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
        """Blocker'ı yolun dışındaki en yakın boş node'a götüren rotayı bul."""
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
        """Sonraki hop: bitti→DONE, yan park→yield_step, yolda→aynı komut,
        çakışma→çöz, yer yok→WAIT, yoksa normal hop."""
        if m.is_done:
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.DONE,
                from_=m.current, reason="mission complete",
            )

        # YIELDING: park'a vardiysa diger AGV'yi bekle, gectiyse don.
        if m.state == MissionState.YIELDING:
            return self._plan_yield_step(m)

        # Araç bu hop'u sürüyor; kararı değiştirme, aynı komutu tekrar gönder.
        if m.inflight is not None and m.pos == m.inflight[0]:
            nxt = m.inflight[1]
            after = None
            if m.pos in m.path:
                i = m.path.index(m.pos)
                if i + 1 < len(m.path) and m.path[i + 1] == nxt:
                    # after'ı ancak biz tutuyorsak taşı, başkası tutuyorsa kes.
                    if i + 2 < len(m.path):
                        cand = m.path[i + 2]
                        owner = self.reservations.node_owner.get(cand)
                        if owner is None or owner == m.agv_id:
                            after = cand
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.NORMAL,
                from_=m.pos, next_=nxt, after_=after, after2_=None,
                reason="enroute (reissue)",
            )

        # Replan: path tek node (ulaşılamaz) ya da pos path dışında.
        if (len(m.path) <= 1 or m.pos not in m.path) and m.pos != m.goal:
            replan = self._plan_path(m.pos, m.goal, m.agv_id)
            if replan and len(replan) > 1:
                m.path = replan

        conflict = self._detect_conflict(m)
        if conflict.type != ConflictType.NONE:
            return self._resolve_conflict(m, conflict)

        if m.next_node is None:
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.WAIT,
                from_=m.current, reason="no reachable path",
            )

        return self._normal_hop(m)

    def _normal_hop(self, m: Mission, reason: str = "normal",
                    send_after: bool = True) -> HopCommand:
        """Hop dispatch. Sert kilit: A→B'de hem A hem B kilitli, A ancak varınca
        (on_hop_complete) bırakılır — yoksa başka araç A'ya girip çarpar."""
        if m.next_node is None:
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.WAIT,
                from_=m.current, reason="no next",
            )
        # Dolu node'a sürme, bekle (çarpışma).
        owner = self.reservations.node_owner.get(m.next_node)
        if owner is not None and owner != m.agv_id:
            return HopCommand(
                agv_id=m.agv_id, action=HopAction.WAIT, from_=m.current,
                reason=f"next {m.next_node} dolu ({owner}), bekle",
            )
        # current'i tut, varınca bırakılır (çarpışma koruması).
        self.reservations.reserve_node(m.current, m.agv_id)
        self.reservations.reserve_node(m.next_node, m.agv_id)
        self.reservations.reserve_edge(m.current, m.next_node, m.agv_id)
        # Gönderdiğimiz after'ı da rezerve et (firmware buffer'layıp sürüyor);
        # dolu ise gönderme, araç next'te bekler. send_after=False: after gönderme.
        after = m.after_node if send_after else None
        if after is not None:
            owner = self.reservations.node_owner.get(after)
            if owner is not None and owner != m.agv_id:
                after = None
            else:
                self.reservations.reserve_node(after, m.agv_id)
                self.reservations.reserve_edge(m.next_node, after, m.agv_id)
        m.state = MissionState.ACTIVE
        m.inflight = (m.current, m.next_node)
        return HopCommand(
            agv_id = m.agv_id,
            action = HopAction.NORMAL,
            from_  = m.current,
            next_  = m.next_node,
            after_ = after,
            after2_ = None,
            reason = reason,
        )

    # -----------------------------------------------------------------------
    # Conflict detection
    # -----------------------------------------------------------------------

    def _detect_conflict(self, me: Mission) -> Conflict:
        """Çakışma tespiti. Tipler:
          - VERTEX parked: duran araç tam next'imizde
          - VERTEX active: next'imiz diğerinin next/after'ında
          - EDGE_SWAP direct: A→B'ye karşı B→A (komşu, kafa kafaya)
          - EDGE_SWAP future: yollar 3-4 hop ileride aynı edge'i ters yön kullanıyor
        """
        if me.next_node is None:
            return Conflict(ConflictType.NONE, me.agv_id, "", "")

        # 1) next'imizde duran araç (DONE/yield) — üstüne sürülemez.
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

            # VERTEX: next'imiz diğerinin next/after'ında (90° koruması).
            other_future = {n for n in (other.next_node, other.after_node) if n}
            if me.next_node in other_future:
                return Conflict(
                    ConflictType.VERTEX, me.agv_id, other.agv_id, me.next_node,
                    detail=f"both target {me.next_node} (other near-future)",
                )

            # EDGE_SWAP direct: A→B'ye karşı B→A (komşu, kafa kafaya).
            if (other.current == me.next_node
                    and other.next_node == me.current):
                return Conflict(
                    ConflictType.EDGE_SWAP, me.agv_id, other.agv_id,
                    edge_key(me.current, me.next_node),
                    detail=f"head-on {me.current}-{me.next_node}",
                )

            # Future head-on: yollar ileride aynı edge'i ters yön kullanıyor.
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
        """start'tan k tane (from,to) edge döndür."""
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
        """İki yol lookahead içinde ters yönde aynı edge'i kullanıyor mu."""
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

        # Kilitlenme (kaybeden kazananın yolunda oturuyor) → kenara çekilme.
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
            # Bitişik olan geçer (aynı-yön konvoy): next'im bu node, diğeri after'la
            # geliyorsa ben geçerim. Head-on'da kapalı (ileri-geri olmasın).
            if (other.next_node != me.next_node
                    and self._find_path_headon(me, other, lookahead=4) is None):
                return self._normal_hop(
                    me, reason=f"yakinlik: {me.next_node} bitisik, gec")
            # Gerçek tie (ikisinin de next'i aynı) → düşük öncelikli bekler.
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
            # Future head-on → çakışma edge'ini bloklayıp reroute.
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
            # Detour yok. Beklemek yerine sınıra ilerle (next diğerinin yolunda
            # değilse bir hop). Bitişik olunca yield/corridor çözer.
            if (me.next_node is not None
                    and not self._on_remaining_path(other, me.next_node)):
                # after gönderme: sınırın ötesine dalmasın.
                return self._normal_hop(
                    me, reason=f"future head-on {other.agv_id}: sinira ilerle",
                    send_after=False)
            return HopCommand(
                agv_id=me.agv_id, action=HopAction.WAIT,
                from_=me.current,
                reason=f"future head-on {other.agv_id}, no detour",
            )

        return HopCommand(
            agv_id=me.agv_id, action=HopAction.WAIT,
            from_=me.current,
            reason=f"yielding to {other.agv_id}",
        )

    # -----------------------------------------------------------------------
    # Kafa kafaya / koridor kilidi cozumu
    # -----------------------------------------------------------------------

    def _on_remaining_path(self, m: Mission, node: str) -> bool:
        """node, m'nin kalan yolunda mı (pos dahil)?"""
        if not node or m.pos not in m.path:
            return False
        return node in m.path[m.path.index(m.pos):]

    def _needs_corridor_resolution(
        self, me: Mission, other: Mission, conflict: Conflict,
    ) -> bool:
        """Kenara çekilme gerekli mi? (kaybeden kazananın yolunda + yan park yok)"""
        direct = (conflict.type == ConflictType.EDGE_SWAP
                  and other.current == me.next_node)
        # Aynı yön takipse çekilme gerekmez.
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
        """Kimin çekileceğini _headon_plan seçer (iki araç da aynı sonucu bulur).
        Çekilecek bensem çekilmeye başla; değilsem o geçene kadar bekle."""
        yielder, path = self._headon_plan(me, other)
        if yielder is None:
            # İki tarafa da yer yok (düz koridor); bekle, watchdog girer.
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
        # Diğeri çekiliyor, bekle.
        return HopCommand(
            agv_id=me.agv_id, action=HopAction.WAIT, from_=me.current,
            reason=f"head-on: {other.agv_id} cekiliyor, bekle",
        )

    def _headon_plan(
        self, a: Mission, b: Mission,
    ) -> Tuple[Optional[Mission], Optional[List[str]]]:
        """Kim nereye çekilir (sıra-bağımsız). Önce kaybeden yan park/refuge,
        olmazsa kazanan. Bulamazsa (None, None)."""
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
        """Diğerinin yolundan uzak en yakın geçişli node'a giden rota.
        Dönüş [me.current, ..., R]. Diğerinin şu anki yerine girmez."""
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
        # other'in su an durdugu node'lara cekilirken girme
        forbidden = {other.current}
        if other.inflight:
            forbidden.add(other.inflight[1])
        prev: Dict[str, Optional[str]] = {me.current: None}
        queue: deque = deque([me.current])
        while queue:
            cur = queue.popleft()
            # Çekilecek yer: diğerinin yolunda olmayan boş geçişli node (ya da hedefimiz).
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
        """Reroute dispatch: eski rezervasyonları bırak, current'i tut, yeni path'i
        ata. Hemen çakışıyorsa ve öncelik bizde değilse WAIT."""
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
        """a, b'den öncelikli mi? Kargo > FIFO > ID. Uzun bekleyen zamanla
        önceliği kazanır (aç kalmasın)."""
        return self._priority_key(a) < self._priority_key(b)

    def _find_alternative_path(
        self, me: Mission, other: Mission,
    ) -> Optional[List[str]]:
        """Parked engeller + diğerinin 2-hop ileri planını bloklayarak alternatif
        yol bul. Mevcut path ile aynıysa None."""
        blocked_e: Set[Tuple[str, str]] = set()
        # other'ın 2-hop ileri planını (edge'leri) blokla.
        if other.current and other.next_node:
            blocked_e.add(edge_key(other.current, other.next_node))
        if other.next_node and other.after_node:
            blocked_e.add(edge_key(other.next_node, other.after_node))
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
        """current'a komşu yan park node'u bul (diğerinin yolu/rezerve/next değil;
        düşük degree önce)."""
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
        # Kopuk araçların son konumunu da dışla (çarpışma).
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
            # Dead-end (degree-1) yan park = tuzak; geçişli node olmalı.
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
        """Aracı kenara çek. path verilirse çok-hop (ilk hop gönderilir, gerisini
        _plan_yield_step yürütür); park verilirse tek-hop; ikisi de yoksa komşudan
        seçilir. Yer yoksa None. Kazanan geçince hedefe yeniden planlanır."""
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
        # Sert kilit: current'i tut, ilk hop + edge rezerve.
        self.reservations.reserve_node(me.current, me.agv_id)
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
        """Yan parkta her tick: parka varmadıysa devam et; vardıysa diğeri geçince
        dön, geçmediyse bekle."""
        park = m.yield_park_node
        ret  = m.yield_return_node
        other = self.missions.get(m.yield_to) if m.yield_to else None

        if park is None or ret is None:
            return self._cancel_yield(m, "yield state corrupt")

        if m.pos != park:
            # Henüz parka varmadık; sıradaki hop'u gönder. Yoldan çıktıysa iptal.
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
                    # Sert kilit: current'i tut, sıradaki hop + edge rezerve.
                    self.reservations.reserve_node(m.pos, m.agv_id)
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
            # Yol verdiğimiz araç dönüş node'una park ettiyse parktan replan.
            if (other is not None and other.is_done
                    and other.current == ret):
                return self._resume_from_park(m)
            # Çok uzun beklersek yield'i bitir, parktan replan.
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
        """Diğer araç node'dan uzaklaştı mı? is_done tek başına yetmez (orada park
        etmiş olabilir); önce konuma bak."""
        if other is None:
            return True
        if other.current == node:        # hâlâ orada (park etmiş olsa bile)
            return False
        if other.is_done:
            return True
        # Koptuysa başka yerdeyse clear say.
        if other.agv_id in self.disconnected_agvs:
            return True
        if other.next_node == node:      # oraya geliyor
            return False
        if node in other.path and other.pos in other.path:
            idx = other.path.index(other.pos)
            if node in other.path[idx:]:
                return False
        return True

    def _cancel_yield(self, m: Mission, reason: str) -> HopCommand:
        """Yield iptal: park rezervasyonu bırak, pos tut, ACTIVE'e dön. WAIT döner."""
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
        """Yield bitti: parktan hedefe yeniden planla, normal akışı çalıştır."""
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
            m.path = [park]    # ulaşılamaz, her tick replan denenir
        return self._plan_hop(m)


class FleetSimulator:
    """Planner.tick() çıktısını araçlara uygular. Test ve simülasyon için;
    gerçekte bu işi WS executor (PC→AGV setHop) yapar."""

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
        """Hepsi bitene/deadlock olana kadar koş. Döner: done/deadlock/timeout."""
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

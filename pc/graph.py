"""
Graph + A* — waypoint grafında yol bulma (çift yönlü, maliyet = cm).

Heuristic ölçeklenmiş Euclidean (_h_scale = min(cost/euclid), admissible +
consistent). blocked_nodes/blocked_edges ile planner her tick A*'ı tekrar çağırır.
Format (pc/waypoints.json): {"nodes": {"A": {"x":0,"y":0}}, "edges": [{"from","to","cost"}]}.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from heapq import heappush, heappop
from typing import Dict, Iterable, List, Optional, Set, Tuple


@dataclass(frozen=True)
class Node:
    name: str
    x:    float
    y:    float


# Undirected: sıralı tuple'a normalize (A-B == B-A).
def edge_key(a: str, b: str) -> Tuple[str, str]:
    return (a, b) if a <= b else (b, a)


@dataclass
class Graph:
    nodes: Dict[str, Node]
    # adjacency: node -> [(neighbor, cost)]
    _adj:  Dict[str, List[Tuple[str, float]]] = field(default_factory=dict)
    # quick edge cost lookup, normalized key
    _ec:   Dict[Tuple[str, str], float]      = field(default_factory=dict)
    # heuristic ölçek: h = _h_scale * euclid (admissible + consistent).
    _h_scale: float = 1.0

    # ---- IO --------------------------------------------------------------
    @classmethod
    def from_json(cls, path: str) -> "Graph":
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
        nodes = {
            name: Node(name=name, x=float(d["x"]), y=float(d["y"]))
            for name, d in doc["nodes"].items()
        }
        # Firmware node adını tek char işler; çok-karakterli adı reddet.
        bad = [n for n in nodes if len(n) != 1]
        if bad:
            raise ValueError(
                f"Node adlari tek karakter olmali (firmware char[] siniri); "
                f"gecersiz: {bad}"
            )
        g = cls(nodes=nodes)
        for n in nodes:
            g._adj[n] = []
        for e in doc["edges"]:
            a, b, c = e["from"], e["to"], float(e["cost"])
            if a not in nodes or b not in nodes:
                raise ValueError(f"Edge {a}-{b} references unknown node")
            g._adj[a].append((b, c))
            g._adj[b].append((a, c))            # undirected
            g._ec[edge_key(a, b)] = c
        # Heuristic ölçek: min(cost/euclid) — admissible + consistent.
        ratios = [
            c / g.euclidean(a, b)
            for (a, b), c in g._ec.items()
            if g.euclidean(a, b) > 1e-9
        ]
        g._h_scale = min(ratios) if ratios else 1.0
        return g

    # ---- Erisim -----------------------------------------------------------
    def neighbors(self, n: str) -> List[Tuple[str, float]]:
        return self._adj.get(n, [])

    def edge_cost(self, a: str, b: str) -> Optional[float]:
        return self._ec.get(edge_key(a, b))

    def euclidean(self, a: str, b: str) -> float:
        na, nb = self.nodes[a], self.nodes[b]
        return math.sqrt((na.x - nb.x) ** 2 + (na.y - nb.y) ** 2)

    # ---- A* ---------------------------------------------------------------
    def _heuristic(self, a: str, b: str) -> float:
        """Olceklenmis Euclidean (admissible + consistent). Bkz. _h_scale."""
        return self._h_scale * self.euclidean(a, b)

    def astar(
        self,
        start: str,
        goal:  str,
        blocked_nodes: Optional[Iterable[str]]              = None,
        blocked_edges: Optional[Iterable[Tuple[str, str]]]  = None,
        heuristic: bool = True,
        edge_penalty: Optional[Dict[Tuple[str, str], float]] = None,
    ) -> Optional[List[str]]:
        """En kisa yolu node isimleri listesi olarak dondur. Yol yoksa None.

        - blocked_nodes: bu node'lar gozardi edilir (passing yasak)
        - blocked_edges: bu kenarlar gozardi edilir; tuple yon-bagimsiz
          normalize edilir (edge_key)
        - heuristic=False: Dijkstra (h=0, optimal garantili)
        - edge_penalty: opsiyonel yumusak ek maliyet (edge_key -> cm). Hard
          block degil; A* mumkunse kacinir. Bos/None ise davranis degismez.
        """
        if start not in self.nodes:
            return None
        if goal not in self.nodes:
            return None

        bn: Set[str]                  = set(blocked_nodes or [])
        be: Set[Tuple[str, str]]      = {edge_key(*e) for e in (blocked_edges or [])}
        ep: Dict[Tuple[str, str], float] = (
            {edge_key(*k): v for k, v in edge_penalty.items()}
            if edge_penalty else {}
        )

        # start == goal kabul; goal bloklu ise yol yok
        if start == goal:
            return [start]
        if goal in bn:
            return None
        # Start bloklu olsa bile başlangıç geçerli (oradayız).

        # f_score min-heap. counter tie-breaker (deterministik siralama).
        open_heap: List[Tuple[float, int, str]] = []
        g_score:   Dict[str, float]            = {start: 0.0}
        came_from: Dict[str, str]              = {}
        counter    = 0
        h0         = self._heuristic(start, goal) if heuristic else 0.0
        heappush(open_heap, (h0, counter, start))

        closed: Set[str] = set()

        while open_heap:
            _, _, current = heappop(open_heap)

            if current == goal:
                return self._reconstruct(came_from, current)

            if current in closed:
                continue
            closed.add(current)

            for neighbor, cost in self._adj.get(current, []):
                if neighbor in bn and neighbor != goal:
                    # goal bloklu degil; ara dugumler bloklu olabilir
                    continue
                ekey = edge_key(current, neighbor)
                if ekey in be:
                    continue

                tentative = g_score[current] + cost + ep.get(ekey, 0.0)
                if tentative < g_score.get(neighbor, float("inf")):
                    g_score[neighbor]   = tentative
                    came_from[neighbor] = current
                    h = self._heuristic(neighbor, goal) if heuristic else 0.0
                    counter += 1
                    heappush(open_heap, (tentative + h, counter, neighbor))

        return None  # ulasilamaz

    # ---- Path utilities ---------------------------------------------------
    @staticmethod
    def _reconstruct(came_from: Dict[str, str], end: str) -> List[str]:
        path = [end]
        while end in came_from:
            end = came_from[end]
            path.append(end)
        path.reverse()
        return path

    def path_cost(self, path: List[str]) -> float:
        """Verilen path'in toplam maliyeti. Path eksik veya geçersiz ise inf."""
        if not path or len(path) < 2:
            return 0.0
        total = 0.0
        for a, b in zip(path, path[1:]):
            c = self.edge_cost(a, b)
            if c is None:
                return float("inf")
            total += c
        return total

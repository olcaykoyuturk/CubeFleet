"""
Kup Deposu (cube_store) — 4 fiziksel kupun saha konumu.

Kullanim akisi (kullanici karari):
  - 4 kup, baslangicta yeri NULL. Kullanici harita UI'sinden node + yon (side)
    isaretleyerek yerlestirir ("ilk basta ben koyuyorum").
  - Tasima sirasinda SISTEM gunceller ("isler sende sonra"): AGV kupu kapinca
    carrier=AGV_ID + node/side=None; birakinca node/side=birakilan yer +
    carrier=None.

Side semantigi: kup, node'un cizgisiz kenarina 90° acida konur. Gecerli yonler
= node'un kenari OLMAYAN yonleri (free_sides). Or. F'nin N/S komsusu var (C/I)
ama E/W bos → kup yalniz E veya W'ye konabilir; AGV faceDir ile o yone doner.

Koordinat konvansiyonu (waypoints.json + firmware waypoint_map.h ile ayni):
  KUZEY = -y (A,B,C tarafi)   GUNEY = +y (G,H,I tarafi)
  DOGU  = +x (C,F,I tarafi)   BATI  = -x (A,D,G tarafi)

Format (pc/cubes.json — runtime'da olusur, gitignore):

    {
      "cubes": {
        "1": {"node": "F", "side": "E", "carrier": null},
        "2": {"node": null, "side": null, "carrier": "AGV_1"},
        ...
      }
    }
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

from graph import Graph

CUBE_COUNT = 4
SIDES = ("N", "E", "S", "W")

# Firmware faceComplete heading string'i → tek harf yon
HEADING_STR_TO_SIDE = {"KUZEY": "N", "DOGU": "E", "GUNEY": "S", "BATI": "W"}
SIDE_LABELS_TR = {"N": "Kuzey", "E": "Dogu", "S": "Guney", "W": "Bati"}


def heading_between(g: Graph, a: str, b: str) -> Optional[str]:
    """a'dan b'ye baskin eksen yonu ('N'/'E'/'S'/'W'). Koordinatlar cm,
    y asagi (guney) artar — firmware waypoint_map.h konvansiyonu."""
    na, nb = g.nodes.get(a), g.nodes.get(b)
    if na is None or nb is None:
        return None
    dx, dy = nb.x - na.x, nb.y - na.y
    if abs(dx) >= abs(dy):
        return "E" if dx >= 0 else "W"
    return "S" if dy >= 0 else "N"


def free_sides(g: Graph, node: str) -> List[str]:
    """Node'un kenari OLMAYAN yonleri — kup yalniz buralara konabilir.
    Kenarli yonlerde kup yolun ustunde olurdu."""
    if node not in g.nodes:
        return []
    used = {heading_between(g, node, nb) for nb, _ in g.neighbors(node)}
    return [s for s in SIDES if s not in used]


@dataclass
class Cube:
    cube_id: int
    node:    Optional[str] = None   # bulundugu waypoint ('A'-'I'), tasimada None
    side:    Optional[str] = None   # node'un hangi kenarinda ('N'/'E'/'S'/'W')
    carrier: Optional[str] = None   # tasiyan AGV id'si (tasimada dolu)

    @property
    def placed(self) -> bool:
        return self.node is not None and self.side is not None

    @property
    def carried(self) -> bool:
        return self.carrier is not None

    def to_dict(self) -> dict:
        return {"node": self.node, "side": self.side, "carrier": self.carrier}

    @classmethod
    def from_dict(cls, cube_id: int, d: dict) -> "Cube":
        return cls(
            cube_id = cube_id,
            node    = d.get("node"),
            side    = d.get("side"),
            carrier = d.get("carrier"),
        )


class CubeStore:
    """JSON tabanli kup konumu persistence (atomik save)."""

    def __init__(self, json_path: str):
        self.path: str = json_path
        self.cubes: Dict[int, Cube] = {
            i: Cube(cube_id=i) for i in range(1, CUBE_COUNT + 1)
        }
        self.load()

    # ---- IO ---------------------------------------------------------------
    def load(self) -> None:
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                doc = json.load(f)
        except (OSError, json.JSONDecodeError):
            return
        for key, d in (doc.get("cubes") or {}).items():
            try:
                cid = int(key)
            except ValueError:
                continue
            if 1 <= cid <= CUBE_COUNT and isinstance(d, dict):
                self.cubes[cid] = Cube.from_dict(cid, d)

    def save(self) -> None:
        doc = {"cubes": {str(i): c.to_dict() for i, c in sorted(self.cubes.items())}}
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=2, ensure_ascii=False)
        os.replace(tmp, self.path)

    # ---- Konum guncellemeleri ----------------------------------------------
    def place(self, cube_id: int, node: str, side: str) -> Cube:
        """Kupu sahaya koy (kullanici yerlestirmesi veya AGV birakmasi)."""
        c = self.cubes[cube_id]
        c.node, c.side, c.carrier = node, side, None
        self.save()
        return c

    def pick_up(self, cube_id: int, agv_id: str) -> Cube:
        """AGV kupu kapti — artik node'da degil, AGV uzerinde."""
        c = self.cubes[cube_id]
        c.node, c.side, c.carrier = None, None, agv_id
        self.save()
        return c

    def drop(self, cube_id: int, node: str, side: str) -> Cube:
        """AGV kupu birakti — yeni saha konumu kaydedilir."""
        return self.place(cube_id, node, side)

    def clear(self, cube_id: int) -> Cube:
        """Kupu haritadan kaldir (yeri yeniden NULL)."""
        c = self.cubes[cube_id]
        c.node, c.side, c.carrier = None, None, None
        self.save()
        return c

    # ---- Sorgulama ----------------------------------------------------------
    def cube_at(self, node: str, side: Optional[str] = None) -> Optional[Cube]:
        """Node'da (istege bagli belirli kenarda) duran kup."""
        for c in self.cubes.values():
            if c.node == node and (side is None or c.side == side):
                return c
        return None

    def carried_by(self, agv_id: str) -> Optional[Cube]:
        for c in self.cubes.values():
            if c.carrier == agv_id:
                return c
        return None

    def placed_cubes(self) -> List[Cube]:
        return [c for c in self.cubes.values() if c.placed]

    def unplaced_cubes(self) -> List[Cube]:
        return [c for c in self.cubes.values() if not c.placed and not c.carried]

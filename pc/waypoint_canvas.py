"""
Waypoint Harita Widget
======================
A-I waypoint grafini Tkinter Canvas uzerinde cizer.
- Mevcut konum yesil dolu daire
- Hedef kirmizi halka
- Yol sari kalin kenar
- Aracin yonu ortada ok
- Bir node'a tiklayinca callback ile bildirir (hedef ayarlamak icin)

Multi-AGV gosterimi icin `set_fleet_state(agvs)` API mevcut.

Kup katmani:
- `set_cubes([(cube_id, node, side), ...])` — sahadaki kupleri turuncu kare
  olarak node'un side kenarinda gosterir (her iki cizim modunda).
- `begin_placement(get_sides_fn, on_pick)` — kup yerlestirme modu: kullanici
  once node tiklar, sonra cizgisiz kenarlardan birini secer;
  on_pick(node, side) cagrilir. `cancel_placement()` iptal eder.

Grid (12 kenar — 2026-06 4×3 A-L duzeni):
    A───B───C───D
    │       │
    E───F───G───H
        │
    I───J───K───L
"""

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple
import customtkinter as ctk
import tkinter as tk


# Waypoint koordinatlari (kanvas uzerinde mantiki konum, 0-1 normalize)
# (x, y) sol-ust koseden, x sag, y asagi. 4×3 grid (A-L).
_WAYPOINTS = {
    "A": (0.12, 0.20), "B": (0.37, 0.20), "C": (0.62, 0.20), "D": (0.87, 0.20),
    "E": (0.12, 0.50), "F": (0.37, 0.50), "G": (0.62, 0.50), "H": (0.87, 0.50),
    "I": (0.12, 0.80), "J": (0.37, 0.80), "K": (0.62, 0.80), "L": (0.87, 0.80),
}

# Komsuluklar (waypoint_map.h ile ayni — 2026-06 4×3 A-L duzeni, 12 kenar)
_EDGES: List[Tuple[str, str]] = [
    ("A", "B"), ("B", "C"), ("C", "D"),    # ust sira
    ("E", "F"), ("F", "G"), ("G", "H"),    # orta sira
    ("I", "J"), ("J", "K"), ("K", "L"),    # alt sira
    ("A", "E"), ("C", "G"), ("F", "J"),    # dikey
]

# Yon harfleri → kanvas birim vektoru (y asagi = guney; firmware konvansiyonu)
_SIDE_VEC: Dict[str, Tuple[int, int]] = {
    "N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0),
}
_SIDE_TR = {"N": "K", "E": "D", "S": "G", "W": "B"}   # kisa Turkce etiket
_CUBE_COLOR = "#ff9f1a"


# Multi-AGV gosterimi icin per-AGV durum nesnesi
@dataclass
class FleetAGVDisplay:
    agv_id:    str
    pos:       str
    path:      List[str]      = field(default_factory=list)
    color:     str             = "#3fbf66"   # AGV rengi
    state:     str             = "ACTIVE"
    heading:   str             = ""          # yon (KUZEY/EAST/SOUTH/...)
    is_active: bool            = False       # Operations'da secili olan
    target:    Optional[str]   = None        # hedef node (kirmizi halka)


class WaypointCanvas(ctk.CTkFrame):
    """A-I haritasini cizen, etkilesimli widget."""

    NODE_R   = 22
    BG_COLOR = "#1a1a1a"

    def __init__(self, master, on_target_click: Optional[Callable[[str], None]] = None,
                 width: int = 360, height: int = 360, **kw):
        super().__init__(master, **kw)
        self.on_target_click = on_target_click

        self.canvas = tk.Canvas(self, width=width, height=height,
                                bg=self.BG_COLOR, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=4, pady=4)
        self.canvas.bind("<Button-1>", self._on_click)
        self.canvas.bind("<Configure>", lambda e: self._redraw())

        # Durum (tek-AGV API, geriye doneuk uyumluluk)
        self.current_wp: str = ""
        self.target_wp:  str = ""
        self.path_wps:   List[str] = []
        self.heading:    str = ""
        # Multi-AGV API (planner viewer)
        self.fleet:      List[FleetAGVDisplay] = []
        # Kup katmani: [(cube_id, node, side), ...]
        self.cubes:      List[Tuple[int, str, str]] = []
        # Yerlestirme modu state'i
        self._place_get_sides: Optional[Callable[[str], List[str]]] = None
        self._place_on_pick:   Optional[Callable[[str, str], None]] = None
        self._place_node:      Optional[str] = None
        self._place_sides:     List[str] = []
        self._place_marker_px: Dict[str, Tuple[float, float]] = {}

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------
    def set_fleet_state(self, agvs: List[FleetAGVDisplay]):
        """Multi-AGV API (planner viewer / fleet dashboard).

        Her AGV kendi rengiyle gosterilir; path kendi rengiyle vurgulanir;
        aktif AGV'nin hedefi kirmizi halka ile vurgulanir.
        """
        self.fleet = list(agvs)
        # tek-AGV state'i temizle
        self.current_wp = ""
        self.target_wp  = ""
        self.path_wps   = []
        self.heading    = ""
        self._redraw()

    def set_cubes(self, cubes: List[Tuple[int, str, str]]):
        """Sahadaki kupler: [(cube_id, node, side), ...]. Tasinanlar dahil
        edilmez (cagiran filtreler)."""
        self.cubes = list(cubes)
        self._redraw()

    def begin_placement(self,
                        get_sides: Callable[[str], List[str]],
                        on_pick:   Callable[[str, str], None],
                        preselect: Optional[str] = None):
        """Kup yerlestirme modu: kullanici node tiklar → get_sides(node)'un
        dondurdugu kenarlardan birini secer → on_pick(node, side).
        preselect: node secimini atla (or. birakma — AGV'nin node'u sabit)."""
        self._place_get_sides = get_sides
        self._place_on_pick   = on_pick
        self._place_node      = None
        self._place_sides     = []
        if preselect is not None and preselect in _WAYPOINTS:
            sides = get_sides(preselect)
            if sides:
                self._place_node  = preselect
                self._place_sides = sides
        self._redraw()

    def cancel_placement(self):
        self._place_get_sides = None
        self._place_on_pick   = None
        self._place_node      = None
        self._place_sides     = []
        self._place_marker_px = {}
        self._redraw()

    @property
    def placing(self) -> bool:
        return self._place_on_pick is not None

    # -------------------------------------------------------------------------
    # Tiklama
    # -------------------------------------------------------------------------
    def _on_click(self, event):
        # Yerlestirme modu oncelikli: once kenar isaretcisi, sonra node secimi
        if self.placing:
            if self._place_node:
                for side, (mx, my) in self._place_marker_px.items():
                    if (event.x - mx) ** 2 + (event.y - my) ** 2 <= 14 ** 2:
                        node, cb = self._place_node, self._place_on_pick
                        self.cancel_placement()
                        if cb:
                            cb(node, side)
                        return
            wp = self._wp_at(event.x, event.y)
            if wp and self._place_get_sides:
                sides = self._place_get_sides(wp)
                if sides:
                    self._place_node  = wp
                    self._place_sides = sides
                self._redraw()
            return
        if self.on_target_click is None:
            return
        wp = self._wp_at(event.x, event.y)
        if wp:
            self.on_target_click(wp)

    def _wp_at(self, x: int, y: int) -> Optional[str]:
        w = max(self.canvas.winfo_width(),  10)
        h = max(self.canvas.winfo_height(), 10)
        for name, (nx, ny) in _WAYPOINTS.items():
            cx, cy = nx * w, ny * h
            if (x - cx) ** 2 + (y - cy) ** 2 <= self.NODE_R ** 2:
                return name
        return None

    # -------------------------------------------------------------------------
    # Cizim
    # -------------------------------------------------------------------------
    def _redraw(self):
        if self.fleet:
            self._redraw_fleet()
        else:
            self._redraw_single()

    def _redraw_single(self):
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(),  10)
        h = max(c.winfo_height(), 10)

        path_set = set()
        for i in range(len(self.path_wps) - 1):
            a, b = self.path_wps[i], self.path_wps[i + 1]
            path_set.add(frozenset([a, b]))

        for a, b in _EDGES:
            ax, ay = _WAYPOINTS[a]
            bx, by = _WAYPOINTS[b]
            x1, y1 = ax * w, ay * h
            x2, y2 = bx * w, by * h
            on_path = frozenset([a, b]) in path_set
            color = "#ffd54a" if on_path else "#666"
            width = 4 if on_path else 2
            c.create_line(x1, y1, x2, y2, fill=color, width=width, tags="edge")

        for name, (nx, ny) in _WAYPOINTS.items():
            cx, cy = nx * w, ny * h
            r = self.NODE_R

            if name == self.current_wp:
                fill, outline, ow = "#3fbf66", "#a0ffc0", 3
            elif name == self.target_wp:
                fill, outline, ow = "#3a3a3a", "#ff5050", 4
            elif name in (self.path_wps or []):
                fill, outline, ow = "#3a3a3a", "#ffd54a", 2
            else:
                fill, outline, ow = "#2a2a2a", "#888", 1

            c.create_oval(cx - r, cy - r, cx + r, cy + r,
                          fill=fill, outline=outline, width=ow, tags=("node", name))
            c.create_text(cx, cy, text=name, fill="white",
                          font=("Helvetica", 14, "bold"))

        if self.current_wp in _WAYPOINTS and self.heading in (
                "NORTH", "EAST", "SOUTH", "WEST",
                "KUZEY", "DOGU", "GUNEY", "BATI"):
            cx, cy = _WAYPOINTS[self.current_wp]
            cx, cy = cx * w, cy * h
            dx, dy = 0, 0
            if   self.heading in ("NORTH", "KUZEY"): dy = -1
            elif self.heading in ("EAST",  "DOGU"):  dx = 1
            elif self.heading in ("SOUTH", "GUNEY"): dy = 1
            elif self.heading in ("WEST",  "BATI"):  dx = -1
            L = self.NODE_R + 16
            c.create_line(cx, cy, cx + dx * L, cy + dy * L,
                          fill="#3fbf66", width=3, arrow=tk.LAST)

        self._draw_cubes(c, w, h)
        self._draw_placement(c, w, h)

        c.create_text(w - 8, h - 30, anchor="se", fill="#aaa", font=("Helvetica", 9),
                      text="● Konum   ◯ Hedef   ─ Yol")
        c.create_text(w - 8, h - 12, anchor="se", fill="#888", font=("Helvetica", 9),
                      text="Tiklayinca hedef ayarla")

    def _redraw_fleet(self):
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(),  10)
        h = max(c.winfo_height(), 10)

        # Once tum kenarlari gri ciz
        for a, b in _EDGES:
            ax, ay = _WAYPOINTS[a]
            bx, by = _WAYPOINTS[b]
            x1, y1 = ax * w, ay * h
            x2, y2 = bx * w, by * h
            c.create_line(x1, y1, x2, y2, fill="#555", width=2, tags="edge")

        # Sonra her AGV'nin path'ini kendi rengiyle vurgula
        for i, agv in enumerate(self.fleet):
            offset = (i - (len(self.fleet) - 1) / 2.0) * 4   # paralel cizgi kaymasi
            for k in range(len(agv.path) - 1):
                a, b = agv.path[k], agv.path[k + 1]
                if a not in _WAYPOINTS or b not in _WAYPOINTS:
                    continue
                ax, ay = _WAYPOINTS[a]
                bx, by = _WAYPOINTS[b]
                x1, y1 = ax * w, ay * h
                x2, y2 = bx * w, by * h
                # Kenara dik kucuk ofset (paralel hatlar ic ice gecmesin)
                ddx, ddy = x2 - x1, y2 - y1
                length = max((ddx * ddx + ddy * ddy) ** 0.5, 1.0)
                ox = -ddy / length * offset
                oy =  ddx / length * offset
                c.create_line(x1 + ox, y1 + oy, x2 + ox, y2 + oy,
                              fill=agv.color, width=3, tags="path")

        # Node'lar — pos sahibi/normal
        owner_at:  dict[str, FleetAGVDisplay] = {}
        target_at: dict[str, FleetAGVDisplay] = {}   # aktif AGV'nin hedefi
        for agv in self.fleet:
            if agv.pos:
                owner_at[agv.pos] = agv
            if agv.is_active and agv.target:
                target_at[agv.target] = agv

        for name, (nx, ny) in _WAYPOINTS.items():
            cx, cy = nx * w, ny * h
            r = self.NODE_R

            if name in owner_at:
                agv = owner_at[name]
                fill, outline, ow = agv.color, "#ffffff", 3
            else:
                fill, outline, ow = "#2a2a2a", "#888", 1

            c.create_oval(cx - r, cy - r, cx + r, cy + r,
                          fill=fill, outline=outline, width=ow,
                          tags=("node", name))
            c.create_text(cx, cy, text=name, fill="white",
                          font=("Helvetica", 14, "bold"))

            # Aktif AGV'nin hedefi: ek kirmizi halka (dis disinda)
            if name in target_at and name not in owner_at:
                c.create_oval(cx - r - 5, cy - r - 5, cx + r + 5, cy + r + 5,
                              outline="#ff5050", width=3, tags="target_ring")

            # AGV ID etiketi node'un altinda
            if name in owner_at:
                agv = owner_at[name]
                c.create_text(cx, cy + r + 12,
                              text=agv.agv_id, fill=agv.color,
                              font=("Helvetica", 9, "bold"))

        # Aktif AGV'nin heading ok'u
        for agv in self.fleet:
            if not (agv.is_active and agv.pos and agv.heading):
                continue
            if agv.pos not in _WAYPOINTS:
                continue
            heading = agv.heading.strip().upper()
            cx, cy = _WAYPOINTS[agv.pos]
            cx, cy = cx * w, cy * h
            dx, dy = 0, 0
            if   heading in ("NORTH", "KUZEY"): dy = -1
            elif heading in ("EAST",  "DOGU"):  dx =  1
            elif heading in ("SOUTH", "GUNEY"): dy =  1
            elif heading in ("WEST",  "BATI"):  dx = -1
            if dx or dy:
                L = self.NODE_R + 16
                c.create_line(cx, cy, cx + dx * L, cy + dy * L,
                              fill=agv.color, width=3, arrow=tk.LAST)

        self._draw_cubes(c, w, h)
        self._draw_placement(c, w, h)

        # Lejand
        c.create_text(8, h - 12, anchor="sw", fill="#888", font=("Helvetica", 9),
                      text=f"{len(self.fleet)} AGV  |  ● konum  ◯ hedef"
                           + ("  |  ■ kup" if self.cubes else ""))

    # -------------------------------------------------------------------------
    # Kup katmani + yerlestirme overlay'i (her iki cizim modunda ortak)
    # -------------------------------------------------------------------------
    def _draw_cubes(self, c, w, h):
        """Sahadaki kupleri node'un side kenarinda turuncu kare olarak ciz."""
        r = self.NODE_R
        for cube_id, node, side in self.cubes:
            if node not in _WAYPOINTS or side not in _SIDE_VEC:
                continue
            nx, ny = _WAYPOINTS[node]
            dx, dy = _SIDE_VEC[side]
            cx = nx * w + dx * (r + 14)
            cy = ny * h + dy * (r + 14)
            s = 8
            c.create_rectangle(cx - s, cy - s, cx + s, cy + s,
                               fill=_CUBE_COLOR, outline="#ffffff", width=1,
                               tags="cube")
            c.create_text(cx, cy, text=str(cube_id), fill="#1a1a1a",
                          font=("Helvetica", 9, "bold"))

    def _draw_placement(self, c, w, h):
        """Yerlestirme modunda secili node'u vurgula + secilebilir kenar
        isaretcilerini ciz. Isaretci merkezleri _place_marker_px'e yazilir
        (_on_click hit testi icin)."""
        self._place_marker_px = {}
        if not self.placing:
            return
        r = self.NODE_R
        if self._place_node is None:
            c.create_text(w / 2, 14, fill=_CUBE_COLOR,
                          font=("Helvetica", 11, "bold"),
                          text="KUP YERLESTIRME: bir node tikla")
            return
        nx, ny = _WAYPOINTS[self._place_node]
        cx, cy = nx * w, ny * h
        c.create_oval(cx - r - 6, cy - r - 6, cx + r + 6, cy + r + 6,
                      outline=_CUBE_COLOR, width=3, tags="place_sel")
        c.create_text(w / 2, 14, fill=_CUBE_COLOR,
                      font=("Helvetica", 11, "bold"),
                      text=f"{self._place_node}: kenar sec "
                           f"({'/'.join(_SIDE_TR[s] for s in self._place_sides)})")
        for side in self._place_sides:
            dx, dy = _SIDE_VEC.get(side, (0, 0))
            mx = cx + dx * (r + 22)
            my = cy + dy * (r + 22)
            self._place_marker_px[side] = (mx, my)
            s = 11
            c.create_rectangle(mx - s, my - s, mx + s, my + s,
                               fill="#3a3a3a", outline=_CUBE_COLOR, width=2,
                               tags="place_side")
            c.create_text(mx, my, text=_SIDE_TR[side], fill=_CUBE_COLOR,
                          font=("Helvetica", 10, "bold"))

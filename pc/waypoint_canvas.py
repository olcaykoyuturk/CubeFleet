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

Grid (10 kenar — 2026-05 cyclic guncellemesi):
    A───B───C
        │   │
    D───E   F
    │   │   │
    G───H───I
"""

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple
import customtkinter as ctk
import tkinter as tk


# Waypoint koordinatlari (kanvas uzerinde mantiki konum, 0-1 normalize)
# (x, y) sol-ust koseden, x sag, y asagi
_WAYPOINTS = {
    "A": (0.10, 0.20),
    "B": (0.40, 0.20),
    "C": (0.70, 0.20),
    "D": (0.10, 0.50),
    "E": (0.40, 0.50),
    "F": (0.70, 0.50),
    "G": (0.10, 0.80),
    "H": (0.40, 0.80),
    "I": (0.70, 0.80),
}

# Komsuluklar (waypoint_map.h ile ayni — G-H ve C-F 2026-05'te eklendi)
_EDGES: List[Tuple[str, str]] = [
    ("A", "B"), ("B", "C"),
    ("B", "E"), ("C", "F"),
    ("D", "E"),
    ("D", "G"),
    ("E", "H"),
    ("F", "I"),
    ("G", "H"), ("H", "I"),
]


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

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------
    def set_state(self, current: str = "", target: str = "",
                  path: str = "", heading: str = ""):
        """Tek-AGV API (geriye donuk)."""
        self.current_wp = (current or "").strip()
        self.target_wp  = (target or "").strip()
        self.heading    = (heading or "").strip().upper()

        # path "A>B>E>H" formatinda
        path = (path or "").strip()
        self.path_wps = [p.strip() for p in path.split(">") if p.strip()] if path else []
        # multi-AGV state'i temizle (mix kullanim olmasin)
        self.fleet = []
        self._redraw()

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

    # -------------------------------------------------------------------------
    # Tiklama
    # -------------------------------------------------------------------------
    def _on_click(self, event):
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

        # Lejand
        c.create_text(8, h - 12, anchor="sw", fill="#888", font=("Helvetica", 9),
                      text=f"{len(self.fleet)} AGV  |  ● konum  ◯ hedef")

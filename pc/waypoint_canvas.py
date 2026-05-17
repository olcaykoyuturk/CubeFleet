"""
Waypoint Harita Widget
======================
A-I waypoint grafini Tkinter Canvas uzerinde cizer.
- Mevcut konum yesil dolu daire
- Hedef kirmizi halka
- Yol sari kalin kenar
- Aracin yonu ortada ok
- Bir node'a tiklayinca callback ile bildirir (hedef ayarlamak icin)

Grid:
    A───B───C
        │
    D───E   F
    │   │   │
    G   H───I
"""

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

# Komsuluklar (waypoint_map.h ile ayni)
_EDGES: List[Tuple[str, str]] = [
    ("A", "B"), ("B", "C"),
    ("B", "E"),
    ("D", "E"),
    ("D", "G"),
    ("E", "H"),
    ("F", "I"),
    ("H", "I"),
]


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

        # Durum
        self.current_wp: str = ""
        self.target_wp:  str = ""
        self.path_wps:   List[str] = []
        self.heading:    str = ""

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------
    def set_state(self, current: str = "", target: str = "",
                  path: str = "", heading: str = ""):
        self.current_wp = (current or "").strip()
        self.target_wp  = (target or "").strip()
        self.heading    = (heading or "").strip().upper()

        # path "A>B>E>H" formatinda
        path = (path or "").strip()
        self.path_wps = [p.strip() for p in path.split(">") if p.strip()] if path else []

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
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(),  10)
        h = max(c.winfo_height(), 10)

        path_set = set()
        for i in range(len(self.path_wps) - 1):
            a, b = self.path_wps[i], self.path_wps[i + 1]
            path_set.add(frozenset([a, b]))

        # Kenarlar
        for a, b in _EDGES:
            ax, ay = _WAYPOINTS[a]
            bx, by = _WAYPOINTS[b]
            x1, y1 = ax * w, ay * h
            x2, y2 = bx * w, by * h
            on_path = frozenset([a, b]) in path_set
            color = "#ffd54a" if on_path else "#666"
            width = 4 if on_path else 2
            c.create_line(x1, y1, x2, y2, fill=color, width=width, tags="edge")

        # Node'lar
        for name, (nx, ny) in _WAYPOINTS.items():
            cx, cy = nx * w, ny * h
            r = self.NODE_R

            if name == self.current_wp:
                fill, outline, ow = "#3fbf66", "#a0ffc0", 3   # yesil dolu
            elif name == self.target_wp:
                fill, outline, ow = "#3a3a3a", "#ff5050", 4   # kirmizi halka
            elif name in (self.path_wps or []):
                fill, outline, ow = "#3a3a3a", "#ffd54a", 2   # yol uzerinde
            else:
                fill, outline, ow = "#2a2a2a", "#888", 1

            c.create_oval(cx - r, cy - r, cx + r, cy + r,
                          fill=fill, outline=outline, width=ow, tags=("node", name))
            c.create_text(cx, cy, text=name, fill="white",
                          font=("Helvetica", 14, "bold"))

        # Mevcut konumdaki yon oku
        if self.current_wp in _WAYPOINTS and self.heading in ("NORTH", "EAST", "SOUTH", "WEST", "KUZEY", "DOGU", "GUNEY", "BATI"):
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

        # Lejand sag-altta
        c.create_text(w - 8, h - 30, anchor="se", fill="#aaa", font=("Helvetica", 9),
                      text="● Konum   ◯ Hedef   ─ Yol")
        c.create_text(w - 8, h - 12, anchor="se", fill="#888", font=("Helvetica", 9),
                      text="Tiklayinca hedef ayarla")

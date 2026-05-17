"""
AGV WebSocket Client
====================
ESP32 AGV server'a baglanir, AGV durumlarini ve loglari ayri thread'te dinler.
UI thread'inden surekli `pop_events()` ile yeni olaylari al ve uygula.

Server protokolu (firmware/server/server.ino):
    PC -> Server:
        {type: "setPosition", agvId: "AGV_1", waypoint: "A"}
        {type: "setTarget",   agvId: "AGV_1", waypoint: "H"}
        {type: "setPID",      agvId: "AGV_1", Kp: 0.012, Ki: 0, Kd: 0.005}
        {type: "setSpeed",    agvId: "AGV_1", speed: 35}
        {type: "command",     agvId: "AGV_1", command: "start|stop|calibrate"}
        {type: "getList"}

    Server -> PC:
        {type: "agvList",   agvs: [{...}, ...]}
        {type: "agvUpdate", id, connected, currentWaypoint, targetWaypoint,
                            path, pathIdx, heading, navState, linePos,
                            Kp, Ki, Kd, baseSpeed, calibrated, isTarget,
                            sonarDist, obstacle, sensors[8]}
        {type: "log",       agvId, message, time}
"""

import json
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import websocket   # websocket-client


# =============================================================================
# AGV durum sinifi
# =============================================================================

@dataclass
class AGVState:
    id:              str   = ""
    connected:       bool  = False
    currentWaypoint: str   = ""
    targetWaypoint:  str   = ""
    path:            str   = ""
    pathIdx:         int   = 0
    heading:         str   = ""
    navState:        str   = ""
    linePos:         int   = 0
    Kp:              float = 0.0
    Ki:              float = 0.0
    Kd:              float = 0.0
    baseSpeed:       int   = 0
    calibrated:      bool  = False
    isTarget:        bool  = False
    sonarDist:       float = 999.0
    obstacle:        bool  = False
    sensors:         List[int] = field(default_factory=lambda: [0] * 8)
    last_update:     float = 0.0

    @classmethod
    def from_dict(cls, d: dict) -> "AGVState":
        s = cls(
            id              = d.get("id", ""),
            connected       = d.get("connected", False),
            currentWaypoint = d.get("currentWaypoint", ""),
            targetWaypoint  = d.get("targetWaypoint", ""),
            path            = d.get("path", ""),
            pathIdx         = d.get("pathIdx", 0),
            heading         = d.get("heading", ""),
            navState        = d.get("navState", ""),
            linePos         = d.get("linePos", 0),
            Kp              = d.get("Kp", 0.0),
            Ki              = d.get("Ki", 0.0),
            Kd              = d.get("Kd", 0.0),
            baseSpeed       = d.get("baseSpeed", 0),
            calibrated      = d.get("calibrated", False),
            isTarget        = d.get("isTarget", False),
            sonarDist       = d.get("sonarDist", 999.0),
            obstacle        = d.get("obstacle", False),
            sensors         = list(d.get("sensors", [0] * 8))[:8],
            last_update     = time.time(),
        )
        # Pad sensors to 8
        while len(s.sensors) < 8:
            s.sensors.append(0)
        return s


@dataclass
class LogEvent:
    agvId:   str
    message: str
    time_ms: int


# =============================================================================
# WebSocket Client
# =============================================================================

class AGVClient:
    """
    Server WS'sine baglanir. Olaylari `events` kuyruguna iter:
        ("connected",   None)
        ("disconnected", None)
        ("error", "<mesaj>")
        ("agv_list", {id: AGVState, ...})
        ("agv_update", AGVState)
        ("log", LogEvent)
    UI bunu surekli pop'lar.
    """

    def __init__(self, url: str):
        self.url     = url
        self.ws:     Optional[websocket.WebSocketApp] = None
        self.thread: Optional[threading.Thread]       = None
        self.events: "queue.Queue[tuple]"             = queue.Queue()
        self.connected = False
        self.agvs: Dict[str, AGVState] = {}
        self._stop_flag = threading.Event()

    # -------------------------------------------------------------------------
    # Yasam dongusu
    # -------------------------------------------------------------------------
    def start(self):
        self._stop_flag.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self._stop_flag.set()
        try:
            if self.ws is not None:
                self.ws.close()
        except Exception:
            pass
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def _run(self):
        while not self._stop_flag.is_set():
            try:
                self.ws = websocket.WebSocketApp(
                    self.url,
                    on_open    = self._on_open,
                    on_message = self._on_message,
                    on_error   = self._on_error,
                    on_close   = self._on_close,
                )
                # run_forever blokludur, ws.close() ile cikar
                self.ws.run_forever(ping_interval=10, ping_timeout=5)
            except Exception as e:
                self.events.put(("error", f"WS run hata: {e}"))

            if self._stop_flag.is_set():
                break

            # Yeniden baglan dene
            self.events.put(("error", "Yeniden baglaniliyor..."))
            time.sleep(2.0)

    # -------------------------------------------------------------------------
    # WS callback'leri (WS thread'inde calisir)
    # -------------------------------------------------------------------------
    def _on_open(self, ws):
        self.connected = True
        self.events.put(("connected", None))
        # Listeyi iste
        self.send_raw({"type": "getList"})

    def _on_close(self, ws, code, reason):
        self.connected = False
        self.events.put(("disconnected", None))

    def _on_error(self, ws, error):
        self.events.put(("error", str(error)))

    def _on_message(self, ws, message):
        try:
            doc = json.loads(message)
        except Exception:
            return

        t = doc.get("type", "")

        if t == "agvList":
            new_map: Dict[str, AGVState] = {}
            for d in doc.get("agvs", []):
                s = AGVState.from_dict(d)
                if s.id:
                    new_map[s.id] = s
            self.agvs = new_map
            self.events.put(("agv_list", dict(self.agvs)))

        elif t == "agvUpdate":
            s = AGVState.from_dict(doc)
            if s.id:
                self.agvs[s.id] = s
                self.events.put(("agv_update", s))

        elif t == "log":
            ev = LogEvent(
                agvId   = doc.get("agvId", ""),
                message = doc.get("message", ""),
                time_ms = int(doc.get("time", 0)),
            )
            self.events.put(("log", ev))

    # -------------------------------------------------------------------------
    # Mesaj gonderme (UI thread'inden cagrilir)
    # -------------------------------------------------------------------------
    def send_raw(self, payload: dict) -> bool:
        if not self.connected or self.ws is None:
            return False
        try:
            self.ws.send(json.dumps(payload))
            return True
        except Exception as e:
            self.events.put(("error", f"Gonderim hata: {e}"))
            return False

    # Komut yardimcilari
    def set_position(self, agv_id: str, waypoint: str) -> bool:
        return self.send_raw({"type": "setPosition", "agvId": agv_id, "waypoint": waypoint})

    def set_target(self, agv_id: str, waypoint: str) -> bool:
        return self.send_raw({"type": "setTarget", "agvId": agv_id, "waypoint": waypoint})

    def set_pid(self, agv_id: str, kp: float, ki: float, kd: float) -> bool:
        return self.send_raw({
            "type": "setPID", "agvId": agv_id,
            "Kp": kp, "Ki": ki, "Kd": kd,
        })

    def set_speed(self, agv_id: str, speed: int) -> bool:
        return self.send_raw({"type": "setSpeed", "agvId": agv_id, "speed": int(speed)})

    def command(self, agv_id: str, cmd: str) -> bool:
        # cmd: "start" | "stop" | "calibrate"
        return self.send_raw({"type": "command", "agvId": agv_id, "command": cmd})

    def request_list(self) -> bool:
        return self.send_raw({"type": "getList"})

    # -------------------------------------------------------------------------
    # UI tarafindan polling
    # -------------------------------------------------------------------------
    def pop_events(self, max_count: int = 64) -> List[tuple]:
        """UI tick'inde cagir, bu cagri sirasinda biriken olaylari dondurur."""
        out: List[tuple] = []
        for _ in range(max_count):
            try:
                out.append(self.events.get_nowait())
            except queue.Empty:
                break
        return out

"""
AGV WebSocket Client
====================
ESP32 AGV server'a baglanir, AGV durumlarini ve loglari ayri thread'te dinler.
UI thread'inden surekli `pop_events()` ile yeni olaylari al ve uygula.

Server protokolu (firmware/server/server.ino):
    PC -> Server:
        {type: "setPosition",      agvId: "AGV_1", waypoint: "A"}
        {type: "setHop",           agvId: "AGV_1", from: "A", next: "B",
                                   after: "E", goal: "I"}    # 2-hop emir
        {type: "clearMission",     agvId: "AGV_1"}
        {type: "setPID",           agvId: "AGV_1", Kp: 0.012, Ki: 0, Kd: 0.005}
        {type: "setSpeed",         agvId: "AGV_1", speed: 35}
        {type: "command",          agvId: "AGV_1", command: "start|stop|calibrate"}
        {type: "applyCalibration", agvId: "AGV_1", sensorMin: [...8], sensorMax: [...8]}
        {type: "pcHeartbeat"}      # her 1.5sn (firmware AGV disconnect tespiti)
        {type: "getList"}

    Server -> PC:
        {type: "agvList",   agvs: [{...}, ...]}
        {type: "agvUpdate", id, connected, currentWaypoint, targetWaypoint,
                            path, pathIdx, heading, navState, linePos,
                            Kp, Ki, Kd, baseSpeed, calibrated, isTarget,
                            sonarDist, obstacle, sensors[8]}
        {type: "log",             agvId, message, time}
        {type: "calibrationData", agvId, sensorMin: [...8], sensorMax: [...8]}
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
    obstacle:        bool  = False    # STOP zone (< OBSTACLE_STOP_CM, AGV durdu)
    sonarSlow:       bool  = False    # SLOW zone (< OBSTACLE_SLOW_CM, hiz dustu)
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
            sonarSlow       = d.get("sonarSlow", False),
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
    time_ms:   int     # server uptime ms (firmware'in zamani)
    wall_time: float = 0.0   # PC wall clock (time.time()) when received


@dataclass
class CalibrationDataEvent:
    """AGV manuel kalibrasyonu bitirdi, ham min/max degerlerini yolladi."""
    agvId:     str
    sensorMin: List[int]
    sensorMax: List[int]


@dataclass
class HopCompleteEvent:
    """AGV bir hop'u tamamladi, yeni node'a vardi. Planner bunu alip
    on_hop_complete cagirir → rezervasyon guncellenir."""
    agvId:     str
    node:      str
    heading:   str   = ""
    time_ms:   int   = 0       # AGV firmware uptime ms (yollarken)
    wall_time: float = 0.0     # PC wall clock (receive zamani)


# =============================================================================
# WebSocket Client
# =============================================================================

class AGVClient:
    """
    Server WS'sine baglanir. Olaylari `events` kuyruguna iter:
        ("connected",        None)
        ("disconnected",     None)
        ("error",            "<mesaj>")
        ("agv_list",         {id: AGVState, ...})
        ("agv_update",       AGVState)
        ("log",              LogEvent)
        ("calibration_data", CalibrationDataEvent)
        ("hop_complete",     HopCompleteEvent)
    UI bunu surekli pop'lar.
    """

    # PC -> AGV heartbeat periyodu (saniye). AGV firmware'i 3sn timeout
    # uyguluyor — 1.5sn periyot guvenli marjin saglar (1 paket kaybolsa bile
    # ikincisi yakalanir).
    HEARTBEAT_INTERVAL_S = 1.5

    def __init__(self, url: str):
        self.url     = url
        self.ws:     Optional[websocket.WebSocketApp] = None
        self.thread: Optional[threading.Thread]       = None
        self.heartbeat_thread: Optional[threading.Thread] = None
        self.events: "queue.Queue[tuple]"             = queue.Queue()
        self.connected = False
        self.agvs: Dict[str, AGVState] = {}
        self._stop_flag = threading.Event()
        # send_raw UI thread'inden, websocket-client'in kendi ping thread'i ile
        # ayni socket'e yazma yarisina girebilir. Lock ile serialize et.
        self._send_lock = threading.Lock()
        # debug_timing=True iken her WS mesaji "ws_debug" event olarak kuyruga
        # girer; UI log paneline yansir. Kapaliyken sifir overhead.
        self.debug_timing = False

    # -------------------------------------------------------------------------
    # Yasam dongusu
    # -------------------------------------------------------------------------
    def start(self):
        self._stop_flag.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        # PC heartbeat sender — firmware'in 3sn timeout'undan dususuk periyot.
        self.heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True,
        )
        self.heartbeat_thread.start()

    def stop(self):
        self._stop_flag.set()
        try:
            if self.ws is not None:
                self.ws.close()
        except Exception:
            pass
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.heartbeat_thread is not None:
            self.heartbeat_thread.join(timeout=1.0)

    def _heartbeat_loop(self):
        """PC -> AGV heartbeat. Server forward eder, AGV firmware'i alip
        lastPCActivityMs guncel tutar. Sn 1.5'lik periyot 3sn timeout'a marjin.
        Disconnect sirasinda send_raw zaten False doner — sessizce devam."""
        while not self._stop_flag.is_set():
            if self.connected:
                # Tip kontrolu firmware'de yapilir; tek field yeterli.
                self.send_raw({"type": "pcHeartbeat"})
            # _stop_flag.wait sleep'ten daha duyarli — disconnect anında break.
            if self._stop_flag.wait(self.HEARTBEAT_INTERVAL_S):
                break

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
                # run_forever blokludur, ws.close() ile cikar.
                # ping_interval=30, ping_timeout=15 → WiFi flicker'a tolerans.
                # Eski 10/5 ESP32 server'in burst broadcast (2 AGV × 2Hz status)
                # sirasinda PC ping'ine yetisemediginde false-positive disconnect
                # yapiyordu. Gercek disconnect yine 45sn icinde tespit edilir.
                self.ws.run_forever(ping_interval=30, ping_timeout=15)
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
        self._connect_time = time.time()
        self.events.put(("connected", None))
        # Listeyi iste
        self.send_raw({"type": "getList"})

    def _on_close(self, ws, code, reason):
        # Drop diagnostics: connection lifetime + close code + reason yardimcidir.
        # code/reason library tarafindan TCP/WS protocol seviyesinden gelir;
        # None ise abrupt TCP RST (genelde WiFi flicker veya server crash).
        lifetime = (time.time() - getattr(self, "_connect_time", time.time()))
        reason_s = reason if reason else "—"
        code_s   = str(code) if code is not None else "—"
        self.connected = False
        self.events.put((
            "error",
            f"WS kapandi: code={code_s} reason={reason_s} (lifetime {lifetime:.1f}s)",
        ))
        self.events.put(("disconnected", None))

    def _on_error(self, ws, error):
        self.events.put(("error", f"WS error: {type(error).__name__}: {error}"))

    def _on_message(self, ws, message):
        try:
            doc = json.loads(message)
        except Exception:
            return

        t = doc.get("type", "")

        # DEBUG: Her gelen mesaj icin timing logu. Status broadcast (agvUpdate)
        # cok sik geldigi icin sadece "ilgilenilen" tipleri logla — yoksa spam.
        if self.debug_timing and t in ("hopComplete", "log", "calibrationData",
                                       "agvList"):
            t_fw = int(doc.get("time", 0))
            aid  = doc.get("agvId") or doc.get("id", "")
            extras = ""
            if t == "hopComplete":
                extras = f" node={doc.get('node','?')}"
            elif t == "log":
                msg = doc.get("message", "")[:40]
                extras = f" \"{msg}\""
            self.events.put((
                "ws_debug",
                f"RX {t} agv={aid or '-'} fw_ms={t_fw}{extras}"
                f" (size={len(message)}B)",
            ))

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
                agvId     = doc.get("agvId", ""),
                message   = doc.get("message", ""),
                time_ms   = int(doc.get("time", 0)),
                wall_time = time.time(),
            )
            self.events.put(("log", ev))

        elif t == "calibrationData":
            sensor_min = list(doc.get("sensorMin", []))[:8]
            sensor_max = list(doc.get("sensorMax", []))[:8]
            ev = CalibrationDataEvent(
                agvId     = doc.get("agvId", ""),
                sensorMin = [int(v) for v in sensor_min],
                sensorMax = [int(v) for v in sensor_max],
            )
            self.events.put(("calibration_data", ev))

        elif t == "hopComplete":
            ev = HopCompleteEvent(
                agvId     = doc.get("agvId", ""),
                node      = str(doc.get("node", "")),
                heading   = str(doc.get("heading", "")),
                time_ms   = int(doc.get("time", 0)),
                wall_time = time.time(),
            )
            self.events.put(("hop_complete", ev))

    # -------------------------------------------------------------------------
    # Mesaj gonderme (UI thread'inden cagrilir)
    # -------------------------------------------------------------------------
    def send_raw(self, payload: dict) -> bool:
        if not self.connected or self.ws is None:
            return False
        # websocket-client'in run_forever() icindeki ping thread'i ile UI
        # thread'inden gelen send cagrilari ayni TCP socket'a yaziyor olabilir;
        # WS frame corruption olmasin diye serialize et.
        t0 = time.time() if self.debug_timing else 0.0
        with self._send_lock:
            try:
                blob = json.dumps(payload)
                self.ws.send(blob)
                # DEBUG: pcHeartbeat'i loglama — 1.5sn'de bir gelir, gurultu.
                if self.debug_timing and payload.get("type") != "pcHeartbeat":
                    dt = (time.time() - t0) * 1000
                    ptype = payload.get("type", "?")
                    aid   = payload.get("agvId", "")
                    # setHop icin detayli, digerleri sadece tip+agv
                    extras = ""
                    if ptype == "setHop":
                        extras = (f" {payload.get('from','?')}>{payload.get('next','?')}"
                                  f">{payload.get('after','-') or '-'}"
                                  f" goal={payload.get('goal','-')}")
                    self.events.put((
                        "ws_debug",
                        f"TX {ptype} agv={aid or '-'}{extras}"
                        f" (size={len(blob)}B, send_ms={dt:.1f})",
                    ))
                return True
            except Exception as e:
                self.events.put(("error", f"Gonderim hata: {e}"))
                return False

    # Komut yardimcilari
    def set_position(self, agv_id: str, waypoint: str) -> bool:
        return self.send_raw({"type": "setPosition", "agvId": agv_id, "waypoint": waypoint})
    # (set_target kaldirildi — path planning PC planner'da, set_hop kullanilir.)

    def set_pid(self, agv_id: str, kp: float, ki: float, kd: float) -> bool:
        return self.send_raw({
            "type": "setPID", "agvId": agv_id,
            "Kp": kp, "Ki": ki, "Kd": kd,
        })

    def set_speed(self, agv_id: str, speed: int) -> bool:
        return self.send_raw({"type": "setSpeed", "agvId": agv_id, "speed": speed})

    def command(self, agv_id: str, cmd: str) -> bool:
        # cmd: "start" | "stop" | "calibrate"
        return self.send_raw({"type": "command", "agvId": agv_id, "command": cmd})

    def apply_calibration(self, agv_id: str,
                          sensor_min: List[int], sensor_max: List[int]) -> bool:
        """PC'de tutulan preset'i AGV'ye runtime olarak uygula."""
        return self.send_raw({
            "type":      "applyCalibration",
            "agvId":     agv_id,
            "sensorMin": list(sensor_min),
            "sensorMax": list(sensor_max),
        })

    # ---- Multi-AGV planner WS API ----
    def set_hop(self, agv_id: str, from_: str, next_: str,
                after: Optional[str] = None,
                goal:  Optional[str] = None) -> bool:
        """2-hop look-ahead emir. AGV `from_`'dan `next_`'e gider, kavsakta
        `after` icin yon belirler ve durmaz. `after` None ise next'te park.

        `goal`: mission'un nihai hedefi. AGV firmware "Hedefe ulasildi"
        kontrolunde local hop after yerine bunu kullanir; AGV ara node'da
        durmaz, sadece gercek goal'da REACHED."""
        payload: dict = {
            "type":  "setHop",
            "agvId": agv_id,
            "from":  from_,
            "next":  next_,
        }
        if after:
            payload["after"] = after
        if goal:
            payload["goal"] = goal
        return self.send_raw(payload)

    def clear_mission(self, agv_id: str) -> bool:
        return self.send_raw({"type": "clearMission", "agvId": agv_id})

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

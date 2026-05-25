"""
AGV Kontrol Merkezi (CustomTkinter)
====================================
PC tarafi tum AGV'leri yonetmek icin merkezi UI.

Sekmeler:
    [Navigasyon]  - Harita + komutlar + telemetri
    [Kamera]      - ESP32-CAM yayini + tespit overlay
    [PID]         - PID + hiz tuning
    [Robot Kol]   - Servo + miknatis (placeholder, ESP32-CAM kontrolu icin)
    [Log]         - Tum AGV loglari

Sol kenar: AGV listesi (tikla ile aktif AGV degis)
Ust serit: WS server URL + Bagian/Kes
"""

import os
import time
from typing import Dict, List, Optional

import customtkinter as ctk
from PIL import Image

import cv2

from agv_ws_client       import AGVClient, AGVState, LogEvent, CalibrationDataEvent
from calibration_presets import PresetStore
from agv_config          import AGVConfigStore
from waypoint_canvas     import WaypointCanvas, FleetAGVDisplay
from stream_reader       import StreamReader
from detector            import get_detector
from graph               import Graph
from fleet_planner       import (
    FleetPlanner, HopAction, HopCommand,
)

# NOT: Tespit (object detection) pipeline'i tamamen kaldirildi — yeniden yazilacak.
# Eski: detector.py (background subtraction tabanli kare/renk tespiti) + hsv_presets.json


SERVER_URL_DEFAULT = "ws://192.168.4.1/ws"
PICKUP_CFG_PATH    = os.path.join(os.path.dirname(__file__), "pickup_config.json")
WAYPOINTS          = list("ABCDEFGHI")


# =============================================================================
# Tema — tek yerden renk/spacing/font sabitleri
# =============================================================================

# Aksiyon renkleri (Material Design 800/700 tonlari, dark theme uyumlu)
COL_SUCCESS         = "#2e7d32"   # baslat / OK
COL_SUCCESS_HOVER   = "#1b5e20"
COL_DANGER          = "#c62828"   # durdur / hata / acil
COL_DANGER_HOVER    = "#8e0000"
COL_DANGER_FLASH    = "#ff6b3d"   # acil-dur buton flash
COL_WARN            = "#f57c00"   # uyari / kalibre
COL_WARN_HOVER      = "#bb6500"
COL_INFO            = "#1565c0"   # bilgi / mavi aksiyon
COL_INFO_HOVER      = "#0d3c75"

# Yardimci/yazi renkleri
COL_MUTED           = "#9aa0a6"   # ikincil yazi (etiketler)
COL_SUBTLE          = "#6c6c6c"   # daha sonuk (dipnot, açiklama)
COL_BG_STATUSBAR    = "#1a1a1a"   # alt durum cubugu
COL_BG_DEEP         = "#141414"   # video / kapali alan zemini

# Durum mesaji renkleri (text_color)
COL_TXT_OK          = "#80ff80"
COL_TXT_WARN        = "#ffb347"
COL_TXT_ERR         = "#ff8080"
COL_TXT_ACCENT      = "#aaccff"
COL_TXT_BAGLI       = "#80ff80"   # "Bagli" yesili (orange yerine yesil olur)

# Standart spacing olcegi
PAD_XS = 2
PAD_SM = 4
PAD_MD = 8
PAD_LG = 12
PAD_XL = 16


# =============================================================================
# UI Yardimcilari
# =============================================================================

def fmt_ago(t_ms: int) -> str:
    """Server'dan gelen millis()'i 'X sn once' gibi gosterir (basit)."""
    if not t_ms:
        return ""
    return time.strftime("%H:%M:%S", time.localtime())


class LabelSlider(ctk.CTkFrame):
    """Etiket + slider + canlı değer (HSV kalibrasyon için)."""

    def __init__(self, master, label, mn, mx, init, on_change=None, step=1):
        super().__init__(master, fg_color="transparent")
        self.on_change = on_change
        self.label = ctk.CTkLabel(self, text=label, width=70, anchor="w")
        self.label.grid(row=0, column=0, sticky="w", padx=(0, 4))
        steps = max(1, int((mx - mn) / step))
        self.slider = ctk.CTkSlider(self, from_=mn, to=mx, number_of_steps=steps,
                                    command=self._cb, width=200)
        self.slider.set(init)
        self.slider.grid(row=0, column=1, padx=4)
        self.value_lbl = ctk.CTkLabel(self, text=str(init), width=40, anchor="e")
        self.value_lbl.grid(row=0, column=2, sticky="e")

    def _cb(self, val):
        v = int(val)
        self.value_lbl.configure(text=str(v))
        if self.on_change is not None:
            self.on_change(v)

    def get(self) -> int:
        return int(self.slider.get())

    def set(self, v):
        self.slider.set(v)
        self.value_lbl.configure(text=str(int(v)))


class SensorBar(ctk.CTkFrame):
    """Tek bir sensor degerini dikey bar olarak gosterir."""

    def __init__(self, master, idx: int):
        super().__init__(master, fg_color="transparent", width=20, height=80)
        self.idx = idx
        self.canvas = ctk.CTkCanvas(self, width=20, height=80, bg="#222",
                                    highlightthickness=0)
        self.canvas.pack()
        self.label = ctk.CTkLabel(self, text=str(idx), font=ctk.CTkFont(size=9))
        self.label.pack()
        self._bar = self.canvas.create_rectangle(2, 80, 18, 80, fill="#3fbf66", outline="")

    def set_value(self, v: int, max_v: int = 1000):
        v = max(0, min(max_v, v))
        # 80 piksel max
        h = int(80 * (v / max_v))
        self.canvas.coords(self._bar, 2, 80 - h, 18, 80)


# =============================================================================
# Ana Uygulama
# =============================================================================

class AGVControlApp(ctk.CTk):
    # UI tick aralıği — WS event queue'sundan pop + dashboard/sensor refresh.
    # 50ms = 20 Hz, queue_wait ortalaması ~25ms (eski 100ms'de ~50ms idi).
    # 3 AGV + 3 ESP32-CAM icin CPU yuku onemsiz; tkinter rahat kaldirir.
    UI_TICK_MS = 50

    def __init__(self):
        super().__init__()

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        self.title("AGV Kontrol Merkezi")
        self.geometry("1280x780")
        self.minsize(1100, 680)

        # Tipografi — Tk basladiktan sonra olusturulmali (modul seviyesinde olmaz)
        self.font_h1        = ctk.CTkFont(weight="bold", size=14)
        self.font_h2        = ctk.CTkFont(weight="bold", size=12)
        self.font_body      = ctk.CTkFont(size=11)
        self.font_small     = ctk.CTkFont(size=10)
        self.font_btn       = ctk.CTkFont(weight="bold", size=13)
        self.font_mono_sm   = ctk.CTkFont(family="Consolas", size=11)
        self.font_mono      = ctk.CTkFont(family="Consolas", size=12)
        self.font_mono_lg   = ctk.CTkFont(family="Consolas", size=13)
        self.font_mono_sm_b = ctk.CTkFont(family="Consolas", size=11, weight="bold")

        # Durum
        # NOT: CTk parent class'ında Wm.client() method'u var ama Optional
        # AGVClient ile çakışmıyor (runtime'da farklı tip), tip-checker
        # uyarısı bilinçli ignore ediliyor.
        self.client: Optional[AGVClient] = None   # type: ignore[assignment]
        self.agvs:   Dict[str, AGVState] = {}
        self.active_agv: Optional[str]   = None
        # F4: Kamera/kol artik AGV-aware — her AGV'nin kendi ESP32-CAM'i.
        # Per-AGV stream readers + per-AGV video labels (Robot Kol tab içinde).
        self.stream_readers:  Dict[str, StreamReader]    = {}
        self.cam_fps:         Dict[str, float]           = {}
        self.cam_last_count:  Dict[str, int]             = {}
        self.cam_last_time:   Dict[str, float]           = {}
        self.cam_video_lbls:  Dict[str, ctk.CTkLabel]    = {}
        self.cam_info_lbls:   Dict[str, ctk.CTkLabel]    = {}
        # CTkImage GC koruması: AGV bazında son frame referansı
        self._cam_img_refs:   Dict[str, ctk.CTkImage]    = {}

        # F4: Robot Kol per-AGV state (servolar + magnet + speed + LED)
        # Her AGV'nin kendi widget set'i, _make_arm_panel her AGV için ayrı build eder.
        self.arm_panels:      Dict[str, ctk.CTkFrame]    = {}
        self.arm_servo_vars:  Dict[str, List[ctk.IntVar]]   = {}
        self.arm_servo_lbls:  Dict[str, List[ctk.CTkLabel]] = {}
        self.arm_magnet_vars: Dict[str, ctk.BooleanVar]  = {}
        self.arm_speed_vars:  Dict[str, ctk.IntVar]      = {}
        self.arm_speed_lbls:  Dict[str, ctk.CTkLabel]    = {}
        self.arm_led_vars:    Dict[str, ctk.IntVar]      = {}
        self.arm_led_lbls:    Dict[str, ctk.CTkLabel]    = {}

        # YOLO tespit — per-AGV toggle (BooleanVar). Detector singleton (detector.py).
        # AGV'lerin ayni CAM modeline baglanmasi gerekmez; her biri kendi frame'i
        # uzerinde inference yapar, ortak model agirligini paylasir.
        self.detect_vars:     Dict[str, ctk.BooleanVar]  = {}
        self.detect_last_ms:  Dict[str, float]           = {}   # son inference suresi
        self.detect_last_n:   Dict[str, int]             = {}   # son frame'de tespit sayisi

        # Görünüm modu: dual (varsayılan) = iki AGV yan yana, single = aktif olan büyük
        self.arm_view_mode:   str  = "dual"   # "dual" | "single"
        self.arm_single_agv:  Optional[str] = None   # single mode'da gösterilen AGV

        # Kapma kalibrasyonu — base tarama state machine + saklanan limitler
        self.scan_active:        bool = False
        self.scan_current_angle: int  = 90    # base'in son komut acisi
        self.scan_target_angle:  int  = 90    # bu adimda gidilecek aci
        self.scan_min:           int  = 0     # aktif taramada alt sinir
        self.scan_max:           int  = 180   # aktif taramada ust sinir
        self.scan_direction:     int  = 1     # +1 saga, -1 sola
        self.scan_loop:          bool = False # bitince geri don / dur
        self.scan_save_field:    str  = ""    # "left" | "right" | "center" | ""
        self.scan_step_deg:      int  = 1
        self.scan_step_ms:       int  = 100
        self.scan_gripper_fwd:   int  = 73    # tarama başlangıcında set olur
        self.scan_gripper_back:  int  = 100
        # JSON'dan yuklenecek kapma konfig
        self.pickup_cfg: Dict = self._pickup_load_config()

        # ESP32-CAM log polling — her AGV icin ayri thread, AGV connect olunca
        # otomatik baslar. Stream sekmesinden bagimsiz: GRIP olaylari Arm
        # sekmesi acik olmasa da Log paneline akar.
        import threading as _t
        self._cam_log_stop_evt:   _t.Event                = _t.Event()
        self._cam_log_threads:    Dict[str, _t.Thread]    = {}
        self._cam_log_last_seq:   Dict[str, int]          = {}
        self._cam_grip_last_seq:  Dict[str, int]          = {}
        # /poll ile gelen son arm state — drag etmeyen UI guncellenmeleri icin
        self._cam_arm_state: Dict = {}

        # UI'da maksimum log satiri
        self.MAX_LOG_LINES = 500

        # Sensor gecmisi (her agvUpdate'da bir satir)
        self.MAX_SENSOR_LINES = 500
        self._sensor_lines: List[tuple] = []   # (timestamp, agv_id, sensors[8], linePos, sonarDist, navState)

        # Kalibrasyon preset deposu — pc/calibration_presets.json
        self.preset_store = PresetStore(
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "calibration_presets.json")
        )
        # F2: AGV donanim/baglanti config (her AGV'nin kendi ESP32-CAM URL'si)
        self.agv_config_store = AGVConfigStore(
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "agv_config.json")
        )
        # AGV bazinda son alinan ham kalibrasyon verisi — "Preset kaydet" icin
        self._last_calib_data: Dict[str, CalibrationDataEvent] = {}
        # AGV'nin onceki baglanti durumu — disconnect→connect gecisinde auto-apply
        self._was_connected: Dict[str, bool] = {}
        # Kalibrasyon debounce — son "calibrate" gonderiminin zamani (per AGV)
        # Kullanici butona art arda basamasin, AGV onboard 8 sn + 2 sn LED = 10 sn
        self._last_calibrate_sent: Dict[str, float] = {}
        self.CALIBRATE_DEBOUNCE_SEC = 12.0
        # Aktif geri sayim state'i — sadece bir AGV ayni anda kalibre olur
        self._calib_countdown_agv:        Optional[str] = None
        self._calib_countdown_remaining:  int           = 0
        self._calib_countdown_after_id                  = None
        self.CALIBRATE_DURATION_SEC = 10   # firmware: 1 + 8 + 1 sn

        # ===== Multi-AGV planner (background) =====
        # AGVControlApp her zaman aktif planner tutar. Operations'ta verilen
        # hedefler planner.add_mission'a yonlendirilir; planner setHop emirleri
        # uretip dispatch eder. Cakisma cozumu, parked AGV obstacle, mid-flight
        # rerouting otomatik. Kullanici sadece hedef verir.
        _wp_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "waypoints.json")
        self.fleet_graph   = Graph.from_json(_wp_path)
        self.fleet_planner = FleetPlanner(self.fleet_graph)
        # Dedup: ayni (from, next, after) tuple'i ust uste gondermek hem ag
        # trafigi hem AGV firmware'inde stale msg overwrite riski yaratir.
        # tick AGV_X icin AGV_Y'nin hopComplete'i ile tekrar tetikledigi zaman
        # AGV_X icin emir degismediyse atla.
        self._last_dispatched_hop: Dict[str, tuple] = {}

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self.UI_TICK_MS, self._tick)

    # =========================================================================
    # TOP BAR — WS bağlantı + ACİL DUR + Aktif AGV göstergesi
    # =========================================================================
    def _build_top_bar(self):
        bar = ctk.CTkFrame(self, height=56)
        bar.pack(fill="x", padx=8, pady=(8, 4))
        bar.pack_propagate(False)

        # --- Sol: WS bağlantı kontrolleri ---
        ctk.CTkLabel(bar, text="WS:", font=self.font_h2
                     ).pack(side="left", padx=(12, 4), pady=10)
        self.url_var = ctk.StringVar(value=SERVER_URL_DEFAULT)
        ctk.CTkEntry(bar, textvariable=self.url_var, width=240,
                     ).pack(side="left", padx=4, pady=10)
        self.connect_btn = ctk.CTkButton(
            bar, text="● Bağlan", width=110, height=34,
            fg_color=COL_SUCCESS, hover_color=COL_SUCCESS_HOVER,
            font=self.font_btn, command=self._connect)
        self.connect_btn.pack(side="left", padx=4, pady=10)
        self.disconnect_btn = ctk.CTkButton(
            bar, text="Kes", width=80, height=34,
            fg_color=COL_SUBTLE, hover_color="#4a4a4a",
            command=self._disconnect, state="disabled")
        self.disconnect_btn.pack(side="left", padx=4, pady=10)

        # --- Sağ (ters sıra): Aktif AGV indicator + ACİL DUR ---
        self.active_agv_lbl = ctk.CTkLabel(
            bar, text="Aktif AGV: -  ●",
            text_color=COL_SUBTLE, font=self.font_h2)
        self.active_agv_lbl.pack(side="right", padx=(8, 16), pady=10)

        self.estop_btn = ctk.CTkButton(
            bar, text="🛑 ACİL DUR  [Esc]", width=180, height=40,
            fg_color=COL_DANGER, hover_color=COL_DANGER_HOVER,
            font=self.font_h1, command=self._emergency_stop)
        self.estop_btn.pack(side="right", padx=8, pady=8)
        self.bind_all("<Escape>", lambda e: self._emergency_stop())

    # Server bağlantı durumu (dot rengi) — _disconnect ve event loop'ta okunur.
    # Eski hb_server_dot yerine connect_btn'in fg_color'ı + active_agv_lbl dot.
    def _update_connection_indicator(self, connected: bool):
        if connected:
            self.connect_btn.configure(text="● Bağlı", fg_color=COL_SUCCESS,
                                       state="disabled")
            self.disconnect_btn.configure(state="normal", fg_color=COL_DANGER)
        else:
            self.connect_btn.configure(text="● Bağlan", fg_color=COL_SUCCESS,
                                       state="normal")
            self.disconnect_btn.configure(state="disabled", fg_color=COL_SUBTLE)

    def _update_active_agv_indicator(self):
        """Top bar'daki Aktif AGV göstergesini active_agv'ye göre yenile."""
        if not hasattr(self, "active_agv_lbl"):
            return
        if self.active_agv and self.active_agv in self.agvs:
            s = self.agvs[self.active_agv]
            color = COL_TXT_BAGLI if s.connected else COL_TXT_ERR
            self.active_agv_lbl.configure(
                text=f"Aktif AGV: {self.active_agv}  ●",
                text_color=color)
        else:
            self.active_agv_lbl.configure(
                text="Aktif AGV: -  ●",
                text_color=COL_SUBTLE)

    # =========================================================================
    # Layout
    # =========================================================================
    def _build_ui(self):
        self._build_top_bar()

        # ----- Govde -----
        body = ctk.CTkFrame(self)
        body.pack(fill="both", expand=True, padx=8, pady=4)

        # Sol: AGV listesi
        self.agv_list_frame = ctk.CTkFrame(body, width=200)
        self.agv_list_frame.pack(side="left", fill="y", padx=(0, 8))
        self.agv_list_frame.pack_propagate(False)

        ctk.CTkLabel(self.agv_list_frame, text="AGV LISTESI",
                     font=self.font_h2).pack(pady=(8, 4))

        self.agv_list_inner = ctk.CTkScrollableFrame(self.agv_list_frame, width=180)
        self.agv_list_inner.pack(fill="both", expand=True, padx=4, pady=4)
        self.agv_buttons: Dict[str, ctk.CTkButton] = {}

        # Sag: 5 sekme — Dashboard / Operations / Arm / Sensors / Log
        # command callback: tab degisince biriken sensor log'u render et
        self.tabs = ctk.CTkTabview(body, command=self._on_tab_changed)
        self.tabs.pack(side="right", fill="both", expand=True)

        self.tab_dashboard  = self.tabs.add("Dashboard")
        self.tab_operations = self.tabs.add("Operations")
        self.tab_arm        = self.tabs.add("Arm")
        self.tab_sensors    = self.tabs.add("Sensors")
        self.tab_log        = self.tabs.add("Log")

        # İnşa sırası: Operations en bastaki map_widget'i diger tab'lar referansliyor
        self._build_tab_operations()
        self._build_tab_dashboard()
        self._build_tab_arm()
        self._build_tab_sensors()
        self._build_tab_log()

        # Dashboard ilk gosterilen tab
        self.tabs.set("Dashboard")

        # Alt durum cubugu
        self.status_bar = ctk.CTkLabel(self, text="Hazir.", anchor="w",
                                       fg_color=COL_BG_STATUSBAR, height=22)
        self.status_bar.pack(fill="x", side="bottom")

    # ---------- Navigasyon sekmesi ----------
    # ---------- Operations sekmesi (eski Navigasyon, F3) ----------
    # Harita + komut paneli + telemetri. Dashboard kaldırıldığı için tüm
    # mission-control içeriği burada toplandı.
    def _build_tab_operations(self):
        nav = self.tab_operations

        # Harita sol (geniş) — sağda komut/telemetri (dar)
        nav.grid_columnconfigure(0, weight=2)   # harita 2x ağırlık
        nav.grid_columnconfigure(1, weight=1)
        nav.grid_rowconfigure(0, weight=1)
        nav.grid_rowconfigure(1, weight=0)

        # Sol-ust: harita
        map_frame = ctk.CTkFrame(nav)
        map_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=(0, 4))
        ctk.CTkLabel(map_frame, text="WAYPOINT HARITASI",
                     font=self.font_h2).pack(pady=(6, 0))
        self.map_widget = WaypointCanvas(map_frame, on_target_click=self._on_map_target)
        self.map_widget.pack(fill="both", expand=True, padx=8, pady=8)

        # Sag-ust: hizli komut
        cmd_frame = ctk.CTkFrame(nav)
        cmd_frame.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=(0, 4))
        ctk.CTkLabel(cmd_frame, text="HIZLI KOMUT",
                     font=self.font_h2).pack(pady=(6, 6))

        # Konum/hedef secim
        sel_frame = ctk.CTkFrame(cmd_frame, fg_color="transparent")
        sel_frame.pack(fill="x", padx=12, pady=4)
        ctk.CTkLabel(sel_frame, text="Konum:").grid(row=0, column=0, padx=4, pady=4, sticky="e")
        self.pos_var = ctk.StringVar(value="A")
        ctk.CTkOptionMenu(sel_frame, values=WAYPOINTS, variable=self.pos_var,
                          width=80).grid(row=0, column=1, padx=4)
        ctk.CTkButton(sel_frame, text="Ayarla", width=70,
                      command=self._cmd_set_position).grid(row=0, column=2, padx=4)

        ctk.CTkLabel(sel_frame, text="Hedef:").grid(row=1, column=0, padx=4, pady=4, sticky="e")
        self.tgt_var = ctk.StringVar(value="H")
        ctk.CTkOptionMenu(sel_frame, values=WAYPOINTS, variable=self.tgt_var,
                          width=80).grid(row=1, column=1, padx=4)
        ctk.CTkButton(sel_frame, text="Ayarla", width=70,
                      command=self._cmd_set_target).grid(row=1, column=2, padx=4)

        # Buyuk butonlar
        btn_frame = ctk.CTkFrame(cmd_frame, fg_color="transparent")
        btn_frame.pack(fill="x", padx=12, pady=8)
        ctk.CTkButton(btn_frame, text="▶ BASLAT", height=44,
                      fg_color=COL_SUCCESS, hover_color=COL_SUCCESS_HOVER,
                      command=lambda: self._cmd_command("start")).pack(fill="x", pady=3)
        ctk.CTkButton(btn_frame, text="■ DURDUR", height=44,
                      fg_color=COL_DANGER, hover_color=COL_DANGER_HOVER,
                      command=lambda: self._cmd_command("stop")).pack(fill="x", pady=3)
        # NOT: Kalibrasyon butonu Sensör sekmesindeki preset paneline taşındı —
        # tek kaynak (geri sayım + sonuç status'u orada gösteriliyor)

        # Aktif yol/yon ozeti
        self.path_lbl = ctk.CTkLabel(cmd_frame, text="Yol: -", anchor="w")
        self.path_lbl.pack(fill="x", padx=12, pady=(8, 2))
        self.heading_lbl = ctk.CTkLabel(cmd_frame, text="Yon: -", anchor="w")
        self.heading_lbl.pack(fill="x", padx=12, pady=(0, 6))

        # --- PID + Hiz (eski PID sekmesi buraya sikistirildi) ---
        ctk.CTkLabel(cmd_frame, text="PID + HIZ",
                     font=self.font_h2).pack(pady=(6, 2))
        pid_row = ctk.CTkFrame(cmd_frame, fg_color="transparent")
        pid_row.pack(fill="x", padx=12, pady=2)
        ctk.CTkLabel(pid_row, text="Kp", width=22).grid(row=0, column=0, padx=(2, 1))
        self.kp_var = ctk.StringVar(value="0.012")
        ctk.CTkEntry(pid_row, textvariable=self.kp_var, width=60
                     ).grid(row=0, column=1, padx=1)
        ctk.CTkLabel(pid_row, text="Ki", width=22).grid(row=0, column=2, padx=(6, 1))
        self.ki_var = ctk.StringVar(value="0.000")
        ctk.CTkEntry(pid_row, textvariable=self.ki_var, width=60
                     ).grid(row=0, column=3, padx=1)
        ctk.CTkLabel(pid_row, text="Kd", width=22).grid(row=0, column=4, padx=(6, 1))
        self.kd_var = ctk.StringVar(value="0.005")
        ctk.CTkEntry(pid_row, textvariable=self.kd_var, width=60
                     ).grid(row=0, column=5, padx=1)
        ctk.CTkButton(cmd_frame, text="PID Uygula", height=26,
                      command=self._cmd_set_pid).pack(fill="x", padx=12, pady=(2, 4))

        speed_row = ctk.CTkFrame(cmd_frame, fg_color="transparent")
        speed_row.pack(fill="x", padx=12, pady=(2, 2))
        ctk.CTkLabel(speed_row, text="Hiz", width=24).pack(side="left")
        self.speed_var = ctk.IntVar(value=35)
        self.speed_slider = ctk.CTkSlider(
            speed_row, from_=0, to=255, number_of_steps=255,
            variable=self.speed_var,
            command=lambda v: self.speed_lbl.configure(text=str(int(v))))
        self.speed_slider.pack(side="left", fill="x", expand=True, padx=4)
        self.speed_lbl = ctk.CTkLabel(speed_row, text="35", width=30)
        self.speed_lbl.pack(side="left")
        ctk.CTkButton(cmd_frame, text="Hiz Uygula", height=26,
                      command=self._cmd_set_speed).pack(fill="x", padx=12, pady=(2, 4))

        self.pid_status_lbl = ctk.CTkLabel(
            cmd_frame, text="Aktif PID/hiz: -",
            anchor="w", text_color=COL_MUTED, font=self.font_small)
        self.pid_status_lbl.pack(fill="x", padx=12, pady=(2, 8))

        # Alt: telemetri
        tel_frame = ctk.CTkFrame(nav)
        tel_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(4, 0))
        ctk.CTkLabel(tel_frame, text="CANLI TELEMETRI",
                     font=self.font_h2).pack(pady=(6, 4))

        # Bilgi satiri
        info_row = ctk.CTkFrame(tel_frame, fg_color="transparent")
        info_row.pack(fill="x", padx=12, pady=2)
        self.tel_state    = ctk.CTkLabel(info_row, text="Durum: -")
        self.tel_state.grid(row=0, column=0, sticky="w", padx=8)
        self.tel_speed    = ctk.CTkLabel(info_row, text="Hiz: -")
        self.tel_speed.grid(row=0, column=1, sticky="w", padx=8)
        self.tel_linepos  = ctk.CTkLabel(info_row, text="Cizgi pos: -")
        self.tel_linepos.grid(row=0, column=2, sticky="w", padx=8)
        self.tel_sonar    = ctk.CTkLabel(info_row, text="Sonar: -")
        self.tel_sonar.grid(row=0, column=3, sticky="w", padx=8)
        self.tel_calib    = ctk.CTkLabel(info_row, text="Kalibre: -")
        self.tel_calib.grid(row=0, column=4, sticky="w", padx=8)

        # Sensor barlari
        sensors_frame = ctk.CTkFrame(tel_frame)
        sensors_frame.pack(padx=12, pady=8)
        ctk.CTkLabel(sensors_frame, text="Sensorler (0-1000):").pack()
        bar_row = ctk.CTkFrame(sensors_frame, fg_color="transparent")
        bar_row.pack()
        self.sensor_bars: List[SensorBar] = []
        for i in range(8):
            b = SensorBar(bar_row, i)
            b.grid(row=0, column=i, padx=2)
            self.sensor_bars.append(b)

    # ---------- Planner tab kaldirildi ----------
    # Multi-AGV fleet planner artik background'da otomatik calisir
    # (AGVControlApp.fleet_planner instance). Operations tab "Hedef Ata"
    # butonu setTarget yerine planner.add_mission + setHop dispatch yapar.
    # Cakisma cozumu, parked AGV obstacle, mid-flight rerouting hepsi
    # otomatik. Kullanici sadece hedef verir.
    #
    # Standalone debug aracı pc/planner_viewer.py kaldirildi.

    # ---------- Kamera tab kaldırıldı (F4) ----------
    # Stream gösterimi + KAPMA KALIBRASYONU içeriği Robot Kol tab'ına alındı.
    # Cam URL'leri agv_config.json'da AGV başına saklı (her AGV'nin kendi
    # ESP32-CAM'i — varsayılan IP 192.168.4.50+N).

    # KAPMA KALIBRASYONU panelini Robot Kol tab'ında inşa eden helper.
    # (Eski _build_tab_cam'in alt yarısının extract'i.)
    def _build_pickup_calibration_panel(self, parent):
        ctk.CTkLabel(parent, text="KAPMA KALIBRASYONU",
                     font=self.font_h2).pack(pady=(8, 4))

        pose = self.pickup_cfg.get("scan_pose", {})
        self.scan_pose_lbl = ctk.CTkLabel(
            parent,
            text=f"Tarama poz: sh={pose.get('shoulder','?')}  el={pose.get('elbow','?')}",
            font=self.font_small, text_color=COL_MUTED)
        self.scan_pose_lbl.pack(anchor="w", padx=12)
        ctk.CTkButton(parent, text="Mevcut sh/el'yi Tarama Pozu Kaydet",
                      command=self._pickup_save_scan_pose,
                      font=self.font_body).pack(fill="x", padx=12, pady=(2, 6))

        ctk.CTkButton(parent, text="◀ Full Sol Tara",
                      command=lambda: self._pickup_scan_start("left"),
                      font=self.font_body).pack(fill="x", padx=12, pady=2)
        ctk.CTkButton(parent, text="Full Sag Tara ▶",
                      command=lambda: self._pickup_scan_start("right"),
                      font=self.font_body).pack(fill="x", padx=12, pady=2)
        ctk.CTkButton(parent, text="◆ On Tara (loop)",
                      command=lambda: self._pickup_scan_start("front"),
                      font=self.font_body).pack(fill="x", padx=12, pady=2)
        ctk.CTkButton(parent, text="Genel Tara (loop)",
                      command=lambda: self._pickup_scan_start("general"),
                      font=self.font_body).pack(fill="x", padx=12, pady=2)

        save_row = ctk.CTkFrame(parent, fg_color="transparent")
        save_row.pack(fill="x", padx=12, pady=(6, 2))
        self.scan_save_start_btn = ctk.CTkButton(
            save_row, text="📍 Baslangic", state="disabled",
            fg_color=COL_SUCCESS, hover_color=COL_SUCCESS_HOVER,
            command=self._pickup_save_start)
        self.scan_save_start_btn.pack(side="left", expand=True, fill="x", padx=2)
        self.scan_save_end_btn = ctk.CTkButton(
            save_row, text="📍 Bitis", state="disabled",
            fg_color=COL_INFO, hover_color=COL_INFO_HOVER,
            command=self._pickup_save_end)
        self.scan_save_end_btn.pack(side="left", expand=True, fill="x", padx=2)

        self.scan_stop_btn = ctk.CTkButton(parent, text="⏹ Taramayi Bitir",
                                           fg_color=COL_DANGER, hover_color=COL_DANGER_HOVER,
                                           command=self._pickup_scan_stop)
        self.scan_stop_btn.pack(fill="x", padx=12, pady=(2, 4))

        self.scan_status_lbl = ctk.CTkLabel(parent, text="Pasif",
                                            font=self.font_small,
                                            text_color=COL_MUTED, justify="left")
        self.scan_status_lbl.pack(anchor="w", padx=12, pady=(2, 4))

        self.scan_limits_lbl = ctk.CTkLabel(
            parent, text=self._pickup_limits_text(),
            font=self.font_small, text_color=COL_TXT_ACCENT, justify="left")
        self.scan_limits_lbl.pack(anchor="w", padx=12, pady=(0, 4))

        ctk.CTkButton(parent, text="💾 JSON'a Kaydet",
                      command=self._pickup_save_config,
                      font=self.font_body).pack(fill="x", padx=12, pady=(4, 8))

    # =========================================================================
    # F4 — Robot Kol tab (dual/single view, per-AGV camera + servo + magnet)
    # =========================================================================
    # Tasarım: üstte view-mode toggle (Çift/Tek) + AGV dropdown (Tek için).
    # Orta: arm_panels_container — view mode değişince yeniden inşa.
    #   Çift: yan yana her bağlı AGV için panel
    #   Tek:  seçili AGV için tek büyük panel
    # Alt: KAPMA KALIBRASYONU (aktif AGV bazlı, tek kaynak).
    #
    # State (per-AGV, agv_id key'li dict):
    #   - arm_servo_vars[id] = [IntVar × 4]   ← rebuild'de korunur
    #   - arm_magnet_vars[id]                  ← rebuild'de korunur
    #   - cam_video_lbls[id], cam_info_lbls[id] ← rebuild'de yeniden yaratılır

    _ARM_THROTTLE_S = 0.10
    SERVO_DEFS: List[tuple] = [
        # (isim, alt, MIN, MAX, HOME) — firmware HOME ile uyumlu
        ("Base",     "GPIO 14 - MG996R",    0, 180,  93),
        ("Shoulder", "GPIO 13 - DS3218MG",  0, 120, 120),
        ("Elbow",    "GPIO 15 - MG996R",    4, 153,  15),
        ("Gripper",  "GPIO 12 - MG90S",     0, 160, 110),
    ]

    # ---------- Arm sekmesi (tek AGV, aktif olanı gösterir) ----------
    def _build_tab_arm(self):
        """Aktif AGV için: sol scrollable (cam+arm+pickup) | sağ scrollable (detection)."""
        arm = self.tab_arm
        arm.grid_columnconfigure(0, weight=2)
        arm.grid_columnconfigure(1, weight=1)
        arm.grid_rowconfigure(1, weight=1)

        head = ctk.CTkFrame(arm, fg_color="transparent")
        head.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=(8, 2))
        self.arm_active_lbl = ctk.CTkLabel(head, text="(AGV seçilmedi)",
                                            font=self.font_h1,
                                            text_color=COL_SUBTLE)
        self.arm_active_lbl.pack(side="left", padx=4)

        # Sol scrollable: cam + arm + pickup
        self.arm_left_scroll = ctk.CTkScrollableFrame(arm)
        self.arm_left_scroll.grid(row=1, column=0, sticky="nsew", padx=(8, 4), pady=4)

        # Sağ scrollable: detection placeholder
        self.arm_right_scroll = ctk.CTkScrollableFrame(arm)
        self.arm_right_scroll.grid(row=1, column=1, sticky="nsew", padx=(4, 8), pady=4)
        ctk.CTkLabel(self.arm_right_scroll, text="GÖRÜNTÜ TANIMA",
                     font=self.font_h2).pack(pady=(8, 4))
        ctk.CTkLabel(self.arm_right_scroll,
                     text=("(boş)\n\nYOLOv8 / model seçimi /\n"
                           "confidence eşiği / bbox toggle\n"
                           "ileride buraya gelecek."),
                     text_color=COL_SUBTLE, justify="center",
                     font=self.font_small).pack(pady=20)

        # Sol kolonu ilk inşa (aktif AGV varsa)
        self._refresh_arm_panel()

    def _ensure_arm_state(self, agv_id: str):
        """Per-AGV servo/magnet/speed/LED IntVar'larını oluştur (idempotent)."""
        if agv_id not in self.arm_servo_vars:
            self.arm_servo_vars[agv_id] = [
                ctk.IntVar(value=defn[4]) for defn in self.SERVO_DEFS
            ]
        if agv_id not in self.arm_magnet_vars:
            self.arm_magnet_vars[agv_id] = ctk.BooleanVar(value=False)
        if agv_id not in self.arm_speed_vars:
            self.arm_speed_vars[agv_id] = ctk.IntVar(value=40)
        if agv_id not in self.arm_led_vars:
            self.arm_led_vars[agv_id] = ctk.IntVar(value=5)
        if agv_id not in self.detect_vars:
            self.detect_vars[agv_id] = ctk.BooleanVar(value=False)

    def _sync_stream_readers(self, visible: List[str]):
        """visible listesi dışındaki AGV'lerin stream reader'larını kapat."""
        for agv_id in list(self.stream_readers.keys()):
            if agv_id not in visible:
                try:
                    self.stream_readers[agv_id].stop()
                except Exception:
                    pass
                del self.stream_readers[agv_id]
                self.cam_fps.pop(agv_id, None)
                self.cam_last_count.pop(agv_id, None)
                self.cam_last_time.pop(agv_id, None)
                self._cam_img_refs.pop(agv_id, None)

    def _refresh_arm_panel(self):
        """Aktif AGV'ye göre sol kolonu yeniden inşa (cam + arm + pickup)."""
        if not hasattr(self, "arm_left_scroll"):
            return

        for child in self.arm_left_scroll.winfo_children():
            child.destroy()
        # Eski widget referansları sıfırlanır (yeni active AGV için yenileri yaratılır)
        self.cam_video_lbls.clear()
        self.cam_info_lbls.clear()

        agv_id = self.active_agv
        if not agv_id or agv_id not in self.agvs:
            self.arm_active_lbl.configure(text="(AGV seçilmedi)",
                                           text_color=COL_SUBTLE)
            ctk.CTkLabel(self.arm_left_scroll,
                         text="Sol AGV listesinden bir AGV seçin",
                         text_color=COL_SUBTLE, font=self.font_small
                         ).pack(pady=20)
            self._sync_stream_readers(visible=[])
            return

        self.arm_active_lbl.configure(text=f"AGV: {agv_id}",
                                       text_color=COL_TXT_BAGLI)
        self._ensure_arm_state(agv_id)
        self._build_arm_cam_section(self.arm_left_scroll, agv_id)
        self._build_arm_controls_section(self.arm_left_scroll, agv_id)

        # KAPMA KALIBRASYONU bölümü
        pickup_card = ctk.CTkFrame(self.arm_left_scroll)
        pickup_card.pack(fill="x", padx=4, pady=4)
        self._build_pickup_calibration_panel(pickup_card)

        # Stream sadece bu AGV için açık olsun
        self._sync_stream_readers(visible=[agv_id])

    def _build_arm_cam_section(self, parent, agv_id: str):
        card = ctk.CTkFrame(parent)
        card.pack(fill="x", padx=4, pady=4)
        ctk.CTkLabel(card, text="📷 ESP32-CAM",
                     font=self.font_h2).pack(anchor="w", padx=12, pady=(8, 4))

        # URL satırı
        url_row = ctk.CTkFrame(card, fg_color="transparent")
        url_row.pack(fill="x", padx=8, pady=2)
        cfg = self.agv_config_store.get(agv_id)
        url_var = ctk.StringVar(value=cfg.cam_control_url)
        ctk.CTkEntry(url_row, textvariable=url_var, font=self.font_small
                     ).pack(side="left", fill="x", expand=True, padx=(0, 4))

        def save_urls():
            from urllib.parse import urlparse
            ctrl = url_var.get().strip().rstrip("/")
            if not ctrl:
                return
            try:
                p = urlparse(ctrl)
                stream = f"{p.scheme}://{p.hostname}:81/stream"
            except Exception:
                stream = ctrl
            self.agv_config_store.update(
                agv_id, cam_control_url=ctrl, cam_stream_url=stream)
            self._set_status(f"{agv_id} cam URL kaydedildi")

        ctk.CTkButton(url_row, text="💾", width=30,
                      command=save_urls, font=self.font_small).pack(side="left")

        # Video display
        video_lbl = ctk.CTkLabel(card, text="Akış kapalı", width=480, height=360,
                                  fg_color=COL_BG_DEEP)
        video_lbl.pack(padx=8, pady=4)
        self.cam_video_lbls[agv_id] = video_lbl

        info_lbl = ctk.CTkLabel(card, text="FPS: -", font=self.font_small,
                                 anchor="w")
        info_lbl.pack(fill="x", padx=8)
        self.cam_info_lbls[agv_id] = info_lbl

        btn_row = ctk.CTkFrame(card, fg_color="transparent")
        btn_row.pack(fill="x", padx=8, pady=(2, 8))
        ctk.CTkButton(btn_row, text="Yayın Aç", width=100, height=28,
                      fg_color=COL_SUCCESS, hover_color=COL_SUCCESS_HOVER,
                      command=lambda: self._cam_connect(agv_id),
                      font=self.font_small).pack(side="left", padx=2)
        ctk.CTkButton(btn_row, text="Kapat", width=70, height=28,
                      fg_color=COL_SUBTLE,
                      command=lambda: self._cam_disconnect(agv_id),
                      font=self.font_small).pack(side="left", padx=2)
        ctk.CTkButton(btn_row, text="Durum Oku", width=110, height=28,
                      command=lambda: self._arm_fetch_state(agv_id),
                      font=self.font_small).pack(side="left", padx=2)

        # YOLO tespit toggle — checkbox aktifken _process_camera inference yapar
        # ve frame'e bbox + merkez overlay basar. Sag tarafta inference suresi (ms)
        # + son frame'deki tespit sayisi gosterilir.
        det_row = ctk.CTkFrame(card, fg_color="transparent")
        det_row.pack(fill="x", padx=8, pady=(0, 8))
        ctk.CTkCheckBox(det_row, text="🎯 Tespit (YOLO)",
                        variable=self.detect_vars[agv_id],
                        font=self.font_small).pack(side="left", padx=2)
        det_info = ctk.CTkLabel(det_row, text="—", font=self.font_small,
                                 anchor="w", text_color=COL_MUTED)
        det_info.pack(side="left", padx=8)
        if not hasattr(self, "detect_info_lbls"):
            self.detect_info_lbls: Dict[str, ctk.CTkLabel] = {}
        self.detect_info_lbls[agv_id] = det_info

    def _build_arm_controls_section(self, parent, agv_id: str):
        card = ctk.CTkFrame(parent)
        card.pack(fill="x", padx=4, pady=4)
        ctk.CTkLabel(card, text="ROBOT KOL",
                     font=self.font_h2).pack(anchor="w", padx=12, pady=(8, 4))

        slbls: List[ctk.CTkLabel] = []
        for idx, (name, sub, vmin, vmax, vhome) in enumerate(self.SERVO_DEFS):
            row = ctk.CTkFrame(card, fg_color="transparent")
            row.pack(fill="x", padx=8, pady=2)
            ctk.CTkLabel(row, text=name, width=80, anchor="w",
                         font=self.font_h2).pack(side="left")
            var = self.arm_servo_vars[agv_id][idx]
            slider = ctk.CTkSlider(
                row, from_=vmin, to=vmax, number_of_steps=vmax - vmin,
                variable=var,
                command=lambda v, i=idx: self._arm_slider_change(agv_id, i, int(v)),
            )
            slider.pack(side="left", fill="x", expand=True, padx=4)
            slider.bind("<ButtonRelease-1>",
                        lambda e, i=idx: self._arm_slider_release(agv_id, i))
            ang = ctk.CTkLabel(row, text=f"{var.get()}°", width=50,
                                font=self.font_mono_sm)
            ang.pack(side="left")
            slbls.append(ang)
        self.arm_servo_lbls[agv_id] = slbls

        # Magnet + HOME + FREEZE
        action = ctk.CTkFrame(card, fg_color="transparent")
        action.pack(fill="x", padx=8, pady=4)
        ctk.CTkSwitch(action, text="🧲 Mıknatıs",
                      variable=self.arm_magnet_vars[agv_id],
                      command=lambda: self._arm_magnet_toggle(agv_id)
                      ).pack(side="left", padx=4)
        ctk.CTkButton(action, text="🏠 HOME", width=90, height=28,
                      fg_color=COL_SUCCESS, hover_color=COL_SUCCESS_HOVER,
                      command=lambda: self._arm_home(agv_id),
                      font=self.font_small).pack(side="left", padx=4)
        ctk.CTkButton(action, text="🛑 FREEZE", width=90, height=28,
                      fg_color=COL_DANGER, hover_color=COL_DANGER_HOVER,
                      command=lambda: self._arm_http_get(agv_id, "/arm/freeze"),
                      font=self.font_small).pack(side="left", padx=4)

        # Speed
        sp_row = ctk.CTkFrame(card, fg_color="transparent")
        sp_row.pack(fill="x", padx=8, pady=2)
        ctk.CTkLabel(sp_row, text="Servo hızı", width=80).pack(side="left")
        sp_slider = ctk.CTkSlider(
            sp_row, from_=10, to=200, number_of_steps=190,
            variable=self.arm_speed_vars[agv_id],
            command=lambda v: self._arm_speed_label_update(agv_id, int(v)),
        )
        sp_slider.pack(side="left", fill="x", expand=True, padx=4)
        sp_slider.bind("<ButtonRelease-1>",
                       lambda e: self._arm_send_speed(agv_id))
        sp_lbl = ctk.CTkLabel(sp_row,
                               text=f"{self.arm_speed_vars[agv_id].get()} ms",
                               width=80, font=self.font_mono_sm)
        sp_lbl.pack(side="left")
        self.arm_speed_lbls[agv_id] = sp_lbl

        # LED
        led_row = ctk.CTkFrame(card, fg_color="transparent")
        led_row.pack(fill="x", padx=8, pady=(2, 8))
        ctk.CTkLabel(led_row, text="LED", width=80).pack(side="left")
        led_slider = ctk.CTkSlider(
            led_row, from_=0, to=255, number_of_steps=255,
            variable=self.arm_led_vars[agv_id],
            command=lambda v: self._arm_led_label_update(agv_id, int(v)),
        )
        led_slider.pack(side="left", fill="x", expand=True, padx=4)
        led_slider.bind("<ButtonRelease-1>",
                        lambda e: self._arm_send_led(agv_id))
        led_lbl = ctk.CTkLabel(led_row,
                                text=f"{self.arm_led_vars[agv_id].get()}/255",
                                width=80, font=self.font_mono_sm)
        led_lbl.pack(side="left")
        self.arm_led_lbls[agv_id] = led_lbl

    # =========================================================================
    # Robot Kol HTTP — per-AGV
    # =========================================================================
    def _arm_base_url(self, agv_id: str) -> Optional[str]:
        cfg = self.agv_config_store.get(agv_id)
        url = (cfg.cam_control_url or "").strip().rstrip("/")
        if not url:
            self._set_status(f"{agv_id}: kol URL'si bos", err=True)
            return None
        return url

    def _arm_http_get(self, agv_id: str, path: str):
        """HTTP GET'i ayri thread'de gonder — UI bloklanmasin."""
        import threading
        import urllib.request

        base = self._arm_base_url(agv_id)
        if base is None:
            return
        url = f"{base}{path}"

        def worker():
            try:
                with urllib.request.urlopen(url, timeout=2.0) as r:
                    _ = r.read()
            except Exception as e:
                self.after(0, lambda: self._set_status(
                    f"{agv_id} kol HTTP hata: {e}", err=True))

        threading.Thread(target=worker, daemon=True).start()

    def _arm_slider_change(self, agv_id: str, servo_id: int, angle: int):
        # Label güncelle
        if agv_id in self.arm_servo_lbls:
            self.arm_servo_lbls[agv_id][servo_id].configure(text=f"{angle}°")

        # Throttle state: (agv_id, servo_id) tuple key
        if not hasattr(self, "_arm_pending"):
            self._arm_pending:   Dict[tuple, int]   = {}
            self._arm_last_send: Dict[tuple, float] = {}
            self._arm_scheduled: set                = set()

        key = (agv_id, servo_id)
        self._arm_pending[key] = angle
        now  = time.time()
        last = self._arm_last_send.get(key, 0.0)

        if now - last >= self._ARM_THROTTLE_S:
            self._arm_flush_servo(agv_id, servo_id)
        elif key not in self._arm_scheduled:
            self._arm_scheduled.add(key)
            delay_ms = max(1, int((self._ARM_THROTTLE_S - (now - last)) * 1000))
            self.after(delay_ms, lambda k=key: self._arm_flush_servo(*k))

    def _arm_flush_servo(self, agv_id: str, servo_id: int):
        key = (agv_id, servo_id)
        if hasattr(self, "_arm_scheduled"):
            self._arm_scheduled.discard(key)
        if not hasattr(self, "_arm_pending") or key not in self._arm_pending:
            return
        angle = self._arm_pending.pop(key)
        self._arm_last_send[key] = time.time()
        self._arm_http_get(agv_id, f"/servo?id={servo_id}&a={angle}")

    def _arm_slider_release(self, agv_id: str, servo_id: int):
        key = (agv_id, servo_id)
        if hasattr(self, "_arm_pending") and key in self._arm_pending:
            self._arm_flush_servo(agv_id, servo_id)

    def _arm_home(self, agv_id: str):
        """Sequential HOME: firmware once shoulder, sonra digerleri."""
        HOME = [93, 120, 15, 110]
        if agv_id in self.arm_servo_vars:
            for i, a in enumerate(HOME):
                self.arm_servo_vars[agv_id][i].set(a)
                if agv_id in self.arm_servo_lbls:
                    self.arm_servo_lbls[agv_id][i].configure(text=f"{a}°")
        self._arm_http_get(agv_id, "/arm/home")
        self._set_status(f"{agv_id} kol HOME")

    def _arm_magnet_toggle(self, agv_id: str):
        if agv_id not in self.arm_magnet_vars:
            return
        state = 1 if self.arm_magnet_vars[agv_id].get() else 0
        self._set_status(f"{agv_id} mıknatıs: {'AÇIK' if state else 'KAPALI'}")
        self._arm_http_get(agv_id, f"/magnet?s={state}")

    def _arm_speed_label_update(self, agv_id: str, ms: int):
        dps = (1000 // ms) if ms > 0 else 0
        if agv_id in self.arm_speed_lbls:
            self.arm_speed_lbls[agv_id].configure(text=f"{ms} ms / {dps}°/sn")

    def _arm_send_speed(self, agv_id: str):
        ms = self.arm_speed_vars[agv_id].get()
        self._arm_http_get(agv_id, f"/servo/speed?ms={ms}")
        self._set_status(f"{agv_id} servo hızı: {ms} ms")

    def _arm_led_label_update(self, agv_id: str, b: int):
        if agv_id in self.arm_led_lbls:
            pct = round(b * 100 / 255)
            self.arm_led_lbls[agv_id].configure(text=f"{b}/255 ({pct}%)")

    def _arm_send_led(self, agv_id: str):
        b = self.arm_led_vars[agv_id].get()
        self._arm_http_get(agv_id, f"/led?b={b}")
        self._set_status(f"{agv_id} LED: {b}/255")

    def _arm_fetch_state(self, agv_id: str):
        """ESP32-CAM'dan mevcut kol durumunu çek, slider'lari guncelle."""
        import threading
        import urllib.request
        import json as _json

        base = self._arm_base_url(agv_id)
        if base is None:
            return

        def worker():
            try:
                with urllib.request.urlopen(f"{base}/arm/state", timeout=2.0) as r:
                    data = _json.loads(r.read().decode("utf-8"))
                servos   = data.get("servos", [90, 90, 90, 90])
                magnet   = bool(data.get("magnet", 0))
                speed_ms = int(data.get("speedMs", 40))
                led_b    = int(data.get("led", 0))

                def apply():
                    if agv_id in self.arm_servo_vars:
                        for i, a in enumerate(servos[:4]):
                            self.arm_servo_vars[agv_id][i].set(int(a))
                            if agv_id in self.arm_servo_lbls:
                                self.arm_servo_lbls[agv_id][i].configure(text=f"{int(a)}°")
                    if agv_id in self.arm_magnet_vars:
                        self.arm_magnet_vars[agv_id].set(magnet)
                    if agv_id in self.arm_speed_vars:
                        self.arm_speed_vars[agv_id].set(speed_ms)
                        self._arm_speed_label_update(agv_id, speed_ms)
                    if agv_id in self.arm_led_vars:
                        self.arm_led_vars[agv_id].set(led_b)
                        self._arm_led_label_update(agv_id, led_b)
                    self._set_status(f"{agv_id} kol durumu okundu")
                self.after(0, apply)
            except Exception as e:
                self.after(0, lambda: self._set_status(
                    f"{agv_id} kol durum hata: {e}", err=True))

        threading.Thread(target=worker, daemon=True).start()
    # ---------- Sensors sekmesi (preset + scrollable sensör log) ----------
    def _build_tab_sensors(self):
        """Üstte sabit 250px preset paneli, altta scrollable sensör log."""
        sen = self.tab_sensors
        sen.grid_columnconfigure(0, weight=1)
        sen.grid_rowconfigure(1, weight=1)

        # ÜST: Sabit 250px preset paneli
        preset_frame = ctk.CTkFrame(sen, height=250)
        preset_frame.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))
        preset_frame.pack_propagate(False)
        self._build_preset_panel(preset_frame)

        # ALT: Scrollable sensör log
        log_frame = ctk.CTkFrame(sen)
        log_frame.grid(row=1, column=0, sticky="nsew", padx=8, pady=(4, 8))

        bar = ctk.CTkFrame(log_frame, fg_color="transparent")
        bar.pack(fill="x", padx=8, pady=6)
        ctk.CTkLabel(bar, text="SENSÖR LOG", font=self.font_h2
                     ).pack(side="left", padx=(0, 12))

        ctk.CTkLabel(bar, text="Filtre:").pack(side="left", padx=(0, 4))
        self.sensor_filter_var = ctk.StringVar(value="Hepsi")
        self.sensor_filter = ctk.CTkOptionMenu(
            bar, values=["Hepsi"], variable=self.sensor_filter_var,
            width=110, command=lambda _: self._refresh_sensor_log())
        self.sensor_filter.pack(side="left", padx=4)

        self.sensor_pause_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(bar, text="⏸ Pause", variable=self.sensor_pause_var
                        ).pack(side="left", padx=8)
        self.sensor_autoscroll_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(bar, text="↓ Auto", variable=self.sensor_autoscroll_var
                        ).pack(side="left", padx=4)

        ctk.CTkButton(bar, text="💾 CSV", width=70,
                      command=self._save_sensor_csv).pack(side="right", padx=2)
        ctk.CTkButton(bar, text="Temizle", width=80,
                      command=self._clear_sensor_log).pack(side="right", padx=2)

        self.sensor_count_lbl = ctk.CTkLabel(log_frame, text="0 satır",
                                              text_color=COL_SUBTLE,
                                              font=self.font_small)
        self.sensor_count_lbl.pack(anchor="w", padx=12)

        header = ctk.CTkLabel(
            log_frame,
            text=("Zaman          AGV      "
                  "S0   S1   S2   S3   S4   S5   S6   S7   "
                  "linePos  sonar  durum"),
            font=self.font_mono_sm_b, anchor="w",
            text_color=COL_TXT_ACCENT)
        header.pack(fill="x", padx=8, pady=(2, 0))

        self.sensor_text = ctk.CTkTextbox(log_frame, font=self.font_mono_sm)
        self.sensor_text.pack(fill="both", expand=True, padx=8, pady=(2, 8))
        self.sensor_text.configure(state="disabled")

    # ---------- Dashboard sekmesi ----------
    def _build_tab_dashboard(self):
        """Üst: read-only harita + son log mesajları.
        Alt: tüm AGV'lerin scrollable kart listesi (sensör barları + telemetri)."""
        dash = self.tab_dashboard
        dash.grid_columnconfigure(0, weight=1)
        dash.grid_rowconfigure(0, weight=1)
        dash.grid_rowconfigure(1, weight=1)

        # ÜST: harita (sol) + son log (sağ)
        top = ctk.CTkFrame(dash, fg_color="transparent")
        top.grid(row=0, column=0, sticky="nsew", padx=8, pady=(8, 4))
        top.grid_columnconfigure(0, weight=7)   # harita ~70%
        top.grid_columnconfigure(1, weight=3)   # log ~30%
        top.grid_rowconfigure(0, weight=1)

        map_card = ctk.CTkFrame(top)
        map_card.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        ctk.CTkLabel(map_card, text="WAYPOINT HARİTASI (salt-okunur)",
                     font=self.font_h2).pack(pady=(8, 4))
        # on_target_click=None → read-only
        self.dash_map = WaypointCanvas(map_card, on_target_click=None,
                                        width=420, height=320)
        self.dash_map.pack(padx=8, pady=(0, 8), expand=True)

        log_card = ctk.CTkFrame(top)
        log_card.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        ctk.CTkLabel(log_card, text="SON LOGLAR",
                     font=self.font_h2).pack(pady=(8, 4))
        self.dash_log_box = ctk.CTkTextbox(log_card, font=self.font_mono_sm)
        self.dash_log_box.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.dash_log_box.configure(state="disabled")

        # ALT: tüm AGV'lerin scrollable sensör + telemetri kartları
        cards_card = ctk.CTkFrame(dash)
        cards_card.grid(row=1, column=0, sticky="nsew", padx=8, pady=(4, 8))
        ctk.CTkLabel(cards_card, text="AKTİF AGV'LER (sensör + telemetri)",
                     font=self.font_h2).pack(pady=(8, 4))

        self.dash_cards_scroll = ctk.CTkScrollableFrame(cards_card, height=200)
        self.dash_cards_scroll.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        # Her AGV için kart (dynamic) — _refresh_dashboard_cards yenilemeyi yönetir
        self.dash_agv_cards:   Dict[str, ctk.CTkFrame]      = {}
        self.dash_agv_widgets: Dict[str, dict]              = {}

    def _refresh_dashboard_log(self):
        """Dashboard'un mini log kutusu — son ~50 satır."""
        if not hasattr(self, "dash_log_box"):
            return
        self.dash_log_box.configure(state="normal")
        self.dash_log_box.delete("1.0", "end")
        for ev in self._log_lines[-50:]:
            t = ev.wall_time if ev.wall_time > 0 else time.time()
            ts = time.strftime("%H:%M:%S", time.localtime(t))
            self.dash_log_box.insert("end",
                f"[{ts}] [{ev.agvId or '-'}] {ev.message}\n")
        self.dash_log_box.see("end")
        self.dash_log_box.configure(state="disabled")

    def _refresh_dashboard_cards(self):
        """Her AGV için kart oluştur/güncelle/sil. Sensör + telemetri."""
        if not hasattr(self, "dash_cards_scroll"):
            return

        # Olmayan AGV'lerin kartlarını sil
        existing = set(self.dash_agv_cards.keys())
        current  = set(self.agvs.keys())
        for agv_id in existing - current:
            self.dash_agv_cards[agv_id].destroy()
            del self.dash_agv_cards[agv_id]
            self.dash_agv_widgets.pop(agv_id, None)

        # Var olanları güncelle, yenileri ekle
        for agv_id in sorted(self.agvs.keys()):
            s = self.agvs[agv_id]
            if agv_id in self.dash_agv_cards:
                self._update_dashboard_card(agv_id, s)
            else:
                self._make_dashboard_card(agv_id, s)

    def _make_dashboard_card(self, agv_id: str, s: AGVState):
        """Tek AGV için Dashboard kartı: status + waypoint + sensör barları + telemetri."""
        card = ctk.CTkFrame(self.dash_cards_scroll)
        card.pack(fill="x", padx=4, pady=4)

        # Üst satır: dot + AGV_ID + konum/hedef + navState
        top = ctk.CTkFrame(card, fg_color="transparent")
        top.pack(fill="x", padx=8, pady=(6, 2))
        dot = ctk.CTkLabel(top, text="●",
                           text_color=COL_SUCCESS if s.connected else COL_DANGER,
                           font=self.font_h1)
        dot.pack(side="left")
        id_lbl = ctk.CTkLabel(top, text=agv_id, font=self.font_h2)
        id_lbl.pack(side="left", padx=4)
        wp_lbl = ctk.CTkLabel(
            top,
            text=f"{s.currentWaypoint or '?'} → {s.targetWaypoint or '?'}",
            font=self.font_mono_lg, text_color=COL_TXT_ACCENT)
        wp_lbl.pack(side="left", padx=12)
        state_lbl = ctk.CTkLabel(top, text=s.navState or "?",
                                  text_color=COL_SUBTLE)
        state_lbl.pack(side="left", padx=8)

        # Orta: sensör barları (8 adet, küçük)
        bars_row = ctk.CTkFrame(card, fg_color="transparent")
        bars_row.pack(fill="x", padx=8, pady=2)
        bars: List[SensorBar] = []
        for i in range(8):
            b = SensorBar(bars_row, i)
            b.grid(row=0, column=i, padx=2)
            bars.append(b)

        # Alt: telemetri (sonar, çizgi, kalibre)
        tel = ctk.CTkLabel(card, text="-", font=self.font_small,
                            text_color=COL_MUTED, anchor="w")
        tel.pack(fill="x", padx=10, pady=(2, 6))

        self.dash_agv_cards[agv_id] = card
        self.dash_agv_widgets[agv_id] = {
            "dot": dot, "wp": wp_lbl, "state": state_lbl,
            "bars": bars, "tel": tel,
        }
        # İlk değerleri yansıt
        self._update_dashboard_card(agv_id, s)

    def _update_dashboard_card(self, agv_id: str, s: AGVState):
        """Var olan kartın içeriğini güncelle (rebuild yerine)."""
        w = self.dash_agv_widgets.get(agv_id)
        if w is None:
            return
        w["dot"].configure(text_color=COL_SUCCESS if s.connected else COL_DANGER)
        w["wp"].configure(text=f"{s.currentWaypoint or '?'} → {s.targetWaypoint or '?'}")
        w["state"].configure(text=s.navState or "?")
        for i, b in enumerate(w["bars"]):
            v = s.sensors[i] if i < len(s.sensors) else 0
            b.set_value(v)
        sonar_txt = f"{s.sonarDist:.0f}cm" if s.sonarDist < 999 else "menzil dışı"
        engel = " ⚠ENGEL" if s.obstacle else ""
        cal = "✓" if s.calibrated else "✗"
        w["tel"].configure(
            text=f"Sonar: {sonar_txt}{engel}   Çizgi: {s.linePos:4d}   "
                 f"Kalibre: {cal}   Hız: {s.baseSpeed}   Yön: {s.heading or '-'}",
            text_color=COL_TXT_ERR if s.obstacle else COL_MUTED)

    # ---------- Kalibrasyon Preset paneli ----------
    def _build_preset_panel(self, parent):
        """Sensör sekmesinin üstüne yerleşen kalibrasyon preset yöneticisi."""
        card = ctk.CTkFrame(parent)
        card.pack(fill="x", padx=8, pady=(8, 4))

        ctk.CTkLabel(card, text="KALİBRASYON PRESET",
                     font=self.font_h1).pack(anchor="w", padx=10, pady=(8, 2))
        self.preset_agv_lbl = ctk.CTkLabel(card, text="Aktif AGV: -",
                                            text_color=COL_SUBTLE,
                                            font=self.font_small)
        self.preset_agv_lbl.pack(anchor="w", padx=10)

        # Satır 1: Dropdown + Yükle + Sil
        row1 = ctk.CTkFrame(card, fg_color="transparent")
        row1.pack(fill="x", padx=10, pady=(6, 4))

        ctk.CTkLabel(row1, text="Aktif preset:", width=90, anchor="w"
                     ).pack(side="left")
        self.preset_var = ctk.StringVar(value="(yok)")
        self.preset_menu = ctk.CTkOptionMenu(
            row1, variable=self.preset_var, values=["(yok)"], width=220)
        self.preset_menu.pack(side="left", padx=(0, 8))
        ctk.CTkButton(row1, text="Yükle", width=70,
                      command=self._preset_apply).pack(side="left", padx=2)
        ctk.CTkButton(row1, text="Sil", width=60, fg_color=COL_DANGER,
                      hover_color=COL_DANGER_HOVER,
                      command=self._preset_delete).pack(side="left", padx=2)

        # Satır 2: Son kalibrasyon verisi (min/max okumaları)
        row2 = ctk.CTkFrame(card, fg_color="transparent")
        row2.pack(fill="x", padx=10, pady=(4, 4))
        ctk.CTkLabel(row2, text="Son okuma:", width=90, anchor="w"
                     ).pack(side="left")
        self.preset_data_lbl = ctk.CTkLabel(
            row2, text="(henüz veri yok — Yeni Kalibrasyon yapın)",
            text_color=COL_SUBTLE, font=self.font_mono_sm, anchor="w",
            justify="left",
        )
        self.preset_data_lbl.pack(side="left", fill="x", expand=True)

        # Satır 3: Eylem butonları
        row3 = ctk.CTkFrame(card, fg_color="transparent")
        row3.pack(fill="x", padx=10, pady=(4, 10))
        ctk.CTkButton(row3, text="▶ Yeni Kalibrasyon",
                      fg_color=COL_WARN, hover_color=COL_WARN_HOVER,
                      command=lambda: self._cmd_command("calibrate")
                      ).pack(side="left", padx=(0, 6))
        ctk.CTkButton(row3, text="💾 Preset olarak Kaydet",
                      command=self._preset_save).pack(side="left")

    def _update_preset_panel(self):
        """Aktif AGV'ye göre preset paneli içeriğini yenile."""
        if not hasattr(self, "preset_menu"):
            return   # henüz build edilmedi

        agv = self.active_agv
        self.preset_agv_lbl.configure(
            text=f"Aktif AGV: {agv or '-'}",
            text_color="white" if agv else COL_SUBTLE,
        )

        # Dropdown'u yenile
        names = self.preset_store.names()
        opts = names if names else ["(yok)"]
        self.preset_menu.configure(values=opts)

        # Bu AGV için aktif preset varsa onu seç
        active = self.preset_store.active_per_agv.get(agv, "") if agv else ""
        if active and active in names:
            self.preset_var.set(active)
        else:
            self.preset_var.set(opts[0])

        # Son ham kalibrasyon verisini göster
        data = self._last_calib_data.get(agv, None) if agv else None
        if data:
            mn = " ".join(f"{v:4d}" for v in data.sensorMin)
            mx = " ".join(f"{v:4d}" for v in data.sensorMax)
            self.preset_data_lbl.configure(
                text=f"Min: [ {mn} ]\nMax: [ {mx} ]",
                text_color="white",
            )
        else:
            self.preset_data_lbl.configure(
                text="(henüz veri yok — Yeni Kalibrasyon yapın)",
                text_color=COL_SUBTLE,
            )

    def _preset_apply(self):
        """Seçili preset'i aktif AGV'ye uygula + activePerAgv kaydet."""
        if self.client is None or not self.client.connected:
            self._set_status("Önce server'a bağlan", err=True)
            return
        if not self.active_agv:
            self._set_status("AGV seçili değil", err=True)
            return
        state = self.agvs.get(self.active_agv)
        if state is None or not state.connected:
            self._set_status(f"{self.active_agv} bağlı değil", err=True)
            return
        name = self.preset_var.get()
        p = self.preset_store.get(name)
        if p is None:
            self._set_status("Preset yok", err=True)
            return
        if not p.is_valid():
            self._set_status(f"'{name}' geçersiz (yetersiz range)", err=True)
            return
        if self.client.apply_calibration(self.active_agv, p.sensorMin, p.sensorMax):
            self.preset_store.set_active(self.active_agv, name)
            self._set_status(f"{self.active_agv}: '{name}' uygulandı")
            self._update_preset_panel()
        else:
            self._set_status("Gönderim hata (WS bağlı mı?)", err=True)

    def _preset_delete(self):
        name = self.preset_var.get()
        if name in ("(yok)", ""):
            return
        if self.preset_store.delete(name):
            self._set_status(f"Preset silindi: {name}")
            self._update_preset_panel()

    def _preset_save(self):
        """Son alınan kalibrasyon verisini isimli preset olarak kaydet."""
        if not self.active_agv:
            self._set_status("AGV seçili değil", err=True)
            return
        data = self._last_calib_data.get(self.active_agv)
        if data is None:
            self._set_status("Önce 'Yeni Kalibrasyon' çalıştırın", err=True)
            return
        dlg = ctk.CTkInputDialog(text="Preset adı:", title="Kalibrasyon Preset")
        name = dlg.get_input()
        if not name:
            return
        p = self.preset_store.add(name, data.sensorMin, data.sensorMax,
                                   agvSource=data.agvId)
        # Yeni kaydedilen aktif yap
        self.preset_store.set_active(self.active_agv, p.name)
        self._update_preset_panel()
        self._set_status(f"Preset kaydedildi: {p.name}")

    def _check_auto_apply(self, s: AGVState):
        """AGV disconnect→connect geçişinde aktif preset'i otomatik uygula."""
        if not s.id:
            return
        was = self._was_connected.get(s.id, False)
        self._was_connected[s.id] = s.connected
        if not (s.connected and not was):
            return
        # Yeni bağlandı — aktif preset varsa uygula. WS bağlı olmadan denenmesin.
        p = self.preset_store.active_for(s.id)
        if p is None or not p.is_valid():
            return
        if self.client is None or not self.client.connected:
            return
        if self.client.apply_calibration(s.id, p.sensorMin, p.sensorMax):
            self._set_status(f"{s.id}: '{p.name}' otomatik uygulandı")

    # ---------- Log sekmesi ----------
    def _build_tab_log(self):
        """Tam ekran olay log — filter + temizle. Sensör history Sensors tab'ında."""
        log = self.tab_log
        log.grid_columnconfigure(0, weight=1)
        log.grid_rowconfigure(1, weight=1)

        bar = ctk.CTkFrame(log, fg_color="transparent")
        bar.grid(row=0, column=0, sticky="ew", padx=8, pady=8)
        ctk.CTkLabel(bar, text="OLAY LOG", font=self.font_h1
                     ).pack(side="left", padx=(4, 12))
        ctk.CTkLabel(bar, text="Filtre:").pack(side="left", padx=(0, 4))
        self.log_filter_var = ctk.StringVar(value="Hepsi")
        self.log_filter = ctk.CTkOptionMenu(
            bar, values=["Hepsi"], variable=self.log_filter_var,
            width=140, command=lambda _: self._refresh_log())
        self.log_filter.pack(side="left", padx=4)
        # DEBUG TIMING toggle — WS mesaj akisi + planner tick + dispatch
        # latency'lerini log paneline yansitir. Default kapali (spam'i onler).
        self.debug_timing_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(
            bar, text="DBG timing", variable=self.debug_timing_var,
            command=self._on_debug_timing_toggle,
        ).pack(side="left", padx=(12, 4))
        ctk.CTkButton(bar, text="Temizle", width=80, fg_color=COL_DANGER,
                      hover_color=COL_DANGER_HOVER,
                      command=self._clear_log).pack(side="right", padx=4)

        self.log_text = ctk.CTkTextbox(log, font=self.font_mono)
        self.log_text.grid(row=1, column=0, sticky="nsew", padx=8, pady=(0, 8))
        self.log_text.configure(state="disabled")
        self._log_lines: List[LogEvent] = []

    def _on_debug_timing_toggle(self):
        """DBG timing checkbox callback — WS client'in debug_timing flag'ini
        guncelle. Aktifken WS trafik + planner tick + dispatch latency log
        paneline yansir."""
        enabled = self.debug_timing_var.get()
        if self.client is not None:
            self.client.debug_timing = enabled
        self._add_log(LogEvent(
            agvId="*",
            message=f"[DBG] timing log {'AKTIF' if enabled else 'kapali'}",
            time_ms=0, wall_time=time.time(),
        ))

    # =========================================================================
    # WS Baglanti
    # =========================================================================
    def _connect(self):
        if self.client is not None:
            return
        url = self.url_var.get().strip()
        if not url:
            self._set_status("URL bos olamaz", err=True)
            return
        self.client = AGVClient(url)
        # Reconnect sonrasi DBG timing seçimini geri uygula
        if hasattr(self, "debug_timing_var"):
            self.client.debug_timing = self.debug_timing_var.get()
        self.client.start()
        self.connect_btn.configure(state="disabled")
        self.disconnect_btn.configure(state="normal")

    def _disconnect(self):
        if self.client is None:
            return
        self.client.stop()
        self.client = None
        self._update_connection_indicator(connected=False)
        self.agvs.clear()
        self._refresh_agv_list()
        self._refresh_dashboard_cards()
        self._update_active_agv_indicator()

    # -------------------------------------------------------------------------
    # ACIL DUR - tum AGV'leri durdur + tum CAM kollarini dondur
    # -------------------------------------------------------------------------
    def _emergency_stop(self):
        actions = []

        # 0. Tarama state machine'i durdur (otomatik takip kaldırıldı)
        if self.scan_active:
            self._pickup_scan_stop()
            actions.append("Tarama iptal")

        # 1. Planner mission'larini iptal et + her AGV'ye clearMission/STOP
        try:
            self._planner_clear_all()
            actions.append("Planner mission'lari iptal")
        except Exception:
            pass

        # 2. Tum AGV'lere STOP komutu (WS broadcast) — clearMission'a ek olarak
        if self.client is not None and self.agvs:
            for agv_id in list(self.agvs.keys()):
                try:
                    self.client.command(agv_id, "stop")
                    actions.append(f"AGV {agv_id} STOP")
                except Exception:
                    pass

        # 2. Tum AGV'lerin ESP32-CAM kollarini freeze (F4: per-AGV URLs)
        if hasattr(self, "agv_config_store") and self.agvs:
            import threading, urllib.request
            def freeze_worker(u: str):
                try:
                    with urllib.request.urlopen(f"{u}/arm/freeze", timeout=2.0) as r:
                        r.read()
                except Exception:
                    pass
            for agv_id in self.agvs:
                cfg = self.agv_config_store.get(agv_id)
                base = (cfg.cam_control_url or "").strip().rstrip("/")
                if base:
                    threading.Thread(target=freeze_worker, args=(base,),
                                     daemon=True).start()
                    actions.append(f"{agv_id} kol freeze")

        # 3. Otomatik kup tutma (eklenince): controller varsa estop
        if hasattr(self, "auto_pickup") and self.auto_pickup is not None:
            try:
                self.auto_pickup.estop()
                actions.append("Auto pickup E-STOP")
            except Exception:
                pass

        # 4. UI feedback
        msg = "ACİL DUR: " + (", ".join(actions) if actions else "yapılacak iş yok")
        self._set_status(msg, err=True)
        # Buton'u kisa sure flash et (kullanici Esc bastiysa visual feedback)
        try:
            orig = self.estop_btn.cget("fg_color")
            self.estop_btn.configure(fg_color=COL_DANGER_FLASH)
            self.after(250, lambda: self.estop_btn.configure(fg_color=orig))
        except Exception:
            pass

    def _on_close(self):
        try:
            if self.client is not None:
                self.client.stop()
        finally:
            # Tüm AGV stream reader'larını kapat (F4 — per-AGV dict)
            for reader in list(self.stream_readers.values()):
                try:
                    reader.stop()
                except Exception:
                    pass
            self.stream_readers.clear()
            try:
                self._cam_log_stop()
            except Exception:
                pass
            self.destroy()

    # =========================================================================
    # AGV listesi
    # =========================================================================
    def _refresh_agv_list(self):
        prev_active = self.active_agv   # Auto-select değişimini yakalamak için

        # Mevcut butonlari sil
        for w in self.agv_list_inner.winfo_children():
            w.destroy()
        self.agv_buttons.clear()

        if not self.agvs:
            ctk.CTkLabel(self.agv_list_inner, text="(bos)", text_color=COL_SUBTLE).pack(pady=8)
            self.active_agv = None
            if prev_active is not None:
                self._update_preset_panel()
            return

        for agv_id, s in sorted(self.agvs.items()):
            label = (f"{agv_id}\n"
                     f"{s.currentWaypoint or '?'} → {s.targetWaypoint or '?'}\n"
                     f"{s.navState or 'IDLE'}")
            color = "#1f6aa5" if s.connected else "#444"
            sel_color = "#3a8acf" if (agv_id == self.active_agv) else None

            btn = ctk.CTkButton(
                self.agv_list_inner, text=label, anchor="w", height=64,
                fg_color=(sel_color or color),
                command=lambda a=agv_id: self._select_agv(a),
            )
            btn.pack(fill="x", padx=2, pady=3)
            self.agv_buttons[agv_id] = btn

        # Aktif AGV gecerli mi?
        if self.active_agv not in self.agvs:
            self.active_agv = None
        if self.active_agv is None and self.agvs:
            # Ilk bagli olani sec
            for agv_id, s in self.agvs.items():
                if s.connected:
                    self.active_agv = agv_id
                    break

        # Log + sensör filtreleri guncelle
        opts = ["Hepsi"] + sorted(self.agvs.keys())
        self.log_filter.configure(values=opts)
        self.sensor_filter.configure(values=opts)

        # Auto-select active_agv'yi değiştirdiyse preset paneli + arm paneli + top bar'ı yenile
        if self.active_agv != prev_active:
            self._update_preset_panel()
            self._refresh_arm_panel()
            self._update_active_agv_indicator()

    def _select_agv(self, agv_id: str):
        self.active_agv = agv_id
        self._refresh_agv_list()
        self._update_active_view()
        self._update_preset_panel()
        self._refresh_arm_panel()          # aktif AGV değişti → kol panelini yeniden inşa
        self._update_active_agv_indicator()

    # =========================================================================
    # Aktif AGV gosterimi
    # =========================================================================
    def _update_active_view(self):
        # Harita: her durumda multi-AGV. Aktif AGV varsa onu vurgula
        # (heading + target ring), digerleri kendi renklerinde gozukur.
        self._refresh_operations_map()

        if self.active_agv is None or self.active_agv not in self.agvs:
            self.path_lbl.configure(text="Yol: -")
            self.heading_lbl.configure(text="Yon: -")
            self.tel_state.configure(text="Durum: -")
            self.tel_speed.configure(text="Hiz: -")
            self.tel_linepos.configure(text="Cizgi pos: -")
            self.tel_sonar.configure(text="Sonar: -")
            self.tel_calib.configure(text="Kalibre: -")
            self.pid_status_lbl.configure(text="Aktif AGV: -")
            for b in self.sensor_bars:
                b.set_value(0)
            for b in getattr(self, "calib_sensor_bars", []):
                b.set_value(0)
            return

        s = self.agvs[self.active_agv]

        # Yan paneller
        self.path_lbl.configure(text=f"Yol: {s.path or '-'}")
        self.heading_lbl.configure(text=f"Yon: {s.heading or '-'}")

        # Telemetri
        self.tel_state.configure(text=f"Durum: {s.navState or '-'}")
        self.tel_speed.configure(text=f"Hiz: {s.baseSpeed}")
        self.tel_linepos.configure(text=f"Cizgi pos: {s.linePos}")
        sonar_txt = f"{s.sonarDist:.0f} cm" if s.sonarDist < 999 else "menzil disi"
        engel = " ENGEL!" if s.obstacle else ""
        self.tel_sonar.configure(text=f"Sonar: {sonar_txt}{engel}",
                                  text_color=COL_TXT_ERR if s.obstacle else "white")
        self.tel_calib.configure(text=f"Kalibre: {'EVET' if s.calibrated else 'HAYIR'}",
                                  text_color=COL_TXT_BAGLI if s.calibrated else "orange")

        # Sensor barlari (Operations tab) + Kalibrasyon tab canli barlari (F5)
        for i, b in enumerate(self.sensor_bars):
            v = s.sensors[i] if i < len(s.sensors) else 0
            b.set_value(v)
        for i, b in enumerate(getattr(self, "calib_sensor_bars", [])):
            v = s.sensors[i] if i < len(s.sensors) else 0
            b.set_value(v)

        # PID/hiz durumu
        self.pid_status_lbl.configure(
            text=f"Mevcut: Kp={s.Kp:.3f}  Ki={s.Ki:.3f}  Kd={s.Kd:.3f}  Hiz={s.baseSpeed}"
        )

    # =========================================================================
    # Komutlar
    # =========================================================================
    def _ensure_active(self) -> Optional[tuple[AGVClient, str]]:
        """Aktif (client, agv_id) tuple'i dondurur. Onkosul: client bagli +
        aktif AGV secili. Aksi halde status hatasi yazip None."""
        if self.client is None or not self.client.connected:
            self._set_status("Once server'a baglan", err=True); return None
        if not self.active_agv:
            self._set_status("Aktif AGV yok", err=True); return None
        return (self.client, self.active_agv)

    def _cmd_set_position(self):
        res = self._ensure_active()
        if res is None: return
        client, agv = res
        wp = self.pos_var.get()
        if client.set_position(agv, wp):
            self._set_status(f"{agv}: konum {wp}")

    def _cmd_set_target(self):
        res = self._ensure_active()
        if res is None: return
        _client, agv = res
        wp = self.tgt_var.get()
        self._planner_assign_target(agv, wp)

    def _cmd_command(self, c: str):
        res = self._ensure_active()
        if res is None: return
        client, agv = res

        # Calibrate debounce — kalibrasyon ~10 sn surer, art arda komut yollanamaz
        if c == "calibrate":
            last = self._last_calibrate_sent.get(agv, 0.0)
            since = time.time() - last
            if since < self.CALIBRATE_DEBOUNCE_SEC:
                wait = self.CALIBRATE_DEBOUNCE_SEC - since
                self._set_status(
                    f"{agv}: Kalibrasyon zaten devam ediyor ({wait:.0f}s bekleyin)",
                    err=True,
                )
                return

        if client.command(agv, c):
            if c == "calibrate":
                self._last_calibrate_sent[agv] = time.time()
                self._start_calib_countdown(agv)
            else:
                self._set_status(f"{agv}: {c}")

    def _cmd_set_pid(self):
        res = self._ensure_active()
        if res is None: return
        client, agv = res
        try:
            kp = float(self.kp_var.get())
            ki = float(self.ki_var.get())
            kd = float(self.kd_var.get())
        except ValueError:
            self._set_status("PID degerleri sayi olmali", err=True); return
        if client.set_pid(agv, kp, ki, kd):
            self._set_status(f"{agv}: PID Kp={kp} Ki={ki} Kd={kd}")

    def _cmd_set_speed(self):
        res = self._ensure_active()
        if res is None: return
        client, agv = res
        speed = self.speed_var.get()
        if client.set_speed(agv, speed):
            self._set_status(f"{agv}: hiz {speed}")

    def _on_map_target(self, wp: str):
        res = self._ensure_active()
        if res is None: return
        _client, agv = res
        self.tgt_var.set(wp)
        self._planner_assign_target(agv, wp)

    # =========================================================================
    # Multi-AGV Planner integration (background)
    # =========================================================================
    # Operations'tan "Hedef Ata" / harita tiklamasi planner.add_mission'a gider.
    # Planner cakisma kontrolu yapar, AGV'ye setHop emirleri uretir.
    # AGV'den gelen hopComplete event'i auto-tick tetikler — AGV durmadan devam.
    # =========================================================================

    def _planner_assign_target(self, agv_id: str, goal: str) -> None:
        """Operations sekmesinden gelen 'Hedef Ata' istegini planner uzerinden
        AGV'ye yonlendir.

        Durum kontrolu:
          - konum yok → reddet
          - gecersiz hedef → reddet
          - start == goal → sessiz "zaten orada" mesaji
          - AGV mesgul (navState != IDLE) → popup: BEKLE (kuyruga ekle) /
            IPTAL (yoksay). Hizli ardisik assignTarget'larin mission churn
            yaratmasini onler — eski log'da firmware "TURNING mesgul, REDDEDILDI"
            ve PC replan kaosu olusuyordu.
        """
        state = self.agvs.get(agv_id)
        start = (state.currentWaypoint or "").strip() if state else ""
        nav_state = (state.navState if state else "")
        # DEBUG log: hangi pos'tan baslatiyoruz
        self._add_log(LogEvent(
            agvId=agv_id,
            message=f"[PLN] assignTarget {agv_id}→{goal}: state.pos={start or '-'}, "
                    f"navState={nav_state}",
            time_ms=0, wall_time=time.time(),
        ))
        if not start:
            self._set_status(
                f"{agv_id}: konum bilinmiyor (RFID okutun veya setPosition)",
                err=True,
            )
            return
        if goal not in self.fleet_graph.nodes:
            self._set_status(f"{agv_id}: gecersiz hedef {goal}", err=True)
            return
        # start == goal: zaten orada, sessizce reddet
        if start == goal:
            self._set_status(f"{agv_id}: zaten {goal} konumunda")
            return

        # AGV mesgul mu? IDLE disinda popup ile sor.
        if nav_state and nav_state != "IDLE":
            import tkinter.messagebox as mb
            choice = mb.askyesno(
                "AGV mesgul",
                f"{agv_id} su anda {nav_state} durumunda.\n"
                f"Yeni hedef: {goal}\n\n"
                f"EVET = mevcut gorev bittikten sonra basla (kuyruga ekle)\n"
                f"HAYIR = yeni hedefi yoksay (mevcut gorev devam)",
                parent=self,
            )
            if not choice:
                self._set_status(
                    f"{agv_id}: yeni hedef yoksayildi ({nav_state} devam)"
                )
                return
            # Kuyruga ekle: existing mission'i SILME, add_mission queue branch'ine
            # dussun. _finish_mission queued'i otomatik aktive edecek.
            m = self.fleet_planner.add_mission(
                agv_id, goal=goal, start=start,
            )
            self._add_log(LogEvent(
                agvId=agv_id,
                message=f"[PLN] mission QUEUED: ->{goal}, FIFO={m.fifo_order} "
                        f"(mevcut bitince basla)",
                time_ms=0, wall_time=time.time(),
            ))
            self._set_status(
                f"{agv_id}: {goal} kuyruga eklendi (FIFO={m.fifo_order})"
            )
            return

        # IDLE — normal akis: eski mission varsa temizle + yeni mission
        if agv_id in self.fleet_planner.missions:
            self.fleet_planner.reservations.release_all_for(agv_id)
            del self.fleet_planner.missions[agv_id]
        # Dedup sifirla — yeni mission yeni emirlerle gelsin
        self._last_dispatched_hop.pop(agv_id, None)
        m = self.fleet_planner.add_mission(agv_id, goal=goal, start=start)
        if len(m.path) <= 1 and start != goal:
            self._set_status(
                f"{agv_id}: {start}->{goal} ulasilamaz (parked AGV mu var?)",
                err=True,
            )
            return
        # DEBUG log — path + cargo bilgisi
        path_str = " -> ".join(m.path)
        cargo_s  = " [CARGO]" if m.has_cargo else ""
        self._add_log(LogEvent(
            agvId=agv_id,
            message=f"[PLN] mission add: {start}->{goal}{cargo_s}, "
                    f"FIFO={m.fifo_order}, path: {path_str}",
            time_ms=0, wall_time=time.time(),
        ))
        # Reservation snapshot
        resv = self.fleet_planner.reservations.node_owner
        if resv:
            resv_str = ", ".join(f"{n}={o}" for n, o in sorted(resv.items()))
            self._add_log(LogEvent(
                agvId="*",
                message=f"[PLN] reservations: {resv_str}",
                time_ms=0, wall_time=time.time(),
            ))
        # Yol karsilastirma: diger AGV'lerin path'i ile kesisen node'lari logla
        for other_id, other_m in self.fleet_planner.missions.items():
            if other_id == agv_id or not other_m.path:
                continue
            common = set(m.path) & set(other_m.path)
            if common:
                self._add_log(LogEvent(
                    agvId=agv_id,
                    message=f"[PLN] CAKISMA NOKTALARI {agv_id} vs {other_id} "
                            f"({other_m.state.value}): {','.join(sorted(common))}",
                    time_ms=0, wall_time=time.time(),
                ))
        self._set_status(f"{agv_id}: hedef {goal} (planner)")
        # Hemen ilk tick'i at — setHop AGV'ye gitsin
        self._planner_tick_and_dispatch()

    def _planner_tick_and_dispatch(self) -> None:
        """planner.tick() cagir, donen komutlari dispatch et + tick ozeti logla.

        DEBUG: tick suresi ve dispatch sayisi olculur — uzun tick'ler darbogaz
        isaretidir."""
        if self.client is None:
            return
        debug = getattr(self.client, "debug_timing", False)
        t0 = time.time() if debug else 0.0
        cmds = self.fleet_planner.tick()
        tick_ms = (time.time() - t0) * 1000 if debug else 0.0
        # Tick ozeti — sadece WAIT veya DONE icin
        # (NORMAL ve YIELD dispatch_hop'ta loglanir)
        non_normal = [(a, c) for a, c in cmds.items()
                      if c.action not in (HopAction.NORMAL, HopAction.YIELD)]
        if non_normal:
            parts = [f"{a}={c.action.value}({c.reason})" for a, c in non_normal]
            self._add_log(LogEvent(
                agvId="*",
                message=f"[PLN] tick: {', '.join(parts)}",
                time_ms=0, wall_time=time.time(),
            ))
        if debug:
            n_dispatch = sum(1 for c in cmds.values()
                             if c.action in (HopAction.NORMAL, HopAction.YIELD))
            self._add_log(LogEvent(
                agvId="*",
                message=f"[DBG] tick {tick_ms:.1f}ms, "
                        f"{len(cmds)} mission, {n_dispatch} dispatch",
                time_ms=0, wall_time=time.time(),
            ))
        for agv_id, c in cmds.items():
            self._planner_dispatch_hop(agv_id, c)

    def _planner_dispatch_hop(self, agv_id: str, c: "HopCommand") -> None:
        """tick() ciktisindaki bir komutu AGV'ye yolla.

        PC tarafinda state-aware filter YOK — firmware race guard zaten
        mid-flight full-hop'lari reddediyor (NAV_FOLLOWING/TURNING'de).
        Ekstra PC filtresi continuous hop'u da engelliyordu (PC'nin
        navState'i 500ms eski olabiliyor). Sadece dedup ile gondermek
        yeterli; firmware sameHop branch'i continuous'u handle eder.

        YIELD action da NORMAL gibi dispatch edilir — firmware acisindan
        yan park bir setHop'tur (park node'una git). State'i planner takip eder."""
        if self.client is None:
            return
        if c.action not in (HopAction.NORMAL, HopAction.YIELD):
            return
        if not c.next_ or c.next_ == c.from_:
            return

        # Dedup — ayni (from, next, after) tekrar gonderme
        key = (c.from_, c.next_, c.after_)
        if self._last_dispatched_hop.get(agv_id) == key:
            return
        self._last_dispatched_hop[agv_id] = key

        m = self.fleet_planner.missions.get(agv_id)
        mpath = " -> ".join(m.path) if m else "(yok)"
        mgoal = m.goal if m else None
        after_s = c.after_ or "-"
        self._add_log(LogEvent(
            agvId=agv_id,
            message=f"[PLN] dispatch {c.from_}>{c.next_}>{after_s} "
                    f"goal={mgoal} ({c.reason}) | path: {mpath}",
            time_ms=0, wall_time=time.time(),
        ))
        self.client.set_hop(agv_id, c.from_, c.next_, c.after_, goal=mgoal)

    def _planner_on_hop_complete(self, agv_id: str, node: str) -> None:
        """AGV'den hopComplete event'i geldi. Planner'i guncelle ve auto-tick
        ile bir sonraki setHop'u gonder (AGV kesintisiz devam).

        DEBUG: bu noktadan dispatch'e kadar gecen sure (PC-side reaction
        latency) olculur. Yuksek deger PC tarafinda darbogaz isaretidir."""
        m = self.fleet_planner.missions.get(agv_id)
        if m is None:
            self._add_log(LogEvent(
                agvId=agv_id,
                message=f"[PLN] hopComplete {node} — mission yok, ignored",
                time_ms=0, wall_time=time.time(),
            ))
            return
        debug = self.client is not None and getattr(self.client, "debug_timing", False)
        t_react = time.time() if debug else 0.0
        pos_before  = m.pos
        path_before = list(m.path)
        self.fleet_planner.on_hop_complete(agv_id, node)
        # Log: pos guncellemesi + replan tetiklendiyse goster
        m = self.fleet_planner.missions.get(agv_id)
        if m is None:
            # Mission tamamlandi → silinmis olabilir (parked durdu)
            self._add_log(LogEvent(
                agvId=agv_id,
                message=f"[PLN] hopComplete {pos_before}→{node}, mission COMPLETE",
                time_ms=0, wall_time=time.time(),
            ))
        elif path_before != m.path:
            self._add_log(LogEvent(
                agvId=agv_id,
                message=f"[PLN] hopComplete {pos_before}→{node}, "
                        f"PATH REPLAN: {'-'.join(path_before)} → "
                        f"{'-'.join(m.path)}",
                time_ms=0, wall_time=time.time(),
            ))
        # Auto-step: AGV durmadan devam etsin
        self._planner_tick_and_dispatch()
        if debug:
            dt = (time.time() - t_react) * 1000
            self._add_log(LogEvent(
                agvId=agv_id,
                message=f"[DBG] hopComplete→dispatch toplam {dt:.1f}ms",
                time_ms=0, wall_time=time.time(),
            ))

    def _planner_on_agv_position(self, agv_id: str, node: str) -> None:
        """AGV status'undeki currentWaypoint planner'in bildiginden farkliysa
        senkronize et (drift correction)."""
        if not node:
            return
        m = self.fleet_planner.missions.get(agv_id)
        if m is None or m.pos == node:
            return
        pos_before  = m.pos
        path_before = list(m.path)
        self.fleet_planner.set_agv_position(agv_id, node)
        m = self.fleet_planner.missions.get(agv_id)
        if m is None:
            return
        if path_before != m.path:
            self._add_log(LogEvent(
                agvId=agv_id,
                message=f"[PLN] drift {pos_before}→{node}, "
                        f"PATH REPLAN: {'-'.join(m.path)}",
                time_ms=0, wall_time=time.time(),
            ))
        else:
            self._add_log(LogEvent(
                agvId=agv_id,
                message=f"[PLN] drift {pos_before}→{node} (pos sync)",
                time_ms=0, wall_time=time.time(),
            ))

    def _planner_clear_all(self) -> None:
        """Acil dur veya manual clear: tum mission'lari iptal et + AGV'lere
        clearMission yolla."""
        affected = list(self.fleet_planner.missions.keys())
        if self.client is not None:
            for agv_id in affected:
                self.client.clear_mission(agv_id)
        # Yeni temiz planner + dedup sifir
        self.fleet_planner = FleetPlanner(self.fleet_graph)
        self._last_dispatched_hop.clear()
        if affected:
            self._add_log(LogEvent(
                agvId="*",
                message=f"[PLN] CLEAR ALL — mission iptal: {', '.join(affected)}",
                time_ms=0, wall_time=time.time(),
            ))

    # AGV renkleri (Operations haritasinda her AGV farkli renk gozuksun)
    _FLEET_COLORS = [
        "#3fbf66",   # yesil
        "#3a9fff",   # mavi
        "#ff8c3a",   # turuncu
        "#d65fd6",   # magenta
        "#f5d142",   # sari
    ]

    def _refresh_operations_map(self) -> None:
        """Operations haritasini multi-AGV moda guncelle. Tum bagli AGV'lerin
        anlik pozisyonlarini gosterir; aktif AGV ek olarak target ring +
        heading arrow gosterir."""
        fleet: List[FleetAGVDisplay] = []
        # Deterministik renk: AGV id'ye gore (AGV_1=yesil, AGV_2=mavi, ...)
        sorted_ids = sorted(self.agvs.keys())
        for idx, agv_id in enumerate(sorted_ids):
            s = self.agvs[agv_id]
            if not s.currentWaypoint:
                continue   # konum bilinmiyor, haritaya konmaz
            color = self._FLEET_COLORS[idx % len(self._FLEET_COLORS)]
            # Planlanan path: önce planner mission'ından, yoksa AGV'den gelen status
            path_list: List[str] = []
            m = self.fleet_planner.missions.get(agv_id)
            if m and m.path:
                # Mevcut pos sonrasini goster (gecmiste kalan node'lar zaten)
                if m.pos in m.path:
                    idx_p = m.path.index(m.pos)
                    path_list = m.path[idx_p:]
                else:
                    path_list = list(m.path)
            elif s.path:
                path_list = [p for p in s.path.split(">") if p]

            fleet.append(FleetAGVDisplay(
                agv_id     = agv_id,
                pos        = s.currentWaypoint,
                path       = path_list,
                color      = color,
                state      = s.navState or "",
                heading    = s.heading or "",
                is_active  = (agv_id == self.active_agv),
                target     = (s.targetWaypoint or None) if agv_id == self.active_agv else None,
            ))
        self.map_widget.set_fleet_state(fleet)

    # =========================================================================
    # F4 — Per-AGV kamera yönetimi
    # =========================================================================
    def _cam_connect(self, agv_id: str):
        """Belirtilen AGV'nin ESP32-CAM'ından MJPEG stream'i aç."""
        if agv_id in self.stream_readers:
            return                                  # zaten açık
        cfg = self.agv_config_store.get(agv_id)
        url = (cfg.cam_stream_url or "").strip()
        if not url:
            self._set_status(f"{agv_id}: cam_stream_url bos", err=True)
            return
        reader = StreamReader(url)
        if not reader.start():
            if agv_id in self.cam_info_lbls:
                self.cam_info_lbls[agv_id].configure(text="HATA: akışa bağlanılamadı")
            return
        self.stream_readers[agv_id] = reader
        self.cam_last_count[agv_id] = 0
        self.cam_last_time[agv_id]  = time.time()
        self.cam_fps[agv_id]        = 0.0
        # ESP32-CAM log polling AGV connect olunca otomatik baslar
        # (agv_update connect transition handler'da), burada gerek yok.

    def _cam_disconnect(self, agv_id: str):
        """Belirtilen AGV'nin stream'ini kapat, görsel referansları temizle."""
        if agv_id not in self.stream_readers:
            return
        try:
            self.stream_readers[agv_id].stop()
        except Exception:
            pass
        del self.stream_readers[agv_id]
        self.cam_fps.pop(agv_id, None)
        self.cam_last_count.pop(agv_id, None)
        self.cam_last_time.pop(agv_id, None)
        # Önce widget image'i temizle, sonra ref'i bırak (Tcl pyimage lifetime)
        if agv_id in self.cam_video_lbls:
            self.cam_video_lbls[agv_id].configure(image=None, text="Akış kapalı")
        self._cam_img_refs.pop(agv_id, None)
        if not self.stream_readers:
            self._cam_log_stop()

    # ----- ESP32-CAM log polling (per-AGV) -----
    def _cam_log_start_for_agv(self, agv_id: str):
        """AGV connect olunca cagrilir — agv_config'deki cam URL'sini al ve
        polling thread baslat. Mevcut thread varsa atla."""
        import threading
        from urllib.parse import urlparse
        if not agv_id:
            return
        existing = self._cam_log_threads.get(agv_id)
        if existing is not None and existing.is_alive():
            return
        cfg = self.agv_config_store.get(agv_id)
        host = urlparse(cfg.cam_stream_url).hostname if cfg.cam_stream_url else None
        if not host:
            return
        self._cam_log_last_seq[agv_id]  = 0
        self._cam_log_threads[agv_id]   = threading.Thread(
            target=self._cam_log_loop, args=(agv_id, host), daemon=True,
        )
        self._cam_log_threads[agv_id].start()

    def _cam_log_stop_for_agv(self, agv_id: str):
        """AGV disconnect olunca cagrilir. Thread bir sonraki iterasyonda cikar
        (stop_evt set + dict'ten kaldirildi → kontrol gor)."""
        self._cam_log_threads.pop(agv_id, None)
        self._cam_log_last_seq.pop(agv_id, None)
        self._cam_grip_last_seq.pop(agv_id, None)

    def _cam_log_stop(self):
        """Tum cam polling thread'lerini durdur."""
        self._cam_log_stop_evt.set()
        self._cam_log_threads.clear()

    def _cam_log_loop(self, agv_id: str, host: str):
        """Per-AGV poll. AGV thread dict'ten cikarilmissa kendi kendine sonlanir."""
        import urllib.request, json
        while not self._cam_log_stop_evt.is_set():
            # AGV disconnect oldu → thread cikar
            if agv_id not in self._cam_log_threads:
                return
            try:
                last_seq = self._cam_log_last_seq.get(agv_id, 0)
                url = f"http://{host}/poll?logSince={last_seq}"
                with urllib.request.urlopen(url, timeout=3.0) as r:
                    data = json.loads(r.read().decode("utf-8", errors="replace"))

                cam_id = str(data.get("id", "CAM?"))
                arm    = data.get("arm", {}) or {}

                # Arm state cache (aktif AGV ise UI guncellenir)
                if agv_id == self.active_agv:
                    self._cam_arm_state = {
                        "camId":   cam_id,
                        "servos":  list(arm.get("servos", [])),
                        "targets": list(arm.get("targets", [])),
                        "magnet":  bool(arm.get("magnet", 0)),
                        "speedMs": int(arm.get("speedMs", 40)),
                        "led":     int(arm.get("led", 0)),
                    }

                # Grip event — seq degisirse vurgulu log (basit text "GRIP: kup
                # TUTULDU" zaten log entry olarak gelecek; bu ekstra prefix ile
                # akiska aciklik ekler ve PC tarafinda state machine'e baglanir).
                grip_seq  = int(arm.get("gripEventSeq", 0))
                grip_type = str(arm.get("gripEvent", "none"))
                last_grip = self._cam_grip_last_seq.get(agv_id, 0)
                if grip_seq > last_grip and grip_type in ("held", "lost"):
                    self._cam_grip_last_seq[agv_id] = grip_seq
                    icon = "🎯" if grip_type == "held" else "❗"
                    label = "KUP TUTULDU" if grip_type == "held" else "KUP DUSTU"
                    self.after(0, lambda a=agv_id, ic=icon, lb=label, s=grip_seq:
                        self._add_log(LogEvent(
                            agvId=a, message=f"{ic} {lb} (seq={s})",
                            time_ms=0, wall_time=time.time(),
                        )))

                # Log entry'leri
                log_blk = data.get("log", {}) or {}
                cur_seq = int(log_blk.get("seq", 0))
                entries = log_blk.get("entries", []) or []
                entries.sort(key=lambda e: int(e.get("seq", 0)))
                for e in entries:
                    seq_i = int(e.get("seq", 0))
                    if seq_i <= self._cam_log_last_seq.get(agv_id, 0):
                        continue
                    self._cam_log_last_seq[agv_id] = seq_i
                    msg  = str(e.get("msg", ""))
                    ms_i = int(e.get("ms", 0))
                    self.after(0, lambda c=cam_id, m=msg, t=ms_i:
                        self._add_log(LogEvent(agvId=c, message=m, time_ms=t)))
                if cur_seq > self._cam_log_last_seq.get(agv_id, 0):
                    self._cam_log_last_seq[agv_id] = cur_seq
            except Exception:
                pass
            # 1.5 sn aralikla yokla, stop sinyalinde hemen cik
            self._cam_log_stop_evt.wait(1.5)

    # NOT: _on_cam_toggle / _capture_background / _clear_background metotlari
    # eski background-subtraction tabanli tespit pipeline'i ile birlikte
    # kaldirildi. Yeni detector eklendiginde burada karsiliklari yazilacak.

    # ---------- Kapma kalibrasyonu: konfig I/O ----------
    def _pickup_load_config(self) -> Dict:
        import json
        try:
            with open(PICKUP_CFG_PATH, "r", encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            return {}
        except Exception as e:
            print(f"pickup_config yuklenemedi: {e}")
            return {}

    def _pickup_save_config(self):
        import json
        try:
            with open(PICKUP_CFG_PATH, "w", encoding="utf-8") as f:
                json.dump(self.pickup_cfg, f, indent=2)
            self._set_status(f"Kapma konfig kaydedildi -> {PICKUP_CFG_PATH}")
        except Exception as e:
            self._set_status(f"Konfig kaydedilemedi: {e}", err=True)

    # Tarama tipi -> default base aralıgı + loop. Hepsi loop, sınıra varınca
    # gripper alternate (ileri:73° / geri:100°) + yön değişimi.
    SCAN_TYPES = {
        "right":   {"start":  0, "end":  96, "loop": True},
        "left":    {"start": 96, "end": 180, "loop": True},
        "front":   {"start": 50, "end": 150, "loop": True},
        "general": {"start":  0, "end": 180, "loop": True},
    }
    SCAN_LABELS = {"left": "Sol", "right": "Sag", "front": "On", "general": "Genel"}

    # Tarama sırasında sabit poz + iki yönlü gripper açıları
    SCAN_POSE_DEFAULT = {
        "shoulder":     120,
        "elbow":         20,
        "gripper_fwd":   73,    # ileri yön süpürmesinde
        "gripper_back": 100,    # sınıra varınca tersine dönmeden önce
    }
    # Sınıra varınca yön değişimi öncesi gripper hareketi tamamlanma süresi
    SCAN_TURNAROUND_MS = 1500

    def _pickup_scan_range(self, save_field: str):
        """Aktif tarama tipinin (start, end, loop) tuple'ini dondur — config'te
        kayitliysa onu, yoksa varsayilani."""
        default = self.SCAN_TYPES.get(save_field, {"start": 0, "end": 180, "loop": True})
        entry = self.pickup_cfg.get(f"scan_{save_field}", {}) if save_field else {}
        start = int(entry.get("start", default["start"]))
        end   = int(entry.get("end",   default["end"]))
        loop  = default["loop"]
        return start, end, loop

    def _pickup_scan_pose(self):
        """Tarama sırasında kullanılacak shoulder/elbow/gripper_fwd/gripper_back —
        config'te varsa onu, yoksa SCAN_POSE_DEFAULT."""
        cfg  = self.pickup_cfg.get("scan_pose", {})
        out  = dict(self.SCAN_POSE_DEFAULT)
        out.update({k: int(v) for k, v in cfg.items() if k in out})
        return out

    def _pickup_limits_text(self) -> str:
        lines = []
        for key in ("left", "right", "front", "general"):
            entry = self.pickup_cfg.get(f"scan_{key}", {})
            s = entry.get("start", "?"); e = entry.get("end", "?")
            lines.append(f"{self.SCAN_LABELS[key]:6s}: {s}° -> {e}°")
        return "\n".join(lines)

    def _pickup_save_scan_pose(self):
        """Robot Kol sekmesinde manuel ayarlanan shoulder+elbow'u tarama pozu olarak sakla."""
        servos = self._cam_arm_state.get("servos") or []
        if len(servos) < 3 or not all(isinstance(s, dict) for s in servos[:3]):
            self._set_status("Tarama pozu icin once /arm/state al (Durum Sorgula)", err=True)
            return
        shoulder = int(servos[1].get("target", 0))
        elbow    = int(servos[2].get("target", 0))
        self.pickup_cfg["scan_pose"] = {"shoulder": shoulder, "elbow": elbow}
        self.scan_pose_lbl.configure(text=f"Tarama poz: sh={shoulder}  el={elbow}")
        self._set_status(f"Tarama pozu kaydedildi (sh={shoulder}, el={elbow})")

    # ---------- Kapma kalibrasyonu: tarama state machine ----------
    def _pickup_scan_start(self, save_field: str):
        agv_id = self.active_agv
        if not agv_id:
            self._set_status("Tarama için önce sol panelden AGV seçin", err=True)
            return
        if self._arm_base_url(agv_id) is None:
            self._set_status("Tarama icin gecerli kamera URL'si gerekli", err=True)
            return
        if self.scan_active:
            self._pickup_scan_stop()

        # Aktif tarama hangi AGV için yapıldı — _pickup_scan_step buradan okur
        self.scan_agv_id = agv_id

        # Sabit poz (shoulder + elbow) + gripper acılari (ileri/geri yon)
        pose = self._pickup_scan_pose()
        self.scan_gripper_fwd  = pose["gripper_fwd"]
        self.scan_gripper_back = pose["gripper_back"]
        try:
            self._arm_http_get(agv_id, f"/servo?id=1&a={pose['shoulder']}")
            self._arm_http_get(agv_id, f"/servo?id=2&a={pose['elbow']}")
            self._arm_http_get(agv_id, f"/servo?id=3&a={self.scan_gripper_fwd}")
        except Exception:
            pass

        # Kayitli (veya default) baslangic/bitis acisi
        start, end, loop = self._pickup_scan_range(save_field)
        low  = min(start, end)
        high = max(start, end)

        self.scan_current_angle = start
        self.scan_min        = low
        self.scan_max        = high
        self.scan_direction  = +1 if end > start else -1
        self.scan_loop       = loop
        self.scan_save_field = save_field
        self.scan_active     = True

        # Base'i baslangica getir (kullanici icin tutarli)
        self._arm_http_get(agv_id, f"/servo?id=0&a={start}")

        # Kayit butonlarini aktifle
        label = self.SCAN_LABELS.get(save_field, save_field)
        self.scan_save_start_btn.configure(state="normal",
                                           text=f"📍 {label} Baslangic")
        self.scan_save_end_btn.configure(state="normal",
                                         text=f"📍 {label} Bitis")

        # Ilk adimi tetikle, after-zinciri kendi devam eder
        self._pickup_scan_step()

    def _pickup_scan_stop(self):
        if not self.scan_active:
            return
        self.scan_active = False
        if hasattr(self, "scan_save_start_btn"):
            self.scan_save_start_btn.configure(state="disabled", text="📍 Baslangic")
        if hasattr(self, "scan_save_end_btn"):
            self.scan_save_end_btn.configure(state="disabled", text="📍 Bitis")
        if hasattr(self, "scan_status_lbl"):
            self.scan_status_lbl.configure(
                text=f"Durdu (son aci: {self.scan_current_angle}°)",
                text_color=COL_MUTED)

    def _pickup_scan_step(self):
        if not self.scan_active:
            return

        next_angle = self.scan_current_angle + self.scan_direction * self.scan_step_deg
        hit_boundary = False

        if next_angle >= self.scan_max:
            next_angle = self.scan_max
            hit_boundary = True
        elif next_angle <= self.scan_min:
            next_angle = self.scan_min
            hit_boundary = True

        # Base'i yeni acıya gönder (sınır olsa bile son adım yazılır)
        self.scan_current_angle = next_angle
        agv_id = getattr(self, "scan_agv_id", None) or self.active_agv
        if agv_id:
            self._arm_http_get(agv_id, f"/servo?id=0&a={next_angle}")

        if hit_boundary:
            if not self.scan_loop:
                self._finish_scan(next_angle)
                return
            # Loop modu — yön değiştir ve gripper'i alternate et.
            # Yeni yön +1 ise gripper_fwd, -1 ise gripper_back kullan.
            self.scan_direction = -self.scan_direction
            new_gripper = (self.scan_gripper_fwd if self.scan_direction == +1
                           else self.scan_gripper_back)
            if agv_id:
                self._arm_http_get(agv_id, f"/servo?id=3&a={new_gripper}")
            self.scan_status_lbl.configure(
                text=f"Sınır {next_angle}°: gripper→{new_gripper}°, yön ters çevriliyor...",
                text_color=COL_TXT_WARN)
            # Gripper hareketinin bitmesi için bekle, sonra base hareketine devam
            self.after(self.SCAN_TURNAROUND_MS, self._pickup_scan_step)
            return

        self.scan_status_lbl.configure(
            text=f"Tarama base={next_angle}°  yön={self.scan_direction:+d}  "
                 f"({self.scan_min}..{self.scan_max})",
            text_color=COL_TXT_OK)
        self.after(self.scan_step_ms, self._pickup_scan_step)

    def _finish_scan(self, last_angle: int):
        self.scan_active = False
        self.scan_save_start_btn.configure(state="disabled", text="📍 Baslangic")
        self.scan_save_end_btn.configure(state="disabled", text="📍 Bitis")
        self.scan_status_lbl.configure(
            text=f"Tarama tamamlandi (son aci: {last_angle}°)", text_color=COL_MUTED)

    def _pickup_save_endpoint(self, which: str):
        """Aktif taramanin start/end alanina current_angle'i yaz."""
        if not self.scan_save_field:
            self._set_status("Aktif tarama yok", err=True)
            return
        key = f"scan_{self.scan_save_field}"
        entry = self.pickup_cfg.setdefault(key, {})
        entry[which] = self.scan_current_angle
        self.scan_limits_lbl.configure(text=self._pickup_limits_text())
        label = self.SCAN_LABELS.get(self.scan_save_field, self.scan_save_field)
        self._set_status(f"{label} {which} = {self.scan_current_angle}° "
                         f"(henuz JSON'a yazilmadi)")

    def _pickup_save_start(self):
        self._pickup_save_endpoint("start")

    def _pickup_save_end(self):
        self._pickup_save_endpoint("end")

    # ---------- Otomatik takip — KALDIRILDI ----------
    # Tüm vision-servoing kodu (_on_track_*, _toggle_tracking, _start_tracking,
    # _stop_tracking, _tracking_step) kaldırıldı. Yeni tespit pipeline'ı
    # yazıldıktan sonra burada yeniden kurulacak.

    def _process_camera(self):
        """F4: Her aktif stream_reader için frame oku + ilgili video label'i güncelle.
        Health bar'daki FPS göstergesi de buradan beslenir."""
        if not self.stream_readers:
            return

        now = time.time()
        for agv_id, reader in list(self.stream_readers.items()):
            frame, count = reader.read()
            if frame is None:
                continue

            # Per-AGV FPS (sağlık şeridinden okunur)
            last_t = self.cam_last_time.get(agv_id, now)
            last_c = self.cam_last_count.get(agv_id, count)
            if now - last_t >= 0.5:
                self.cam_fps[agv_id]       = (count - last_c) / (now - last_t)
                self.cam_last_count[agv_id] = count
                self.cam_last_time[agv_id]  = now

            # YOLO tespit (checkbox aktifse) — frame'i in-place modify et
            if self.detect_vars.get(agv_id) and self.detect_vars[agv_id].get():
                det = get_detector()
                dets = det.detect(frame)
                det.draw(frame, dets)
                self.detect_last_ms[agv_id] = det.last_infer_ms
                self.detect_last_n[agv_id] = len(dets)
                info = self.detect_info_lbls.get(agv_id) if hasattr(self, "detect_info_lbls") else None
                if info is not None:
                    info.configure(
                        text=f"{len(dets)} tespit  |  {det.last_infer_ms:.0f} ms")

            # Video label varsa frame'i göster (dual'da küçük, single'da büyük)
            vlbl = self.cam_video_lbls.get(agv_id)
            if vlbl is None:
                continue
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil = Image.fromarray(rgb)
            # Hedef boyut: dual=320×240, single=640×480 (vlbl genişliğinden tahmin)
            try:
                target_w = int(vlbl.cget("width") or 320)
                target_h = int(vlbl.cget("height") or 240)
            except Exception:
                target_w, target_h = 320, 240
            pil.thumbnail((target_w, target_h), Image.Resampling.LANCZOS)
            ctk_img = ctk.CTkImage(light_image=pil, dark_image=pil, size=pil.size)
            vlbl.configure(image=ctk_img, text="")
            self._cam_img_refs[agv_id] = ctk_img

            # Info label (FPS + frame count)
            ilbl = self.cam_info_lbls.get(agv_id)
            if ilbl is not None:
                ilbl.configure(
                    text=f"FPS: {self.cam_fps.get(agv_id, 0):.1f}  |  Frame: {count}")

    # =========================================================================
    # Sensör geçmişi
    # =========================================================================
    def _record_sensor(self, s: AGVState):
        """Her agvUpdate'da çağrılır; bağlı AGV'nin sensör verisini kaydet."""
        if self.sensor_pause_var.get():
            return
        ts = time.time()
        entry = (ts, s.id, list(s.sensors), s.linePos, s.sonarDist, s.navState)
        self._sensor_lines.append(entry)
        if len(self._sensor_lines) > self.MAX_SENSOR_LINES:
            self._sensor_lines = self._sensor_lines[-self.MAX_SENSOR_LINES:]
        # Performans: textbox redraw maliyetli (500 satira kadar yeniden yazma).
        # Sadece Sensors sekmesi gorunurken yenile; diger sekmedeyken sessizce
        # _sensor_lines'a biriksin. Sekme degisiminde _on_tab_changed tek seferlik
        # tam refresh atar (asagida).
        if hasattr(self, "tabs") and self.tabs.get() == "Sensors":
            self._refresh_sensor_log()

    def _on_tab_changed(self):
        """Sekme değişiminde biriken veriyi tek seferlik render et.
        Sensors tab'ına geçildiğinde _record_sensor o sekme aktif olmadığı için
        textbox'ı atlatmıştı; buraya gelince son birikim göster."""
        if not hasattr(self, "tabs"):
            return
        current = self.tabs.get()
        if current == "Sensors" and hasattr(self, "sensor_text"):
            self._refresh_sensor_log()

    def _refresh_sensor_log(self):
        flt = self.sensor_filter_var.get()
        # Sayaç güncelle
        shown = sum(1 for e in self._sensor_lines if flt == "Hepsi" or e[1] == flt)
        self.sensor_count_lbl.configure(text=f"{shown} satır (toplam {len(self._sensor_lines)})")

        self.sensor_text.configure(state="normal")
        self.sensor_text.delete("1.0", "end")
        for ts, agv_id, sensors, line_pos, sonar, nav_state in self._sensor_lines:
            if flt != "Hepsi" and agv_id != flt:
                continue
            ts_str = time.strftime("%H:%M:%S", time.localtime(ts))
            ms = int((ts - int(ts)) * 1000)
            ts_full = f"{ts_str}.{ms:03d}"

            sens_str = " ".join(f"{v:4d}" for v in sensors[:8])
            sonar_str = f"{sonar:5.1f}" if sonar < 999 else "  --"
            line = (f"{ts_full}  {agv_id:<6}  {sens_str}   "
                    f"{line_pos:5d}    {sonar_str}  {nav_state}\n")
            self.sensor_text.insert("end", line)

        if self.sensor_autoscroll_var.get():
            self.sensor_text.see("end")
        self.sensor_text.configure(state="disabled")

    def _clear_sensor_log(self):
        self._sensor_lines.clear()
        self._refresh_sensor_log()

    def _save_sensor_csv(self):
        """Sensör geçmişini .csv dosyasına yaz."""
        from tkinter import filedialog
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All", "*.*")],
            initialfile=f"agv_sensors_{time.strftime('%Y%m%d_%H%M%S')}.csv",
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write("timestamp,agv,s0,s1,s2,s3,s4,s5,s6,s7,linePos,sonar,navState\n")
                for ts, agv_id, sensors, line_pos, sonar, nav_state in self._sensor_lines:
                    sensors_str = ",".join(str(v) for v in sensors[:8])
                    while sensors_str.count(",") < 7:
                        sensors_str += ",0"
                    f.write(f"{ts:.3f},{agv_id},{sensors_str},"
                            f"{line_pos},{sonar:.1f},{nav_state}\n")
            self._set_status(f"Kaydedildi: {os.path.basename(path)}")
        except Exception as e:
            self._set_status(f"Kayıt hatası: {e}", err=True)

    # =========================================================================
    # Log
    # =========================================================================
    def _add_log(self, ev: LogEvent):
        self._log_lines.append(ev)
        if len(self._log_lines) > self.MAX_LOG_LINES:
            self._log_lines = self._log_lines[-self.MAX_LOG_LINES:]
        self._refresh_log()
        self._refresh_dashboard_log()
        # Global mini log kaldırıldı — Dashboard tab'ında dash_log_box var.

    def _refresh_log(self):
        flt = self.log_filter_var.get()
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        for ev in self._log_lines:
            if flt != "Hepsi" and ev.agvId != flt:
                continue
            t = ev.wall_time if ev.wall_time > 0 else time.time()
            ts = time.strftime("%H:%M:%S", time.localtime(t))
            line = f"[{ts}] [{ev.agvId or '-'}] {ev.message}\n"
            self.log_text.insert("end", line)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _clear_log(self):
        self._log_lines.clear()
        self._refresh_log()

    # =========================================================================
    # Tick
    # =========================================================================
    def _tick(self):
        try:
            self._tick_once()
        except Exception as e:
            print("Tick hata:", e)
        self.after(self.UI_TICK_MS, self._tick)

    def _tick_once(self):
        # Kamera
        self._process_camera()

        if self.client is None:
            return

        # WS olaylari
        evs = self.client.pop_events(max_count=128)
        if evs:
            need_list_refresh = False
            need_active_refresh = False

            need_health_refresh = False

            for kind, payload in evs:
                if kind == "connected":
                    self._update_connection_indicator(connected=True)
                    self._set_status("Server'a baglandi")
                elif kind == "disconnected":
                    self._update_connection_indicator(connected=False)
                    self._set_status("Baglanti kesildi")
                elif kind == "error":
                    self._set_status(f"WS: {payload}", err=True)
                elif kind == "agv_list":
                    self.agvs = dict(payload)
                    # Auto-apply: her AGV icin disconnect→connect gecisini kontrol et
                    for s_ in self.agvs.values():
                        self._check_auto_apply(s_)
                        # Cam log polling her bagli AGV icin (ilk list'te de baslat)
                        if s_.connected:
                            self._cam_log_start_for_agv(s_.id)
                    need_list_refresh   = True
                    need_active_refresh = True
                    need_health_refresh = True
                elif kind == "agv_update":
                    s: AGVState = payload
                    if s.id:
                        prev = self.agvs.get(s.id)
                        prev_wp        = prev.currentWaypoint if prev else None
                        prev_connected = prev.connected if prev else None
                        self.agvs[s.id] = s
                        self._check_auto_apply(s)
                        need_list_refresh   = True
                        need_health_refresh = True
                        if self.active_agv == s.id:
                            need_active_refresh = True
                        # Sensör geçmişine ekle (tüm AGV'lerden)
                        self._record_sensor(s)
                        # DEBUG: currentWaypoint degisirse logla (drift / pos sync gorunsun)
                        if prev_wp != s.currentWaypoint:
                            self._add_log(LogEvent(
                                agvId=s.id,
                                message=f"[PLN] agvs[{s.id}].currentWaypoint: "
                                        f"{prev_wp or '-'} → {s.currentWaypoint or '-'} "
                                        f"(navState={s.navState})",
                                time_ms=0, wall_time=time.time(),
                            ))
                        # Connect/disconnect gecisi → planner'a bildir.
                        # Disconnect'te mission + rezervasyonlar korunur,
                        # son pos diger AGV'ler icin engel sayilir.
                        # Reconnect'te dispatch tekrar baslar (drift correction
                        # asagidaki on_agv_position ile pos senkronlar).
                        if prev_connected != s.connected:
                            self.fleet_planner.set_agv_connected(
                                s.id, s.connected, s.currentWaypoint or None,
                            )
                            self._add_log(LogEvent(
                                agvId=s.id,
                                message=f"[PLN] {s.id} "
                                        f"{'BAGLANDI' if s.connected else 'KOPTU'}"
                                        f" (pos={s.currentWaypoint or '-'})",
                                time_ms=0, wall_time=time.time(),
                            ))
                            if s.connected:
                                # Cam log polling baslat (Arm sekmesi gerek yok —
                                # GRIP olaylari her zaman Log paneline gelir)
                                self._cam_log_start_for_agv(s.id)
                                self._planner_tick_and_dispatch()
                            else:
                                self._cam_log_stop_for_agv(s.id)
                        # Planner drift correction: AGV gercek pozisyonu planner'in
                        # bildiginden farkliysa senkronla. (Connected AGV'ler icin.)
                        if s.connected and s.currentWaypoint:
                            self._planner_on_agv_position(s.id, s.currentWaypoint)
                            # last_known_pos da guncellensin (disconnect aninda
                            # son guncel pos kullanilsin diye)
                            self.fleet_planner.last_known_pos[s.id] = s.currentWaypoint
                elif kind == "hop_complete":
                    ev = payload   # HopCompleteEvent
                    # DEBUG TIMING: WS thread'inde ev.wall_time set edildi.
                    # UI tick'i en fazla UI_TICK_MS (100ms) sonra bunu islar.
                    # queue_ms = WS thread'den UI _tick'e kadar gecen sure.
                    if (self.client is not None
                        and getattr(self.client, "debug_timing", False)):
                        queue_ms = (time.time() - ev.wall_time) * 1000 \
                                   if ev.wall_time else 0.0
                        self._add_log(LogEvent(
                            agvId=ev.agvId,
                            message=f"[DBG] hop_complete recv {ev.node} "
                                    f"fw_ms={ev.time_ms} "
                                    f"queue_wait={queue_ms:.1f}ms",
                            time_ms=0, wall_time=time.time(),
                        ))
                    self._planner_on_hop_complete(ev.agvId, ev.node)
                elif kind == "ws_debug":
                    # WS client debug timing event'i — log paneline yansit
                    self._add_log(LogEvent(
                        agvId="*",
                        message=f"[WS] {payload}",
                        time_ms=0, wall_time=time.time(),
                    ))
                elif kind == "log":
                    self._add_log(payload)
                    # Kalibrasyon basarisiz log'unda countdown'i durdur + kirmizi feedback
                    log_ev: LogEvent = payload
                    if (log_ev.agvId
                        and "Kalibrasyon basarisiz" in log_ev.message):
                        # Detayi mesajdan al (": "den sonrasi)
                        detail = log_ev.message.split(":", 1)[1].strip() \
                                 if ":" in log_ev.message else ""
                        self._on_calib_result(log_ev.agvId, success=False,
                                              detail=detail)
                elif kind == "calibration_data":
                    ev: CalibrationDataEvent = payload
                    if ev.agvId:
                        self._last_calib_data[ev.agvId] = ev
                        # Defensive log: event geldiğini log panelinden de gör
                        # (eski server firmware bunu forward etmezse hic gelmez)
                        self._add_log(LogEvent(
                            agvId=ev.agvId,
                            message=f"[UI] calibration_data alindi ({len(ev.sensorMin)} min + {len(ev.sensorMax)} max)",
                            time_ms=0,
                            wall_time=time.time(),
                        ))
                        # Basari: countdown'i durdur + yesil feedback
                        self._on_calib_result(ev.agvId, success=True)
                        if self.active_agv == ev.agvId:
                            self._update_preset_panel()

            if need_list_refresh:
                self._refresh_agv_list()
            if need_active_refresh:
                self._update_active_view()
            if need_health_refresh:
                # Dashboard kartlarını ve top bar aktif AGV göstergesini yenile
                self._refresh_dashboard_cards()
                self._update_active_agv_indicator()
                # Arm tab aktif AGV değişince kendi içeriğini yeniler (rebuild gerek yok)

    # =========================================================================
    # Durum cubugu
    # =========================================================================
    def _set_status(self, text: str, err: bool = False):
        self.status_bar.configure(
            text=f"{time.strftime('%H:%M:%S')} - {text}",
            text_color=COL_TXT_ERR if err else "white",
        )

    def _set_status_color(self, text: str, color: str):
        """Aciksiz renk verilebilen status — sonuc gosterimi (yesil/kirmizi)."""
        self.status_bar.configure(
            text=f"{time.strftime('%H:%M:%S')} - {text}",
            text_color=color,
        )

    # =========================================================================
    # Kalibrasyon geri sayim — PC-side timer, firmware'e ekstra WS trafigi yok
    # =========================================================================
    def _start_calib_countdown(self, agv: str):
        """Yeni kalibrasyon basladi: status bar'da 10 → 0 geri sayim."""
        # Eski timer varsa iptal
        self._cancel_calib_countdown()
        self._calib_countdown_agv       = agv
        self._calib_countdown_remaining = self.CALIBRATE_DURATION_SEC
        self._tick_calib_countdown()

    def _tick_calib_countdown(self):
        agv = self._calib_countdown_agv
        if agv is None:
            return
        n = self._calib_countdown_remaining
        if n <= 0:
            # Firmware'den sonuc gelmedi → timeout uyarisi (15 sn kadar bekliyor olabilir)
            self._set_status_color(
                f"{agv}: ⏱ Kalibrasyon süresi doldu, sonuç bekleniyor...",
                COL_WARN,
            )
            return
        self._set_status_color(
            f"{agv}: 🟡 Kalibrasyon yapılıyor... ({n} sn kaldı) "
            f"— aracı çizgi/zemin üstünde gezdir",
            COL_WARN,
        )
        self._calib_countdown_remaining = n - 1
        self._calib_countdown_after_id = self.after(1000, self._tick_calib_countdown)

    def _cancel_calib_countdown(self):
        if self._calib_countdown_after_id is not None:
            try:
                self.after_cancel(self._calib_countdown_after_id)
            except Exception:
                pass
        self._calib_countdown_after_id  = None
        self._calib_countdown_agv       = None
        self._calib_countdown_remaining = 0

    def _on_calib_result(self, agv: str, success: bool, detail: str = ""):
        """Sonuç geldi — geri sayım timer'ini durdur + renkli status göster."""
        self._cancel_calib_countdown()
        self._last_calibrate_sent.pop(agv, None)
        if success:
            self._set_status_color(
                f"{agv}: ✓ Kalibrasyon başarılı — preset olarak kaydedebilirsin",
                COL_SUCCESS,
            )
        else:
            msg = f"{agv}: ✗ Kalibrasyon başarısız"
            if detail:
                msg += f" — {detail}"
            self._set_status_color(msg, COL_DANGER)


# =============================================================================
# Entry
# =============================================================================

if __name__ == "__main__":
    AGVControlApp().mainloop()

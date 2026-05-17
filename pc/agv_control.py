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
import numpy as np

from agv_ws_client    import AGVClient, AGVState, LogEvent
from waypoint_canvas  import WaypointCanvas
from stream_reader    import StreamReader
from detector         import (
    Presets, detect_red, detect_blue, draw_detections,
    load_presets,
)


SERVER_URL_DEFAULT = "ws://192.168.4.1/ws"
PRESET_PATH        = os.path.join(os.path.dirname(__file__), "hsv_presets.json")
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
    UI_TICK_MS = 100

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
        self.client: Optional[AGVClient] = None
        self.agvs:   Dict[str, AGVState] = {}
        self.active_agv: Optional[str]   = None
        self.stream_reader: Optional[StreamReader] = None
        self.presets: Presets = load_presets(PRESET_PATH)
        self.show_red_det:   bool = True
        self.show_blue_det:  bool = True
        self.show_mask:      bool = False
        self.last_stream_count: int   = 0
        self.last_stream_time:  float = time.time()
        self.stream_fps:        float = 0.0
        self._cam_video_img_ref       = None   # son CTkImage'i GC'den koru

        # Otomatik takip (vision-servoing base servo)
        self.tracking_active:       bool  = False
        self.tracking_color:        str   = "red"     # "red" | "blue"
        self.tracking_base_angle:   int   = 93        # son gonderilen base acisi (HOME=93)
        self.tracking_last_send_ts: float = 0.0

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

        # ESP32-CAM log polling (kamera baglandiginda baslar)
        import threading as _t
        self._cam_log_stop_evt   = _t.Event()
        self._cam_log_thread     = None     # type: Optional[_t.Thread]
        self._cam_log_last_seq:   int = 0
        # /poll ile gelen son arm state (drag etmeyen UI guncellenmeleri icin)
        self._cam_arm_state: Dict = {}

        # UI'da maksimum log satiri
        self.MAX_LOG_LINES = 500

        # Sensor gecmisi (her agvUpdate'da bir satir)
        self.MAX_SENSOR_LINES = 500
        self._sensor_lines: List[tuple] = []   # (timestamp, agv_id, sensors[8], linePos, sonarDist, navState)

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self.UI_TICK_MS, self._tick)

    # =========================================================================
    # Layout
    # =========================================================================
    def _build_ui(self):
        # ----- Ust serit -----
        topbar = ctk.CTkFrame(self, height=56)
        topbar.pack(fill="x", padx=8, pady=(8, 4))

        ctk.CTkLabel(topbar, text="WS:").pack(side="left", padx=(8, 4))
        self.url_var = ctk.StringVar(value=SERVER_URL_DEFAULT)
        ctk.CTkEntry(topbar, textvariable=self.url_var, width=300).pack(side="left", padx=4)

        self.connect_btn = ctk.CTkButton(topbar, text="Baglan", width=80,
                                         command=self._connect)
        self.disconnect_btn = ctk.CTkButton(topbar, text="Kes", width=60,
                                            command=self._disconnect, state="disabled")
        self.connect_btn.pack(side="left", padx=4)
        self.disconnect_btn.pack(side="left", padx=4)

        self.conn_lbl = ctk.CTkLabel(topbar, text="● Bagli degil", text_color=COL_WARN)
        self.conn_lbl.pack(side="left", padx=12)

        self.agv_count_lbl = ctk.CTkLabel(topbar, text="AGV: 0 aktif")
        self.agv_count_lbl.pack(side="right", padx=12)

        # Acil Dur - en sagda, kirmizi, surekli erisilebilir (Esc kisayolu da var)
        self.estop_btn = ctk.CTkButton(
            topbar, text="🛑 ACİL DUR  [Esc]", width=180, height=40,
            fg_color=COL_DANGER, hover_color=COL_DANGER_HOVER,
            font=self.font_h1,
            command=self._emergency_stop,
        )
        self.estop_btn.pack(side="right", padx=8, pady=4)
        # Global kisayol: Esc her yerden tetikler (slider focus'unda bile)
        self.bind_all("<Escape>", lambda e: self._emergency_stop())

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

        # Sag: sekmeler
        self.tabs = ctk.CTkTabview(body)
        self.tabs.pack(side="right", fill="both", expand=True)

        self.tab_dashboard = self.tabs.add("Dashboard")
        self.tab_nav       = self.tabs.add("Navigasyon")
        self.tab_cam       = self.tabs.add("Kamera")
        self.tab_arm       = self.tabs.add("Robot Kol")
        self.tab_sensor    = self.tabs.add("Sensör")
        self.tab_log       = self.tabs.add("Log")

        self._build_tab_nav()        # Once Navigasyon - map_widget olusturulur
                                      # ve PID/Hiz kontrolleri buraya sikistirildi
        self._build_tab_cam()
        self._build_tab_arm()
        self._build_tab_sensor()
        self._build_tab_log()
        self._build_tab_dashboard()  # Sona - diger sekmelerin metodlarini referansliyor

        # Dashboard ilk gosterilen sekme olsun
        self.tabs.set("Dashboard")

        # Alt durum cubugu
        self.status_bar = ctk.CTkLabel(self, text="Hazir.", anchor="w",
                                       fg_color=COL_BG_STATUSBAR, height=22)
        self.status_bar.pack(fill="x", side="bottom")

    # ---------- Navigasyon sekmesi ----------
    def _build_tab_nav(self):
        nav = self.tab_nav

        nav.grid_columnconfigure(0, weight=1)
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
        ctk.CTkButton(btn_frame, text="⚙ KALIBRE", height=36,
                      fg_color=COL_WARN, hover_color=COL_WARN_HOVER,
                      command=lambda: self._cmd_command("calibrate")).pack(fill="x", pady=3)

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

    # ---------- Kamera sekmesi ----------
    def _build_tab_cam(self):
        cam = self.tab_cam
        cam.grid_columnconfigure(0, weight=1)
        cam.grid_columnconfigure(1, weight=0)
        cam.grid_rowconfigure(0, weight=1)

        # Sol: video
        left = ctk.CTkFrame(cam)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=4)

        self.cam_video_lbl = ctk.CTkLabel(left, text="Akis kapali", width=640, height=480,
                                           fg_color=COL_BG_DEEP)
        self.cam_video_lbl.pack(padx=8, pady=8, expand=True)

        self.cam_info_lbl = ctk.CTkLabel(left, text="-", anchor="w")
        self.cam_info_lbl.pack(fill="x", padx=8, pady=(0, 8))

        # Sag: ayarlar
        right = ctk.CTkFrame(cam, width=300)
        right.grid(row=0, column=1, sticky="ns", pady=4)
        right.grid_propagate(False)

        ctk.CTkLabel(right, text="KAMERA YAYINI",
                     font=self.font_h2).pack(pady=(8, 4))

        ctk.CTkLabel(right, text="Stream URL:").pack(anchor="w", padx=12)
        self.cam_url_var = ctk.StringVar(value="http://192.168.4.50:81/stream")
        ctk.CTkEntry(right, textvariable=self.cam_url_var, width=260).pack(padx=12, pady=4)

        btn_row = ctk.CTkFrame(right, fg_color="transparent")
        btn_row.pack(fill="x", padx=12, pady=4)
        self.cam_connect_btn    = ctk.CTkButton(btn_row, text="Yayini Ac", width=110,
                                                command=self._cam_connect)
        self.cam_disconnect_btn = ctk.CTkButton(btn_row, text="Kapat", width=110,
                                                command=self._cam_disconnect, state="disabled")
        self.cam_connect_btn.pack(side="left", padx=2)
        self.cam_disconnect_btn.pack(side="left", padx=2)

        ctk.CTkLabel(right, text="TESPIT OVERLAY",
                     font=self.font_h2).pack(pady=(16, 4))
        self.cam_red_var  = ctk.BooleanVar(value=True)
        self.cam_blue_var = ctk.BooleanVar(value=True)
        self.cam_mask_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(right, text="Kirmizi tespit",  variable=self.cam_red_var,
                        command=self._update_cam_toggles).pack(anchor="w", padx=12, pady=2)
        ctk.CTkCheckBox(right, text="Mavi tespit",     variable=self.cam_blue_var,
                        command=self._update_cam_toggles).pack(anchor="w", padx=12, pady=2)
        ctk.CTkCheckBox(right, text="Maske gosterimi", variable=self.cam_mask_var,
                        command=self._update_cam_toggles).pack(anchor="w", padx=12, pady=2)

        ctk.CTkLabel(right,
                     text=("HSV preset'leri vision_app.py\n"
                           "uzerinden ayarlanip JSON'a\n"
                           "kaydedilir; bu sayfa onlari\n"
                           "okur."),
                     justify="left", font=self.font_small,
                     text_color=COL_MUTED, wraplength=270).pack(padx=12, pady=8)
        ctk.CTkButton(right, text="Preset'leri Yeniden Yukle",
                      command=self._reload_presets).pack(padx=12, pady=4)

        # --- Otomatik takip ---
        ctk.CTkLabel(right, text="OTOMATIK TAKIP",
                     font=self.font_h2).pack(pady=(16, 4))

        self.track_color_var = ctk.StringVar(value="red")
        color_frame = ctk.CTkFrame(right, fg_color="transparent")
        color_frame.pack(fill="x", padx=12)
        ctk.CTkRadioButton(color_frame, text="Kirmizi", variable=self.track_color_var,
                           value="red",  command=self._on_track_color_change).pack(side="left", padx=4)
        ctk.CTkRadioButton(color_frame, text="Mavi",    variable=self.track_color_var,
                           value="blue", command=self._on_track_color_change).pack(side="left", padx=4)

        self.track_btn = ctk.CTkButton(right, text="Takibi Baslat",
                                       fg_color=COL_SUCCESS, hover_color=COL_SUCCESS_HOVER,
                                       command=self._toggle_tracking)
        self.track_btn.pack(fill="x", padx=12, pady=6)

        self.track_status_lbl = ctk.CTkLabel(right, text="Pasif",
                                             font=self.font_small,
                                             text_color=COL_MUTED, justify="left")
        self.track_status_lbl.pack(anchor="w", padx=12, pady=(0, 8))

        # --- Kapma kalibrasyonu ---
        ctk.CTkLabel(right, text="KAPMA KALIBRASYONU",
                     font=self.font_h2).pack(pady=(16, 4))

        # Shoulder/elbow tarama pozisyonu (Robot Kol sekmesinde manuel ayarla,
        # sonra burda kaydet — tarama baslarken bu pozisyon servoya gonderilir)
        pose = self.pickup_cfg.get("scan_pose", {})
        self.scan_pose_lbl = ctk.CTkLabel(
            right,
            text=f"Tarama poz: sh={pose.get('shoulder','?')}  el={pose.get('elbow','?')}",
            font=self.font_small, text_color=COL_MUTED)
        self.scan_pose_lbl.pack(anchor="w", padx=12)
        ctk.CTkButton(right, text="Mevcut sh/el'yi Tarama Pozu Kaydet",
                      command=self._pickup_save_scan_pose,
                      font=self.font_body).pack(fill="x", padx=12, pady=(2, 6))

        # Tarama tipi butonlari — kayitli aralik varsa onu kullanir, yoksa default
        ctk.CTkButton(right, text="◀ Full Sol Tara",
                      command=lambda: self._pickup_scan_start("left"),
                      font=self.font_body).pack(fill="x", padx=12, pady=2)
        ctk.CTkButton(right, text="Full Sag Tara ▶",
                      command=lambda: self._pickup_scan_start("right"),
                      font=self.font_body).pack(fill="x", padx=12, pady=2)
        ctk.CTkButton(right, text="◆ On Tara (loop)",
                      command=lambda: self._pickup_scan_start("front"),
                      font=self.font_body).pack(fill="x", padx=12, pady=2)
        ctk.CTkButton(right, text="Genel Tara (loop)",
                      command=lambda: self._pickup_scan_start("general"),
                      font=self.font_body).pack(fill="x", padx=12, pady=2)

        # Tarama anindaki iki nokta kaydi: start / end
        save_row = ctk.CTkFrame(right, fg_color="transparent")
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

        self.scan_stop_btn = ctk.CTkButton(right, text="⏹ Taramayi Bitir",
                                           fg_color=COL_DANGER, hover_color=COL_DANGER_HOVER,
                                           command=self._pickup_scan_stop)
        self.scan_stop_btn.pack(fill="x", padx=12, pady=(2, 4))

        self.scan_status_lbl = ctk.CTkLabel(right, text="Pasif",
                                            font=self.font_small,
                                            text_color=COL_MUTED, justify="left")
        self.scan_status_lbl.pack(anchor="w", padx=12, pady=(2, 4))

        # Mevcut kayitli araliklar (4 satir)
        self.scan_limits_lbl = ctk.CTkLabel(
            right, text=self._pickup_limits_text(),
            font=self.font_small, text_color=COL_TXT_ACCENT, justify="left")
        self.scan_limits_lbl.pack(anchor="w", padx=12, pady=(0, 4))

        ctk.CTkButton(right, text="💾 JSON'a Kaydet",
                      command=self._pickup_save_config,
                      font=self.font_body).pack(fill="x", padx=12, pady=(4, 8))

    # ---------- PID sekmesi ----------
    # ---------- Robot Kol sekmesi ----------
    def _build_tab_arm(self):
        arm = self.tab_arm

        ctk.CTkLabel(arm, text="ROBOT KOL KONTROL",
                     font=self.font_h1).pack(pady=(10, 4))

        # URL satiri
        url_frame = ctk.CTkFrame(arm, fg_color="transparent")
        url_frame.pack(fill="x", padx=12, pady=4)
        ctk.CTkLabel(url_frame, text="ESP32-CAM:").pack(side="left", padx=(8, 4))
        self.arm_url_var = ctk.StringVar(value="http://192.168.4.50")
        ctk.CTkEntry(url_frame, textvariable=self.arm_url_var, width=200).pack(side="left", padx=4)
        ctk.CTkButton(url_frame, text="Durum Oku", width=110,
                      command=self._arm_fetch_state).pack(side="left", padx=4)

        # Servo slider'lar - her servo'nun fiziksel araligi farkli (mekanik kalibrasyon)
        servos_frame = ctk.CTkFrame(arm)
        servos_frame.pack(fill="x", padx=20, pady=10)
        ctk.CTkLabel(servos_frame, text="SERVOLAR (mekanik kalibrasyonlu aralik)",
                     font=self.font_h2).pack(pady=(8, 4))

        self.arm_servo_vars: List[ctk.IntVar] = []
        self.arm_servo_lbls: List[ctk.CTkLabel] = []
        # (isim, alt_yazi, MIN, MAX, HOME) - HOME firmware ile ayni olmali
        SERVO_DEFS = [
            ("Base",     "GPIO 14 - MG996R",    0, 180,  93),
            ("Shoulder", "GPIO 13 - DS3218MG",  0, 120, 120),
            ("Elbow",    "GPIO 15 - MG996R",    4, 153,  15),
            ("Gripper",  "GPIO 12 - MG90S",     0, 160, 110),
        ]
        for idx, (name, sub, vmin, vmax, vhome) in enumerate(SERVO_DEFS):
            row = ctk.CTkFrame(servos_frame, fg_color="transparent")
            row.pack(fill="x", padx=12, pady=3)

            name_block = ctk.CTkFrame(row, fg_color="transparent", width=130)
            name_block.pack(side="left")
            ctk.CTkLabel(name_block, text=name, anchor="w",
                         font=self.font_h2).pack(anchor="w")
            ctk.CTkLabel(name_block, text=f"{sub}  [{vmin}-{vmax}°]", anchor="w",
                         font=self.font_small, text_color=COL_SUBTLE).pack(anchor="w")

            var = ctk.IntVar(value=vhome)
            self.arm_servo_vars.append(var)
            slider = ctk.CTkSlider(
                row, from_=vmin, to=vmax, number_of_steps=vmax - vmin, width=350,
                variable=var,
                command=lambda v, i=idx: self._arm_slider_change(i, int(v)),
            )
            slider.pack(side="left", padx=4)
            slider.bind("<ButtonRelease-1>",
                        lambda e, i=idx: self._arm_slider_release(i))

            angle_lbl = ctk.CTkLabel(row, text=f"{vhome}°", width=50,
                                      font=self.font_mono)
            angle_lbl.pack(side="left")
            self.arm_servo_lbls.append(angle_lbl)

        # Hazir pozisyon butonlari - mekanik kalibrasyona uygun
        # Home = en dik (yukari bakar). Diger preset'ler tahmini, kullanici slider ile fine-tune eder.
        preset_frame = ctk.CTkFrame(arm)
        preset_frame.pack(fill="x", padx=20, pady=8)
        ctk.CTkLabel(preset_frame, text="HAZIR POZISYONLAR",
                     font=self.font_h2).pack(pady=(8, 4))

        btns = ctk.CTkFrame(preset_frame, fg_color="transparent")
        btns.pack(pady=6)
        # HOME ozel: /arm/home endpoint cagirir - once shoulder, sonra digerleri
        ctk.CTkButton(btns, text="🏠 Home (Sequential)", width=160,
                      fg_color=COL_SUCCESS, hover_color=COL_SUCCESS_HOVER,
                      command=self._arm_home).grid(row=0, column=0, padx=4, pady=2)
        ctk.CTkButton(btns, text="Aşağı Uzan", width=160,
                      command=lambda: self._arm_apply_preset([93, 20, 30, 110])).grid(row=0, column=1, padx=4, pady=2)
        ctk.CTkButton(btns, text="Tutmaya Hazır", width=160,
                      command=lambda: self._arm_apply_preset([93, 50, 80, 110])).grid(row=1, column=0, padx=4, pady=2)
        ctk.CTkButton(btns, text="Yatay İleri", width=160,
                      command=lambda: self._arm_apply_preset([93, 0, 4, 110])).grid(row=1, column=1, padx=4, pady=2)

        # Hiz + LED parlaklik
        extra_frame = ctk.CTkFrame(arm)
        extra_frame.pack(fill="x", padx=20, pady=8)
        ctk.CTkLabel(extra_frame, text="HIZ & AYDINLATMA",
                     font=self.font_h2).pack(pady=(8, 4))

        # Servo hizi (ramping ms — kucuk daha hizli)
        speed_row = ctk.CTkFrame(extra_frame, fg_color="transparent")
        speed_row.pack(fill="x", padx=12, pady=4)
        ctk.CTkLabel(speed_row, text="Servo Hizi", width=100,
                     anchor="w").pack(side="left")
        self.arm_speed_var = ctk.IntVar(value=40)
        self.arm_speed_slider = ctk.CTkSlider(
            speed_row, from_=10, to=200, number_of_steps=190,
            variable=self.arm_speed_var,
            command=lambda v: self._arm_speed_label_update(int(v)),
        )
        self.arm_speed_slider.pack(side="left", fill="x", expand=True, padx=4)
        self.arm_speed_slider.bind("<ButtonRelease-1>",
                                   lambda e: self._arm_send_speed())
        self.arm_speed_lbl = ctk.CTkLabel(
            speed_row, text="40 ms / 25°/sn",
            width=120, font=self.font_mono_sm)
        self.arm_speed_lbl.pack(side="left")

        # Flash LED parlaklik (0..255)
        led_row = ctk.CTkFrame(extra_frame, fg_color="transparent")
        led_row.pack(fill="x", padx=12, pady=4)
        ctk.CTkLabel(led_row, text="LED Parlaklik", width=100,
                     anchor="w").pack(side="left")
        self.arm_led_var = ctk.IntVar(value=5)
        self.arm_led_slider = ctk.CTkSlider(
            led_row, from_=0, to=255, number_of_steps=255,
            variable=self.arm_led_var,
            command=lambda v: self._arm_led_label_update(int(v)),
        )
        self.arm_led_slider.pack(side="left", fill="x", expand=True, padx=4)
        self.arm_led_slider.bind("<ButtonRelease-1>",
                                 lambda e: self._arm_send_led())
        self.arm_led_lbl = ctk.CTkLabel(
            led_row, text="5/255 (2%)",
            width=120, font=self.font_mono_sm)
        self.arm_led_lbl.pack(side="left")

        # Mıknatıs (henüz devre dışı)
        magnet_frame = ctk.CTkFrame(arm)
        magnet_frame.pack(fill="x", padx=20, pady=8)
        ctk.CTkLabel(magnet_frame, text="ELEKTROMIKNATIS",
                     font=self.font_h2).pack(pady=(8, 4))

        self.arm_magnet_var = ctk.BooleanVar(value=False)
        magnet_row = ctk.CTkFrame(magnet_frame, fg_color="transparent")
        magnet_row.pack(pady=6)
        self.arm_magnet_btn = ctk.CTkSwitch(
            magnet_row, text="Mıknatıs", variable=self.arm_magnet_var,
            command=self._arm_magnet_toggle,
        )
        self.arm_magnet_btn.pack(side="left", padx=8)
        ctk.CTkLabel(magnet_row, text="GPIO 2 → MOSFET gate (220Ω+10kΩ pull-down)",
                     text_color=COL_SUBTLE,
                     font=self.font_small).pack(side="left", padx=8)

        # Bilgi
        ctk.CTkLabel(arm, text=(
            "Slider'i hareket ettir → komut ESP32-CAM'a HTTP ile gönderilir.\n"
            "Servo güvenli aralık 0-180°. Mekanik sınır servo özeline göre değişir."),
                     justify="left", font=self.font_small,
                     text_color=COL_MUTED).pack(pady=8)

    # =========================================================================
    # Robot Kol - HTTP komut gönder
    # =========================================================================
    def _arm_base_url(self) -> Optional[str]:
        url = self.arm_url_var.get().strip().rstrip("/")
        if not url:
            self._set_status("Kol URL'si bos", err=True)
            return None
        return url

    def _arm_http_get(self, path: str):
        """HTTP GET'i ayri thread'de gonder — UI bloklanmasin."""
        import threading
        import urllib.request

        base = self._arm_base_url()
        if base is None:
            return
        url = f"{base}{path}"

        def worker():
            try:
                with urllib.request.urlopen(url, timeout=2.0) as r:
                    _ = r.read()
            except Exception as e:
                # UI thread'e güvenli aktar (after kullan)
                self.after(0, lambda: self._set_status(f"Kol HTTP hata: {e}", err=True))

        threading.Thread(target=worker, daemon=True).start()

    # Slider throttle: drag sirasinda en fazla 1 istek/100ms; gec gelen degerler
    # pending olarak tutulur ve gec timer'la flush edilir. ButtonRelease pending
    # olani hemen gonderir -> son deger asla kaybolmaz.
    _ARM_THROTTLE_S = 0.10

    def _arm_slider_change(self, servo_id: int, angle: int):
        self.arm_servo_lbls[servo_id].configure(text=f"{angle}°")

        if not hasattr(self, "_arm_pending"):
            self._arm_pending:   Dict[int, int]   = {}
            self._arm_last_send: Dict[int, float] = {}
            self._arm_scheduled: set              = set()

        self._arm_pending[servo_id] = angle
        now  = time.time()
        last = self._arm_last_send.get(servo_id, 0.0)

        if now - last >= self._ARM_THROTTLE_S:
            self._arm_flush_servo(servo_id)
        elif servo_id not in self._arm_scheduled:
            self._arm_scheduled.add(servo_id)
            delay_ms = max(1, int((self._ARM_THROTTLE_S - (now - last)) * 1000))
            self.after(delay_ms, lambda sid=servo_id: self._arm_flush_servo(sid))

    def _arm_flush_servo(self, servo_id: int):
        if hasattr(self, "_arm_scheduled"):
            self._arm_scheduled.discard(servo_id)
        if not hasattr(self, "_arm_pending") or servo_id not in self._arm_pending:
            return
        angle = self._arm_pending.pop(servo_id)
        self._arm_last_send[servo_id] = time.time()
        self._arm_http_get(f"/servo?id={servo_id}&a={angle}")

    def _arm_slider_release(self, servo_id: int):
        # Throttle yuzunden son hareketi atlam olabilir -> garantili flush
        if hasattr(self, "_arm_pending") and servo_id in self._arm_pending:
            self._arm_flush_servo(servo_id)

    def _arm_apply_preset(self, angles: List[int]):
        for i, a in enumerate(angles[:4]):
            self.arm_servo_vars[i].set(a)
            self.arm_servo_lbls[i].configure(text=f"{a}°")
            self._arm_http_get(f"/servo?id={i}&a={a}")
        self._set_status(f"Kol preset: {angles}")

    def _arm_home(self):
        """Sequential HOME: firmware once shoulder'i ayarlar, sonra digerleri.
        UI sliderlari home degerlerine set edilir (gostermelik - asil hareketi
        firmware state machine yonetir, /poll ile geri okunabilir)."""
        HOME = [93, 120, 15, 110]
        for i, a in enumerate(HOME):
            self.arm_servo_vars[i].set(a)
            self.arm_servo_lbls[i].configure(text=f"{a}°")
        self._arm_http_get("/arm/home")
        self._set_status("Kol HOME: once shoulder, sonra digerleri")

    def _arm_magnet_toggle(self):
        state = 1 if self.arm_magnet_var.get() else 0
        self._arm_http_get(f"/magnet?s={state}")
        self._set_status(f"Mıknatıs: {'AÇIK' if state else 'KAPALI'}")

    # ---- Servo hizi + LED parlaklik ----
    def _arm_speed_label_update(self, ms: int):
        dps = (1000 // ms) if ms > 0 else 0
        self.arm_speed_lbl.configure(text=f"{ms} ms / {dps}°/sn")

    def _arm_send_speed(self):
        ms = self.arm_speed_var.get()
        self._arm_http_get(f"/servo/speed?ms={ms}")
        self._set_status(f"Servo hizi: {ms} ms/adim")

    def _arm_led_label_update(self, b: int):
        pct = round(b * 100 / 255)
        self.arm_led_lbl.configure(text=f"{b}/255 ({pct}%)")

    def _arm_send_led(self):
        b = self.arm_led_var.get()
        self._arm_http_get(f"/led?b={b}")
        self._set_status(f"LED parlaklik: {b}/255")

    def _arm_fetch_state(self):
        """ESP32-CAM'dan mevcut kol durumunu çek, slider'lari guncelle."""
        import threading
        import urllib.request
        import json as _json

        base = self._arm_base_url()
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
                    for i, a in enumerate(servos[:4]):
                        self.arm_servo_vars[i].set(int(a))
                        self.arm_servo_lbls[i].configure(text=f"{int(a)}°")
                    self.arm_magnet_var.set(magnet)
                    self.arm_speed_var.set(speed_ms)
                    self._arm_speed_label_update(speed_ms)
                    self.arm_led_var.set(led_b)
                    self._arm_led_label_update(led_b)
                    self._set_status(
                        f"Kol durumu okundu: servos={servos} mag={magnet} "
                        f"speed={speed_ms}ms led={led_b}")
                self.after(0, apply)
            except Exception as e:
                self.after(0, lambda: self._set_status(f"Kol durum hata: {e}", err=True))

        threading.Thread(target=worker, daemon=True).start()

    # ---------- Sensör sekmesi ----------
    def _build_tab_sensor(self):
        sen = self.tab_sensor

        # Üst bar
        bar = ctk.CTkFrame(sen, fg_color="transparent")
        bar.pack(fill="x", padx=8, pady=8)

        ctk.CTkLabel(bar, text="Filtre:").pack(side="left", padx=(8, 4))
        self.sensor_filter_var = ctk.StringVar(value="Hepsi")
        self.sensor_filter = ctk.CTkOptionMenu(
            bar, values=["Hepsi"], variable=self.sensor_filter_var,
            width=120, command=lambda _: self._refresh_sensor_log())
        self.sensor_filter.pack(side="left", padx=4)

        self.sensor_pause_var = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(bar, text="Duraklat", variable=self.sensor_pause_var
                        ).pack(side="left", padx=12)

        self.sensor_autoscroll_var = ctk.BooleanVar(value=True)
        ctk.CTkCheckBox(bar, text="Oto-kaydır", variable=self.sensor_autoscroll_var
                        ).pack(side="left", padx=4)

        ctk.CTkButton(bar, text="Kaydet (.csv)", width=110,
                      command=self._save_sensor_csv).pack(side="right", padx=4)
        ctk.CTkButton(bar, text="Temizle", width=80,
                      command=self._clear_sensor_log).pack(side="right", padx=4)

        # Sayaç
        self.sensor_count_lbl = ctk.CTkLabel(sen, text="0 satır",
                                              text_color=COL_SUBTLE,
                                              font=self.font_small)
        self.sensor_count_lbl.pack(anchor="w", padx=12)

        # Üst başlık satırı (sabit)
        header = ctk.CTkLabel(
            sen,
            text=("Zaman      AGV     "
                  "S0   S1   S2   S3   S4   S5   S6   S7   "
                  " linePos  sonar  durum"),
            font=self.font_mono_sm_b,
            anchor="w", text_color=COL_TXT_ACCENT,
        )
        header.pack(fill="x", padx=8, pady=(0, 0))

        # Kaydırılabilir text alanı
        self.sensor_text = ctk.CTkTextbox(
            sen, font=self.font_mono_sm)
        self.sensor_text.pack(fill="both", expand=True, padx=8, pady=(2, 8))
        self.sensor_text.configure(state="disabled")

    # ---------- Log sekmesi ----------
    def _build_tab_log(self):
        log = self.tab_log

        bar = ctk.CTkFrame(log, fg_color="transparent")
        bar.pack(fill="x", padx=8, pady=8)

        ctk.CTkLabel(bar, text="Filtre:").pack(side="left", padx=(8, 4))
        self.log_filter_var = ctk.StringVar(value="Hepsi")
        self.log_filter = ctk.CTkOptionMenu(bar, values=["Hepsi"], variable=self.log_filter_var,
                                             width=120, command=lambda _: self._refresh_log())
        self.log_filter.pack(side="left", padx=4)

        ctk.CTkButton(bar, text="Temizle", width=80, command=self._clear_log).pack(side="right", padx=4)

        self.log_text = ctk.CTkTextbox(log, font=self.font_mono)
        self.log_text.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.log_text.configure(state="disabled")

        self._log_lines: List[LogEvent] = []

    # ---------- Dashboard sekmesi ----------
    # Hizli ozet: aktif AGV durumu + mini harita + hizli komut + son log feed.
    # Detay islemler kendi sekmesinde (Navigasyon/Kamera/PID/Robot Kol) yapilir.
    def _build_tab_dashboard(self):
        dash = self.tab_dashboard

        # ---- Ust satir: 2 yan yana panel ----
        top = ctk.CTkFrame(dash, fg_color="transparent")
        top.pack(fill="x", padx=8, pady=8)

        # Sol: aktif AGV ozet kart
        info_card = ctk.CTkFrame(top)
        info_card.pack(side="left", fill="both", expand=True, padx=(0, 4))
        ctk.CTkLabel(info_card, text="AKTİF AGV",
                     font=self.font_h1).pack(pady=(8, 4))

        self.dash_agv_id   = ctk.CTkLabel(info_card, text="ID:    -",
                                          font=self.font_mono_lg)
        self.dash_state    = ctk.CTkLabel(info_card, text="Durum: -",
                                          font=self.font_mono_lg)
        self.dash_wp       = ctk.CTkLabel(info_card, text="Konum: -",
                                          font=self.font_mono_lg)
        self.dash_target   = ctk.CTkLabel(info_card, text="Hedef: -",
                                          font=self.font_mono_lg)
        self.dash_heading  = ctk.CTkLabel(info_card, text="Yön:   -",
                                          font=self.font_mono_lg)
        self.dash_sonar    = ctk.CTkLabel(info_card, text="Sonar: -",
                                          font=self.font_mono_lg)
        self.dash_line     = ctk.CTkLabel(info_card, text="Çizgi: -",
                                          font=self.font_mono_lg)
        for w in (self.dash_agv_id, self.dash_state, self.dash_wp,
                  self.dash_target, self.dash_heading, self.dash_sonar, self.dash_line):
            w.pack(anchor="w", padx=14, pady=2)

        # Sag: mini harita
        map_card = ctk.CTkFrame(top)
        map_card.pack(side="left", fill="both", expand=True, padx=(4, 0))
        ctk.CTkLabel(map_card, text="HARİTA",
                     font=self.font_h1).pack(pady=(8, 4))
        self.dash_map = WaypointCanvas(
            map_card,
            on_target_click=lambda wp: self.tgt_var.set(wp),  # tikla -> hedef set
            width=260, height=240,
        )
        self.dash_map.pack(padx=8, pady=(0, 8))

        # ---- Orta: hizli komutlar ----
        cmd_card = ctk.CTkFrame(dash)
        cmd_card.pack(fill="x", padx=8, pady=4)
        ctk.CTkLabel(cmd_card, text="HIZLI KOMUTLAR",
                     font=self.font_h1).pack(pady=(8, 4))
        btn_row = ctk.CTkFrame(cmd_card, fg_color="transparent")
        btn_row.pack(pady=(0, 10))

        ctk.CTkButton(btn_row, text="▶ Başlat", width=140, height=44,
                      fg_color=COL_SUCCESS, hover_color=COL_SUCCESS_HOVER,
                      font=self.font_h1,
                      command=lambda: self._cmd_command("start")).pack(side="left", padx=4)
        ctk.CTkButton(btn_row, text="■ Dur", width=140, height=44,
                      fg_color=COL_WARN, hover_color=COL_WARN_HOVER,
                      font=self.font_h1,
                      command=lambda: self._cmd_command("stop")).pack(side="left", padx=4)
        ctk.CTkButton(btn_row, text="⚙ Kalibre Et", width=140, height=44,
                      font=self.font_h1,
                      command=lambda: self._cmd_command("calibrate")).pack(side="left", padx=4)
        ctk.CTkButton(btn_row, text="🏠 Kol Home", width=140, height=44,
                      font=self.font_h1,
                      command=self._arm_home).pack(side="left", padx=4)

        # ---- Alt: rolling log feed (son 8 satir) ----
        feed = ctk.CTkFrame(dash)
        feed.pack(fill="both", expand=True, padx=8, pady=(4, 8))
        ctk.CTkLabel(feed, text="SON OLAYLAR",
                     font=self.font_h1).pack(pady=(8, 2))
        self.dash_log_box = ctk.CTkTextbox(
            feed, font=self.font_mono_sm, height=160,
        )
        self.dash_log_box.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.dash_log_box.configure(state="disabled")

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
        self.client.start()
        self.connect_btn.configure(state="disabled")
        self.disconnect_btn.configure(state="normal")

    def _disconnect(self):
        if self.client is None:
            return
        self.client.stop()
        self.client = None
        self.connect_btn.configure(state="normal")
        self.disconnect_btn.configure(state="disabled")
        self.conn_lbl.configure(text="● Bagli degil", text_color=COL_WARN)
        self.agvs.clear()
        self._refresh_agv_list()

    # -------------------------------------------------------------------------
    # ACIL DUR - tum AGV'leri durdur + tum CAM kollarini dondur
    # -------------------------------------------------------------------------
    def _emergency_stop(self):
        actions = []

        # 0. Otomatik takibi + tarama state machine'i durdur
        if self.tracking_active:
            self._stop_tracking()
            actions.append("Takip iptal")
        if self.scan_active:
            self._pickup_scan_stop()
            actions.append("Tarama iptal")

        # 1. Tum AGV'lere STOP komutu (WS broadcast)
        if self.client is not None and self.agvs:
            for agv_id in list(self.agvs.keys()):
                try:
                    self.client.command(agv_id, "stop")
                    actions.append(f"AGV {agv_id} STOP")
                except Exception:
                    pass

        # 2. Tum ESP32-CAM kollarini freeze (HTTP GET /arm/freeze)
        #    Sadece aktif arm_url_var icin yapiyoruz - cam registry gelene kadar yeterli
        base = self.arm_url_var.get().strip().rstrip("/") if hasattr(self, "arm_url_var") else ""
        if base:
            import threading, urllib.request
            def freeze_worker(u: str):
                try:
                    with urllib.request.urlopen(f"{u}/arm/freeze", timeout=2.0) as r:
                        r.read()
                except Exception:
                    pass
            threading.Thread(target=freeze_worker, args=(base,), daemon=True).start()
            actions.append(f"Kol freeze ({base})")

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
            try:
                if self.stream_reader is not None:
                    self.stream_reader.stop()
            except Exception:
                pass
            try:
                self._cam_log_stop()
            except Exception:
                pass
            self.destroy()

    # =========================================================================
    # AGV listesi
    # =========================================================================
    def _refresh_agv_list(self):
        # Mevcut butonlari sil
        for w in self.agv_list_inner.winfo_children():
            w.destroy()
        self.agv_buttons.clear()

        if not self.agvs:
            ctk.CTkLabel(self.agv_list_inner, text="(bos)", text_color=COL_SUBTLE).pack(pady=8)
            self.agv_count_lbl.configure(text="AGV: 0 aktif")
            self.active_agv = None
            return

        active_count = 0
        for agv_id, s in sorted(self.agvs.items()):
            if s.connected:
                active_count += 1

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

        self.agv_count_lbl.configure(text=f"AGV: {active_count} aktif")

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

    def _select_agv(self, agv_id: str):
        self.active_agv = agv_id
        self._refresh_agv_list()
        self._update_active_view()

    # =========================================================================
    # Aktif AGV gosterimi
    # =========================================================================
    def _update_active_view(self):
        if self.active_agv is None or self.active_agv not in self.agvs:
            self.map_widget.set_state()
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
            # Dashboard ozeti
            self.dash_map.set_state()
            self.dash_agv_id.configure(text="ID:    -")
            self.dash_state.configure(text="Durum: -")
            self.dash_wp.configure(text="Konum: -")
            self.dash_target.configure(text="Hedef: -")
            self.dash_heading.configure(text="Yön:   -")
            self.dash_sonar.configure(text="Sonar: -")
            self.dash_line.configure(text="Çizgi: -")
            return

        s = self.agvs[self.active_agv]

        # Harita
        self.map_widget.set_state(
            current = s.currentWaypoint,
            target  = s.targetWaypoint,
            path    = s.path,
            heading = s.heading,
        )

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

        # Sensor barlari
        for i, b in enumerate(self.sensor_bars):
            v = s.sensors[i] if i < len(s.sensors) else 0
            b.set_value(v)

        # PID/hiz durumu
        self.pid_status_lbl.configure(
            text=f"Mevcut: Kp={s.Kp:.3f}  Ki={s.Ki:.3f}  Kd={s.Kd:.3f}  Hiz={s.baseSpeed}"
        )

        # Dashboard ozeti (aktif AGV)
        self.dash_map.set_state(
            current = s.currentWaypoint,
            target  = s.targetWaypoint,
            path    = s.path,
            heading = s.heading,
        )
        self.dash_agv_id.configure(text=f"ID:    {self.active_agv}")
        self.dash_state.configure(text=f"Durum: {s.navState or '-'}")
        self.dash_wp.configure(text=f"Konum: {s.currentWaypoint or '-'}")
        self.dash_target.configure(text=f"Hedef: {s.targetWaypoint or '-'}")
        self.dash_heading.configure(text=f"Yön:   {s.heading or '-'}")
        self.dash_sonar.configure(
            text=f"Sonar: {sonar_txt}{engel}",
            text_color=COL_TXT_ERR if s.obstacle else "white",
        )
        self.dash_line.configure(text=f"Çizgi: {s.linePos}")

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
        client, agv = res
        wp = self.tgt_var.get()
        if client.set_target(agv, wp):
            self._set_status(f"{agv}: hedef {wp}")

    def _cmd_command(self, c: str):
        res = self._ensure_active()
        if res is None: return
        client, agv = res
        if client.command(agv, c):
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
        client, agv = res
        self.tgt_var.set(wp)
        client.set_target(agv, wp)
        self._set_status(f"{agv}: harita > hedef {wp}")

    # =========================================================================
    # Kamera sekmesi
    # =========================================================================
    def _cam_connect(self):
        if self.stream_reader is not None:
            return
        url = self.cam_url_var.get().strip()
        if not url:
            return
        self.stream_reader = StreamReader(url)
        if not self.stream_reader.start():
            self.stream_reader = None
            self.cam_info_lbl.configure(text="HATA: Akisa baglanılamadı")
            return
        self.cam_connect_btn.configure(state="disabled")
        self.cam_disconnect_btn.configure(state="normal")
        self.last_stream_count = 0
        self.last_stream_time  = time.time()
        self._cam_log_start(url)

    def _cam_disconnect(self):
        if self.stream_reader is None:
            return
        self.stream_reader.stop()
        self.stream_reader = None
        # Takibi de kapat - yeniden baglanana kadar komut gondermesin
        if self.tracking_active:
            self._stop_tracking()
        self.cam_connect_btn.configure(state="normal")
        self.cam_disconnect_btn.configure(state="disabled")
        # Onemli: once widget'in image referansini bosalt, sonra bizim tuttugumuz
        # CTkImage referansini birak. Tersi yaparsak widget hala silinmis Tcl
        # pyimage'a baglidir ve "image pyimageN doesn't exist" hatasi olusur.
        self.cam_video_lbl.configure(image=None, text="Akis kapali")
        self._cam_video_img_ref = None
        self._cam_log_stop()

    # ----- ESP32-CAM log polling -----
    def _cam_log_start(self, stream_url: str):
        import threading
        from urllib.parse import urlparse
        if self._cam_log_thread is not None and self._cam_log_thread.is_alive():
            return
        host = urlparse(stream_url).hostname
        if not host:
            return
        self._cam_log_stop_evt.clear()
        self._cam_log_last_seq = 0
        self._cam_log_thread = threading.Thread(
            target=self._cam_log_loop, args=(host,), daemon=True
        )
        self._cam_log_thread.start()

    def _cam_log_stop(self):
        self._cam_log_stop_evt.set()

    def _cam_log_loop(self, host: str):
        # /poll?logSince=N - arm state + log birlikte gelir (tek istek = 2 islem)
        import urllib.request, json
        while not self._cam_log_stop_evt.is_set():
            try:
                url = f"http://{host}/poll?logSince={self._cam_log_last_seq}"
                with urllib.request.urlopen(url, timeout=3.0) as r:
                    data = json.loads(r.read().decode("utf-8", errors="replace"))

                cam_id = str(data.get("id", "CAM?"))

                # --- Arm state cache (drag etmeyen slider'lar UI'de guncellenir) ---
                arm = data.get("arm", {}) or {}
                self._cam_arm_state = {
                    "camId":   cam_id,
                    "servos":  list(arm.get("servos", [])),
                    "targets": list(arm.get("targets", [])),
                    "magnet":  bool(arm.get("magnet", 0)),
                    "speedMs": int(arm.get("speedMs", 40)),
                    "led":     int(arm.get("led", 0)),
                }

                # --- Log entry'leri ---
                log_blk = data.get("log", {}) or {}
                cur_seq = int(log_blk.get("seq", 0))
                entries = log_blk.get("entries", []) or []
                entries.sort(key=lambda e: int(e.get("seq", 0)))
                for e in entries:
                    seq_i = int(e.get("seq", 0))
                    if seq_i <= self._cam_log_last_seq:
                        continue
                    self._cam_log_last_seq = seq_i
                    msg  = str(e.get("msg", ""))
                    ms_i = int(e.get("ms", 0))
                    self.after(
                        0,
                        lambda c=cam_id, m=msg, t=ms_i:
                            self._add_log(LogEvent(agvId=c, message=m, time_ms=t))
                    )
                if cur_seq > self._cam_log_last_seq:
                    self._cam_log_last_seq = cur_seq
            except Exception:
                pass
            # 1.5 sn aralikla yokla, stop sinyalinde hemen cik
            self._cam_log_stop_evt.wait(1.5)

    def _update_cam_toggles(self):
        self.show_red_det  = self.cam_red_var.get()
        self.show_blue_det = self.cam_blue_var.get()
        self.show_mask     = self.cam_mask_var.get()

    def _reload_presets(self):
        self.presets = load_presets(PRESET_PATH)
        self._set_status("Preset'ler yeniden yuklendi")

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
        if self._arm_base_url() is None:
            self._set_status("Tarama icin gecerli kamera URL'si gerekli", err=True)
            return
        if self.scan_active:
            self._pickup_scan_stop()

        # Sabit poz (shoulder + elbow) + gripper acılari (ileri/geri yon)
        pose = self._pickup_scan_pose()
        self.scan_gripper_fwd  = pose["gripper_fwd"]
        self.scan_gripper_back = pose["gripper_back"]
        try:
            self._arm_http_get(f"/servo?id=1&a={pose['shoulder']}")
            self._arm_http_get(f"/servo?id=2&a={pose['elbow']}")
            self._arm_http_get(f"/servo?id=3&a={self.scan_gripper_fwd}")
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
        self._arm_http_get(f"/servo?id=0&a={start}")

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
        self._arm_http_get(f"/servo?id=0&a={next_angle}")

        if hit_boundary:
            if not self.scan_loop:
                self._finish_scan(next_angle)
                return
            # Loop modu — yön değiştir ve gripper'i alternate et.
            # Yeni yön +1 ise gripper_fwd, -1 ise gripper_back kullan.
            self.scan_direction = -self.scan_direction
            new_gripper = (self.scan_gripper_fwd if self.scan_direction == +1
                           else self.scan_gripper_back)
            self._arm_http_get(f"/servo?id=3&a={new_gripper}")
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
        entry[which] = int(self.scan_current_angle)
        self.scan_limits_lbl.configure(text=self._pickup_limits_text())
        label = self.SCAN_LABELS.get(self.scan_save_field, self.scan_save_field)
        self._set_status(f"{label} {which} = {self.scan_current_angle}° "
                         f"(henuz JSON'a yazilmadi)")

    def _pickup_save_start(self):
        self._pickup_save_endpoint("start")

    def _pickup_save_end(self):
        self._pickup_save_endpoint("end")

    # ---------- Otomatik takip ----------
    def _on_track_color_change(self):
        self.tracking_color = self.track_color_var.get()

    def _toggle_tracking(self):
        if self.tracking_active:
            self._stop_tracking()
        else:
            self._start_tracking()

    def _start_tracking(self):
        if self.stream_reader is None:
            self._set_status("Takip icin once kamera yayinini ac", err=True)
            return
        if self._arm_base_url() is None:
            self._set_status("Takip icin gecerli kamera URL'si gerekli", err=True)
            return
        # Mevcut base acisini state'ten al (varsa)
        try:
            servos = self._cam_arm_state.get("servos") or []
            if servos and isinstance(servos[0], dict):
                self.tracking_base_angle = int(servos[0].get("target", 93))
        except Exception:
            pass
        self.tracking_active       = True
        self.tracking_last_send_ts = 0.0
        self.tracking_color        = self.track_color_var.get()
        self.track_btn.configure(text="Takibi Durdur",
                                 fg_color=COL_DANGER, hover_color=COL_DANGER_HOVER)
        self.track_status_lbl.configure(text=f"Aktif — hedef: {self.tracking_color}",
                                        text_color=COL_TXT_OK)

    def _stop_tracking(self):
        was_active = self.tracking_active
        self.tracking_active = False
        if hasattr(self, "track_btn"):
            self.track_btn.configure(text="Takibi Baslat",
                                     fg_color=COL_SUCCESS, hover_color=COL_SUCCESS_HOVER)
        if hasattr(self, "track_status_lbl"):
            self.track_status_lbl.configure(text="Pasif", text_color=COL_MUTED)
        return was_active

    def _tracking_step(self, frame, red_dets, blue_dets):
        """Aktif takip adimi: hedef rengin en buyuk detection'inin merkezini frame
        merkezine getirecek sekilde base servo'ya delta uygula. P-only kontrol."""
        target_dets = red_dets if self.tracking_color == "red" else blue_dets
        if not target_dets:
            self.track_status_lbl.configure(
                text=f"Aktif — hedef: {self.tracking_color} (bulunamadi)",
                text_color=COL_TXT_WARN)
            return

        det = target_dets[0]   # en buyuk (detector.py'da sort'lu)
        fh, fw = frame.shape[:2]
        cx_frame = fw // 2
        dx = det.center[0] - cx_frame

        # Olu bant: kucuk sapma -> komut gonderme
        if abs(dx) < 20:
            self.track_status_lbl.configure(
                text=f"Aktif — hedef: {self.tracking_color}  hizali (dx={dx:+d})",
                text_color=COL_TXT_OK)
            return

        # P kontrol: kameranin FoV'u yatayda ~65°, base 0..180. Frame yarisi
        # (fw/2 px) ~32.5° karsiligi. Kp = 32.5 / (fw/2) ~ 0.10 derece/px.
        # Yumusak hareket icin 0.06 secildi; tek adimda max ~6° degisim.
        kp = 0.06
        delta = int(round(kp * dx))
        delta = max(-6, min(6, delta))

        new_angle = self.tracking_base_angle - delta   # dx>0 (kutu sagda) -> aci azalt
        new_angle = max(0, min(180, new_angle))

        # Throttle: 200 ms'de bir komut
        now = time.time()
        if now - self.tracking_last_send_ts < 0.20:
            return
        if new_angle == self.tracking_base_angle:
            return

        self.tracking_base_angle   = new_angle
        self.tracking_last_send_ts = now
        self._arm_http_get(f"/servo?id=0&a={new_angle}")
        self.track_status_lbl.configure(
            text=f"Aktif — dx={dx:+d}  base={new_angle}",
            text_color=COL_TXT_OK)

    def _process_camera(self):
        if self.stream_reader is None:
            return

        frame, count = self.stream_reader.read()
        if frame is None:
            return

        # FPS
        now = time.time()
        if now - self.last_stream_time >= 0.5:
            self.stream_fps = (count - self.last_stream_count) / (now - self.last_stream_time)
            self.last_stream_count = count
            self.last_stream_time  = now

        # Tespit
        red_dets,  red_mask  = ([], None)
        blue_dets, blue_mask = ([], None)

        # Tespit gerekirse calistir: overlay icin gorunurluk toggle'larina, takip
        # icin tracking_active'e bagli. Hedef rengi tespit etmek zorunlu.
        need_red  = self.show_red_det  or (self.tracking_active and self.tracking_color == "red")
        need_blue = self.show_blue_det or (self.tracking_active and self.tracking_color == "blue")

        if need_red:
            red_dets,  red_mask  = detect_red(frame,  self.presets.red,  self.presets.common)
        if need_blue:
            blue_dets, blue_mask = detect_blue(frame, self.presets.blue, self.presets.common)

        # Otomatik takip adimi (varsa)
        if self.tracking_active:
            self._tracking_step(frame, red_dets, blue_dets)

        all_dets = red_dets + blue_dets

        if self.show_mask:
            combined = np.zeros(frame.shape[:2], dtype=np.uint8)
            if red_mask  is not None: combined = cv2.bitwise_or(combined, red_mask)
            if blue_mask is not None: combined = cv2.bitwise_or(combined, blue_mask)
            display = cv2.cvtColor(combined, cv2.COLOR_GRAY2BGR)
            display = draw_detections(display, all_dets)
        else:
            display = draw_detections(frame, all_dets)

        # Bilgi
        info = (f"FPS: {self.stream_fps:.1f}  |  Kirmizi: {len(red_dets)}  "
                f"Mavi: {len(blue_dets)}  |  Frame: {count}")
        if all_dets:
            top = all_dets[0]
            info += f"  |  Buyuk {top.color.upper()}: ({top.center[0]},{top.center[1]})"
        self.cam_info_lbl.configure(text=info)

        # Goruntuye don
        rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(rgb)
        pil.thumbnail((640, 480), Image.Resampling.LANCZOS)
        ctk_img = ctk.CTkImage(light_image=pil, dark_image=pil, size=pil.size)
        self.cam_video_lbl.configure(image=ctk_img, text="")
        # GC engelle - CTkLabel'in dahili `_image` attribute'una dokunmadan
        # kendi attribute'umuzda tut (override edersek Tcl pyimage yasam dongusu bozulur).
        self._cam_video_img_ref = ctk_img

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
        self._refresh_dash_log()

    # Dashboard'un son-olaylar kutusu: son 8 satir
    def _refresh_dash_log(self):
        if not hasattr(self, "dash_log_box"):
            return
        self.dash_log_box.configure(state="normal")
        self.dash_log_box.delete("1.0", "end")
        for ev in self._log_lines[-8:]:
            ts = time.strftime("%H:%M:%S", time.localtime())
            self.dash_log_box.insert("end", f"[{ts}] [{ev.agvId or '-'}] {ev.message}\n")
        self.dash_log_box.see("end")
        self.dash_log_box.configure(state="disabled")

    def _refresh_log(self):
        flt = self.log_filter_var.get()
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        for ev in self._log_lines:
            if flt != "Hepsi" and ev.agvId != flt:
                continue
            ts = time.strftime("%H:%M:%S", time.localtime())
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

            for kind, payload in evs:
                if kind == "connected":
                    self.conn_lbl.configure(text="● Bagli", text_color=COL_TXT_BAGLI)
                    self._set_status("Server'a baglandi")
                elif kind == "disconnected":
                    self.conn_lbl.configure(text="● Baglanti yok", text_color=COL_WARN)
                    self._set_status("Baglanti kesildi")
                elif kind == "error":
                    self._set_status(f"WS: {payload}", err=True)
                elif kind == "agv_list":
                    self.agvs = dict(payload)
                    need_list_refresh = True
                    need_active_refresh = True
                elif kind == "agv_update":
                    s: AGVState = payload
                    if s.id:
                        self.agvs[s.id] = s
                        need_list_refresh = True
                        if self.active_agv == s.id:
                            need_active_refresh = True
                        # Sensör geçmişine ekle (tüm AGV'lerden)
                        self._record_sensor(s)
                elif kind == "log":
                    self._add_log(payload)

            if need_list_refresh:
                self._refresh_agv_list()
            if need_active_refresh:
                self._update_active_view()

    # =========================================================================
    # Durum cubugu
    # =========================================================================
    def _set_status(self, text: str, err: bool = False):
        self.status_bar.configure(
            text=f"{time.strftime('%H:%M:%S')} - {text}",
            text_color=COL_TXT_ERR if err else "white",
        )


# =============================================================================
# Entry
# =============================================================================

if __name__ == "__main__":
    AGVControlApp().mainloop()

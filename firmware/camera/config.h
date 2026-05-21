#pragma once

// =============================================================================
// config.h - ESP32-CAM (OV3660 sensorlu) MJPEG yayin sunucusu
// =============================================================================

// ===== Aractaki ESP32-CAM Kimligi =====
// Her bir CAM'a benzersiz bir ID ata. Birden cok aractaki CAM aynagi gondericek
// ID ile PC tarafinda hangi aracin log'u oldugu ayirt edilir.
// FLASH EDERKEN BU SATIRI HER BOARD ICIN DEGISTIR:
#define CAM_ID          "CAM_1"     // CAM_1 | CAM_2 | CAM_3 ...

// ===== WiFi (AGV Server'in AP'ina baglan) =====
#define WIFI_SSID       "AGV_SERVER"
#define WIFI_PASS       "agv12345"

// Statik IP son okteti - her CAM icin farkli olmali (50, 51, 52...)
// server: 192.168.4.1, AP DHCP havuzu 192.168.4.2-11
#define CAM_IP_OCT_4    50

// ===== Log Ring Buffer =====
// PC tarafindan HTTP polling ile cekilen son N log girdisi tutulur.
#define CAM_LOG_RING    32
#define CAM_LOG_MSGLEN  80

// ===== Kamera (OV3660) =====
// OV3660 hardware JPEG destekliyor; PSRAM ile yuksek cozunurluk rahat kalkar.
//
// FRAMESIZE_QVGA   320x240   ~25-30 fps  HSV renk tespiti icin yeterli
// FRAMESIZE_HVGA   480x320   ~18-22 fps  <-- secildi: YOLO inference icin
//                                            detay yeterli, AP modunda stabil
// FRAMESIZE_VGA    640x480   ~10 fps     daha cok detay, AP modunda zorlanabilir
// FRAMESIZE_SVGA   800x600   ~7-8 fps
// FRAMESIZE_HD     1280x720  ~5 fps
//
// YOLOv8 inference (main_yolo.py) tipik olarak imgsz=320 ile calisir →
// HVGA 480x320 zaten o boyuta yakin, ekstra detay PC'de model.predict
// downscale ile kaybolur. Daha buyuk = bandwidth + CPU israfi.
#define CAM_FRAMESIZE   FRAMESIZE_HVGA

// JPEG kalitesi: 0 (en iyi, en buyuk) - 63 (en kotu, en kucuk)
// Hardware JPEG'de dusuk sayi CPU'yu yormaz, 10-12 ideal
#define CAM_JPEG_QUALITY  10

// PSRAM ile cift tampon -> kesintisiz yakalama
#define CAM_FB_COUNT       2

// ===== HTTP / MJPEG =====
// Port 80: kontrol API (servo, magnet, status) - WebServer (synchronous)
// Port 81: MJPEG stream - ayri WiFiServer + xTask (paralel, kontrolu bloklamaz)
#define HTTP_PORT         80
#define STREAM_PORT       81
#define MJPEG_BOUNDARY   "esp32cam"

// ===== Servo Yumusatma (Ramping) =====
// Hedef aciya kucuk adimlarla yaklasarak ani sicrayisi onler.
// Per-servo step degeri arm.ino'da SERVO_STEP_DEG_PER[] dizisinde tanimli.
// Hareket suresi ~ (mesafe / step) * INTERVAL
// Tum servolar 1°/40ms = 25°/sn  ->  90° hareket = 3.6 sn (akici, titremesiz)
#define SERVO_STEP_INTERVAL_MS  40    // her adim arasi gecikme (ms)

// ===== Debug =====
#define DEBUG_INTERVAL  2000
#define BOARD_LED_PIN     33   // Kirmizi durum LED (aktif LOW)

// ===== Flash LED (Beyaz, GPIO4) =====
// Aydinlatma icin PWM ile kismen yakilir. 0 = kapali, 255 = max parlak.
// Renkler solduysa parlakligi artir (50-100), kameraya yansima yapiyorsa azalt.
#define FLASH_LED_PIN          4
#define FLASH_LED_BRIGHTNESS   0    // BOOT'ta kapali (PC tarafından /led ile acilir)
                                    // 5-10 hafif aydinlatma, 20+ renk soldurur

// ===== Robot Kol Pinleri =====
// 4 servo + 1 elektromiknatis. Pin secimi ESP32-CAM'in sinirli serbest pinlerinden.
// UYARI: GPIO 16 KULLANILMAZ - dahili PSRAM CS hatti. PWM uygulanirsa PSRAM
// bozulur, kamera frame buffer ve WebServer heap'i corrupt olur -> crash.
// GPIO 12 strapping pin (boot'ta LOW olmali); servo signal cogu zaman LOW
// oldugu icin sorun yok, ama reset siralamasinda dikkat.
#define SERVO_BASE_PIN     14   // Base (MG996R) - kol etrafinda donme
#define SERVO_SHOULDER_PIN 13   // Shoulder (DS3218MG) - ana yuk tasiyici
#define SERVO_ELBOW_PIN    15   // Elbow (MG996R) - dirsek
#define SERVO_GRIPPER_PIN  12   // Gripper (MG90S) - GPIO 16 PSRAM CS oldugu icin 12'ye tasindi
#define MAGNET_PIN          2   // MOSFET gate (10kΩ pull-down dısarıda)

// Servo PWM hassasiyet aralıgı (genis aralik DS3218MG icin de tam isler)
#define SERVO_MIN_US       500
#define SERVO_MAX_US      2500

// =============================================================================
// Servo aci sinirlari ve home pozisyonlari (MEKANIK KALIBRASYON)
// Her servo'nun fiziksel olarak ulasabilecegi aralik ve "yukari bakan" home aci.
// Bu degerler kolun govde/kablo carpismalarini onler - DEGISTIRMEDEN ONCE TEST ET.
// =============================================================================
//                       MIN    MAX   HOME (guvenli park / yatay)
// NOT: HOME'a SEQUENTIAL gecis - once shoulder ayarlanir (carpisma onleme),
// shoulder hedefe ulastiktan sonra base/elbow/gripper paralel ayarlanir.
// Bunun icin /arm/home endpoint'i kullanilir (armGoHome fonksiyonu).
#define SERVO_BASE_MIN       0
#define SERVO_BASE_MAX     180
#define SERVO_BASE_HOME     93

#define SERVO_SHOULDER_MIN   0
#define SERVO_SHOULDER_MAX 120
#define SERVO_SHOULDER_HOME 120    // park konumu - HOME sirasinda ILK ayarlanir

#define SERVO_ELBOW_MIN      4
#define SERVO_ELBOW_MAX    153
#define SERVO_ELBOW_HOME    15     // shoulder hazir olunca

#define SERVO_GRIPPER_MIN    0
#define SERVO_GRIPPER_MAX  160
#define SERVO_GRIPPER_HOME 110

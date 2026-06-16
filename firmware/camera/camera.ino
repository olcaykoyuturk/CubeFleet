// =============================================================================
// ESP32-CAM MJPEG Yayin Sunucusu
//
// AGV Server'in AP'ina (AGV_SERVER, 192.168.4.1) statik IP ile baglanir.
// MJPEG akisini PC tarafindaki Python alir, kup tespiti yapar.
//
// Kullanim:
//   1. Server (firmware/server) calisir durumda olmali
//   2. ESP32-CAM'i flash'la (AI-Thinker board, GPIO0->GND ile programla)
//   3. Reset; IP'yi onayla (192.168.4.50)
//   4. PC: http://192.168.4.50:81/stream (port 81!)
// =============================================================================

#include "types.h"

static unsigned long lastDebug   = 0;
static unsigned long framesAtLog = 0;
static uint8_t       flashLedBrightness = 0;   // runtime'da degistirilebilir

// ============================================================================
// Kamera init + frame buffer alma/iade
// AI-Thinker ESP32-CAM + OV3660. PSRAM'de cift framebuffer (kesintisiz capture).
// LEDC kanal/timer 0 -> sensor XCLK; diger LEDC kullanicilari (servo, flash LED)
// farkli kanal/timer kullanir.
// ============================================================================
bool cameraInit() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;             // hardware JPEG
    config.frame_size   = CAM_FRAMESIZE;
    config.jpeg_quality = CAM_JPEG_QUALITY;

    // PSRAM runtime fallback — PSRAM yoksa DRAM tek-fb modu. (DS3225 titremesi
    // icin PSRAM kapatilabilir; o durumda PSRAM init fail eder, psramFound()
    // ile karar veriyoruz.)
    if (psramFound()) {
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.fb_count    = CAM_FB_COUNT;        // 2: kesintisiz capture
        config.grab_mode   = CAMERA_GRAB_LATEST;  // en taze frame
    } else {
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count    = 1;                    // DRAM kucuk, tek fb
        config.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
        camLog("UYARI: PSRAM yok, DRAM 1 fb modu");
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        camLog("esp_camera_init hata: 0x%x", err);
        return false;
    }

    // OV3660 ince ayar — varsayilan oryantasyon ters/gri olabilir
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        if (s->id.PID == OV3660_PID) {
            s->set_vflip(s,    1);     // dik goruntu
            s->set_brightness(s, 1);
            s->set_saturation(s, -2);  // OV3660 sat varsayilani cok yuksek
        }
        // Beyaz dengesi ve pozlama otomatik
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_gain_ctrl(s, 1);
    }

    camLog("Kamera hazir (PID=0x%x)", s ? s->id.PID : 0);
    return true;
}

camera_fb_t* cameraCapture() {
    return esp_camera_fb_get();
}

void cameraReturn(camera_fb_t* fb) {
    if (fb) esp_camera_fb_return(fb);
}

bool cameraIsHardwareJpeg() {
    return true;   // PIXFORMAT_JPEG ile init ettik
}

// ---- Flash LED parlakligi (GPIO 4, PWM 5 kHz 8-bit) ----
void setFlashLedBrightness(uint8_t v) {
    flashLedBrightness = v;
    ledcWrite(FLASH_LED_PIN, v);
    camLog("Flash LED parlaklik: %u/255", v);
}

uint8_t getFlashLedBrightness() {
    return flashLedBrightness;
}

void setup() {
    // Serial.begin yok — GPIO 3 (UART0 RX) grip mikroswitch icin lazim, UART
    // pini sahiplenmesin. Log camLog ring buffer + /poll ile PC'ye gider.
    delay(500);

    pinMode(BOARD_LED_PIN, OUTPUT);
    digitalWrite(BOARD_LED_PIN, HIGH);   // aktif LOW -> kapali

    // Log buffer once - diger init'ler camLog'u cagirabilsin
    camLogInit();
    camLog("Boot: ESP32-CAM (%s)", CAM_ID);

    // 1. Kamera (LEDC timer 0)
    if (!cameraInit()) {
        camLog("FATAL: Kamera baslamadi");
        while (true) {
            digitalWrite(BOARD_LED_PIN, LOW);  delay(100);
            digitalWrite(BOARD_LED_PIN, HIGH); delay(100);
        }
    }

    // 2. Flash LED (kameradan sonra; timer 1'i otomatik alir)
    ledcAttach(FLASH_LED_PIN, 5000, 8);
    setFlashLedBrightness(FLASH_LED_BRIGHTNESS);

    // 3. WiFi
    if (!wifiInit()) {
        camLog("FATAL: WiFi yok, restart");
        delay(2000);
        ESP.restart();
    }
    wifiPrintInfo();

    // 4. Robot kol (servo timer 2-3 + miknatis GPIO + grip sensoru GPIO 3)
    armInit();

    // 5. HTTP / MJPEG + Kol endpoint'leri
    mjpegServerStart();

    digitalWrite(BOARD_LED_PIN, LOW);   // hazir, LED yanik
    camLog("Sistem hazir");
}

void loop() {
    mjpegServerLoop();
    armUpdate();           // servo ramping adimi (her ~20 ms'de 2°)
    gripUpdate();          // grip sensoru debounce + edge logu
    armMagnetUpdate();     // miknatis termal timeout korumasi (kupsuz auto-off)

    // FPS metrik — sadece sayac guncelle (/status endpoint'inden okunur).
    unsigned long now = millis();
    if (now - lastDebug >= DEBUG_INTERVAL) {
        framesAtLog = mjpegFrameCount();
        lastDebug   = now;
    }

    // WiFi reconnect — non-blocking. delay ana loop'u (armUpdate, mjpegServerLoop)
    // bloklar, onun yerine 5 sn aralikla deneriz.
    static bool          wifiWasUp     = true;
    static unsigned long lastReconnect = 0;
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiWasUp) {
            camLog("WiFi koptu");
            wifiWasUp     = false;
            lastReconnect = 0;        // hemen ilk denemeye izin ver
        }
        if (millis() - lastReconnect > 5000) {
            WiFi.reconnect();
            lastReconnect = millis();
        }
    } else if (!wifiWasUp) {
        camLog("WiFi geri geldi");
        wifiWasUp = true;
    }
}

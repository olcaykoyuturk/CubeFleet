#include <WebServer.h>
#include <WiFi.h>
#include "img_converters.h"   // frame2jpg
#include "types.h"

// =============================================================================
// mjpeg_server.ino - MJPEG yayin + Robot Kol kontrol API
//
// Iki ayri sunucu paralel calisir:
//
//   Port 80 (WebServer, synchronous, ana loop'ta handleClient):
//     GET /                       - hoslama sayfasi
//     GET /capture                - tek kare JPEG (yuksek kalite)
//     GET /status                 - JSON: frames, clients, heap, RSSI, logSeq
//     GET /log?since=N            - ring buffer log girdileri (since'tan sonra)
//     GET /poll?logSince=N        - kol durumu + log birlestirilmis (PC polling)
//     GET /servo?id=0..3&a=0..180 - servo aci komutu (per-servo MIN/MAX clamp)
//     GET /servo/speed?ms=5..500  - ramping hizi (kucuk=hizli, default 40ms = 25°/sn)
//     GET /magnet?s=0|1           - elektromiknatis ac/kapat
//     GET /led?b=0..255           - flash LED parlakligi (PWM duty)
//     GET /arm/state              - JSON: servos, targets, magnet, speedMs, led
//     GET /arm/freeze             - ACIL DUR (target = current, ramping durur)
//     GET /arm/home               - sequential HOME (shoulder once, sonra digerleri)
//
//   Port 81 (WiFiServer + xTask core 0):
//     GET /                       - MJPEG canli akis (multipart boundary)
//
// Iki port ayri olunca stream sonsuz dongusu kontrol API'sini bloklamaz.
// Kamera erisimi cameraMutex ile korunur (capture + stream cakismasin).
// =============================================================================

static WebServer            httpServer(HTTP_PORT);
static WiFiServer           streamServer(STREAM_PORT);
static SemaphoreHandle_t    cameraMutex   = NULL;
static unsigned long        totalFrames   = 0;
static volatile int         activeClients = 0;

unsigned long mjpegFrameCount()  { return totalFrames; }
int           mjpegClientCount() { return activeClients; }

static const char* INDEX_PAGE =
    "<html><body style='font-family:sans-serif'>"
    "<h2>ESP32-CAM MJPEG + Robot Kol</h2>"
    "<p>Canli yayin: <code>http://[cam-ip]:81/</code> (ayri port)</p>"
    "<p><a href='/capture'>/capture</a> - tek kare</p>"
    "<p><a href='/status'>/status</a> - durum</p>"
    "<p><a href='/arm/state'>/arm/state</a> - kol durumu</p>"
    "<p>/servo?id=0..3&amp;a=0..180 - servo kontrol</p>"
    "<p>/servo/speed?ms=5..500 - servo hizi (kucuk=hizli, 40=25dps default)</p>"
    "<p>/magnet?s=0|1 - miknatis</p>"
    "<p>/led?b=0..255 - flash LED parlakligi</p>"
    "</body></html>";

static void handleRoot() {
    httpServer.send(200, "text/html", INDEX_PAGE);
}

static void handleCapture() {
    // Stream task'a karsi mutex - 200ms timeout (kamera donmus olabilir)
    if (cameraMutex && xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        httpServer.send(503, "text/plain", "camera busy");
        return;
    }
    camera_fb_t* fb = cameraCapture();
    if (!fb) {
        if (cameraMutex) xSemaphoreGive(cameraMutex);
        httpServer.send(500, "text/plain", "capture failed");
        return;
    }

    httpServer.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
    httpServer.sendHeader("Connection", "close");

    if (fb->format == PIXFORMAT_JPEG) {
        httpServer.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
        cameraReturn(fb);
        if (cameraMutex) xSemaphoreGive(cameraMutex);
    } else {
        uint8_t* jpg_buf = NULL;
        size_t   jpg_len = 0;
        bool ok = frame2jpg(fb, CAM_JPEG_QUALITY, &jpg_buf, &jpg_len);
        cameraReturn(fb);
        if (cameraMutex) xSemaphoreGive(cameraMutex);
        if (!ok || !jpg_buf) {
            httpServer.send(500, "text/plain", "jpeg encode failed");
            return;
        }
        httpServer.send_P(200, "image/jpeg", (const char*)jpg_buf, jpg_len);
        free(jpg_buf);
    }
}

static void handleStatus() {
    char buf[200];
    snprintf(buf, sizeof(buf),
             "{\"id\":\"%s\",\"frames\":%lu,\"clients\":%d,\"heap\":%u,\"rssi\":%d,\"logSeq\":%u}",
             CAM_ID, totalFrames, activeClients,
             ESP.getFreeHeap(), WiFi.RSSI(),
             (unsigned)camLogCurrentSeq());
    httpServer.send(200, "application/json", buf);
}

// GET /log?since=N  - N'den sonraki tum log girdileri JSON dizi olarak doner
static void handleLog() {
    uint32_t since = 0;
    if (httpServer.hasArg("since")) {
        since = (uint32_t) httpServer.arg("since").toInt();
    }
    String out;
    out.reserve(2048);
    camLogToJson(since, out);
    httpServer.send(200, "application/json", out);
}

// GET /poll?logSince=N
//   PC'nin periyodik tek istegi: arm state + log birlikte doner
//   -> /log + /arm/state polling'i tek HTTP cagrisina birlestirir
static void handlePoll() {
    uint32_t since = 0;
    if (httpServer.hasArg("logSince")) {
        since = (uint32_t) httpServer.arg("logSince").toInt();
    }

    String out;
    out.reserve(2200);
    out  = "{\"id\":\"" CAM_ID "\"";
    out += ",\"arm\":{\"servos\":[";
    for (int i = 0; i < 4; i++) {
        if (i) out += ',';
        out += armGetAngle(i);
    }
    out += "],\"targets\":[";
    for (int i = 0; i < 4; i++) {
        if (i) out += ',';
        out += armGetTarget(i);
    }
    out += "],\"magnet\":";
    out += (armGetMagnetState() ? 1 : 0);
    out += ",\"grip\":";
    out += (armGetGripSensor() ? 1 : 0);
    out += ",\"gripEvent\":\"";
    out += armGripEventType();
    out += "\",\"gripEventSeq\":";
    out += (unsigned)armGripEventSeq();
    out += ",\"speedMs\":";
    out += armGetStepInterval();
    out += ",\"led\":";
    out += (int)getFlashLedBrightness();
    out += "},\"log\":";

    String logJson;
    logJson.reserve(2000);
    camLogToJson(since, logJson);
    out += logJson;
    out += "}";

    httpServer.send(200, "application/json", out);
}

// --- Robot Kol Endpoint'leri --------------------------------------------------

// GET /servo?id=0..3&a=0..180
static void handleServo() {
    if (!httpServer.hasArg("id") || !httpServer.hasArg("a")) {
        httpServer.send(400, "text/plain", "missing id or a");
        return;
    }
    int id = httpServer.arg("id").toInt();
    int a  = httpServer.arg("a").toInt();
    if (id < 0 || id > 3) {
        httpServer.send(400, "text/plain", "id out of range");
        return;
    }
    armSetAngle(id, a);
    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"id\":%d,\"target\":%d,\"current\":%d}",
             id, armGetTarget(id), armGetAngle(id));
    httpServer.send(200, "application/json", buf);
}

// GET /arm/freeze  - ACIL DUR: kol mevcut pozisyonda dondurulur
static void handleArmFreeze() {
    armFreeze();
    httpServer.send(200, "application/json", "{\"frozen\":1}");
}

// GET /arm/home  - Sequential HOME: ONCE shoulder, sonra digerleri (carpisma onleme)
static void handleArmHome() {
    armGoHome();
    httpServer.send(200, "application/json", "{\"home\":1}");
}

// GET /magnet?s=0|1
static void handleMagnet() {
    if (!httpServer.hasArg("s")) {
        httpServer.send(400, "text/plain", "missing s");
        return;
    }
    int state = httpServer.arg("s").toInt();
    if (state) armMagnetOn();
    else       armMagnetOff();
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"magnet\":%d}", state ? 1 : 0);
    httpServer.send(200, "application/json", buf);
}

// GET /arm/state
static void handleArmState() {
    char buf[336];
    snprintf(buf, sizeof(buf),
             "{\"id\":\"%s\",\"servos\":[%d,%d,%d,%d],\"targets\":[%d,%d,%d,%d],"
             "\"magnet\":%d,\"grip\":%d,"
             "\"gripEvent\":\"%s\",\"gripEventSeq\":%u,"
             "\"speedMs\":%d,\"led\":%u}",
             CAM_ID,
             armGetAngle(0), armGetAngle(1), armGetAngle(2), armGetAngle(3),
             armGetTarget(0), armGetTarget(1), armGetTarget(2), armGetTarget(3),
             armGetMagnetState() ? 1 : 0,
             armGetGripSensor() ? 1 : 0,
             armGripEventType(), (unsigned)armGripEventSeq(),
             armGetStepInterval(),
             (unsigned)getFlashLedBrightness());
    httpServer.send(200, "application/json", buf);
}

// GET /servo/speed?ms=N  (5..500). Kucuk = hizli (1°/ms).
static void handleServoSpeed() {
    if (!httpServer.hasArg("ms")) {
        httpServer.send(400, "text/plain", "missing ms");
        return;
    }
    int ms = httpServer.arg("ms").toInt();
    armSetStepInterval(ms);
    char buf[48];
    snprintf(buf, sizeof(buf), "{\"speedMs\":%d}", armGetStepInterval());
    httpServer.send(200, "application/json", buf);
}

// GET /led?b=N  (0..255 PWM duty, GPIO 4 flash LED).
static void handleLed() {
    if (!httpServer.hasArg("b")) {
        httpServer.send(400, "text/plain", "missing b");
        return;
    }
    int b = httpServer.arg("b").toInt();
    if (b < 0) b = 0;
    if (b > 255) b = 255;
    setFlashLedBrightness((uint8_t)b);
    char buf[32];
    snprintf(buf, sizeof(buf), "{\"led\":%d}", b);
    httpServer.send(200, "application/json", buf);
}

// GET /reboot - remote restart (dev/teshis icin)
static void handleReboot() {
    httpServer.send(200, "application/json", "{\"reboot\":1}");
    delay(100);
    ESP.restart();
}

// GET /grip-debug - grip sensor diagnostic. Switch ile sorun yasaniyorsa bu
// endpoint ile ham GPIO durumu, debounce state, son event sira no okunur.
//   raw_gpio:    digitalRead(GRIP_SENSE_PIN) ham deger (0=LOW, 1=HIGH)
//   pressed:     active_low yorumu sonrasi "basili mi" (0/1)
//   stable:      debounce sonrasi stable state (0/1)
//   pending:     suan bir debounce timer'i calisiyor mu (0/1)
//   event:       "held" | "lost" | "none"
//   seq:         son event sira numarasi
static void handleGripDebug() {
    int  raw     = armGripRawGpio();
    bool pressed = (raw == LOW);   // active_low varsayim — config.h ile uyumlu
    char buf[224];
    snprintf(buf, sizeof(buf),
        "{\"raw_gpio\":%d,\"pressed\":%d,\"stable\":%d,\"pending\":%d,"
        "\"event\":\"%s\",\"seq\":%u,\"magnet\":%d}",
        raw, pressed ? 1 : 0,
        armGetGripSensor() ? 1 : 0,
        armGripPending() ? 1 : 0,
        armGripEventType(), (unsigned)armGripEventSeq(),
        armGetMagnetState() ? 1 : 0);
    httpServer.send(200, "application/json", buf);
}

// Tek MJPEG istemcisini servis et (streamTask icinden cagrilir)
static void serveMjpegClient(WiFiClient& client) {
    // HTTP request header'ini hizla tuket - "\r\n\r\n" gorunce hemen kir.
    // Tipik GET header ~80-150 byte, 5-15 ms'de tukenir (eski 200 ms while loop'tan cok daha hizli).
    unsigned long t0 = millis();
    int newlineRun = 0;          // ardisik "\n" sayaci - 2 olursa bos satir = header sonu
    while (client.connected() && millis() - t0 < 100) {
        if (!client.available()) { delay(1); continue; }
        int c = client.read();
        if (c == '\n') {
            if (++newlineRun >= 2) break;
        } else if (c != '\r') {
            newlineRun = 0;
        }
    }

    activeClients++;
    camLog("Stream client baglandi (toplam:%d)", activeClients);

    static const char* HDR =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=" MJPEG_BOUNDARY "\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n\r\n";
    client.print(HDR);

    while (client.connected()) {
        // Mutex'i 100ms ile dene; alinamazsa frame'i atla (kamera donmus olabilir)
        if (cameraMutex && xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        camera_fb_t* fb = cameraCapture();
        if (!fb) {
            if (cameraMutex) xSemaphoreGive(cameraMutex);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        const uint8_t* data    = NULL;
        size_t         dlen    = 0;
        uint8_t*       jpg_buf = NULL;

        if (fb->format == PIXFORMAT_JPEG) {
            data = fb->buf;
            dlen = fb->len;
        } else {
            size_t jpg_len = 0;
            bool ok = frame2jpg(fb, CAM_JPEG_QUALITY, &jpg_buf, &jpg_len);
            if (!ok || !jpg_buf) {
                cameraReturn(fb);
                if (cameraMutex) xSemaphoreGive(cameraMutex);
                if (jpg_buf) free(jpg_buf);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            data = jpg_buf;
            dlen = jpg_len;
        }

        char part[96];
        int  pn = snprintf(part, sizeof(part),
                           "--" MJPEG_BOUNDARY "\r\n"
                           "Content-Type: image/jpeg\r\n"
                           "Content-Length: %u\r\n\r\n",
                           (unsigned)dlen);

        size_t w1 = client.write((const uint8_t*)part, pn);
        size_t w2 = client.write(data, dlen);
        size_t w3 = client.write((const uint8_t*)"\r\n", 2);
        bool wrote_ok = (w1 == (size_t)pn) && (w2 == dlen) && (w3 == 2);

        cameraReturn(fb);
        if (cameraMutex) xSemaphoreGive(cameraMutex);
        if (jpg_buf) free(jpg_buf);

        if (!wrote_ok) break;
        totalFrames++;
    }

    activeClients--;
    camLog("Stream client koptu (toplam:%d)", activeClients);
    client.stop();
}

// Ayri FreeRTOS task'i: port 81'i dinler, accept ettigi her client'i MJPEG ile besler.
// Core 0'da calisir, ana loop'taki WebServer'i (port 80) bloklamaz.
static void streamTask(void* arg) {
    streamServer.begin();
    camLog("MJPEG sunucu hazir (port %d)", STREAM_PORT);
    for (;;) {
        WiFiClient client = streamServer.available();
        if (client) {
            serveMjpegClient(client);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mjpegServerStart() {
    // Kamera mutex - stream task ve capture handler ayni anda fb_get cagiramaz
    cameraMutex = xSemaphoreCreateMutex();

    // Port 80: kontrol API (stream YOK - port 81'de)
    httpServer.on("/",          HTTP_GET, handleRoot);
    httpServer.on("/capture",   HTTP_GET, handleCapture);
    httpServer.on("/status",    HTTP_GET, handleStatus);
    httpServer.on("/log",       HTTP_GET, handleLog);
    httpServer.on("/poll",      HTTP_GET, handlePoll);
    httpServer.on("/servo",      HTTP_GET, handleServo);
    httpServer.on("/magnet",     HTTP_GET, handleMagnet);
    httpServer.on("/arm/state",  HTTP_GET, handleArmState);
    httpServer.on("/arm/freeze", HTTP_GET, handleArmFreeze);   // acil dur
    httpServer.on("/arm/home",   HTTP_GET, handleArmHome);     // sequential home
    httpServer.on("/servo/speed", HTTP_GET, handleServoSpeed); // ramping ms ayari
    httpServer.on("/led",         HTTP_GET, handleLed);        // flash LED parlaklik
    httpServer.on("/reboot",      HTTP_GET, handleReboot);     // remote restart
    httpServer.on("/grip-debug",  HTTP_GET, handleGripDebug);  // diagnostic
    httpServer.onNotFound([]() { httpServer.send(404, "text/plain", "Not Found"); });
    httpServer.begin();

    // Port 81: MJPEG stream - ayri task, core 0
    xTaskCreatePinnedToCore(
        streamTask,     // function
        "mjpegStream",  // name
        8192,           // stack
        NULL,           // arg
        1,              // priority
        NULL,           // handle
        0               // core 0 (WiFi event'leri burada calisir, stream burada)
    );
}

// Bu fonksiyon ana loop'tan cagrilir.
void mjpegServerLoop() {
    httpServer.handleClient();
}

// ===== İletişim Modülü =====
// Sorumluluklar: WiFi bağlantısı, WebSocket protokolü, gelen mesaj yönlendirme.
// Navigasyon iç state'ine doğrudan erişmez — navCommand* arayüzünü kullanır.

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "types.h"

// =============================================================================
// Bağlantı durumu — bu modülün sahibi websocket.ino
// =============================================================================
WebSocketsClient webSocket;
bool wsConnected     = false;

// PC heartbeat tracker — PC kaynaklı son mesaj zamanı. Server'in pong/registered
// cevapları sayılmaz. PC her ~1.5sn pcHeartbeat gönderir; arada gelen setHop/komut
// da reset eder. Timeout aşılırsa AGV moving'se motoru durdurur (PC öldü varsayımı).
static volatile unsigned long lastPCActivityMs = 0;
static const unsigned long PC_HEARTBEAT_TIMEOUT_MS = 3000;

static unsigned long lastStatusSend = 0;
static unsigned long lastPing       = 0;

// Kalibrasyon sırasında WS koparsa sendCalibrationData() için pending flag —
// WStype_CONNECTED'da yeniden gönderilir, PC countdown takılmaz.
static bool s_pendingCalibData = false;

// ===== WiFi / Sunucu Ayarları =====
static const char* WIFI_SSID  = "AGV_SERVER";
static const char* WIFI_PASS  = "agv12345";
static const char* SERVER_IP  = "192.168.4.1";
static const int   SERVER_PORT = 80;
static const char* AGV_ID     = "AGV_1";   // Her araca benzersiz ID

// =============================================================================
// Gelen mesaj yönlendirici — webSocketEvent'ten önce tanımlanmalı
// =============================================================================

static void handleServerMessage(uint8_t* payload, size_t length) {
    // applyCalibration mesajında 16 int'lik dizi var, doc büyük olmalı
    StaticJsonDocument<768> doc;
    if (deserializeJson(doc, payload, length)) {
        return;
    }

    const char* type = doc["type"];

    // --- AGV'den sunucuya giden mesajların echo'ları (PC kaynakli DEGIL) ---
    if (strcmp(type, "registered") == 0) {
        return;
    }
    if (strcmp(type, "pong") == 0) return;

    // PC kaynakli her mesaj heartbeat sayilir → timer reset.
    // (pcHeartbeat, setHop, command, vs hepsi PC'den gelir.)
    lastPCActivityMs = millis();

    // pcHeartbeat sadece timer reset icin — no-op, return.
    if (strcmp(type, "pcHeartbeat") == 0) return;

    // --- Konum bildir (başlangıç noktası) ---
    if (strcmp(type, "setPosition") == 0) {
        const char* wp = doc["waypoint"];
        if (wp && wp[0]) navCommandSetPosition(wp[0]);
        return;
    }

    // (setTarget kaldirildi — path planning PC'de yapiliyor, PC setHop ile
    // 2-hop emir gonderir. setTarget gelirse sessizce yoksay.)

    // --- PID ayarı ---
    if (strcmp(type, "setPID") == 0) {
        navCommandSetPID(
            doc["Kp"] | pidParams.Kp,
            doc["Ki"] | pidParams.Ki,
            doc["Kd"] | pidParams.Kd
        );
        return;
    }

    // --- Hız ayarı ---
    if (strcmp(type, "setSpeed") == 0) {
        navCommandSetSpeed(doc["speed"] | baseSpeed);
        return;
    }

    // --- Komutlar ---
    if (strcmp(type, "command") == 0) {
        const char* cmd = doc["command"];
        if      (strcmp(cmd, "start")     == 0) navCommandStart();
        else if (strcmp(cmd, "stop")      == 0) navCommandStop();
        else if (strcmp(cmd, "calibrate") == 0) navCommandCalibrate();
        else if (strcmp(cmd, "boostTurn") == 0) navCommandBoostTurn();
        else if (strcmp(cmd, "carryOff")  == 0) navCommandCarryOff();
        return;
    }

    // --- Kalibrasyon preset uygulama ---
    if (strcmp(type, "applyCalibration") == 0) {
        JsonArray minArr = doc["sensorMin"].as<JsonArray>();
        JsonArray maxArr = doc["sensorMax"].as<JsonArray>();
        if (minArr.size() != SENSOR_COUNT || maxArr.size() != SENSOR_COUNT) {
            sendLog("applyCalibration: dizi uzunlugu hatali");
            return;
        }
        int minVals[SENSOR_COUNT], maxVals[SENSOR_COUNT];
        for (int i = 0; i < SENSOR_COUNT; i++) {
            minVals[i] = minArr[i].as<int>();
            maxVals[i] = maxArr[i].as<int>();
        }
        navCommandApplyCalibration(minVals, maxVals);
        return;
    }

    // --- Planner: setHop (look-ahead emir) ---
    // payload: {type:"setHop", agvId, from, next, after?, goal?}
    // goal: mission nihai hedefi (REACHED kontrolü için); after lokal devam için.
    if (strcmp(type, "setHop") == 0) {
        const char* from   = doc["from"];
        const char* next_  = doc["next"];
        const char* after  = doc["after"];    // null olabilir
        const char* after2 = doc["after2"];   // null olabilir (3-hop)
        const char* goal   = doc["goal"];     // null olabilir
        if (from && from[0] && next_ && next_[0]) {
            navCommandHop(
                from[0],
                next_[0],
                (after  && after[0])  ? after[0]  : 0,
                (after2 && after2[0]) ? after2[0] : 0,
                (goal   && goal[0])   ? goal[0]   : 0
            );
        }
        return;
    }

    // --- Multi-AGV Planner: clearMission ---
    if (strcmp(type, "clearMission") == 0) {
        navCommandStop();
        return;
    }

    // --- Kup yonune don (faceDir) ---
    // payload: {type:"faceDir", agvId, dir:"N"|"E"|"S"|"W"}
    // Yalniz NAV_IDLE'da kabul edilir; bitince faceComplete doner.
    if (strcmp(type, "faceDir") == 0) {
        const char* dir = doc["dir"];
        if (dir && dir[0]) navCommandFaceDir(dir[0]);
        return;
    }
}

// =============================================================================
// Başlatma
// =============================================================================

void wifiInit() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500);
    }
}

void webSocketInit() {
    webSocket.begin(SERVER_IP, SERVER_PORT, "/ws");

    // Lambda kullanılıyor: Arduino IDE named function için prototype üretemez,
    // bu yüzden WStype_t include'dan önce görülür ve derleme hata verir.
    // Lambda bu sorunu tamamen ortadan kaldırır.
    webSocket.onEvent([](WStype_t type, uint8_t* payload, size_t length) {
        (void)length;   // ERROR/TEXT dışında kullanılmaz
        switch (type) {
            case WStype_CONNECTED:
                wsConnected      = true;
                lastPCActivityMs = millis();   // fresh start
                {
                    StaticJsonDocument<64> doc;
                    doc["type"] = "register";
                    doc["id"]   = AGV_ID;
                    String msg; serializeJson(doc, msg);
                    webSocket.sendTXT(msg);
                }
                // NOT: pending calibrationData gonderimi BURADAN cikti — callback
                // icinden delay() yapip motorlari bloklamak yerine webSocketLoop()
                // her tick'te flag'i kontrol edip yolluyor (asagida).
                break;

            case WStype_DISCONNECTED:
                wsConnected = false;
                navCommandStop();
                break;

            case WStype_TEXT:
                handleServerMessage(payload, length);
                break;

            default:
                break;
        }
    });

    webSocket.setReconnectInterval(3000);
}

// =============================================================================
// Ana döngü — loop() tarafından çağrılır
// =============================================================================

// PC heartbeat timeout kontrolu — moving AGV'yi PC ile baglanti kesilirse durdur.
// AGV-server WS link saglikli olsa bile PC server'dan ayrildiysa (PC crash, WiFi
// drop) heartbeat mesaji gelmez → asagida motorStop + IDLE.
static void checkPCHeartbeat() {
    if (!wsConnected) return;
    if (lastPCActivityMs == 0) return;   // henuz mesaj yok, sayim baslamadi
    if (millis() - lastPCActivityMs < PC_HEARTBEAT_TIMEOUT_MS) return;
    // Timeout — moving ise durdur
    if (navState == NAV_IDLE || navState == NAV_REACHED) return;
    sendLog("PC heartbeat timeout — motor STOP, IDLE'a gecildi");
    motorStop();
    navState      = NAV_IDLE;
    isTarget      = false;
    navPathIndex  = 0;
    navPathLength = 0;
    lastPCActivityMs = millis();   // resetle, log spam onle
}

void webSocketLoop() {
    webSocket.loop();
    checkPCHeartbeat();

    if (!wsConnected) return;

    // Bekleyen kalibrasyon verisi: bağlantı callback'inde göndermiyoruz (orada
    // delay() motorları bloklardı). Her tick flag'e bakıp yolluyoruz; başarısız
    // olursa flag kalır, sonraki tick tekrar denenir.
    if (s_pendingCalibData && calibData.isCalibrated) {
        sendCalibrationData();
    }

    unsigned long now = millis();
    // Kalibrasyon sirasinda sendStatus'u atla — WS trafigini azalt, disconnect
    // riskini dusur. sendPing devam eder ki server timeout'a dusmesin.
    if (!calibrationActive && now - lastStatusSend >= STATUS_INTERVAL) {
        sendStatus();
        lastStatusSend = now;
    }
    if (now - lastPing >= PING_INTERVAL) {
        sendPing();
        lastPing = now;
    }
}

// =============================================================================
// Gönderim fonksiyonları
// =============================================================================

void sendStatus() {
    // IDLE'da bile sensorler fresh okunsun — UI'da hardware kontrolu icin.
    // Kalibre yoksa sensorCalibrated[] her zaman 0 oluyor (range=0 fallback);
    // bu durumda asagida RAW sensorValues[] gonderiyoruz ki kullanici QTR'in
    // gercekten okuyup okumadigini gorebilsin.
    readCalibratedSensors();

    StaticJsonDocument<768> doc;
    doc["type"]      = "status";
    char cwStr[2] = {currentWaypoint ? currentWaypoint : '?', 0};
    char twStr[2] = {targetWaypoint  ? targetWaypoint  : '?', 0};
    doc["currentWaypoint"] = cwStr;
    doc["targetWaypoint"]  = twStr;
    // Yol dizisini "A>B>E>H" formatında gönder
    char pathStr[MAX_PATH_LENGTH * 2 + 1] = "";
    for (int i = 0; i < navPathLength; i++) {
        char seg[3] = {navPath[i], (i < navPathLength - 1 ? '>' : '\0'), '\0'};
        strncat(pathStr, seg, sizeof(pathStr) - strlen(pathStr) - 1);
    }
    doc["path"]      = pathStr;
    doc["pathIdx"]   = navPathIndex;
    doc["heading"]   = getHeadingName();
    doc["linePos"]   = linePosition;
    doc["Kp"]        = pidParams.Kp;
    doc["Ki"]        = pidParams.Ki;
    doc["Kd"]        = pidParams.Kd;
    doc["baseSpeed"] = baseSpeed;
    doc["calibrated"]= calibData.isCalibrated;
    doc["isTarget"]  = isTarget;

    switch (navState) {
        case NAV_IDLE:        doc["navState"] = "IDLE";        break;
        case NAV_TURNING:     doc["navState"] = "TURNING";     break;
        case NAV_FOLLOWING:   doc["navState"] = "FOLLOWING";   break;
        case NAV_AT_JUNCTION: doc["navState"] = "JUNCTION";    break;
        case NAV_REACHED:     doc["navState"] = "REACHED";     break;
        case NAV_LINE_SEARCH: doc["navState"] = "LINE_SEARCH"; break;
        default:              doc["navState"] = "UNKNOWN";     break;
    }

    doc["sonarDist"] = getSonarDistance();
    doc["obstacle"]  = isObstacleDetected();   // STOP zone
    doc["sonarSlow"] = isObstacleSlow();       // SLOW zone

    JsonArray sensors = doc.createNestedArray("sensors");
    // Kalibreyse 0-1000 normalize, degilse RAW ADC (0-4095) gonder.
    // UI 4095 araliginda buyuk degerler gorurse hardware OK, hep 0 ise donanim sorunu.
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensors.add(calibData.isCalibrated ? sensorCalibrated[i] : sensorValues[i]);
    }

    String msg; serializeJson(doc, msg);
    webSocket.sendTXT(msg);
}

void sendPing() {
    StaticJsonDocument<32> doc;
    doc["type"] = "ping";
    String msg; serializeJson(doc, msg);
    webSocket.sendTXT(msg);
}

void sendLog(const char* message) {
    if (!wsConnected) return;
    StaticJsonDocument<128> doc;
    doc["type"]    = "log";
    doc["agvId"]   = AGV_ID;
    doc["message"] = message;
    String msg; serializeJson(doc, msg);
    webSocket.sendTXT(msg);
}

// Bir hop'u tamamladigimizi PC planner'a bildir. Yeni node + (ops) heading.
// Planner bunu alip kendi `on_hop_complete(agv_id, node)` cagrir → rezervasyon
// guncellenir + sonraki tick yeni setHop emri uretebilir.
void sendHopComplete(char node, const char* heading) {
    if (!wsConnected) return;
    StaticJsonDocument<128> doc;
    doc["type"]    = "hopComplete";
    doc["agvId"]   = AGV_ID;
    char nodeStr[2] = { node, 0 };
    doc["node"]    = nodeStr;
    if (heading && heading[0]) doc["heading"] = heading;
    // PC debug timing: firmware uptime ms — server'da ve PC'de latency olcumu
    // icin kullanilir. Server bunu degistirmeden forward eder.
    doc["time"]    = millis();
    String msg; serializeJson(doc, msg);
    webSocket.sendTXT(msg);
}

// faceDir tamamlandi: yeni heading + bulunulan node PC'ye bildirilir.
// PC bunu alip otonom kapma akisini (kamera + kol) baslatabilir.
void sendFaceComplete(char node, const char* heading) {
    if (!wsConnected) return;
    StaticJsonDocument<128> doc;
    doc["type"]    = "faceComplete";
    doc["agvId"]   = AGV_ID;
    char nodeStr[2] = { node, 0 };
    doc["node"]    = nodeStr;
    if (heading && heading[0]) doc["heading"] = heading;
    doc["time"]    = millis();
    String msg; serializeJson(doc, msg);
    webSocket.sendTXT(msg);
}

// Önemli WS gönderiminden sonra flush — TX buffer TCP'ye aksın diye webSocketLoop'a
// vakit ver. Hızlı ardışık göndermelerde mesaj kaybını önler (örn. kalibrasyon sonu).
void wsFlush(int yields) {
    for (int i = 0; i < yields; i++) {
        webSocket.loop();
        delay(15);
    }
}

// Manuel kalibrasyon bitiminde PC'ye ham min/max degerlerini gonder.
// PC bunu preset olarak kaydedebilir veya direkt kullanmaya devam edebilir.
// WS kopuksa pending flag set edilir → WStype_CONNECTED'da otomatik resend.
// sendTXT basarisiz olursa flag tekrar set edilir (bir sonraki firsat'ta retry).
void sendCalibrationData() {
    if (!wsConnected) {
        s_pendingCalibData = true;
        return;
    }
    StaticJsonDocument<512> doc;
    doc["type"]  = "calibrationData";
    doc["agvId"] = AGV_ID;
    JsonArray minArr = doc.createNestedArray("sensorMin");
    JsonArray maxArr = doc.createNestedArray("sensorMax");
    for (int i = 0; i < SENSOR_COUNT; i++) {
        minArr.add(calibData.minVal[i]);
        maxArr.add(calibData.maxVal[i]);
    }
    String msg; serializeJson(doc, msg);
    s_pendingCalibData = !webSocket.sendTXT(msg);   // false = retry icin pending
}

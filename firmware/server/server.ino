// ===== ESP32 AGV Server =====
// WiFi Access Point + WebSocket relay
// Dashboard tarafi PC uzerinde (CustomTkinter), bu sunucu sadece WS koprulemesi yapar.

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ===== WiFi AP Ayarlari =====
const char* AP_SSID = "AGV_SERVER";
const char* AP_PASS = "agv12345";

// ===== Server Nesneleri =====
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ===== AGV Yapisi =====
#define MAX_AGVS 5
#define PING_TIMEOUT 20000  // 20 saniye

struct AGVClient {
    bool connected;
    String id;
    uint32_t clientId;
    unsigned long lastPing;

    // Navigasyon (waypoint tabanlı)
    String currentWaypoint;   // "A"-"I", "?" = bilinmiyor
    String targetWaypoint;
    String path;              // "A>B>E>H"
    int    pathIdx;
    String heading;
    String navState;

    // Sensor verileri
    int sensorValues[8];
    int linePosition;

    // PID ve hiz
    float Kp;
    float Ki;
    float Kd;
    int baseSpeed;

    // Kalibrasyon ve hedef
    bool isCalibrated;
    bool isTarget;

    // Engel / sonar
    float sonarDist;
    bool  obstacle;     // STOP zone
    bool  sonarSlow;    // SLOW zone
};

AGVClient agvs[MAX_AGVS];

// ===== Fonksiyon Prototipleri =====
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len);
void handleWebSocketMessage(AsyncWebSocketClient *client, uint8_t *data, size_t len);
void broadcastAGVList();
void sendAGVUpdate(int idx);
void broadcastLog(String agvId, String message);
void checkTimeouts();
int findAGVByClientId(uint32_t clientId);
int findAGVById(String id);
int findEmptySlot();

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("================================");
    Serial.println("   ESP32 AGV Server Starting    ");
    Serial.println("================================");

    // AGV listesini temizle
    for (int i = 0; i < MAX_AGVS; i++) {
        agvs[i].connected = false;
        agvs[i].id = "";
        agvs[i].clientId = 0;
    }

    // WiFi Access Point baslat
    Serial.print("WiFi AP baslatiliyor: ");
    Serial.println(AP_SSID);

    WiFi.mode(WIFI_AP);
    // Parametreler: ssid, pass, channel, hidden, max_connection
    // 3 AGV + 3 ESP32-CAM + 1 PC = 7 cihaz icin max=8 ayarliyoruz
    WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 8);

    Serial.print("AP IP adresi: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("Max bagli cihaz: 8");

    // WebSocket ayarla
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);

    // Server baslat (sadece WebSocket icin)
    server.begin();
    Serial.println("WebSocket Server baslatildi.");
    Serial.println("WS endpoint: ws://192.168.4.1/ws");
    Serial.println("================================");
}

void loop() {
    // WebSocket temizligi
    ws.cleanupClients();

    // Timeout kontrolu
    checkTimeouts();

    delay(100);
}

// ===== WebSocket Event Handler =====
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u baglandi: %s "
                          "(total=%u, uptime=%lu ms, heap=%u)\n",
                         client->id(),
                         client->remoteIP().toString().c_str(),
                         ws.count(), millis(), ESP.getFreeHeap());
            break;

        case WS_EVT_DISCONNECT:
            // Detayli drop logu — sebep teshisi icin: kalan client, uptime,
            // heap. WiFi RSSI da yardimci olur ama AsyncWS bu evente vermez;
            // bunu periyodik baska yerde basabiliriz.
            {
                int idx = findAGVByClientId(client->id());
                Serial.printf("WebSocket client #%u ayrildi "
                              "(role=%s, total_after=%u, uptime=%lu ms, "
                              "heap=%u, rssi=%d dBm)\n",
                              client->id(),
                              (idx >= 0) ? agvs[idx].id.c_str() : "PC?",
                              ws.count(), millis(),
                              ESP.getFreeHeap(),
                              WiFi.RSSI());
                if (idx >= 0) {
                    agvs[idx].connected = false;
                    broadcastAGVList();
                }
            }
            break;

        case WS_EVT_DATA: {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len)
                handleWebSocketMessage(client, data, len);
            break;
        }

        case WS_EVT_PONG:
            break;

        case WS_EVT_ERROR:
            Serial.printf("WebSocket error client #%u\n", client->id());
            break;
    }
}

// ===== WebSocket Mesaj Isleyici =====
void handleWebSocketMessage(AsyncWebSocketClient *client, uint8_t *data, size_t len) {
    String message((char*)data, len);

    // JSON parse
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, message);

    if (error) {
        Serial.println("JSON parse hatasi");
        return;
    }

    String type = doc["type"].as<String>();

    // ===== AGV'den gelen mesajlar =====
    if (type == "register") {
        // Yeni AGV kaydı
        String agvId = doc["id"].as<String>();
        int idx = findAGVById(agvId);

        if (idx < 0) {
            idx = findEmptySlot();
        }

        if (idx >= 0) {
            agvs[idx].connected = true;
            agvs[idx].id = agvId;
            agvs[idx].clientId = client->id();
            agvs[idx].lastPing = millis();
            agvs[idx].isCalibrated = false;

            Serial.printf("AGV kayit: %s (slot %d)\n", agvId.c_str(), idx);

            // Onay gonder
            StaticJsonDocument<128> response;
            response["type"] = "registered";
            response["status"] = "ok";
            String responseStr;
            serializeJson(response, responseStr);
            client->text(responseStr);

            broadcastAGVList();
        }
    }
    else if (type == "ping") {
        // Ping - baglanti canli
        int idx = findAGVByClientId(client->id());
        if (idx >= 0) {
            agvs[idx].lastPing = millis();
        }

        // Pong gonder
        StaticJsonDocument<64> pong;
        pong["type"] = "pong";
        String pongStr;
        serializeJson(pong, pongStr);
        client->text(pongStr);
    }
    else if (type == "pcHeartbeat") {
        // PC -> AGV heartbeat. PC her ~1.5sn gonderir, AGV bunu alip
        // lastPCActivityMs guncel tutar. Tum clientlere broadcast — AGV'ler
        // PC kaynakli mesaj olarak gorur, ekstra tip kontrolu firmware'de.
        // (Hafif: <30 byte, 1.5sn periyot = ~20 byte/sn ek trafik.)
        ws.textAll(message);
    }
    else if (type == "status") {
        // AGV durum guncellemesi
        int idx = findAGVByClientId(client->id());
        if (idx >= 0) {
            agvs[idx].lastPing = millis();
            agvs[idx].currentWaypoint = doc["currentWaypoint"].as<String>();
            agvs[idx].targetWaypoint  = doc["targetWaypoint"].as<String>();
            agvs[idx].path    = doc["path"].as<String>();
            agvs[idx].pathIdx = doc["pathIdx"] | 0;
            agvs[idx].heading  = doc["heading"].as<String>();
            agvs[idx].navState = doc["navState"].as<String>();
            agvs[idx].linePosition = doc["linePos"] | 0;
            agvs[idx].Kp = doc["Kp"] | 0.0f;
            agvs[idx].Ki = doc["Ki"] | 0.0f;
            agvs[idx].Kd = doc["Kd"] | 0.0f;
            agvs[idx].baseSpeed   = doc["baseSpeed"] | 0;
            agvs[idx].isCalibrated = doc["calibrated"] | false;
            agvs[idx].isTarget    = doc["isTarget"] | false;
            agvs[idx].sonarDist   = doc["sonarDist"] | 999.0f;
            agvs[idx].obstacle    = doc["obstacle"] | false;
            agvs[idx].sonarSlow   = doc["sonarSlow"] | false;

            // Sensor verileri
            JsonArray sensors = doc["sensors"].as<JsonArray>();
            int i = 0;
            for (JsonVariant v : sensors) {
                if (i < 8) agvs[idx].sensorValues[i++] = v.as<int>();
            }

            // Her status'ta broadcast yapma - sadece dashboard'a guncelleme gonder
            sendAGVUpdate(idx);
        }
    }
    else if (type == "log") {
        // AGV'den gelen log mesaji
        String agvId = doc["agvId"].as<String>();
        String message = doc["message"].as<String>();
        broadcastLog(agvId, message);
    }
    else if (type == "calibrationData") {
        // AGV manuel kalibrasyonu bitirdi, ham min/max degerlerini PC'ye yolla.
        // Server bunu sadece tum clientlere broadcast eder; PC dinler + preset
        // olarak kaydedebilir.
        ws.textAll(message);
        Serial.printf("[CALIB] %s'ten kalibrasyon verisi alindi, broadcast edildi\n",
                     doc["agvId"].as<String>().c_str());
    }

    // ===== Dashboard'dan gelen komutlar =====
    else if (type == "setPosition") {
        String agvId = doc["agvId"].as<String>();
        String waypoint = doc["waypoint"].as<String>();

        int idx = findAGVById(agvId);
        if (idx >= 0 && agvs[idx].connected) {
            StaticJsonDocument<128> cmd;
            cmd["type"]     = "setPosition";
            cmd["waypoint"] = waypoint;
            String cmdStr;
            serializeJson(cmd, cmdStr);
            ws.text(agvs[idx].clientId, cmdStr);
            Serial.printf("Konum gonderildi %s: %s\n", agvId.c_str(), waypoint.c_str());
            broadcastLog(agvId, "Konum ayarlandi: " + waypoint);
        }
    }
    // setTarget kaldirildi: path planning PC tarafinda (fleet_planner),
    // 2-hop emir setHop ile gonderilir. PC istemcisi de bu komutu uretmiyor.
    else if (type == "setPID") {
        String agvId = doc["agvId"].as<String>();
        float Kp = doc["Kp"] | 0.0;
        float Ki = doc["Ki"] | 0.0;
        float Kd = doc["Kd"] | 0.0;

        int idx = findAGVById(agvId);
        if (idx >= 0 && agvs[idx].connected) {
            StaticJsonDocument<128> cmd;
            cmd["type"] = "setPID";
            cmd["Kp"] = Kp;
            cmd["Ki"] = Ki;
            cmd["Kd"] = Kd;
            String cmdStr;
            serializeJson(cmd, cmdStr);

            ws.text(agvs[idx].clientId, cmdStr);
            Serial.printf("PID gonderildi %s: Kp=%.3f Ki=%.3f Kd=%.3f\n",
                         agvId.c_str(), Kp, Ki, Kd);

            // Log gonder
            char buf[64];
            sprintf(buf, "PID: Kp=%.3f Ki=%.3f Kd=%.3f", Kp, Ki, Kd);
            broadcastLog(agvId, String(buf));
        }
    }
    else if (type == "setSpeed") {
        String agvId = doc["agvId"].as<String>();
        int speed = doc["speed"] | 25;

        int idx = findAGVById(agvId);
        if (idx >= 0 && agvs[idx].connected) {
            StaticJsonDocument<64> cmd;
            cmd["type"] = "setSpeed";
            cmd["speed"] = speed;
            String cmdStr;
            serializeJson(cmd, cmdStr);

            ws.text(agvs[idx].clientId, cmdStr);
            Serial.printf("Hiz gonderildi %s: %d\n", agvId.c_str(), speed);

            // Log gonder
            broadcastLog(agvId, "Hiz ayarlandi: " + String(speed));
        }
    }
    else if (type == "command") {
        String agvId = doc["agvId"].as<String>();
        String command = doc["command"].as<String>();

        int idx = findAGVById(agvId);
        if (idx >= 0 && agvs[idx].connected) {
            StaticJsonDocument<64> cmd;
            cmd["type"] = "command";
            cmd["command"] = command;
            String cmdStr;
            serializeJson(cmd, cmdStr);

            ws.text(agvs[idx].clientId, cmdStr);
            Serial.printf("Komut gonderildi %s: %s\n", agvId.c_str(), command.c_str());

            // Log gonder
            String logMsg = "Komut: " + command;
            broadcastLog(agvId, logMsg);
        }
    }
    else if (type == "applyCalibration") {
        // PC'den gelen preset uygulama komutu. sensorMin + sensorMax dizilerini
        // hedef AGV'ye aynen yonlendir; AGV runtime'da calibData'sini gunceller.
        String agvId = doc["agvId"].as<String>();
        int idx = findAGVById(agvId);
        if (idx >= 0 && agvs[idx].connected) {
            // Orijinal payload'i parametreyi tekrar serialize etmeden dogrudan yonlendir
            ws.text(agvs[idx].clientId, message);
            Serial.printf("applyCalibration gonderildi %s\n", agvId.c_str());
            broadcastLog(agvId, "Kalibrasyon preset'i uygulandi");
        }
    }
    // ===== Multi-AGV Planner mesajlari (PC -> AGV) =====
    else if (type == "setHop") {
        // PC planner'dan gelen 3-hop look-ahead emir.
        // payload: {type:"setHop", agvId, from, next, after?, after2?, goal?}
        // AGV from'dan next'e cizgi takibiyle gider, kavsakta after/after2 icin
        // yon doner. goal: mission nihai hedefi — "Hedefe ulasildi" kontrolu buna.
        // message AYNEN forward edilir (after2 dahil her alan otomatik gecer).
        String agvId = doc["agvId"].as<String>();
        int idx = findAGVById(agvId);
        if (idx >= 0 && agvs[idx].connected) {
            ws.text(agvs[idx].clientId, message);
            const char* fromS   = doc["from"]   | "?";
            const char* nextS   = doc["next"]   | "?";
            const char* afterS  = doc["after"]  | "-";
            const char* after2S = doc["after2"] | "-";
            const char* goalS   = doc["goal"]   | "-";
            Serial.printf("setHop %s: %s>%s>%s>%s goal=%s\n",
                          agvId.c_str(), fromS, nextS, afterS, after2S, goalS);
        }
    }
    else if (type == "clearMission") {
        // Mission iptal: AGV duracak ve plana donmeyecek.
        String agvId = doc["agvId"].as<String>();
        int idx = findAGVById(agvId);
        if (idx >= 0 && agvs[idx].connected) {
            ws.text(agvs[idx].clientId, message);
            Serial.printf("clearMission gonderildi %s\n", agvId.c_str());
            broadcastLog(agvId, "Mission iptal edildi");
        }
    }
    else if (type == "hopComplete") {
        // AGV bir hop'u tamamladi, yeni node'da. Planner senkron olmali.
        // payload: {type:"hopComplete", agvId, node, heading?}
        // calibrationData pattern'i: tum clientlere broadcast.
        ws.textAll(message);
        Serial.printf("[HOP] %s -> %s\n",
                      doc["agvId"].as<String>().c_str(),
                      doc["node"].as<String>().c_str());
    }
    else if (type == "faceDir") {
        // PC'den kup yonune donus emri: hedef AGV'ye aynen yonlendir.
        // payload: {type:"faceDir", agvId, dir:"N"|"E"|"S"|"W"}
        String agvId = doc["agvId"].as<String>();
        int idx = findAGVById(agvId);
        if (idx >= 0 && agvs[idx].connected) {
            ws.text(agvs[idx].clientId, message);
            Serial.printf("faceDir %s: %s\n", agvId.c_str(),
                          doc["dir"].as<String>().c_str());
        }
    }
    else if (type == "faceComplete") {
        // AGV kup yonune donusu bitirdi. hopComplete pattern'i: broadcast.
        // payload: {type:"faceComplete", agvId, node, heading?}
        ws.textAll(message);
        Serial.printf("[FACE] %s @ %s -> %s\n",
                      doc["agvId"].as<String>().c_str(),
                      doc["node"].as<String>().c_str(),
                      doc["heading"].as<String>().c_str());
    }
    else if (type == "getList") {
        // Dashboard AGV listesi istiyor
        broadcastAGVList();
    }
}

// ===== Tek AGV Guncelleme =====
void sendAGVUpdate(int idx) {
    if (idx < 0 || agvs[idx].id == "") return;

    StaticJsonDocument<640> doc;
    doc["type"]             = "agvUpdate";
    doc["id"]               = agvs[idx].id;
    doc["connected"]        = agvs[idx].connected;
    doc["currentWaypoint"]  = agvs[idx].currentWaypoint;
    doc["targetWaypoint"]   = agvs[idx].targetWaypoint;
    doc["path"]             = agvs[idx].path;
    doc["pathIdx"]          = agvs[idx].pathIdx;
    doc["heading"]          = agvs[idx].heading;
    doc["navState"]         = agvs[idx].navState;
    doc["linePos"]          = agvs[idx].linePosition;
    doc["Kp"]               = agvs[idx].Kp;
    doc["Ki"]               = agvs[idx].Ki;
    doc["Kd"]               = agvs[idx].Kd;
    doc["baseSpeed"]        = agvs[idx].baseSpeed;
    doc["calibrated"]       = agvs[idx].isCalibrated;
    doc["isTarget"]         = agvs[idx].isTarget;
    doc["sonarDist"]        = agvs[idx].sonarDist;
    doc["obstacle"]         = agvs[idx].obstacle;
    doc["sonarSlow"]        = agvs[idx].sonarSlow;

    JsonArray sensors = doc.createNestedArray("sensors");
    for (int j = 0; j < 8; j++) sensors.add(agvs[idx].sensorValues[j]);

    String output;
    serializeJson(doc, output);
    ws.textAll(output);
}

// ===== Log Mesaji Broadcast =====
void broadcastLog(String agvId, String message) {
    StaticJsonDocument<256> doc;
    doc["type"] = "log";
    doc["agvId"] = agvId;
    doc["message"] = message;
    doc["time"] = millis();

    String output;
    serializeJson(doc, output);
    ws.textAll(output);

    Serial.print("[LOG] ");
    Serial.print(agvId);
    Serial.print(": ");
    Serial.println(message);
}

// ===== AGV Listesini Broadcast Et =====
void broadcastAGVList() {
    // 5 AGV × ~380 byte = ~1900 byte serialize + JSON overhead doc memory
    // 2048 sınırda kalıyordu ve sessiz truncate riskı var. 4096'ya çıkardık.
    StaticJsonDocument<4096> doc;
    doc["type"] = "agvList";

    JsonArray list = doc.createNestedArray("agvs");

    for (int i = 0; i < MAX_AGVS; i++) {
        if (agvs[i].id != "") {
            JsonObject agv = list.createNestedObject();
            agv["id"]              = agvs[i].id;
            agv["connected"]       = agvs[i].connected;
            agv["currentWaypoint"] = agvs[i].currentWaypoint;
            agv["targetWaypoint"]  = agvs[i].targetWaypoint;
            agv["path"]            = agvs[i].path;
            agv["pathIdx"]         = agvs[i].pathIdx;
            agv["heading"]         = agvs[i].heading;
            agv["navState"]        = agvs[i].navState;
            agv["linePos"]         = agvs[i].linePosition;
            agv["Kp"]              = agvs[i].Kp;
            agv["Ki"]              = agvs[i].Ki;
            agv["Kd"]              = agvs[i].Kd;
            agv["baseSpeed"]       = agvs[i].baseSpeed;
            agv["calibrated"]      = agvs[i].isCalibrated;
            agv["isTarget"]        = agvs[i].isTarget;
            agv["sonarDist"]       = agvs[i].sonarDist;
            agv["obstacle"]        = agvs[i].obstacle;
            agv["sonarSlow"]       = agvs[i].sonarSlow;

            JsonArray sensors = agv.createNestedArray("sensors");
            for (int j = 0; j < 8; j++) sensors.add(agvs[i].sensorValues[j]);
        }
    }

    String output;
    serializeJson(doc, output);
    ws.textAll(output);
}

// ===== Timeout Kontrolu =====
void checkTimeouts() {
    unsigned long now = millis();

    for (int i = 0; i < MAX_AGVS; i++) {
        if (agvs[i].connected && (now - agvs[i].lastPing > PING_TIMEOUT)) {
            Serial.printf("AGV %s timeout - baglanti kesildi\n", agvs[i].id.c_str());
            agvs[i].connected = false;
            broadcastAGVList();
        }
    }
}

// ===== Yardimci Fonksiyonlar =====
int findAGVByClientId(uint32_t clientId) {
    for (int i = 0; i < MAX_AGVS; i++) {
        if (agvs[i].clientId == clientId && agvs[i].connected) {
            return i;
        }
    }
    return -1;
}

int findAGVById(String id) {
    for (int i = 0; i < MAX_AGVS; i++) {
        if (agvs[i].id == id) {
            return i;
        }
    }
    return -1;
}

int findEmptySlot() {
    for (int i = 0; i < MAX_AGVS; i++) {
        if (agvs[i].id == "") {
            return i;
        }
    }
    return -1;
}

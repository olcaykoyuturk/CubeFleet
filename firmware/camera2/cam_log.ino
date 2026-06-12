#include "types.h"
#include <stdarg.h>

// =============================================================================
// cam_log.ino - Anlamli olaylari hem Serial'e hem ring buffer'a yazar.
//
// Buffer JSON formatinda /log?since=N endpoint'i ile PC tarafindan cekilir.
// Thread-safe (FreeRTOS mutex) - hem ana loop hem stream task'tan cagrilabilir.
//
// Sadece YUKSEK SEVIYE olaylar loglanir:
//   - boot/init durumlari
//   - servo hedefe ulasti
//   - magnet on/off
//   - stream client baglandi/koptu
//   - hata mesajlari
// FPS, frame sayilari, debug verileri loglanmaz.
// =============================================================================

struct CamLogEntry {
    uint32_t seq;
    uint32_t ms;
    char     msg[CAM_LOG_MSGLEN];
};

static CamLogEntry        logRing[CAM_LOG_RING];
static volatile uint32_t  logSeqCounter = 0;
static volatile int       logHead       = 0;     // next write slot
static SemaphoreHandle_t  logMutex      = NULL;

void camLogInit() {
    logMutex = xSemaphoreCreateMutex();
    for (int i = 0; i < CAM_LOG_RING; i++) {
        logRing[i].seq = 0;
        logRing[i].ms  = 0;
        logRing[i].msg[0] = '\0';
    }
}

void camLog(const char* fmt, ...) {
    char msg[CAM_LOG_MSGLEN];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    // Ring buffer (thread-safe). Serial init edilmiyor — PC /poll endpoint
    // ile log entry'leri ceker, ekran disi kalan mesaj olmaz.
    if (logMutex) xSemaphoreTake(logMutex, portMAX_DELAY);
    uint32_t seq = ++logSeqCounter;
    logRing[logHead].seq = seq;
    logRing[logHead].ms  = millis();
    strncpy(logRing[logHead].msg, msg, CAM_LOG_MSGLEN - 1);
    logRing[logHead].msg[CAM_LOG_MSGLEN - 1] = '\0';
    logHead = (logHead + 1) % CAM_LOG_RING;
    if (logMutex) xSemaphoreGive(logMutex);
}

uint32_t camLogCurrentSeq() {
    return logSeqCounter;
}

// since'tan buyuk seq'li girdileri JSON olarak doldurur.
// Format: {"id":"CAM_1","seq":42,"entries":[{"seq":40,"ms":12345,"msg":"..."},...]}
// out'un en az ~2KB kapasitesi olmali (32 entry * ~60 byte).
void camLogToJson(uint32_t since, String& out) {
    out  = "{\"id\":\"";
    out += CAM_ID;
    out += "\",\"seq\":";

    if (logMutex) xSemaphoreTake(logMutex, portMAX_DELAY);
    uint32_t curSeq = logSeqCounter;
    out += (uint32_t)curSeq;
    out += ",\"entries\":[";

    bool first = true;
    for (int i = 0; i < CAM_LOG_RING; i++) {
        if (logRing[i].seq == 0 || logRing[i].seq <= since) continue;
        if (!first) out += ',';
        first = false;
        out += "{\"seq\":";
        out += (uint32_t)logRing[i].seq;
        out += ",\"ms\":";
        out += (uint32_t)logRing[i].ms;
        out += ",\"msg\":\"";
        // JSON string escape (sadece minimum)
        for (const char* p = logRing[i].msg; *p; p++) {
            char c = *p;
            if (c == '"' || c == '\\') { out += '\\'; out += c; }
            else if (c == '\n')        { out += "\\n"; }
            else if (c == '\r')        { out += "\\r"; }
            else if ((unsigned char)c < 0x20) { /* skip */ }
            else                       { out += c; }
        }
        out += "\"}";
    }
    if (logMutex) xSemaphoreGive(logMutex);

    out += "]}";
}

// ===== Debug Durum Değişkenleri =====
// Serial debug çıktısı kaldırıldı — bu modül sadece global dbg* değişkenlerini tutar.
// Bu değişkenler pid.ino tarafından yazılır, oled.ino tarafından okunur.

#include "types.h"

int dbgLeftSpeed  = 0;
int dbgRightSpeed = 0;
int dbgError      = 0;
int dbgCorrection = 0;

void debugInit() {
    dbgLeftSpeed = dbgRightSpeed = dbgError = dbgCorrection = 0;
}

void printDebugInfo() {
    // Serial çıktısı kaldırıldı — no-op
}

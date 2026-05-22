// PID + OLED tarafindan paylasilan runtime debug degiskenleri.
// pid.ino yazar, oled.ino okur.

#include "types.h"

int dbgLeftSpeed  = 0;
int dbgRightSpeed = 0;
int dbgError      = 0;
int dbgCorrection = 0;

void debugInit() {
    dbgLeftSpeed = dbgRightSpeed = dbgError = dbgCorrection = 0;
}

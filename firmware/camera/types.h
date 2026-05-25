#pragma once

#include <WiFi.h>
#include <Arduino.h>
#include "esp_camera.h"
#include "pin.h"
#include "config.h"

// =============================================================================
// types.h - Paylasilan prototipler
// =============================================================================

// --- cam_log.ino ---
// Anlamli olaylari hem Serial'e hem ring buffer'a yazar (PC HTTP polling ile ceker)
void     camLogInit();
void     camLog(const char* fmt, ...);
uint32_t camLogCurrentSeq();
void     camLogToJson(uint32_t since, String& out);

// --- camera.ino ---
bool         cameraInit();
camera_fb_t* cameraCapture();
void         cameraReturn(camera_fb_t* fb);
bool         cameraIsHardwareJpeg();   // true = sensor JPEG cikariyor, frame2jpg gerekmez
void         setFlashLedBrightness(uint8_t v);   // 0..255 PWM
uint8_t      getFlashLedBrightness();

// --- wifi_setup.ino ---
bool wifiInit();
void wifiPrintInfo();

// --- mjpeg_server.ino ---
void mjpegServerStart();
void mjpegServerLoop();
unsigned long mjpegFrameCount();   // toplam yayinlanan frame
int           mjpegClientCount();  // anlik bagli istemci sayisi

// --- arm.ino ---
// Servo ID'leri: 0=base, 1=shoulder, 2=elbow, 3=gripper
void armInit();
void armSetAngle(int servoId, int angle);   // hedef aci yaz (per-servo MIN/MAX'a kistirilir)
void armUpdate();                           // loop'ta cagir, ramping adimi
void armFreeze();                           // acil dur - tum target = current
void armGoHome();                           // sequential home (once shoulder)
int  armGetAngle(int servoId);              // mevcut (gercek) aci
int  armGetTarget(int servoId);             // hedef aci
void armMagnetOn();
void armMagnetOff();
bool armGetMagnetState();
bool armGetGripSensor();                    // STABLE durum (debounce sonrasi)
const char* armGripEventType();             // "held" | "lost" | "none"
uint32_t    armGripEventSeq();              // monotonic; PC last_seq ile karsilastirir
int  armGripRawGpio();                      // diagnostic: ham GPIO degeri (0/1)
bool armGripPending();                      // diagnostic: debounce timer aktif mi
void gripUpdate();                          // periyodik kontrol (loop'tan cagrilir)
void armSetStepInterval(int ms);            // 5..500, kucuk = hizli
int  armGetStepInterval();

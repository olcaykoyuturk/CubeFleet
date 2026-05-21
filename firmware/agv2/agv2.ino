// ===== ESP32 AGV Client =====
// Giriş noktası: setup ve loop.
// İş mantığı navigation.ino, websocket.ino ve diğer modüllerdedir.

#include <Arduino.h>
#include "pin.h"
#include "config.h"
#include "types.h"

// =============================================================================
// Sensör ve PID durumu — bu modülün sahibi main.ino
// =============================================================================
CalibrationData calibData;
PIDParams       pidParams;
int sensorValues[SENSOR_COUNT];
int sensorCalibrated[SENSOR_COUNT];
int linePosition = 0;
int baseSpeed    = DEFAULT_BASE_SPEED;

// =============================================================================
// setup / loop
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(LED_PIN, OUTPUT);

    // Modülleri başlat
    muxInit();
    motorInit();
    pidInit();
    calibrationInit();
    navigationInit();
    debugInit();

    // Yeni sensörler
    sonarInit();
    rfidInit();
    oledInit();

    // Bağlantıyı başlat
    wifiInit();
    webSocketInit();
}

void loop() {
    updateSonar();      // HC-SR04 periyodik ölçüm (SONAR_INTERVAL ms)
    webSocketLoop();    // Gelen komutları işle, durum gönder
    runNavigation();    // Navigasyon state machine'i çalıştır
    oledUpdate();       // OLED ekran güncelle (OLED_INTERVAL ms'de bir)
    printDebugInfo();   // Serial debug çıktısı (DEBUG_INTERVAL ms'de bir)
}

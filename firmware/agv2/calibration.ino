// ===== Kalibrasyon Modülü =====

#include "types.h"

void calibrationInit() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        calibData.minVal[i] = 4095;   // ESP32 ADC max
        calibData.maxVal[i] = 0;
    }
    calibData.isCalibrated = false;
}

void updateCalibrationValues() {
    readAllSensors();
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensorValues[i] < calibData.minVal[i]) calibData.minVal[i] = sensorValues[i];
        if (sensorValues[i] > calibData.maxVal[i]) calibData.maxVal[i] = sensorValues[i];
    }
}

// Elle hareket ettirerek kalibrasyon — robot durur, kullanıcı çizginin üstünde iter
void runManualCalibration() {
    calibrationInit();

    unsigned long start = millis();
    while (millis() - start < CALIBRATION_TIME) {
        webSocketLoop();          // STOP komutu kalibrasyon sırasında da çalışsın
        updateCalibrationValues();
        delay(10);
    }

    // Range doğrulaması + her sensörün range'ini tek log'da rapor et.
    // '!' işareti CALIB_MIN_RANGE eşiğinin altında olan sensörü gösterir
    // (donanım sorunu varsa kullanıcı hangi kanal olduğunu görür).
    bool valid = true;
    char rangeBuf[140];
    int  rangeLen = snprintf(rangeBuf, sizeof(rangeBuf), "Sensor ranges:");
    for (int i = 0; i < SENSOR_COUNT; i++) {
        int range = calibData.maxVal[i] - calibData.minVal[i];
        if (range < CALIB_MIN_RANGE) {
            valid = false;
        }
        rangeLen += snprintf(rangeBuf + rangeLen, sizeof(rangeBuf) - rangeLen,
                             " S%d=%d%s", i, range,
                             range < CALIB_MIN_RANGE ? "!" : "");
    }
    sendLog(rangeBuf);
    wsFlush();   // ranges log'unun TCP'ye yazilmasini bekle

    if (valid) {
        calibData.isCalibrated = true;
        sendLog("Kalibrasyon tamamlandi.");
        wsFlush();   // tamamlandi log'u once gitsin, sonra JSON
        sendCalibrationData();   // KRITIK: PC countdown bunu bekliyor
        wsFlush(5);              // JSON ~250 byte, daha uzun yield (3 yerine 5)
    } else {
        calibData.isCalibrated = false;
        sendLog("Kalibrasyon basarisiz: Bazi sensorler beyaz/siyah yuzey goremedi. Yeniden kalibre edin.");
        wsFlush();
    }
}

void printCalibrationData() {
    // Debug çıktısı kaldırıldı
}

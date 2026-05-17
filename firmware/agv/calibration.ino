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

    // Range doğrulaması — her sensör yeterli kontrast gördü mü?
    bool valid = true;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        int range = calibData.maxVal[i] - calibData.minVal[i];
        if (range < CALIB_MIN_RANGE) {
            valid = false;
        }
    }

    if (valid) {
        calibData.isCalibrated = true;
        sendLog("Kalibrasyon tamamlandi.");
    } else {
        calibData.isCalibrated = false;
        sendLog("Kalibrasyon basarisiz: Bazi sensorler beyaz/siyah yuzey goremedi. Yeniden kalibre edin.");
    }
}

void printCalibrationData() {
    // Debug çıktısı kaldırıldı
}

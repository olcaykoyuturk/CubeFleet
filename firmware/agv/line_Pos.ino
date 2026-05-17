// ===== Çizgi Pozisyon Hesaplama =====

#include "types.h"

// Ağırlıklı ortalama ile çizgi pozisyonu.
// Çıktı: -3500 (en sol) … 0 (merkez) … +3500 (en sağ)
int calculateLinePosition() {
    readCalibratedSensors();

    long weightedSum = 0;
    long sum         = 0;

    for (int i = 0; i < SENSOR_COUNT; i++) {
        // Sensör ağırlıkları: -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
        int weight = (i - 3) * 1000 - 500;
        weightedSum += (long)sensorCalibrated[i] * weight;
        sum         += sensorCalibrated[i];
    }

    if (sum > 0)
        linePosition = (int)(weightedSum / sum);
    // sum == 0 ise çizgi kayıp — son pozisyonu koru

    return linePosition;
}

bool isLineDetected() {
    for (int i = 0; i < SENSOR_COUNT; i++)
        if (sensorCalibrated[i] > LINE_THRESHOLD) return true;
    return false;
}

// JUNCTION_MIN_SENSORS veya daha fazla sensör aktifse kavşak
bool isJunction() {
    int count = 0;
    for (int i = 0; i < SENSOR_COUNT; i++)
        if (sensorCalibrated[i] > LINE_THRESHOLD) count++;
    return count >= JUNCTION_MIN_SENSORS;
}

bool isLineLost() {
    return !isLineDetected();
}

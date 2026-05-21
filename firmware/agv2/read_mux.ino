// ===== MUX Okuma Modülü (74HC4051 + QTR-8A) =====
// tests/qtr8a/qtr8a.ino ile birebir aynı okuma davranışı:
//   - MUX_SETTLE_US = 200 µs (op-amp + 1k/2k bölücü RC için yeterli)
//   - SAMPLE_COUNT = 8 (gürültü düşüren ortalama)
//   - ADC explicit 12-bit + 11dB attenuation

#include "types.h"

#define MUX_SETTLE_US   200
#define MUX_SAMPLES       8

void muxInit() {
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_SIG, INPUT);

    // Baslangicta S pinlerini LOW yap
    digitalWrite(MUX_S0, LOW);
    digitalWrite(MUX_S1, LOW);
    digitalWrite(MUX_S2, LOW);

    // ADC — test sketch'iyle birebir aynı ayar
    analogReadResolution(12);            // 0-4095
    analogSetAttenuation(ADC_11db);      // 0-3.3V tam aralık
}

// MUX kanal secimi (0-7)
void selectMuxChannel(uint8_t channel) {
    digitalWrite(MUX_S0, (channel & 0x01));
    digitalWrite(MUX_S1, (channel >> 1) & 0x01);
    digitalWrite(MUX_S2, (channel >> 2) & 0x01);
}

// Tek bir kanaldan okuma — kanal seçtikten sonra settle, sonra 8 örnek ortalaması
int readMuxChannel(uint8_t channel) {
    selectMuxChannel(channel);
    delayMicroseconds(MUX_SETTLE_US);    // op-amp + RC oturma süresi
    int sum = 0;
    for (int j = 0; j < MUX_SAMPLES; j++) sum += analogRead(MUX_SIG);
    return sum / MUX_SAMPLES;
}

// Tum sensorleri oku ve sensorValues dizisine kaydet
void readAllSensors() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensorValues[i] = readMuxChannel(i);
    }
}

// Kalibrasyon verilerine gore normalize edilmis degerler (0-1000)
void readCalibratedSensors() {
    readAllSensors();

    for (int i = 0; i < SENSOR_COUNT; i++) {
        int range = calibData.maxVal[i] - calibData.minVal[i];

        if (range > 0) {
            int value = sensorValues[i] - calibData.minVal[i];
            value = (value * 1000) / range;
            sensorCalibrated[i] = constrain(value, 0, 1000);
        } else {
            sensorCalibrated[i] = 0;
        }
    }
}

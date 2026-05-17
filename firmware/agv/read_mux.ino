// ===== MUX Okuma Modülü (74HC4051 + QTR-8A) =====

#include "types.h"

void muxInit() {
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_SIG, INPUT);

    // Baslangicta S pinlerini LOW yap
    digitalWrite(MUX_S0, LOW);
    digitalWrite(MUX_S1, LOW);
    digitalWrite(MUX_S2, LOW);
}

// MUX kanal secimi (0-7)
void selectMuxChannel(uint8_t channel) {
    digitalWrite(MUX_S0, (channel & 0x01));
    digitalWrite(MUX_S1, (channel >> 1) & 0x01);
    digitalWrite(MUX_S2, (channel >> 2) & 0x01);
}

// Tek bir kanaldan okuma yap
int readMuxChannel(uint8_t channel) {
    selectMuxChannel(channel);
    delayMicroseconds(100);  // 74HC4051 kanal gecisi + analog giris RC oturma suresi
    return analogRead(MUX_SIG);
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

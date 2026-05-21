// ===== Motor Kontrol Modülü =====
// ESP32 core 3.x yeni LEDC API kullanir: kanali otomatik secer, pin uzerinden yazar.

#include "types.h"

void motorInit() {
    // Yeni LEDC API: ledcAttach(pin, freq, resolution) kanalı otomatik secer
    ledcAttach(MOTOR_L1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_L2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_R1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_R2, PWM_FREQ, PWM_RESOLUTION);

    motorStop();
}

// Sol motor kontrolu (-255 ile +255 arasi)
void setLeftMotor(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed >= 0) {
        ledcWrite(MOTOR_L1, speed);
        ledcWrite(MOTOR_L2, 0);
    } else {
        ledcWrite(MOTOR_L1, 0);
        ledcWrite(MOTOR_L2, -speed);
    }
}

// Sag motor kontrolu (-255 ile +255 arasi)
void setRightMotor(int speed) {
    speed = constrain(speed, -255, 255);

    if (speed >= 0) {
        ledcWrite(MOTOR_R1, speed);
        ledcWrite(MOTOR_R2, 0);
    } else {
        ledcWrite(MOTOR_R1, 0);
        ledcWrite(MOTOR_R2, -speed);
    }
}

// Her iki motoru ayri ayri kontrol et
void setMotors(int leftSpeed, int rightSpeed) {
    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
}

// Motorlari durdur
void motorStop() {
    ledcWrite(MOTOR_L1, 0);
    ledcWrite(MOTOR_L2, 0);
    ledcWrite(MOTOR_R1, 0);
    ledcWrite(MOTOR_R2, 0);
}

// Ileri git (MOTOR_TRIM ile sol/sag denge ayarı)
void motorForward(int speed) {
    setMotors(speed - MOTOR_TRIM, speed + MOTOR_TRIM);
}


// Sola don (yerinde) - yonler ters cevrildi
void motorTurnLeft(int speed) {
    setMotors(speed, -speed);  // Sol ileri, Sag geri
}

// Saga don (yerinde) - yonler ters cevrildi
void motorTurnRight(int speed) {
    setMotors(-speed, speed);  // Sol geri, Sag ileri
}

// ===== PID Kontrol Modülü =====

#include "types.h"

void pidInit() {
    pidParams.Kp       = PID_DEFAULT_KP;
    pidParams.Ki       = PID_DEFAULT_KI;
    pidParams.Kd       = PID_DEFAULT_KD;
    pidParams.lastError = 0;
    pidParams.integral  = 0;
    pidParams.lastTime  = millis();
}

void setPIDParams(float kp, float ki, float kd) {
    pidParams.Kp = kp;
    pidParams.Ki = ki;
    pidParams.Kd = kd;
}

void resetPID() {
    pidParams.lastError = 0;
    pidParams.integral  = 0;
    pidParams.lastTime  = millis();
}

int calculatePID(int error) {
    unsigned long now = millis();

    // Δt hesabı (saniye) — uzun aralık için varsayılan 20ms
    float dt = (now - pidParams.lastTime) / 1000.0f;
    if (dt > 0.5f || dt <= 0.0f) dt = 0.02f;
    pidParams.lastTime = now;

    float P = pidParams.Kp * error;

    pidParams.integral += error * dt;
    pidParams.integral  = constrain(pidParams.integral, -5000.0f, 5000.0f);
    float I = pidParams.Ki * pidParams.integral;

    float D = (dt > 0) ? pidParams.Kd * (error - pidParams.lastError) / dt : 0;
    D = constrain(D, -50.0f, 50.0f);   // ani hata sıçramalarında D patlamasını önle
    pidParams.lastError = error;

    return (int)(P + I + D);
}

void lineFollowPID() {
    int error      = calculateLinePosition();
    int correction = calculatePID(error);

    // Sonar SLOW zone — engele yumusak inis. STOP zone navigation.ino'da
    // motorStop() ile zaten devre disi (bu fonksiyon cagrilmaz). Slow'da
    // baseSpeed'i scale ederiz, PID korelasyonu da olceklenir.
    int effectiveBase = baseSpeed;
    if (isObstacleSlow()) {
        effectiveBase = (baseSpeed * OBSTACLE_SLOW_PCT) / 100;
    }

    correction = constrain(correction, -effectiveBase, effectiveBase);

    int leftSpeed  = constrain(effectiveBase - correction, 0, 255);
    int rightSpeed = constrain(effectiveBase + correction, 0, 255);

    // Debug modülü için güncelle
    dbgError      = error;
    dbgCorrection = correction;
    dbgLeftSpeed  = leftSpeed;
    dbgRightSpeed = rightSpeed;

    setMotors(leftSpeed, rightSpeed);
}

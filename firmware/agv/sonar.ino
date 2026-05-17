// ===== HC-SR04 Ultrasonik Mesafe Sensörü (interrupt-based) =====
// AGV'de WiFi + FreeRTOS task switching pulseIn/pulseInLong timing'ini bozuyor.
// Bu yüzden echo'yu ISR ile yakalıyoruz: edge'lerde anında micros() alınır.
// Bu yöntem WiFi/motor/MUX trafiğinden bağımsız çalışır.

#include "types.h"

float sonarDistance = 999.0f;   // cm — başlangıçta engel yok
bool  sonarObstacle = false;

static unsigned long lastSonarTime = 0;

// ISR ile paylaşılan değişkenler
static volatile unsigned long echoStartUs  = 0;
static volatile unsigned long echoEndUs    = 0;
static volatile bool          echoComplete = false;
static volatile bool          waitingEcho  = false;

// Echo pini RISING/FALLING'de tetiklenir, micros() yakalar.
// IRAM_ATTR: ISR yorumlanan kodda değil, IRAM'da çalışsın (hızlı/güvenli).
static void IRAM_ATTR onEchoChange() {
    if (!waitingEcho) return;
    if (digitalRead(SONAR_ECHO) == HIGH) {
        echoStartUs = micros();
    } else {
        echoEndUs    = micros();
        echoComplete = true;
        waitingEcho  = false;
    }
}

void sonarInit() {
    pinMode(SONAR_TRIG, OUTPUT);
    pinMode(SONAR_ECHO, INPUT);
    digitalWrite(SONAR_TRIG, LOW);
    // Echo pin'i her iki yönde dinle (RISING + FALLING)
    attachInterrupt(digitalPinToInterrupt(SONAR_ECHO), onEchoChange, CHANGE);
}

// SONAR_INTERVAL ms aralıklarla ölçüm alır.
// Engel tespiti DEBOUNCE'lu: tek noise spike AGV'yi durdurmaz.
void updateSonar() {
    if (millis() - lastSonarTime < SONAR_INTERVAL) return;
    lastSonarTime = millis();

    // ISR'i sıfırla ve "echo bekleme" modunu aç
    noInterrupts();
    echoComplete = false;
    waitingEcho  = true;
    echoStartUs  = 0;
    echoEndUs    = 0;
    interrupts();

    // Trig: 10µs HIGH
    digitalWrite(SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG, LOW);

    // ISR'in echoComplete=true yapmasını bekle (max 30ms)
    unsigned long startWait = millis();
    while (!echoComplete && millis() - startWait < 30) {
        delay(1);
    }
    waitingEcho = false;   // beklemiyoruz artık

    // Ham okuma
    float rawDistance;
    if (echoComplete && echoEndUs > echoStartUs) {
        unsigned long duration = echoEndUs - echoStartUs;
        if (duration < 25000) rawDistance = duration / 58.0f;
        else                  rawDistance = 999.0f;
    } else {
        rawDistance = 999.0f;
    }

    // 3-nokta median filtre — tek spike'ları eler, gerçek değişimi korur.
    // Örnek: [20, 17, 21] → ortanca = 20 (17 spike yutulur)
    //        [22, 22, 23] → ortanca = 22 (gerçek mesafe değişimi geçer)
    static float hist[3] = {999.0f, 999.0f, 999.0f};
    static int   histIdx = 0;
    hist[histIdx] = rawDistance;
    histIdx = (histIdx + 1) % 3;

    float a = hist[0], b = hist[1], c = hist[2];
    sonarDistance = max(min(a, b), min(max(a, b), c));   // medyan

    // Debounce: 2 ardışık yakın → engel; 2 ardışık uzak → temiz.
    // Tek noise spike AGV'yi durdurmaz; gerçek engel ~200 ms içinde tespit.
    static int closeCount = 0;
    static int farCount   = 0;
    const int  DEBOUNCE_THRESHOLD = 2;
    const int  DEBOUNCE_MAX       = 4;   // overflow korumasi

    bool close = (sonarDistance > 0 && sonarDistance < OBSTACLE_DISTANCE_CM);
    if (close) {
        farCount = 0;
        if (closeCount < DEBOUNCE_MAX) closeCount++;
        if (closeCount >= DEBOUNCE_THRESHOLD) sonarObstacle = true;
    } else {
        closeCount = 0;
        if (farCount < DEBOUNCE_MAX) farCount++;
        if (farCount >= DEBOUNCE_THRESHOLD) sonarObstacle = false;
    }
}

bool isObstacleDetected() {
    return sonarObstacle;
}

float getSonarDistance() {
    return sonarDistance;
}

// =============================================================================
// arm.ino - Robot Kol Kontrolu (4 servo + elektromiknatis)
//
// ESP32 core 3.x dahili LEDC API'sini DOGRUDAN kullanir.
// ESP32Servo kutuphanesi core 3.x ile background interrupt crash'i yaratiyor;
// bu yuzden kullanmiyoruz.
//
// Per-servo PWM ayarlari (SERVO_FREQ_HZ + SERVO_PULSE_MIN/MAX tablolarinda):
//   - Base / Elbow / Gripper: 50 Hz @ 500-2500 µs  (MG996R, MG90S)
//   - Shoulder DS3225:        333 Hz @ 1000-2000 µs (datasheet, 1520 µs neutral)
//   - PWM cozunurluk: 14-bit (16-bit'te 50 Hz APB clock'tan tam bolunmedigi
//     icin jitter dogar — 14-bit dijital servo deadband icinde tolere edilebilir)
//   - ledcAttach farkli frekansli pinleri otomatik ayri timer'a atar
//
// armUpdate() her loop'ta ramping adimi (1°/servoStepIntervalMs).
// armGoHome() sequential: ONCE shoulder, sonra base/elbow/gripper (carpisma onleme).
// Boot soft-start: ilk PWM HOME'a yakin "off-target" konuma, ardindan ramped HOME.
// =============================================================================

#include "types.h"
#include "driver/gpio.h"   // gpio_reset_pin — UART matrisinden GPIO 3'u ayir

static int  curAngle[4]    = {90, 90, 90, 90};   // gercek (fiziksel) konum
static int  targetAngle[4] = {90, 90, 90, 90};   // hedef konum (slider'dan gelir)
static bool servoReached[4] = {true, true, true, true};  // hedefe ulasti mi (tek seferlik log)
static unsigned long lastServoStep = 0;
static int  servoStepIntervalMs = SERVO_STEP_INTERVAL_MS;   // runtime'da degistirilebilir
static bool magnetState = false;
static unsigned long magnetOnSince = 0;   // millis() — termal timeout sayaci
static const char* SERVO_NAME[4] = {"Base", "Shoulder", "Elbow", "Gripper"};

static const int SERVO_PINS[4] = {
    SERVO_BASE_PIN, SERVO_SHOULDER_PIN, SERVO_ELBOW_PIN, SERVO_GRIPPER_PIN
};

// Mekanik kalibrasyon: her servo'nun ulasabilecegi gerçek aci araligi
static const int SERVO_MIN_DEG[4] = {
    SERVO_BASE_MIN, SERVO_SHOULDER_MIN, SERVO_ELBOW_MIN, SERVO_GRIPPER_MIN
};
static const int SERVO_MAX_DEG[4] = {
    SERVO_BASE_MAX, SERVO_SHOULDER_MAX, SERVO_ELBOW_MAX, SERVO_GRIPPER_MAX
};

// Per-servo ramp adim buyuklugu. Hepsi 1° -> akici/titremesiz hareket.
// Hiz: 1°/40ms = 25°/sn (config.h SERVO_STEP_INTERVAL_MS).
static const int SERVO_STEP_DEG_PER[4] = {
    1,   // 0 Base
    1,   // 1 Shoulder
    1,   // 2 Elbow
    1    // 3 Gripper
};

// Sequential HOME state machine - shoulder ONCE, sonra digerleri.
// Bu sirayla gitmezsek shoulder yukari kalkmadan elbow/base/gripper acilir,
// kol govdeye veya kabloya carpabilir.
static int homeStage = -1;   // -1 = inactive, 0 = shoulder, 1 = digerleri

// LEDC PWM cozunurlugu (14-bit). 50 Hz icin ESP32 APB clock'tan tam
// bolunmuyor ama 14-bit jitter'i tolere edilebilir seviyeye iner.
#define SERVO_PWM_BITS  14
#define SERVO_PWM_MAX   ((1U << SERVO_PWM_BITS) - 1)

// Per-servo PWM frekansi + pulse araligi.
// DS3225 (shoulder) datasheet: 1520 µs neutral / 333 Hz refresh.
//   Standart 50 Hz veririsek "geç refresh" sinyali servo deadband icinde
//   düzeltme tetikler -> sürekli mikro-titreme.
// Digerleri klasik analog/dijital 50 Hz @ 500-2500 µs (MG90S, MG996R).
// ledcAttach farkli frekanstaki pinleri otomatik ayri timer'a atar.
static const int SERVO_FREQ_HZ[4] = {
    50,    // 0 Base     - MG996R
    333,   // 1 Shoulder - DS3225 (datasheet)
    50,    // 2 Elbow    - MG996R
    50,    // 3 Gripper  - MG90S
};

// Pulse range — 500-2500 µs genel, DS3225 simetrik 1000-2000 µs (neutral 1520)
static const int SERVO_PULSE_MIN_US[4] = { 500, 1000,  500,  500 };
static const int SERVO_PULSE_MAX_US[4] = {2500, 2000, 2500, 2500 };

// Aci (0-180) -> LEDC duty. Per-servo frekans+pulse araligini kullanir.
static uint32_t angleToDuty(int id, int angle) {
    angle = constrain(angle, 0, 180);
    int min_us = SERVO_PULSE_MIN_US[id];
    int max_us = SERVO_PULSE_MAX_US[id];
    long pulse_us  = min_us + ((long)angle * (max_us - min_us)) / 180;
    long period_us = 1000000L / SERVO_FREQ_HZ[id];   // 50 Hz=20000, 333 Hz=3003
    return (uint32_t)(((uint32_t)pulse_us * SERVO_PWM_MAX) / (uint32_t)period_us);
}

void armInit() {
    // GPIO 12 (Gripper) STRAPPING PIN — boot anında HIGH ise flash voltajı
    // 1.8V'a çekilir, ESP32 boot etmez. ledcAttach'tan ONCE input-pulldown
    // ile LOW garantile. Bu yazılım yine de boot ROM aşamasında etkili degil
    // (boot ROM pinMode bilmez), ama runtime'da hafif ek koruma. KESIN cözüm:
    // harici 10 kΩ pull-down GPIO 12 ↔ GND. (Daha guvenli pin: GPIO 13 olur
    // ama o sonar'a denk gelir.)
    pinMode(SERVO_GRIPPER_PIN, INPUT_PULLDOWN);
    delay(5);

    for (int i = 0; i < 4; i++) {
        if (!ledcAttach(SERVO_PINS[i], SERVO_FREQ_HZ[i], SERVO_PWM_BITS)) {
            camLog("HATA: %s (pin %d) ledcAttach basarisiz",
                   SERVO_NAME[i], SERVO_PINS[i]);
        }
    }

    // Boot soft-start: ilk PWM curAngle'a fırlar (servo fizigi, kacinilmaz).
    // Bu yüzden curAngle = BOOT pozu (kolun güç kesikken durdugu yer) -> ilk
    // snap MINIMUM olur, ardindan TÜM servolar HOME'a ayarlanan hizda (25°/sn)
    // YAVASCA rampalanir. Eskiden base/elbow/gripper dogrudan HOME'a yaziliyordu
    // (anlik firlatma); artik hepsi BOOT'tan yumusak rampalar.
    // BOOT degerlerini config.h'da kolun gerçek dinlenme pozuna ayarla.
    curAngle[0] = constrain(SERVO_BASE_BOOT, SERVO_BASE_MIN, SERVO_BASE_MAX);
    curAngle[1] = constrain(SERVO_SHOULDER_BOOT, SERVO_SHOULDER_MIN, SERVO_SHOULDER_MAX);
    curAngle[2] = constrain(SERVO_ELBOW_BOOT, SERVO_ELBOW_MIN, SERVO_ELBOW_MAX);
    curAngle[3] = constrain(SERVO_GRIPPER_BOOT, SERVO_GRIPPER_MIN, SERVO_GRIPPER_MAX);

    for (int i = 0; i < 4; i++) {
        targetAngle[i]  = curAngle[i];
        servoReached[i] = true;
        ledcWrite(SERVO_PINS[i], angleToDuty(i, curAngle[i]));
    }
    lastServoStep = millis();

    // Miknatis MOSFET (boot guvenli LOW)
    pinMode(MAGNET_PIN, OUTPUT);
    digitalWrite(MAGNET_PIN, LOW);
    magnetState = false;

    // Grip mikroswitch GPIO 3 (UART RX); gpio_reset_pin UART peripheral
    // baglantisini kopart — pinMode tek basina IO_MUX'u override etmiyordu.
    gpio_reset_pin((gpio_num_t)GRIP_SENSE_PIN);
    pinMode(GRIP_SENSE_PIN, INPUT_PULLUP);

    camLog("Soft-start: sh=%d el=%d -> Sequential HOME baslatildi",
           curAngle[1], curAngle[2]);

    // Sequential HOME — once shoulder yavaşça yukari (40°/25°/sn = 1.6 sn),
    // sonra base/elbow/gripper paralel olarak gerçek HOME'a.
    armGoHome();
}

// Hedef aci yaz - gercek hareket armUpdate() icinde adim adim
// angle, ilgili servo'nun mekanik araligina kistirilir (carpisma onleme)
void armSetAngle(int id, int angle) {
    id = constrain(id, 0, 3);
    angle = constrain(angle, SERVO_MIN_DEG[id], SERVO_MAX_DEG[id]);
    targetAngle[id] = angle;
}

// Acil dur: tum servolar mevcut pozisyonda dondurulur (target = current).
// Ramping durur, hareket bitter ama servolar power'li kalir (pozisyonu tutar).
void armFreeze() {
    for (int i = 0; i < 4; i++) {
        targetAngle[i]  = curAngle[i];
        servoReached[i] = true;
    }
    homeStage = -1;            // home sequence iptal
    camLog("ACIL DUR: kol donduruldu (%d/%d/%d/%d)",
           curAngle[0], curAngle[1], curAngle[2], curAngle[3]);
}

// Sequential HOME — 3 asama (carpisma onleme; kol AGV USTUNDE):
//   0) once ELBOW yukari (clearance) — alcaktayken shoulder donerse araca
//      sikisir, once forearm'i kaldir
//   1) sonra SHOULDER home'a otursun
//   2) sonra ELBOW asagi (final) + base/gripper
void armGoHome() {
    homeStage = 0;
    targetAngle[2]  = SERVO_ELBOW_CLEAR;
    servoReached[2] = (curAngle[2] == targetAngle[2]);
    camLog("HOME baslatildi: once elbow -> %d° (clearance)", SERVO_ELBOW_CLEAR);
}

// Her loop'ta cagrilir; SERVO_STEP_INTERVAL_MS gecmisse hedefe yaklas.
// HOME sequence aktifse: shoulder hedefe varinca digerlerini de target'a koy.
void armUpdate() {
    unsigned long now = millis();
    if (now - lastServoStep < (unsigned long)servoStepIntervalMs) return;
    lastServoStep = now;

    for (int i = 0; i < 4; i++) {
        if (curAngle[i] == targetAngle[i]) {
            if (!servoReached[i]) {
                servoReached[i] = true;
                camLog("%s: %d° tamamlandi", SERVO_NAME[i], curAngle[i]);
            }
            continue;
        }
        servoReached[i] = false;
        int diff = targetAngle[i] - curAngle[i];
        int step = SERVO_STEP_DEG_PER[i];
        if (abs(diff) <= step) {
            curAngle[i] = targetAngle[i];
        } else {
            curAngle[i] += (diff > 0) ? step : -step;
        }
        ledcWrite(SERVO_PINS[i], angleToDuty(i, curAngle[i]));
    }

    // ---- HOME sequence ilerletme (3 asama) ----
    if (homeStage == 0 && curAngle[2] == SERVO_ELBOW_CLEAR) {
        // elbow clearance'a ulasti -> shoulder home'a
        homeStage = 1;
        targetAngle[1] = SERVO_SHOULDER_HOME;
        servoReached[1] = (curAngle[1] == targetAngle[1]);
        camLog("Elbow clearance hazir -> Shoulder home");
    } else if (homeStage == 1 && curAngle[1] == SERVO_SHOULDER_HOME) {
        // shoulder oturdu -> elbow asagi (final) + base/gripper
        homeStage = 2;
        targetAngle[0] = SERVO_BASE_HOME;
        targetAngle[2] = SERVO_ELBOW_HOME;
        targetAngle[3] = SERVO_GRIPPER_HOME;
        servoReached[0] = (curAngle[0] == targetAngle[0]);
        servoReached[2] = (curAngle[2] == targetAngle[2]);
        servoReached[3] = (curAngle[3] == targetAngle[3]);
        camLog("Shoulder hazir -> Base/Elbow(asagi)/Gripper");
    } else if (homeStage == 2 &&
               curAngle[0] == SERVO_BASE_HOME &&
               curAngle[2] == SERVO_ELBOW_HOME &&
               curAngle[3] == SERVO_GRIPPER_HOME) {
        homeStage = -1;
        camLog("HOME tamamlandi");
    }
}

int armGetAngle(int id) {
    return curAngle[constrain(id, 0, 3)];
}

int armGetTarget(int id) {
    return targetAngle[constrain(id, 0, 3)];
}

// ---- Grip sensoru state machine ----
// Spam yerine kenarda log: kup yapisinca "TUTULDU", temas kaybinda "DUSTU".
// Press/release ayni 50ms debounce — sadece mekanik bounce filtresi.

static bool          gripStable        = false;
static bool          gripPendingState  = false;
static unsigned long gripPendingSince  = 0;
static const unsigned long GRIP_DEBOUNCE_MS = 50;

static volatile uint32_t gripEventSeq  = 0;
static const char*       gripEventType = "none";

bool armGetGripSensor()           { return gripStable; }
const char* armGripEventType()    { return gripEventType; }
uint32_t    armGripEventSeq()     { return gripEventSeq; }
// Diagnostic — /grip-debug endpoint icin ham GPIO + pending state
int  armGripRawGpio()             { return digitalRead(GRIP_SENSE_PIN); }
bool armGripPending()             { return gripPendingSince > 0; }

void gripUpdate() {
    bool raw = (digitalRead(GRIP_SENSE_PIN) == LOW) == GRIP_SENSE_ACTIVE_LOW;
    unsigned long now = millis();

    if (raw == gripStable) {
        gripPendingSince = 0;
        return;
    }
    if (gripPendingSince == 0 || raw != gripPendingState) {
        gripPendingState = raw;
        gripPendingSince = now;
        return;
    }

    if (now - gripPendingSince >= GRIP_DEBOUNCE_MS) {
        gripStable = raw;
        gripPendingSince = 0;
        gripEventType = raw ? "held" : "lost";
        gripEventSeq++;
        if (raw) camLog("GRIP: kup TUTULDU (seq=%u)", (unsigned)gripEventSeq);
        else     camLog("GRIP: kup DUSTU (seq=%u)", (unsigned)gripEventSeq);
    }
}

// ---- Magnet kontrol ----
void armMagnetOn() {
    if (!magnetState) camLog("Miknatis ACILDI");
    digitalWrite(MAGNET_PIN, HIGH);
    magnetState   = true;
    magnetOnSince = millis();   // termal timeout sayacini baslat
}

void armMagnetOff() {
    if (magnetState) camLog("Miknatis KAPATILDI");
    digitalWrite(MAGNET_PIN, LOW);
    magnetState = false;
}

bool armGetMagnetState() {
    return magnetState;
}

// Termal koruma: miknatis kup TUTMADAN MAGNET_MAX_ON_MS gecerse otomatik kapat.
// Kup tutuldugunda (grip 'held') sayac resetlenir → tasima boyunca acik kalir.
// loop()'tan her cagrilir; sadece millis karsilastirir, PWM/timer kullanmaz.
void armMagnetUpdate() {
    if (!magnetState) return;
    if (armGetGripSensor()) {        // kup tutuldu → sayaci ertele, acik kal
        magnetOnSince = millis();
        return;
    }
    if (millis() - magnetOnSince >= MAGNET_MAX_ON_MS) {
        camLog("Miknatis TIMEOUT (%lu ms, kup yok) — termal koruma kapatti",
               (unsigned long)MAGNET_MAX_ON_MS);
        armMagnetOff();
    }
}

// ---- Servo hizi (ramping interval) ----
// ms ne kadar kucukse o kadar hizli. step_deg=1 sabit oldugu icin dps = 1000/ms.
//   ms=10  -> 100°/sn (cok hizli, mekanik stres)
//   ms=20  ->  50°/sn (hizli)
//   ms=40  ->  25°/sn (varsayilan, akici)
//   ms=80  ->  12.5°/sn (yavas, hassas)
//   ms=200 ->   5°/sn (cok yavas)
void armSetStepInterval(int ms) {
    ms = constrain(ms, 5, 500);
    servoStepIntervalMs = ms;
    camLog("Servo hizi: %d ms/adim (~%d°/sn)", ms, 1000 / ms);
}

int armGetStepInterval() {
    return servoStepIntervalMs;
}

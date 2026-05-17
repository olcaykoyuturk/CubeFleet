// ===== Navigasyon Modülü =====
// Waypoint graf + BFS tabanlı navigasyon.
// Koordinat yerine isimli waypoint'ler kullanılır (A–I).
//
// Dış dünya yalnızca navCommand* fonksiyonlarını çağırır (facade).

#include "types.h"
#include "waypoint_map.h"

// =============================================================================
// Navigasyon durumu (paylaşılan global'ler)
// =============================================================================
char     currentWaypoint = 0;                 // Şu anki konum, 0 = bilinmiyor
char     targetWaypoint  = 0;                 // Hedef
char     navPath[MAX_PATH_LENGTH];            // BFS çıktısı
int      navPathLength   = 0;
int      navPathIndex    = 0;                 // Yolda kaçıncı waypoint'teyiz
Heading  heading         = NORTH;
NavState navState        = NAV_IDLE;
bool     isTarget        = false;

// =============================================================================
// İç runtime state — tek struct, merkezi reset
// =============================================================================
struct NavRuntime {
    unsigned long lineLostStart;    // çizgi kaybı zamanı (NAV_FOLLOWING)
    char          rfidDetectedWP;   // NAV_FOLLOWING → handleJunction köprüsü
    char          lastReadWP;       // son okunan kart, tekrar okumayı önler
    int           lastCorrection;   // kavşak öncesi son PID düzeltmesi
    bool          obstacleLogged;   // engel logu flood önleme
    unsigned long lastIdleRfidMs;   // NAV_IDLE'da RFID polling zaman damgası
};
static NavRuntime nav = {};

static void resetNavRuntime() {
    nav = {};
}

// =============================================================================
// Yardımcılar
// =============================================================================

static const char* headingToStr(Heading h) {
    switch (h) {
        case NORTH: return "KUZEY";
        case EAST:  return "DOGU";
        case SOUTH: return "GUNEY";
        case WEST:  return "BATI";
        default:    return "?";
    }
}
const char* getHeadingName() { return headingToStr(heading); }

// Saat yönünde 90°'de o yönde komşu çizgi var mı?
// turn180'de sensorFindLine'ın ara çizgide durup durmayacağını belirler.
static bool hasNeighborAt(char wp, Heading dir) {
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        if (WAYPOINT_MAP[i].name != wp) continue;
        for (int j = 0; j < WAYPOINT_MAP[i].numNeighbors; j++)
            if (WAYPOINT_MAP[i].neighbors[j].dir == dir) return true;
        return false;
    }
    return false;
}

// =============================================================================
// Sensör tabanlı dönüş yardımcıları
// =============================================================================

static bool sensorExitLine(bool turnRight) {
    unsigned long noLineStart = 0;
    bool counting = false;
    unsigned long timeoutStart = millis();

    while (millis() - timeoutStart < TURN_TIMEOUT) {
        webSocketLoop();
        if (navState == NAV_IDLE) return false;

        turnRight ? motorTurnRight(TURN_SPEED) : motorTurnLeft(TURN_SPEED);
        readCalibratedSensors();

        bool onLine = sensorCalibrated[MID_LEFT_SENSOR]  > LINE_THRESHOLD ||
                      sensorCalibrated[MID_RIGHT_SENSOR] > LINE_THRESHOLD;

        if (!onLine) {
            if (!counting) { noLineStart = millis(); counting = true; }
            else if (millis() - noLineStart >= LINE_EXIT_MS) return true;
        } else {
            counting = false;
        }
        delay(2);
    }
    return false;
}

static bool sensorFindLine(bool turnRight) {
    unsigned long lineStart = 0;
    bool counting = false;
    unsigned long timeoutStart = millis();

    while (millis() - timeoutStart < TURN_TIMEOUT) {
        webSocketLoop();
        if (navState == NAV_IDLE) return false;

        readCalibratedSensors();

        // Herhangi sensör çizgi gördü mü?
        bool anyLine = false;
        for (int i = 0; i < SENSOR_COUNT; i++) {
            if (sensorCalibrated[i] > LINE_THRESHOLD) { anyLine = true; break; }
        }

        // Çizgi sezilince anında yavaş hıza in (atalete yetişmek için).
        int curSpeed = anyLine ? TURN_SPEED_SLOW : TURN_SPEED;
        turnRight ? motorTurnRight(curSpeed) : motorTurnLeft(curSpeed);

        bool onLine = sensorCalibrated[MID_LEFT_SENSOR]  > LINE_THRESHOLD ||
                      sensorCalibrated[MID_RIGHT_SENSOR] > LINE_THRESHOLD;

        if (onLine) {
            if (!counting) { lineStart = millis(); counting = true; }
            else if (millis() - lineStart >= LINE_FIND_MS) return true;
        } else {
            counting = false;
        }
        delay(2);
    }
    return false;
}

// Dönüş sonrası çizgiyi tam merkeze hizala (overshoot düzeltme)
static void centerOnLine() {
    const int           TOLERANCE      = 400;
    const unsigned long CENTER_TIMEOUT = 800;

    unsigned long start = millis();
    int lastDir = 0;

    while (millis() - start < CENTER_TIMEOUT) {
        webSocketLoop();
        if (navState == NAV_IDLE) { motorStop(); return; }

        int pos = calculateLinePosition();
        if (abs(pos) < TOLERANCE) { motorStop(); return; }

        int newDir = (pos > 0) ? 1 : -1;
        if (lastDir != 0 && newDir != lastDir) {
            motorStop();
            delay(30);   // back-EMF darbe önlemi
        }
        lastDir = newDir;

        if (newDir > 0) motorTurnRight(TURN_SPEED_SLOW);
        else            motorTurnLeft(TURN_SPEED_SLOW);
        delay(5);
    }
    motorStop();
}

// =============================================================================
// Tek dönüş motoru (90° / 180° hepsi)
// =============================================================================

// Spin turn: exitLine + findLine adımları toplam 'segments' kez tekrar.
// 90° → 1 segment, 180° (varsa ara çizgi) → 2 segment.
// headingDelta: heading update miktarı (-1 = sol 90°, +1 = sağ 90°, +2 = 180°)
static bool executeSpinTurn(bool turnRight, int segments, int headingDelta,
                            const char* tag) {
    motorStop(); webSocketLoop(); delay(50);

    for (int s = 0; s < segments; s++) {
        if (!sensorExitLine(turnRight)) {
            motorStop();
            char buf[40]; snprintf(buf, sizeof(buf), "%s: Cikis-%d TIMEOUT", tag, s + 1);
            sendLog(buf);
            return false;
        }
        if (!sensorFindLine(turnRight)) {
            motorStop();
            char buf[40]; snprintf(buf, sizeof(buf), "%s: Bulma-%d TIMEOUT", tag, s + 1);
            sendLog(buf);
            return false;
        }
    }

    centerOnLine();
    motorStop(); webSocketLoop(); delay(50);
    heading = (Heading)((heading + headingDelta + 4) % 4);
    return true;
}

static void turnLeft90()         { executeSpinTurn(false, 1, -1, "SOLA"); }
static void turnRight90()        { executeSpinTurn(true,  1, +1, "SAGA"); }
static void turn180(char wp) {
    // Saat yönünde 90°'de komşu çizgi varsa ara çizgide takılırız → 2 segment.
    Heading cwDir   = (Heading)((heading + 1) % 4);
    bool hasMidLine = hasNeighborAt(wp, cwDir);
    executeSpinTurn(true, hasMidLine ? 2 : 1, +2, "180");
}

// Mevcut yönden hedef yöne döner, heading'i günceller.
// wp: dönüşün yapıldığı waypoint — 180° için ara çizgi tespitinde kullanılır.
static void executeTurn(Heading targetDir, char wp = 0) {
    if (targetDir == heading) return;
    int diff = (targetDir - heading + 4) % 4;
    switch (diff) {
        case 1: turnRight90();   break;
        case 2: turn180(wp);     break;
        case 3: turnLeft90();    break;
    }
}

// =============================================================================
// Çizgi Arama (çizgi kaybolunca)
// =============================================================================

static void searchForLine() {
    unsigned long start = millis();
    bool turnRight = (dbgError >= 0);

    while (millis() - start < LINE_SEARCH_MS) {
        webSocketLoop();
        if (navState == NAV_IDLE) { motorStop(); return; }

        // Terminal node'da çizgiyi atlayıp line search'e düştüysek RFID hâlâ okunabilir
        char rfidWP = 0;
        if (readRFIDWaypointFast(&rfidWP) && rfidWP != nav.lastReadWP) {
            nav.lastReadWP     = rfidWP;
            nav.rfidDetectedWP = rfidWP;
            motorStop();
            navState = NAV_AT_JUNCTION;
            sendLog("Line search'te RFID okundu");
            return;
        }

        readCalibratedSensors();
        if (isLineDetected()) {
            motorStop();
            resetPID();
            sendLog("Cizgi bulundu, devam ediliyor");
            navState = NAV_FOLLOWING;
            return;
        }
        turnRight ? motorTurnRight(TURN_SPEED) : motorTurnLeft(TURN_SPEED);
        delay(5);
    }
    motorStop();
    sendLog("Cizgi bulunamadi! Duruldu.");
    navState = NAV_IDLE;
}

// =============================================================================
// Kavşak İşleyici
// =============================================================================

// Yolu logla (yardımcı)
static void logPath(const char* prefix) {
    char buf[MAX_PATH_LENGTH * 2 + 32];
    snprintf(buf, sizeof(buf), "%s", prefix);
    int len = strlen(buf);
    for (int i = 0; i < navPathLength && len < (int)sizeof(buf) - 3; i++) {
        buf[len++] = navPath[i];
        if (i < navPathLength - 1) buf[len++] = '>';
    }
    buf[len] = 0;
    sendLog(buf);
}

static void handleJunction() {
    // 1. RFID konumunu uygula
    char prevWP = currentWaypoint;
    char rfidWP = nav.rfidDetectedWP;
    nav.rfidDetectedWP = 0;

    if (rfidWP != 0) {
        currentWaypoint = rfidWP;
        char buf[32]; sprintf(buf, "RFID: %c", rfidWP);
        sendLog(buf);

        // Yön kalibrasyonu (önceki node ile şimdiki arasından)
        if (prevWP != 0 && rfidWP != prevWP) {
            Heading inferred;
            if (getDirection(prevWP, rfidWP, &inferred)) {
                if (inferred != heading) {
                    char hbuf[56];
                    sprintf(hbuf, "RFID yon duzeltme: %s -> %s",
                            headingToStr(heading), headingToStr(inferred));
                    sendLog(hbuf);
                }
                heading = inferred;
            }
        }
    }

    // 2. Hedefe ulaştık mı?
    if (currentWaypoint == targetWaypoint) {
        navState = NAV_REACHED;
        return;
    }

    // 3. Yolda ilerle
    navPathIndex++;
    if (navPathIndex >= navPathLength) {
        sendLog("Hata: Yol tukendi ama hedefe ulasilamadi!");
        navState = NAV_IDLE;
        return;
    }

    // 4. Sonraki waypoint'e yön bul
    char nextWP = navPath[navPathIndex];
    Heading targetDir;
    if (!getDirection(currentWaypoint, nextWP, &targetDir)) {
        // Beklenmedik node — re-path
        char rbuf[64];
        snprintf(rbuf, sizeof(rbuf), "Beklenmedik node %c! Re-path: %c->%c",
                 currentWaypoint, currentWaypoint, targetWaypoint);
        sendLog(rbuf);

        if (!findPath(currentWaypoint, targetWaypoint, navPath, &navPathLength)
            || navPathLength < 2) {
            sendLog("Re-path basarisiz! Duruluyor.");
            navState = NAV_IDLE;
            return;
        }
        navPathIndex = 1;
        nextWP = navPath[1];
        if (!getDirection(currentWaypoint, nextWP, &targetDir)) {
            sendLog("Re-path yon bulunamadi! Duruluyor.");
            navState = NAV_IDLE;
            return;
        }
        logPath("Yeni yol: ");
    }

    char buf[48];
    snprintf(buf, sizeof(buf), "%c -> %c (%s)", currentWaypoint, nextWP,
             headingToStr(targetDir));
    sendLog(buf);

    // Kart kavşak köşesinde okunuyor → ileri gitmeden direkt dönüş.
    sendLog("[KAVSAK] Donus basliyor (kart konumda)");
    executeTurn(targetDir, currentWaypoint);
    if (navState == NAV_IDLE) return;

    resetPID();
    nav.lastCorrection = 0;
    navState = NAV_FOLLOWING;
}

// =============================================================================
// State handler'lar (her case için ayrı fonksiyon)
// =============================================================================

// runNavigation öncesi ortak kontroller. AGV durdurulması gerekiyorsa false.
static bool preNavigationChecks() {
    // Engel SADECE NAV_FOLLOWING'de aktif (dönüşlerde çevre objelerini yok say)
    if (navState == NAV_FOLLOWING && isObstacleDetected()) {
        motorStop();
        if (!nav.obstacleLogged) {
            sendLog("Engel algilandi! Bekleniyor...");
            nav.obstacleLogged = true;
        }
        return false;
    }
    if (nav.obstacleLogged) {
        sendLog("Engel gecti, devam ediliyor.");
        nav.obstacleLogged = false;
    }

    if (!wsConnected)             { motorStop(); return false; }
    if (!calibData.isCalibrated)  { motorStop(); return false; }
    if (!isTarget &&
        navState != NAV_TURNING &&
        navState != NAV_LINE_SEARCH) {
        motorStop();
        navState = NAV_IDLE;
        return false;
    }
    return true;
}

static void handleTurning() {
    motorStop();
    if (navPathLength <= 1) {
        navState = NAV_REACHED;
        return;
    }
    Heading firstDir;
    if (getDirection(navPath[0], navPath[1], &firstDir)) {
        executeTurn(firstDir, navPath[0]);
    }
    if (navState != NAV_IDLE) {
        navPathIndex = 1;
        resetPID();
        nav.lastCorrection = 0;
        navState = NAV_FOLLOWING;
        sendLog("Navigasyon baslatildi");
    }
}

static void handleFollowing() {
    readCalibratedSensors();

    if (isJunction()) {
        // Kavşakta PID yerine son düzeltmeyi uygula (büyük salınımı önle)
        int safeCorr = constrain(nav.lastCorrection, -10, 10);
        setMotors(baseSpeed - safeCorr, baseSpeed + safeCorr);
        nav.lineLostStart = 0;
    } else if (isLineLost()) {
        if (nav.lineLostStart == 0) nav.lineLostStart = millis();
        else if (millis() - nav.lineLostStart > LINE_LOST_MS) {
            motorStop();
            sendLog("Cizgi kayboldu! Araniyor...");
            nav.lineLostStart = 0;
            navState = NAV_LINE_SEARCH;
            return;
        }
        motorForward(baseSpeed);
    } else {
        nav.lineLostStart = 0;
        lineFollowPID();
        nav.lastCorrection = dbgCorrection;
    }

    // RFID waypoint tespiti — okunduğu anda dur, kavşağa geç
    char rfidWP = 0;
    if (readRFIDWaypointFast(&rfidWP) && rfidWP != nav.lastReadWP) {
        nav.lastReadWP     = rfidWP;
        nav.rfidDetectedWP = rfidWP;
        nav.lineLostStart  = 0;
        motorStop();
        navState = NAV_AT_JUNCTION;
    }
}

static void handleReached() {
    motorStop();
    isTarget      = false;
    navPathIndex  = 0;
    navPathLength = 0;
    navState      = NAV_IDLE;
    sendLog("Hedefe ulasildi");
}

static void handleIdle() {
    motorStop();

    // Konum bilinmiyorsa periyodik RFID polling (her 200 ms).
    // Manuel navCommandSetPosition aynen çalışmaya devam eder (last-write-wins).
    if (currentWaypoint == 0 &&
        millis() - nav.lastIdleRfidMs >= 200) {
        nav.lastIdleRfidMs = millis();
        char rfidWP = 0;
        if (readRFIDWaypointFast(&rfidWP)) {
            currentWaypoint   = rfidWP;
            nav.lastReadWP    = rfidWP;
            char buf[40];
            snprintf(buf, sizeof(buf), "Konum otomatik: %c (RFID)", rfidWP);
            sendLog(buf);
        }
    }
}

// =============================================================================
// State Machine
// =============================================================================

void navigationInit() {
    currentWaypoint = 0;
    targetWaypoint  = 0;
    navPathLength   = 0;
    navPathIndex    = 0;
    heading         = NORTH;
    navState        = NAV_IDLE;
    isTarget        = false;
    resetNavRuntime();
}

void runNavigation() {
    if (!preNavigationChecks()) return;

    switch (navState) {
        case NAV_TURNING:     handleTurning();   break;
        case NAV_FOLLOWING:   handleFollowing(); break;
        case NAV_LINE_SEARCH: searchForLine();   break;
        case NAV_AT_JUNCTION: handleJunction();  break;
        case NAV_REACHED:     handleReached();   break;
        case NAV_IDLE:        handleIdle();      break;
        default: break;
    }
}

// =============================================================================
// Komut Arayüzü (facade)
// =============================================================================

void navCommandSetPosition(char waypoint) {
    currentWaypoint = waypoint;
    char buf[32]; snprintf(buf, sizeof(buf), "Konum ayarlandi: %c", waypoint);
    sendLog(buf);
}

void navCommandSetTarget(char waypoint) {
    if (currentWaypoint == 0) {
        sendLog("Hata: Once konumu ayarlayin (setPosition)!");
        return;
    }

    targetWaypoint = waypoint;

    if (!findPath(currentWaypoint, targetWaypoint, navPath, &navPathLength)) {
        char buf[48];
        sprintf(buf, "Hata: %c->%c yolu bulunamadi!", currentWaypoint, waypoint);
        sendLog(buf);
        return;
    }

    isTarget           = true;
    nav.lastReadWP     = currentWaypoint;   // Başlangıç kartı tekrar tetiklemesin
    nav.rfidDetectedWP = 0;

    logPath("Yol: ");

    if (calibData.isCalibrated) {
        navState = NAV_TURNING;
    } else {
        sendLog("Hedef alindi. Kalibrasyon yapinca Baslat'a basin.");
    }
}

void navCommandStart() {
    if (!calibData.isCalibrated) { sendLog("Hata: Kalibrasyon gerekli!"); return; }
    if (!isTarget)               { sendLog("Hata: Hedef belirlenmedi!");  return; }
    if (currentWaypoint == 0)    { sendLog("Hata: Konum bilinmiyor!");    return; }
    navState = NAV_TURNING;
    sendLog("Navigasyon baslatiliyor...");
}

void navCommandStop() {
    motorStop();
    navState = NAV_IDLE;
    resetNavRuntime();   // tüm runtime state'i temizle
    sendLog("Durduruldu");
}

void navCommandSetPID(float kp, float ki, float kd) {
    setPIDParams(kp, ki, kd);
    char buf[48];
    sprintf(buf, "PID: Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
    sendLog(buf);
}

void navCommandSetSpeed(int speed) {
    baseSpeed = speed;
    char buf[24]; sprintf(buf, "Hiz: %d", speed);
    sendLog(buf);
}

void navCommandCalibrate() {
    navCommandStop();
    isTarget = false;
    digitalWrite(LED_PIN, HIGH); delay(1000); digitalWrite(LED_PIN, LOW);
    runManualCalibration();
    digitalWrite(LED_PIN, HIGH); delay(1000); digitalWrite(LED_PIN, LOW);
}

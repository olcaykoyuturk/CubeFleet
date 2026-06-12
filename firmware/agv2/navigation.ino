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
bool     calibrationActive = false;           // Kalibrasyon sirasinda true,
                                              // webSocketLoop sendStatus'u atlar

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

// ÖĞRENİLEN 90° DÖNÜŞ SÜRESİ — her başarılı SENSÖRLÜ 90° segment ölçülür,
// EMA (%70 eski + %30 yeni) ile güncellenir. faceDir bunun üzerinden çizgisiz
// (küp yönü) dönüş yapar. Default yalnız ilk sensörlü dönüşe kadar geçerli.
static unsigned long avgTurn90Ms = TIMED_TURN90_DEFAULT_MS;

static void updateTurn90Avg(unsigned long measuredMs) {
    if (measuredMs < TIMED_TURN90_MIN_MS || measuredMs > TIMED_TURN90_MAX_MS)
        return;   // sahte/takılmalı ölçüm — ortalamayı kirletme
    avgTurn90Ms = (avgTurn90Ms * 7 + measuredMs * 3) / 10;
}

// Spin turn: exitLine + findLine adımları toplam 'segments' kez tekrar.
// 90° → 1 segment, 180° (varsa ara çizgi) → 2 segment.
// headingDelta: heading update miktarı (-1 = sol 90°, +1 = sağ 90°, +2 = 180°)
static bool executeSpinTurn(bool turnRight, int segments, int headingDelta,
                            const char* tag) {
    motorStop(); webSocketLoop(); delay(50);

    for (int s = 0; s < segments; s++) {
        unsigned long segStart = millis();
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
        // Başarılı 90° segment — süresini öğrenilen ortalamaya işle
        updateTurn90Avg(millis() - segStart);
    }

    centerOnLine();
    motorStop(); webSocketLoop(); delay(50);
    heading = (Heading)((heading + headingDelta + 4) % 4);
    return true;
}

// ZAMANLI 90° dönüş — çizgi OLMAYAN yöne (küp tarafı). Öğrenilen ortalama
// süre kadar döner, sonra durur. STOP her an işler (navCommandStop
// navState'i IDLE yapar → döngü kırılır). Dönüşte biriken küçük açı hatası
// kalıcı DEĞİL: bir sonraki sensörlü dönüş (setHop ilk dönüşü) çizgiye
// kilitlenirken hatayı sıfırlar.
static bool timedTurn90(bool turnRight) {
    motorStop(); webSocketLoop(); delay(50);
    unsigned long start = millis();
    while (millis() - start < avgTurn90Ms) {
        webSocketLoop();
        if (navState == NAV_IDLE) { motorStop(); return false; }   // STOP
        turnRight ? motorTurnRight(TURN_SPEED) : motorTurnLeft(TURN_SPEED);
        delay(2);
    }
    motorStop();
    heading = (Heading)((heading + (turnRight ? 1 : 3)) % 4);
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
// faceDir — küp yönüne dönüş (NAV_IDLE'da, PC komutuyla)
// =============================================================================

// Bekleyen faceDir istegi (-1 = yok). webSocket handler navCommandFaceDir ile
// yazar; handleIdle bir sonraki loop'ta yurutur (facade deseni).
static int facePendingDir = -1;

// Hedef yone don: her 90° segment icin o yonde CIZGI varsa sensorlu donus
// (hassas + ortalamayi besler), yoksa ZAMANLI donus (ogrenilen sure).
// 180° = iki segment (saat yonu). Bitince faceComplete PC'ye gider.
// navState yurutme boyunca NAV_TURNING yapilir ki navCommandStop (IDLE set
// eder) donusu her an kesebilsin; sonunda IDLE'a doner.
static void executeFaceDir(Heading target) {
    if (target == heading) {
        sendFaceComplete(currentWaypoint, headingToStr(heading));
        return;
    }
    navState = NAV_TURNING;
    int diff = (target - heading + 4) % 4;     // 1=sag, 2=arka, 3=sol
    bool turnRight = (diff != 3);
    int  segments  = (diff == 2) ? 2 : 1;

    char buf[64];
    snprintf(buf, sizeof(buf), "faceDir: %s -> %s (%d seg, avg90=%lums)",
             headingToStr(heading), headingToStr(target),
             segments, avgTurn90Ms);
    sendLog(buf);

    for (int s = 0; s < segments; s++) {
        Heading segTarget = (Heading)((heading + (turnRight ? 1 : 3)) % 4);
        bool lineThere = (currentWaypoint != 0)
                         && hasNeighborAt(currentWaypoint, segTarget);
        bool ok = lineThere
                  ? executeSpinTurn(turnRight, 1, turnRight ? 1 : -1, "FACE")
                  : timedTurn90(turnRight);
        if (!ok || navState == NAV_IDLE) {
            motorStop();
            navState = NAV_IDLE;
            sendLog("faceDir iptal/timeout");
            return;
        }
    }
    navState = NAV_IDLE;
    sendFaceComplete(currentWaypoint, headingToStr(heading));
}

// Komut arayuzu: 'N'/'E'/'S'/'W' karakteriyle cagrilir (websocket.ino).
// Yalniz NAV_IDLE'da kabul — gorev ortasinda kup donusu anlamsiz/tehlikeli.
void navCommandFaceDir(char dirChar) {
    int d = -1;
    switch (dirChar) {
        case 'N': d = NORTH; break;
        case 'E': d = EAST;  break;
        case 'S': d = SOUTH; break;
        case 'W': d = WEST;  break;
    }
    if (d < 0) { sendLog("faceDir: gecersiz yon"); return; }
    // NAV_REACHED da kabul: hopComplete ile faceDir arasinda firmware henuz
    // handleReached'i islememis olabilir (setHop race-guard ile ayni mantik).
    if (navState != NAV_IDLE && navState != NAV_REACHED) {
        sendLog("faceDir reddedildi: arac mesgul");
        return;
    }
    facePendingDir = d;
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

// Yolu logla yardimcisi setTarget ile birlikte kaldirildi — setHop kendi
// detayli log mesajini icerir.

static void handleJunction() {
    // 1. RFID konumunu uygula
    char prevWP = currentWaypoint;
    char rfidWP = nav.rfidDetectedWP;
    nav.rfidDetectedWP = 0;

    if (rfidWP != 0) {
        currentWaypoint = rfidWP;
        char buf[32]; sprintf(buf, "RFID: %c", rfidWP);
        sendLog(buf);

        // Planner senkronizasyonu: hop tamamlandi bildir.
        sendHopComplete(rfidWP, headingToStr(heading));

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
        // navPath sona vardi ama hedefe ulasilamadi — PC continuous update
        // mesaji gec ulasmis olabilir. Kisa sure webSocketLoop ile bekle.
        sendLog("Yol son node'a vardi, setHop bekleniyor...");
        motorStop();
        unsigned long waitStart = millis();
        // 3 saniye bekle (WS gecikmesi ~1s olabilir, 1.5s yetmiyor)
        const unsigned long SETHOP_WAIT_MS = 3000;
        while (millis() - waitStart < SETHOP_WAIT_MS) {
            webSocketLoop();
            if (navState == NAV_IDLE) return;
            if (navPathIndex < navPathLength) break;
            delay(10);
        }
        if (navPathIndex >= navPathLength) {
            sendLog("setHop gelmedi, IDLE");
            navState = NAV_IDLE;
            return;
        }
        sendLog("setHop yetisti, devam");
    }

    // 4. Sonraki waypoint'e yön bul
    char nextWP = navPath[navPathIndex];
    Heading targetDir;
    if (!getDirection(currentWaypoint, nextWP, &targetDir)) {
        // Beklenmedik kart — PC drift correction devreye girip yeni setHop
        // gonderene kadar dur.
        char rbuf[80];
        snprintf(rbuf, sizeof(rbuf),
                 "Beklenmedik node %c (next=%c): PC'ye bildirildi, IDLE",
                 currentWaypoint, nextWP);
        sendLog(rbuf);
        motorStop();
        navState      = NAV_IDLE;
        isTarget      = false;
        navPathIndex  = 0;
        navPathLength = 0;
        return;
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

    // Bekleyen faceDir istegi varsa yurut (kup yonune donus — bloklayan ama
    // STOP-kesilebilir; bitince/iptal edilince IDLE'a doner).
    if (facePendingDir >= 0) {
        Heading t = (Heading)facePendingDir;
        facePendingDir = -1;
        executeFaceDir(t);
        return;
    }

    // IDLE'da SÜREKLI RFID polling (her 200 ms). AGV bir karta yerleştirilirse
    // konum hemen güncellenir — kullanıcı manuel setPosition yapmasına gerek yok.
    // Manuel navCommandSetPosition yine çalışır (last-write-wins).
    //
    // Filtre: sadece konum DEĞIŞTIĞINDE update + log (aynı kart üst üste okunsa
    // sessiz). currentWaypoint != rfidWP kontrolü bu spam'i engeller.
    if (millis() - nav.lastIdleRfidMs >= 200) {
        nav.lastIdleRfidMs = millis();
        char rfidWP = 0;
        if (readRFIDWaypointFast(&rfidWP) && rfidWP != currentWaypoint) {
            currentWaypoint = rfidWP;
            nav.lastReadWP  = rfidWP;
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
    // IDLE state'i HER ZAMAN çalıştır — kalibrasyon/target/connect gerektirmez.
    // Sadece motorStop + RFID polling yapıyor; AGV açıldığında bir karta
    // yerleştirilirse konum hemen tespit edilir (kalibrasyondan ÖNCE bile).
    if (navState == NAV_IDLE) {
        handleIdle();
        return;
    }

    // Diğer state'ler için tam kontrol: WS bağlantısı, kalibrasyon, hedef
    if (!preNavigationChecks()) return;

    switch (navState) {
        case NAV_TURNING:     handleTurning();   break;
        case NAV_FOLLOWING:   handleFollowing(); break;
        case NAV_LINE_SEARCH: searchForLine();   break;
        case NAV_AT_JUNCTION: handleJunction();  break;
        case NAV_REACHED:     handleReached();   break;
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
    // Re-entry guard: runManualCalibration() içinde webSocketLoop() çağrılıyor,
    // bu sırada gelen yeni "calibrate" komutu recursive çağrı yapardı (kullanıcı
    // butona art arda basarsa kalibrasyon hep yeniden başlardı, asla bitmezdi).
    // Statik bayrak ile yeni çağrıları engelle.
    static bool calibrating = false;
    if (calibrating) {
        sendLog("Kalibrasyon devam ediyor, yeni komut yoksayildi");
        return;
    }
    calibrating = true;
    calibrationActive = true;   // webSocketLoop sendStatus'u atlasin — WS trafigi dussun

    // Kalibrasyon hazırlığı — motorlari sessizce durdur (navCommandStop "Durduruldu"
    // log'u atiyor, bu kullaniciya "iptal oldu" gibi gorunuyor)
    sendLog("Kalibrasyon basliyor — araci cizgi/zemin uzerinde gezdir (~8 sn)");
    motorStop();
    navState = NAV_IDLE;
    isTarget = false;
    resetNavRuntime();

    digitalWrite(LED_PIN, HIGH); delay(1000); digitalWrite(LED_PIN, LOW);
    runManualCalibration();
    digitalWrite(LED_PIN, HIGH); delay(1000); digitalWrite(LED_PIN, LOW);

    calibrationActive = false;   // status broadcast geri acilsin
    calibrating = false;
}

// PC'den gelen preset'i runtime'da uygula. NVS'ye yazmaz; reboot'ta uçar.
// Doğrulama: her sensör için max-min >= CALIB_MIN_RANGE olmalı. Aksi halde reddet.
// GUARD: Aktif kalibrasyon sirasinda reddet — yoksa runManualCalibration'in
// guncellemekte oldugu calibData.minVal/maxVal araylarını ezer, hibrit/bozuk
// kalibrasyon ortaya cikar.
bool navCommandApplyCalibration(const int* minVals, const int* maxVals) {
    if (calibrationActive) {
        sendLog("applyCalibration: kalibrasyon devam ediyor, reddedildi");
        return false;
    }
    if (minVals == nullptr || maxVals == nullptr) {
        sendLog("applyCalibration: bos veri");
        return false;
    }

    for (int i = 0; i < SENSOR_COUNT; i++) {
        int range = maxVals[i] - minVals[i];
        if (range < CALIB_MIN_RANGE) {
            char buf[64];
            snprintf(buf, sizeof(buf),
                     "applyCalibration: S%d range %d yetersiz", i, range);
            sendLog(buf);
            return false;
        }
    }

    for (int i = 0; i < SENSOR_COUNT; i++) {
        calibData.minVal[i] = minVals[i];
        calibData.maxVal[i] = maxVals[i];
    }
    calibData.isCalibrated = true;
    sendLog("Kalibrasyon preset uygulandi");
    return true;
}

// =============================================================================
// Multi-AGV Planner emirleri
// =============================================================================

static const char* navStateName(NavState s) {
    switch (s) {
        case NAV_IDLE:        return "IDLE";
        case NAV_TURNING:     return "TURNING";
        case NAV_FOLLOWING:   return "FOLLOWING";
        case NAV_AT_JUNCTION: return "AT_JUNCTION";
        case NAV_REACHED:     return "REACHED";
        case NAV_LINE_SEARCH: return "LINE_SEARCH";
        default:              return "?";
    }
}

void navCommandHop(char from, char next, char after, char goal) {
    char dlog[128];
    snprintf(dlog, sizeof(dlog),
             "[HOP-IN] from=%c next=%c after=%c goal=%c | AGV=%c state=%s "
             "navPath[%d/%d]=%c",
             from, next, (after != 0) ? after : '-', (goal != 0) ? goal : '-',
             currentWaypoint, navStateName(navState),
             navPathIndex, navPathLength,
             (navPathLength > 0 && navPathIndex < navPathLength)
                 ? navPath[navPathIndex] : '-');
    sendLog(dlog);

    if (from == next) {
        sendLog("setHop: from==next, gozardi");
        return;
    }

    if (currentWaypoint == 0) {
        currentWaypoint = from;
    } else if (currentWaypoint != from) {
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "setHop: stale (from=%c, AGV=%c) reddedildi",
                 from, currentWaypoint);
        sendLog(buf);
        return;
    }

    bool sameHop = (currentWaypoint == from
                    && navPathLength > navPathIndex
                    && navPath[navPathIndex] == next);
    bool moving  = (navState == NAV_FOLLOWING
                    || navState == NAV_TURNING
                    || navState == NAV_AT_JUNCTION);

    if (sameHop && moving) {
        int afterIdx = navPathIndex + 1;
        if (after != 0 && afterIdx < MAX_PATH_LENGTH) {
            navPath[afterIdx] = after;
            if (afterIdx >= navPathLength) navPathLength = afterIdx + 1;
        } else {
            navPathLength  = navPathIndex + 1;
        }
        targetWaypoint = (goal != 0) ? goal
                                     : ((after != 0) ? after : next);
        char buf[48];
        snprintf(buf, sizeof(buf), "Hop after=%c (kesintisiz)",
                 (after != 0) ? after : '-');
        sendLog(buf);
        return;
    }

    // RACE GUARD: NAV_FOLLOWING/TURNING/LINE_SEARCH'de yeni full hop reddedilir.
    // NAV_AT_JUNCTION'da AGV setHop bekliyor (handleJunction wait) → kabul.
    if (navState == NAV_FOLLOWING ||
        navState == NAV_TURNING ||
        navState == NAV_LINE_SEARCH) {
        char buf[80];
        snprintf(buf, sizeof(buf),
                 "setHop: %s mesgul, yeni hop reddedildi (from=%c next=%c)",
                 navStateName(navState), from, next);
        sendLog(buf);
        return;
    }

    // Fresh full hop. navPath[0]=current, navPath[1]=hedef.
    // navPathIndex=1 (sonraki hedef konvansiyonu — handleTurning idempotent).
    navPath[0]    = from;
    navPath[1]    = next;
    navPathLength = 2;
    if (after != 0) {
        navPath[2]    = after;
        navPathLength = 3;
    }
    navPathIndex   = 1;
    targetWaypoint = (goal != 0) ? goal
                                 : ((after != 0) ? after : next);
    isTarget       = true;

    nav.lastReadWP     = currentWaypoint;
    nav.rfidDetectedWP = 0;

    char buf[64];
    if (after != 0) {
        snprintf(buf, sizeof(buf), "Hop: %c -> %c -> %c (goal=%c)",
                 from, next, after, (goal != 0) ? goal : '-');
    } else {
        snprintf(buf, sizeof(buf), "Hop: %c -> %c (goal=%c)",
                 from, next, (goal != 0) ? goal : '-');
    }
    sendLog(buf);

    if (calibData.isCalibrated) {
        navState = NAV_TURNING;
    } else {
        sendLog("Kalibrasyon gerekli, hop bekliyor.");
    }
}


#pragma once
#include "pin.h"
#include "config.h"

// =============================================================================
// types.h — Paylaşılan veri tipleri, extern bildirimleri, fonksiyon prototipleri
// =============================================================================

// ===== Veri Yapıları =====

struct CalibrationData {
    int  minVal[SENSOR_COUNT];
    int  maxVal[SENSOR_COUNT];
    bool isCalibrated;
};

struct PIDParams {
    float         Kp;
    float         Ki;
    float         Kd;
    float         lastError;
    float         integral;
    unsigned long lastTime;
};

// ===== Enum'lar =====

enum Heading {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

enum NavState {
    NAV_IDLE,
    NAV_TURNING,        // Başlangıç yönü ayarlanıyor
    NAV_FOLLOWING,      // Çizgi takibi
    NAV_AT_JUNCTION,    // Kavşak işleniyor
    NAV_REACHED,        // Hedefe ulaşıldı
    NAV_LINE_SEARCH     // Çizgi kayboldu, aranıyor
};

// =============================================================================
// Global Değişken Bildirimleri
// =============================================================================

// --- main.ino ---
extern CalibrationData calibData;
extern PIDParams       pidParams;
extern int             sensorValues[SENSOR_COUNT];
extern int             sensorCalibrated[SENSOR_COUNT];
extern int             linePosition;
extern int             baseSpeed;

// --- navigation.ino ---
extern char     currentWaypoint;            // Şu anki konum ('A'-'I', 0=bilinmiyor)
extern char     targetWaypoint;             // Hedef waypoint
extern char     navPath[MAX_PATH_LENGTH];   // BFS yolu
extern int      navPathLength;              // Yol uzunluğu
extern int      navPathIndex;              // Yolda şu anki konum indeksi
extern Heading  heading;                    // Aracın baktığı yön
extern NavState navState;
extern bool     isTarget;

// --- websocket.ino ---
extern bool wsConnected;

// --- debug.ino ---
extern int dbgLeftSpeed;
extern int dbgRightSpeed;
extern int dbgError;
extern int dbgCorrection;

// --- sonar.ino ---
extern float sonarDistance;
extern bool  sonarObstacle;

// =============================================================================
// Fonksiyon Prototipleri
// =============================================================================

// --- motor.ino ---
void motorInit();
void setMotors(int leftSpeed, int rightSpeed);
void motorStop();
void motorForward(int speed);
void motorTurnLeft(int speed);
void motorTurnRight(int speed);

// --- read_mux.ino ---
void muxInit();
void readAllSensors();
void readCalibratedSensors();

// --- calibration.ino ---
void calibrationInit();
void updateCalibrationValues();
void runManualCalibration();
void printCalibrationData();

// --- line_Pos.ino ---
int  calculateLinePosition();
bool isLineDetected();
bool isJunction();
bool isLineLost();

// --- pid.ino ---
void pidInit();
void setPIDParams(float kp, float ki, float kd);
int  calculatePID(int error);
void resetPID();
void lineFollowPID();

// --- navigation.ino ---
void navigationInit();
void runNavigation();
const char* getHeadingName();

// --- navigation.ino: Komut arayüzü ---
void navCommandSetTarget(char waypoint);
void navCommandSetPosition(char waypoint);
void navCommandStart();
void navCommandStop();
void navCommandSetPID(float kp, float ki, float kd);
void navCommandSetSpeed(int speed);
void navCommandCalibrate();

// --- pathfinder.ino ---
bool findPath(char from, char to, char* path, int* pathLength);
bool getDirection(char from, char to, Heading* dir);

// --- websocket.ino ---
void wifiInit();
void webSocketInit();
void webSocketLoop();
void sendStatus();
void sendLog(const char* message);

// --- sonar.ino ---
void  sonarInit();
void  updateSonar();
bool  isObstacleDetected();
float getSonarDistance();

// --- rfid.ino ---
void rfidInit();
bool readRFIDWaypointFast(char* waypoint);   // non-blocking, NAV_FOLLOWING içinde kart tespiti

// --- debug.ino ---
void debugInit();
void printDebugInfo();

// --- oled.ino ---
void oledInit();
void oledUpdate();

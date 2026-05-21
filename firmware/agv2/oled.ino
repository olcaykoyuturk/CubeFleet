// ===== OLED Ekran Modülü (SSD1306 128×64) =====

#include "types.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_RESET    -1
#define OLED_INTERVAL 200   // ms

static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
static unsigned long lastOledUpdate = 0;

void oledInit() {
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        return;
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.display();
}

// Sabit bölüm: konum/hedef + hız/PID + ayırıcı
static void drawFixed() {
    display.setTextSize(1);

    // Satır 1: Konum → Hedef
    display.setCursor(0, 0);
    display.print("Pos:");
    display.print(currentWaypoint ? currentWaypoint : '?');
    display.print("   Hedef:");
    display.print(targetWaypoint  ? targetWaypoint  : '?');

    // Satır 2: Hız + PID hatası
    display.setCursor(0, 12);
    display.print("Spd:");
    display.print(baseSpeed);
    display.print("   Err:");
    display.print(dbgError);

    // Ayırıcı çizgi
    display.drawLine(0, 22, 127, 22, SSD1306_WHITE);
}

// Değişken bölüm: state'e göre 2 satır (y=28 ve y=46)
static void drawStateLines() {
    display.setTextSize(1);

    // Engel öncelikli
    if (sonarObstacle) {
        display.setCursor(0, 28);
        display.print("!! ENGEL !!");
        display.setCursor(0, 46);
        display.print("Dist: ");
        display.print((int)sonarDistance);
        display.print(" cm");
        return;
    }

    switch (navState) {

        case NAV_IDLE:
            display.setCursor(0, 28);
            display.print("HAZIR");
            display.setCursor(0, 46);
            display.print(wsConnected ? "WiFi: BAGLI" : "WiFi: YOK");
            break;

        case NAV_FOLLOWING: {
            // Sensör bar (8 nokta)
            display.setCursor(0, 28);
            display.print("Sen:");
            for (int i = 0; i < SENSOR_COUNT; i++)
                display.print(sensorCalibrated[i] > LINE_THRESHOLD ? '#' : '.');
            // Sonar mesafe
            display.setCursor(0, 46);
            display.print("Obs: ");
            display.print((int)sonarDistance);
            display.print(" cm");
            break;
        }

        case NAV_TURNING:
            display.setCursor(0, 28);
            display.print("DONUYOR...");
            display.setCursor(0, 46);
            display.print(getHeadingName());
            break;

        case NAV_AT_JUNCTION:
            display.setCursor(0, 28);
            display.print("KAVSAK");
            display.setCursor(0, 46);
            display.print("WP: ");
            display.print(currentWaypoint ? currentWaypoint : '?');
            break;

        case NAV_REACHED:
            display.setTextSize(2);
            display.setCursor(0, 28);
            display.print("ULASTI!");
            break;

        case NAV_LINE_SEARCH:
            display.setCursor(0, 28);
            display.print("CIZGI ARANIYOR");
            display.setCursor(0, 46);
            display.print("...");
            break;
    }
}

void oledUpdate() {
    if (millis() - lastOledUpdate < OLED_INTERVAL) return;
    lastOledUpdate = millis();

    display.clearDisplay();
    drawFixed();
    drawStateLines();
    display.display();
}

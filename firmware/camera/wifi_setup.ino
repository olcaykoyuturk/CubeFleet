#include <WiFi.h>
#include "types.h"

// =============================================================================
// wifi_setup.ino - Server'in AP'ina statik IP ile baglan
// =============================================================================

bool wifiInit() {
    WiFi.mode(WIFI_STA);

    // modem sleep kapali — yoksa periyodik clock dipi LEDC PWM zamanlamasini
    // bozup DS3225'te titreme yapiyor. Karsiligi ~10-20 mA fazla cekim.
    WiFi.setSleep(false);

    IPAddress local(192, 168, 4, CAM_IP_OCT_4);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dns(192, 168, 4, 1);

    if (!WiFi.config(local, gateway, subnet, dns)) {
        camLog("HATA: Statik IP atanamadi");
        return false;
    }

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
        delay(250);
    }

    if (WiFi.status() != WL_CONNECTED) {
        camLog("HATA: WiFi'ya baglanamadi (%s)", WIFI_SSID);
        return false;
    }

    // tekrar setSleep(false) — bazi core surumlerinde baglanti sirasinda
    // sleep kendiliginden aciliyor.
    WiFi.setSleep(false);
    return true;
}

void wifiPrintInfo() {
    IPAddress ip = WiFi.localIP();
    camLog("WiFi BAGLI: %d.%d.%d.%d RSSI:%d",
           ip[0], ip[1], ip[2], ip[3], WiFi.RSSI());
}

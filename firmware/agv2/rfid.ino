// ===== RC522 RFID Okuyucu =====
// Kavşaktaki kartı okur, waypoint harfini döndürür.
// Kart formatı: MIFARE Classic 1K, Sector 1 Block 4
//   byte[0] = waypoint harfi ('A'–'I' arası ASCII)
//
// Kart yazmak için rfid_writer/ sketch'ini kullanın.

#include "types.h"
#include <SPI.h>
#include <MFRC522.h>

static MFRC522 rfid(RC522_SS_PIN, UINT8_MAX);  // RST → 3.3V

static MFRC522::MIFARE_Key rfidKey = {
    .keyByte = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
};

#define RFID_DATA_BLOCK   4     // Sector 1, ilk data bloğu
#define RFID_TIMEOUT_MS 300     // Kart arama timeout'u

void rfidInit() {
    SPI.begin(RC522_SCK_PIN, RC522_MISO_PIN, RC522_MOSI_PIN, RC522_SS_PIN);
    rfid.PCD_Init();

    (void)rfid.PCD_ReadRegister(MFRC522::VersionReg);
}

// Non-blocking anlık kart kontrolü — döngü/timeout YOK.
// Kart sensör altındaysa okur, yoksa hemen false döner.
// NAV_FOLLOWING içinde terminal waypoint tespiti için kullanılır.
bool readRFIDWaypointFast(char* waypoint) {
    if (!rfid.PICC_IsNewCardPresent()) return false;
    if (!rfid.PICC_ReadCardSerial())   return false;
    if (!rfid.uid.size)                return false;

    MFRC522::StatusCode status = rfid.PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A,
        RFID_DATA_BLOCK, &rfidKey, &(rfid.uid)
    );
    if (status != MFRC522::STATUS_OK) {
        rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();
        return false;
    }

    byte buffer[18]; byte bufferSize = sizeof(buffer);
    status = rfid.MIFARE_Read(RFID_DATA_BLOCK, buffer, &bufferSize);
    rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();

    if (status != MFRC522::STATUS_OK) return false;
    char wp = (char)buffer[0];
    if (wp < 'A' || wp > 'L') return false;

    *waypoint = wp;
    return true;
}


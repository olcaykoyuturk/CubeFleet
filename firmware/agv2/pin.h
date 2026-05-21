#pragma once

// =============================================================================
// pin.h — AGV_2 (FARKLI ESP32 KARTI — pin haritası kontrol edilmeli)
//
// ⚠ ESP32-S3 / ESP32-C3 / ESP32-S2 kullanılıyorsa bu pinler ÇALIŞMAZ —
// onlarda GPIO numaraları farklıdır. Klasik ESP32 (WROOM-32 / WROVER /
// NodeMCU) için bu pinler doğrudur.
//
// Kart kontrol etmek için Arduino Serial Monitor'de boot log'una bak:
//   "Chip: ESP32-D0WDQ6 rev 1" → klasik ESP32, pinler OK
//   "Chip: ESP32-S3FH4R2"      → ESP32-S3, pinler revize edilmeli
//   "Chip: ESP32-C3FN4"        → ESP32-C3, sadece 22 GPIO var, pinler revize
// =============================================================================

// ===== MUX Pinleri (74HC4051) =====
// AGV_2'de GPIO 5 SPI library default SS'idir; SPI.begin sonrasında her
// transferde MUX_S2'yi bozuyor. Pin GPIO 27'ye taşındı (boş, strapping
// değil, hiçbir modülle çakışmaz).
#define MUX_S0    16
#define MUX_S1    17
#define MUX_S2    27   // ⚠ AGV_1: GPIO 5 → AGV_2: GPIO 27 (SPI çakışması yüzünden)
#define MUX_SIG   34   // Analog giriş (GPIO34 input-only)

// ===== Motor Pinleri (ZK-BM1 10A Sürücü) =====
#define MOTOR_L1  32   // Sol IN1
#define MOTOR_L2  33   // Sol IN2
#define MOTOR_R1  25   // Sağ IN1
#define MOTOR_R2  26   // Sağ IN2

// ===== PWM Ayarları =====
#define PWM_FREQ       2000   // ZK-BM1 max ~2kHz
#define PWM_RESOLUTION    8   // 8-bit: 0-255

// ===== HC-SR04 Ultrasonik Mesafe Sensörü =====
#define SONAR_TRIG   23
#define SONAR_ECHO   13

// ===== RC522 RFID Okuyucu (SPI — HSPI) =====
// RST pinini 3.3V'a bağlayın, GPIO gerekmez.
// Kod: MFRC522 rfid(RC522_SS_PIN, UINT8_MAX);
// DİKKAT: RC522'yi 3.3V ile besleyin, 5V yakar!
#define RC522_SS_PIN    15   // CS/SDA — strapping, boot'ta HIGH = güvenli
#define RC522_SCK_PIN   18   // SPI saat
#define RC522_MOSI_PIN  19   // SPI MOSI
#define RC522_MISO_PIN  35   // SPI MISO (GPIO35 input-only — MISO için ideal)

// ===== I2C Pinleri (OLED) =====
#define I2C_SDA   21
#define I2C_SCL   22
#define OLED_ADDR 0x3C   // Tipik adres; bazı modüllerde 0x3D

// NOT: Servolar ve elektromıknatıs ESP32-CAM'de (firmware/camera/) —
// AGV üzerinde robot kol yok. Pin tanımları buradan kaldırıldı.
// AYRICA: Eski SERVO_1_PIN=27 idi → MUX_S2=27 ile çakışıyordu (kod yazılmamıştı
// neyse ki). Şimdi GPIO 27 sadece MUX_S2 olarak kullanılıyor.

// ===== Sensör Sayısı =====
#define SENSOR_COUNT   8   // QTR-8A fiziksel sensör adedi

// ===== LED =====
#define LED_PIN   2

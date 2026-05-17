#pragma once

// =============================================================================
// pin.h — Sadece donanım pin tanımları ve sabit donanım parametreleri
// Davranış sabitleri için config.h'a bakın.
// Tam bağlantı tablosu ve güç dağılımı için pins.txt'e bakın.
// =============================================================================

// ===== MUX Pinleri (74HC4051) =====
#define MUX_S0    16
#define MUX_S1    17
#define MUX_S2     5   // Strapping pin — boot sonrası güvenli
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

// ===== Servo Motorlar (3x Tower Pro MG90S, 180°) =====
// DİKKAT: GPIO12 strapping pin (MTDI). Boot sırasında HIGH olursa sorun çıkar.
#define SERVO_1_PIN   27
#define SERVO_2_PIN   14
#define SERVO_3_PIN   12   // Strapping — boot'ta LOW kalmalı

// ===== Elektromıknatıs (P16/25 5V, ~500mA) =====
// GPIO doğrudan süremez. IRLZ44N MOSFET + flyback diyot kullan.
#define MAGNET_PIN    4

// ===== Sensör Sayısı =====
#define SENSOR_COUNT   8   // QTR-8A fiziksel sensör adedi

// ===== LED =====
#define LED_PIN   2

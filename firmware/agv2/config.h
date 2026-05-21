#pragma once

// =============================================================================
// config.h — Tüm ayarlanabilir sabitler tek yerde
// Donanım pinleri için pin.h'a bakın.
// =============================================================================

// ===== Navigasyon / Dönüş =====
// Ağır araç (1 kg) statik sürtünmeyi yenmek için 22 PWM gerek
#define TURN_SPEED                25   // Dönüş başlangıç PWM hızı (statik sürtünmeyi yenmek için)
#define TURN_SPEED_SLOW           20   // Çizgi yaklaşınca yavaşlama hızı (hassas hizalama)
#define TURN_TIMEOUT            5000   // Maksimum dönüş süresi (ms) — ağır araç biraz uzun döner

// ===== Debounce =====
#define LINE_EXIT_MS   200   // Çizgiden çıkış: bu kadar ms çizgi görünmemeli
#define LINE_FIND_MS    25   // Çizgi bulundu: bu kadar ms kesintisiz görünmeli
                             // Düşük TURN_SPEED ile sensör çizgide kalır, false positive'i azaltır

// ===== Kavşak Tespiti =====
#define JUNCTION_MIN_SENSORS   4   // Bu kadar veya daha fazla sensör aktifse kavşak
#define MID_LEFT_SENSOR        3   // Dönüş kontrolü: sol orta sensör indeksi
#define MID_RIGHT_SENSOR       4   // Dönüş kontrolü: sağ orta sensör indeksi

// ===== Çizgi Algılama =====
#define LINE_THRESHOLD   600   // Kalibre sensör eşiği (0-1000 arası)

// ===== WebSocket Zamanlaması =====
#define STATUS_INTERVAL   500   // Durum paketi gönderme aralığı (ms)
#define PING_INTERVAL    5000   // Canlı kalma ping aralığı (ms)

// ===== Varsayılan PID Değerleri =====
#define PID_DEFAULT_KP   0.008f
#define PID_DEFAULT_KI   0.000f
#define PID_DEFAULT_KD   0.003f

// ===== Varsayılan Hız =====
#define DEFAULT_BASE_SPEED   30   // Temel motor PWM hızı

// ===== Motor Trim (Düz Gidiş Dengesi) =====
// Araç sağa kayıyorsa: pozitif artır (+2, +4...)
// Araç sola  kayıyorsa: negatif azalt (-2, -4...)
#define MOTOR_TRIM   0

// ===== Kalibrasyon =====
#define CALIBRATION_TIME         8000   // Toplam kalibrasyon süresi (ms)
#define CALIBRATION_MOTOR_SPEED   120   // Kalibrasyon sırasında motor hızı
#define CALIBRATION_TURN_INTERVAL 400   // Sağ-sol yön değiştirme aralığı (ms)
#define CALIB_MIN_RANGE           500   // Sensör başına minimum kabul edilebilir range (ham ADC)

// ===== HC-SR04 Engel Tespiti =====
// Iki seviyeli yaklasim: SLOW zone'da hiz dustur, STOP zone'da tamamen dur.
// Boylece engele yumusak iner, ezici fren yok. Multi-AGV koridorunda da
// yan AGV'ye yaklasinca asagi geciyoruz.
#define OBSTACLE_STOP_CM      10    // Bu cm altinda tam dur (eski OBSTACLE_DISTANCE_CM)
#define OBSTACLE_SLOW_CM      20    // Bu cm altinda yavasla (slow zone)
#define OBSTACLE_SLOW_PCT     40    // Slow zone'da baseSpeed yuzdesi (40 = %40 hiz)
#define SONAR_INTERVAL       100    // Sonar olcum araligi (ms)

// ===== Kavşak Davranışı =====
// Kartlar dönüş köşesine yerleştirilmiştir; RFID okunduğu an AGV kavşaktadır.
// İleri gitme adımı YOK; doğrudan executeTurn çağrılır.
// (Eski JUNCTION_REACH_TIMEOUT / JUNCTION_CONFIRM_MS / RFID_FORWARD_MS kaldırıldı)

// ===== Çizgi Kayıp =====
#define LINE_LOST_MS    800    // Bu kadar ms çizgi görünmezse aksiyon al
#define LINE_SEARCH_MS 1200    // Çizgi arama süresi (ms)

// ===== Yol Bulma =====
#define MAX_PATH_LENGTH  20    // BFS yolunun maksimum uzunluğu

// ===== Debug =====
#define DEBUG_INTERVAL   100   // Serial debug çıktı aralığı (ms)

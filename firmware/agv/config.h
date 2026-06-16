#pragma once

// =============================================================================
// config.h — Tüm ayarlanabilir sabitler tek yerde
// Donanım pinleri için pin.h'a bakın.
// =============================================================================

// ===== Navigasyon / Dönüş =====
// Dönüş 45'te başlar, çizgiye yaklaşınca 35'e iner (hassas hizalama).
#define TURN_SPEED                45   // Dönüş başlangıç PWM hızı
#define TURN_SPEED_SLOW           35   // Çizgi yaklaşınca yavaşlama hızı
// Yük alındığı node'daki İLK dönüş — kalkış için en güçlü.
#define BOOST_TURN_SPEED          50   // ilk dönüş başlangıç hızı
#define BOOST_TURN_SPEED_SLOW     40   // ilk dönüş yavaşlama hızı
// Yük taşırken (ilk dönüşten sonra, bırakana kadar) tüm dönüşler: 50 → 38.
#define CARRY_TURN_SPEED          50   // taşıma dönüşü başlangıç
#define CARRY_TURN_SPEED_SLOW     38   // taşıma dönüşü yavaşlama
#define TURN_TIMEOUT            5000   // Maksimum dönüş süresi (ms) — ağır araç biraz uzun döner

// ===== Zamanlı Dönüş (faceDir — küp yönüne, çizgisiz 90°) =====
// Çizgi OLMAYAN yöne (küp tarafı) dönerken motor SABİT bu süre kadar döner.
// (Öğrenilen-süre/EMA kaldırıldı — saha tercihi sabit 680 ms.)
#define TIMED_TURN90_MS 680

// ===== Debounce =====
#define LINE_EXIT_MS   200   // Çizgiden çıkış: bu kadar ms çizgi görünmemeli
#define LINE_FIND_MS    25   // Çizgi bulundu: bu kadar ms kesintisiz görünmeli
                             // (düşük TURN_SPEED ile sensör çizgide kalır, false positive azalır)

// ===== Kavşak Tespiti =====
#define JUNCTION_MIN_SENSORS   4   // Bu kadar veya daha fazla sensör aktifse kavşak
#define MID_LEFT_SENSOR        3   // Dönüş kontrolü: sol orta sensör indeksi
#define MID_RIGHT_SENSOR       4   // Dönüş kontrolü: sağ orta sensör indeksi

// ===== Çizgi Algılama =====
#define LINE_THRESHOLD   600   // Kalibre sensör eşiği (0-1000 arası)

// ===== WebSocket Zamanlaması =====
// 500 -> 1000: çoklu AGV'de status broadcast setHop forward'ini geciktiriyordu
// (AGV path sonunda komut bekliyordu). 1 sn UI/sensör için yeterli; pozisyon
// zaten hopComplete'ten geliyor.
#define STATUS_INTERVAL  1000   // Durum paketi gönderme aralığı (ms)
#define PING_INTERVAL    5000   // Canlı kalma ping aralığı (ms)

// ===== Varsayılan PID Değerleri =====
#define PID_DEFAULT_KP   0.010f
#define PID_DEFAULT_KI   0.000f
#define PID_DEFAULT_KD   0.003f

// ===== Varsayılan Hız =====
#define DEFAULT_BASE_SPEED   40   // Temel motor PWM hızı

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
// İki seviyeli: SLOW zone'da yavaşla, STOP zone'da tam dur. Engele yumuşak iner,
// çoklu AGV koridorunda yan araca yaklaşınca hız düşer.
#define OBSTACLE_STOP_CM      10    // Bu cm altında tam dur
#define OBSTACLE_SLOW_CM      20    // Bu cm altında yavaşla (slow zone)
#define OBSTACLE_SLOW_PCT     40    // Slow zone'da baseSpeed yüzdesi (40 = %40 hız)
#define SONAR_INTERVAL       100    // Sonar olcum araligi (ms)

// ===== Kavşak Davranışı =====
// Kartlar dönüş köşesine yerleştirilmiştir; RFID okunduğu an AGV kavşaktadır.
// İleri gitme adımı YOK; doğrudan executeTurn çağrılır.
// (Eski JUNCTION_REACH_TIMEOUT / JUNCTION_CONFIRM_MS / RFID_FORWARD_MS kaldırıldı)

// ===== Çizgi Kayıp =====
#define LINE_LOST_MS    800    // Bu kadar ms çizgi görünmezse aksiyon al
#define LINE_SEARCH_MS 1200    // Çizgi arama süresi (ms)

// ===== Yol Bulma =====
#define MAX_PATH_LENGTH  20    // setHop path buffer (from + next + after)

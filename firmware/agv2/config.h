#pragma once

// =============================================================================
// config.h — Tüm ayarlanabilir sabitler tek yerde
// Donanım pinleri için pin.h'a bakın.
// =============================================================================

// ===== Navigasyon / Dönüş =====
// ⚠ Sadece DÖNÜŞ hızları AGV1'in %22 ALTINDA = AGV1 × 0.78 (en yakın tama
// yuvarlı) → AGV2 dönüşlerde %22 yavaş. Base/düz hız AGV1 ile AYNI (aşağıda 40).
// Bilinçli per-araç fark (MUX_S2 gibi). AGV1 değişirse burayı yeniden ölçekle.
//   AGV1: dönüş 45→35, boost 50→40, yükte 50→38
#define TURN_SPEED                35   // 45 × 0.78
#define TURN_SPEED_SLOW           27   // 35 × 0.78 (çizgiye yaklaşınca)
// Yük alındığı node'daki İLK dönüş — kalkış için en güçlü.
#define BOOST_TURN_SPEED          39   // 50 × 0.78
#define BOOST_TURN_SPEED_SLOW     31   // 40 × 0.78
// Yük taşırken (ilk dönüşten sonra, bırakana kadar) tüm dönüşler.
#define CARRY_TURN_SPEED          39   // 50 × 0.78
#define CARRY_TURN_SPEED_SLOW     30   // 38 × 0.78
#define TURN_TIMEOUT            5000   // Maksimum dönüş süresi (ms) — ağır araç biraz uzun döner

// ===== Zamanlı Dönüş (faceDir — küp yönüne, çizgisiz 90°) =====
// Çizgi OLMAYAN yöne (küp tarafı) dönerken motor SABİT bu süre kadar döner.
// (Öğrenilen-süre/EMA kaldırıldı — saha tercihi sabit 680 ms.)
#define TIMED_TURN90_MS 680

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
// 500 -> 1000: multi-AGV'de status broadcast (server textAll, her AGV) setHop
// forward'ini geciktiriyordu (AGV path-sonu node'larinda komut bekleyip
// duruyordu). 1 sn UI/sensor icin yeterli; pozisyon zaten hopComplete'ten
// gelir. WS kuyrugunu hafifletip setHop gecikmesini dusurur.
#define STATUS_INTERVAL  1000   // Durum paketi gönderme aralığı (ms)
#define PING_INTERVAL    5000   // Canlı kalma ping aralığı (ms)

// ===== Varsayılan PID Değerleri =====
#define PID_DEFAULT_KP   0.010f
#define PID_DEFAULT_KI   0.000f
#define PID_DEFAULT_KD   0.003f

// ===== Varsayılan Hız =====
#define DEFAULT_BASE_SPEED   40   // AGV1 ile AYNI (düz hız ölçeklenmedi)

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
#define MAX_PATH_LENGTH  20    // setHop path buffer (from + next + after)

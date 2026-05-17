#pragma once

// =============================================================================
// pin.h - AI-Thinker ESP32-CAM kamera pinleri
// Standart referans: https://docs.ai-thinker.com/en/esp32-cam
// =============================================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26   // I2C SDA (kamera)
#define SIOC_GPIO_NUM     27   // I2C SCL (kamera)

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

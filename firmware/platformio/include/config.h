#pragma once

// ========================= USER / DEVICE SETTINGS =========================
// If you want local-only overrides, create `config.local.h` (not tracked).
#ifdef __has_include
  #if __has_include("config.local.h")
    #include "config.local.h"
  #endif
#endif

// E.164 format recommended, example: +14145551234
#ifndef GG_OWNER_NUMBER
  #define GG_OWNER_NUMBER "+10000000000"
#endif

#ifndef GG_DEVICE_NAME
  #define GG_DEVICE_NAME "GearGuard"
#endif

// ========================= PIN MAP =========================
// IMPORTANT: For deep-sleep wake via EXT1, pins should be RTC-capable.
// ESP32-S3 RTC IO typically includes 0..21 (confirm for your exact module).

#ifndef GG_PIN_MPU_INT
  #define GG_PIN_MPU_INT   14   // MPU6050 INT -> wake
#endif

#ifndef GG_PIN_BTN_ARM
  #define GG_PIN_BTN_ARM   15   // ARM button -> wake + hold
#endif
#ifndef GG_PIN_BTN_A
  #define GG_PIN_BTN_A     16
#endif
#ifndef GG_PIN_BTN_B
  #define GG_PIN_BTN_B     17
#endif
#ifndef GG_PIN_BTN_C
  #define GG_PIN_BTN_C     18
#endif
#ifndef GG_PIN_MODEM_RI
  #define GG_PIN_MODEM_RI  19   // Modem RI/RING -> wake (required for instant SMS disarm)
#endif
#ifndef GG_PIN_BTN_D
  #define GG_PIN_BTN_D     21
#endif

#ifndef GG_PIN_I2C_SCL
  #define GG_PIN_I2C_SCL    7
#endif
#ifndef GG_PIN_I2C_SDA
  #define GG_PIN_I2C_SDA    8
#endif

#ifndef GG_PIN_SIREN
  #define GG_PIN_SIREN     13   // Siren/piezo via transistor
#endif

#ifndef GG_PIN_NEOPIXEL
  #define GG_PIN_NEOPIXEL  47   // NeoPixel DIN (1 LED)
#endif

#ifndef GG_PIN_PHOTO_ADC
  #define GG_PIN_PHOTO_ADC  1   // LDR/photodiode analog input (awake-only)
#endif

// Modem UART pins (edit to match your board)
#ifndef GG_PIN_MODEM_RX
  #define GG_PIN_MODEM_RX   4   // ESP32 RX  <- modem TX
#endif
#ifndef GG_PIN_MODEM_TX
  #define GG_PIN_MODEM_TX   5   // ESP32 TX  -> modem RX
#endif
#ifndef GG_MODEM_BAUD
  #define GG_MODEM_BAUD 115200
#endif

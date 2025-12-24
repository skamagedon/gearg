# GearGuard (ESP32) — Codex-ready repo

This repository contains the firmware skeleton for **GearGuard**:
- ESP32-S3 firmware (dual-core FreeRTOS)
- Motion wake + tiered movement classification (bump → warn → carry-away)
- Siren/beep patterns (fully tunable)
- SMS control (ARM/DISARM/WHERE/SETCODE) via LTE modem
- GNSS location reply as a **Google Maps pin link** + raw coordinates

## What’s here

- `firmware/platformio/` — primary build (PlatformIO)
- `firmware/arduino/` — Arduino IDE sketch copy (same logic, single file)
- `docs/` — wiring, tuning, SMS commands, sleep/wake notes

## Quick start (PlatformIO)

1. Install VS Code + PlatformIO.
2. Open `firmware/platformio/` as the project.
3. Edit `include/config.h` to match your wiring and phone number.
4. Build / Upload.

## Quick start (Arduino IDE)

1. Open `firmware/arduino/GearGuardDualCore/GearGuardDualCore.ino`
2. Install library: **Adafruit NeoPixel**
3. Select an ESP32-S3 board and build/upload.
4. Edit the `// CONFIG` section near the top.

## Security / secrets

Do not commit real phone numbers or keys if this repo will be public.
If you want local-only settings, create `include/config.local.h` (ignored by git)
and override `GG_OWNER_NUMBER`, pins, etc.

## Instant SMS disarm while “sleeping”

For true *instant* SMS disarm while the ESP32 is in deep sleep:
- the modem must remain powered enough to receive SMS
- modem **RI/RING** should be wired to an ESP32 **RTC-capable** GPIO
- deep sleep wake uses EXT1 on RI + buttons + MPU INT

See `docs/SLEEP_SMS.md`.

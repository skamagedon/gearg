# Hardware / wiring

## Buttons (required for wake + code entry)

This firmware expects buttons wired for EXT1 wake:

- GPIO configured as `INPUT_PULLDOWN`
- Wire each button between **GPIO** and **3.3V**
- Idle = LOW, pressed = HIGH

Pins: see `firmware/platformio/include/config.h`.

## MPU6050 (required)

- I2C: SDA/SCL from `GG_PIN_I2C_SDA` / `GG_PIN_I2C_SCL`
- INT: MPU INT pin -> `GG_PIN_MPU_INT`

The code configures a motion interrupt so the MPU can wake the ESP32 from deep sleep.

## Modem (LTE + SMS + GNSS)

- UART: modem TX/RX -> ESP32 RX/TX pins in config
- RI/RING: modem RI (or RING) -> `GG_PIN_MODEM_RI` (RTC-capable GPIO)

### Why RI matters
If you want **instant SMS disarm** while the ESP32 is asleep, the modem must:
- stay powered enough to receive SMS
- assert RI/RING when a message arrives
- wake the ESP32 via EXT1

If RI is not wired, the device can still disarm by SMS only when it wakes for other reasons.

## Siren / piezo

Do not drive a real siren directly from a GPIO.
Use a transistor (and a diode if inductive load).

## Light tamper (optional)

Any light sensor that produces an analog voltage into `GG_PIN_PHOTO_ADC`.
The firmware only checks this while awake; it cannot wake the ESP32 from deep sleep.

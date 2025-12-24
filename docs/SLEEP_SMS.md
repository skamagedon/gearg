# Sleep + instant SMS disarm

## Requirements for instant SMS disarm
1) Modem stays powered and registered enough to receive SMS.
2) Modem asserts RI/RING on SMS arrival.
3) RI/RING is wired to an ESP32 RTC-capable pin and included in EXT1 wake mask.

If you fully cut modem power during sleep, it cannot receive SMS and cannot wake the ESP32.

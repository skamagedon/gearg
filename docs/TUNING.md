# Tuning (thresholds + beep patterns)

Open:
- `firmware/platformio/src/main.cpp`

Edit:
- `CFG` struct (motion thresholds, durations, siren sweep, timeouts)
- `PATTERN_*` arrays (beep frequency + on/off)

Recommended: adjust durations first (`warn_over_ms`, `carry_over_ms`), then peaks.

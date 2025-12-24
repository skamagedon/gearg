# SMS commands

All commands are accepted only from `GG_OWNER_NUMBER`.

Codes are 4 digits using only 1–4 (matches A/B/C/D keypad):
- A = 1
- B = 2
- C = 3
- D = 4

## Commands

- `DISARM 1234` -> replies `DISARMED`
- `ARM 1234` -> replies `ARMED`
- `WHERE 1234` -> replies Google Maps pin link + coordinates
- `SETCODE 1234 4321` -> updates code (old then new)

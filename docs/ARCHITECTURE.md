# Architecture

## Core 0 (Motion/UI task)
- reads buttons/keypad
- runs motion investigation window after motion wake
- controls NeoPixel + siren
- triggers events for comms (carry-away, GPS request)

## Core 1 (Comms task)
- owns the modem UART (AT commands)
- processes inbound unread SMS
- sends outbound SMS
- enables GNSS and retrieves a fix (then sends a maps link)

Rule: only the comms task touches the modem UART.

# ESP32-P4 GPS Tracker

GPS tracker project for the ESP32-P4 Function EV Board V1.4 with a SIM7600E-H module.

## Current status

Working:

- ESP32-P4 boots correctly
- LCD initializes correctly
- Backlight works
- Touch controller GT911 initializes
- SIM7600E-H responds over UART
- GNSS polling via `AT+CGPSINFO`
- GPS data is shown on the LCD when a fix is available
- Local track display area is prepared

## Hardware

### ESP32-P4 Function EV Board

Flash/monitor port:

```text
/dev/cu.SLAB_USBtoUART

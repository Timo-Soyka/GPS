# GPS

ESP32-P4 Function EV Board V1.4 + SIM7600E-H GPS/LTE tracker.

## Hardware

### SIM7600E-H UART

SIM7600 UART jumpers: **B**

| ESP32-P4 Function EV Board | SIM7600E-H |
|---|---|
| GPIO21 TX | RXD |
| GPIO22 RX | TXD |
| GND | GND |

Do not connect 5V or 3V3 between the boards if the SIM7600 module is powered separately.

### LCD

| LCD adapter | ESP32-P4 Function EV Board |
|---|---|
| PWM / Backlight | GPIO23 |
| RST_LCD | GPIO27 |

## Flash / Monitor Port

Use:

```bash
/dev/cu.SLAB_USBtoUART

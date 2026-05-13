# GPS

ESP32-P4 Function EV Board V1.4 + SIM7600E-H GPS/LTE test project.

## Current working setup

The SIM7600E-H is connected to the ESP32-P4 via UART pins.

## Wiring

SIM7600 UART jumpers: **B**

| ESP32-P4 | SIM7600E-H |
|---|---|
| GPIO7 TX | RXD |
| GPIO8 RX | TXD |
| GND | GND |

Do not connect 5V or 3V3 between the boards if the SIM7600 module is powered separately.

## Flash / Monitor port

Use:

```bash
/dev/cu.SLAB_USBtoUART

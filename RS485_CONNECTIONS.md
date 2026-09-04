# ESP32-S3 <-> XIAO ESP32-C3 RS485 bench link

This is a two-way UART-over-RS485 test link. The XIAO-C3 sends `PING` packets and the ESP32-S3 replies with `PONG` packets.

## MAX3485 wiring

Use one 3.3 V MAX3485 module at each board.

| XIAO ESP32-C3 | MAX3485 #1 |
|---|---|
| GPIO21 / D6 / TX | **RX** / RXO |
| GPIO20 / D7 / RX | **TX** / TXD |
| GPIO10 / D10 | EN |
| 3V3 | VCC |
| GND | GND |

| ESP32-S3 | MAX3485 #2 |
|---|---|
| GPIO17 (MCU TX) | **RX** / RXO |
| GPIO18 (MCU RX) | **TX** / TXD |
| GPIO16 | EN |
| 3V3 | VCC |
| GND | GND |

The XIAO D6/D7 mapping is GPIO21 TX and GPIO20 RX according to Seeed's pinout. [Seeed XIAO ESP32-C3 pinout](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/)

**Important module-label convention:** on this MAX3485 board, `RX`/`RXO` is the module's TTL input and connects to the MCU TX pin. `TX`/`TXD` is the module's TTL output and connects to the MCU RX pin. The labels describe the module signal direction, not the MCU pin to which they are connected.

## Twisted-pair wiring

```text
MAX3485 #1 A  ----------------  MAX3485 #2 A
MAX3485 #1 B  ----------------  MAX3485 #2 B
GND/reference ----------------- GND/reference
```

Use one twisted pair for A/B. Add 120 ohm termination at the two physical ends of the cable only. Do not cross A/B. Do not connect the two MCU TX/RX pins directly.

## Direction control

`EN` controls direction on this module. Connect the module `EN` pin to the listed MCU direction GPIO. It is active HIGH for transmit and LOW for receive:

```text
DE_RE = 0  -> receive
DE_RE = 1  -> transmit
```

The test files use the module's `EN` pin. If a different module has automatic direction control and no usable EN pin, set `USE_DIRECTION_PIN = False` in both files and leave that module's automatic-direction wiring unchanged.

## UART settings

```text
9600 baud, 8 data bits, no parity, 1 stop bit
```

The first test is intentionally slow and ASCII-based so it is easy to inspect. After the link works, replace the text packets with a binary packet containing sensor ID, sequence number, timestamp, payload, and CRC.

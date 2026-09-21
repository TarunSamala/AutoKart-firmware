# ESP32-S3 ↔ ESP8266MOD RA-02 communication test

For the initial test, use one RA-02 on each microcontroller. The LAY37
emergency-stop switch is not required for this communication-only test.

```text
ESP32-S3 ── SPI ── RA-02 ))) 433 MHz LoRa ((( RA-02 ── SPI ── ESP8266MOD
```

## ESP32-S3 radio

| RA-02 | ESP32-S3 |
|---|---|
| 3.3V | 3V3 |
| GND | GND |
| NSS/CS | GPIO16 |
| SCK | GPIO14 |
| MOSI | GPIO13 |
| MISO | GPIO15 |
| DIO0 | GPIO18 |
| RESET | GPIO10 |

## ESP8266MOD radio board

| RA-02 | ESP8266 board label |
|---|---|
| 3.3V | 3V |
| GND | G / GND |
| NSS/CS | D4 / GPIO2 |
| SCK | D5 / GPIO14 |
| MOSI | D7 / GPIO13 |
| MISO | D6 / GPIO12 |
| DIO0 | D2 / GPIO4 |
| RESET | D1 / GPIO5 |

Both radios must use the same frequency and LoRa parameters. Keep antennas
connected before transmitting. Use 3.3 V only, common ground, and local
decoupling at each RA-02 (`100 nF` plus `10–100 µF`).

The first firmware test should send a short `PING` packet from one board and
return `PONG` from the other. Add sequence number, timeout, and CRC before
using the link for a safety command.

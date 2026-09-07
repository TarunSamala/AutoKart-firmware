# RA-02 ↔ ESP8266MOD connection sheet

This pinout uses the ESP8266 hardware HSPI interface.

## RA-02 to ESP8266MOD

| RA-02 pin | ESP8266MOD pin | Function |
|---|---|---|
| 3.3V | Stable 3.3 V supply | Power |
| GND | GND | Common ground |
| MISO | **D6 / GPIO12** | SPI data from RA-02 |
| MOSI | **D7 / GPIO13** | SPI data to RA-02 |
| SCK | **D5 / GPIO14** | SPI clock |
| NSS / CS | **D4 / GPIO2** | Chip select, active LOW |
| RESET | **D1 / GPIO5** | RA-02 reset |
| DIO0 | **D2 / GPIO4** | IRQ / packet interrupt |

```text
ESP8266 D6/GPIO12  ───── RA-02 MISO
ESP8266 D7/GPIO13  ───── RA-02 MOSI
ESP8266 D5/GPIO14  ───── RA-02 SCK
ESP8266 D4/GPIO2   ───── RA-02 NSS/CS
ESP8266 D1/GPIO5   ───── RA-02 RESET
ESP8266 D2/GPIO4   ───── RA-02 DIO0
ESP8266 3.3 V   ───── RA-02 3.3V
ESP8266 GND     ───── RA-02 GND
```

## Power and safety

- Use 3.3 V only. Never connect the RA-02 to 5 V.
- Use a regulated supply capable of handling ESP8266 and RA-02 transmit
  current together. Do not rely on a weak USB-TTL 3.3 V output.
- Place `100 nF` and `10–100 µF` decoupling close to the RA-02.
- Keep the antenna connected before transmitting.
- Keep all signal grounds common.

## ESP8266 programming connection

This development board has USB and a 3V output. For a bare ESP8266MOD,
use a 3.3 V USB-to-TTL adapter:

| USB-to-TTL | ESP8266MOD |
|---|---|
| 3.3V | 3.3V / VCC |
| GND | GND |
| TX | RX / GPIO3 |
| RX | TX / GPIO1 |
| DTR/RTS, if available | Through an auto-reset circuit |

For manual flashing, hold `GPIO0` LOW while resetting or powering the
ESP8266. Release `GPIO0` after the bootloader starts. Keep `EN/CH_PD` HIGH
at 3.3 V. The USB serial port will commonly be `/dev/ttyUSB0`.

## SPI settings

```text
Bus: HSPI
SCK: GPIO14
MOSI: GPIO13
MISO: GPIO12
CS: GPIO2
SPI mode: 0
Logic level: 3.3 V
```

GPIO2 is used for CS because GPIO15 is an ESP8266 boot-strap pin. Keep the
RA-02 CS line HIGH when idle after the ESP8266 has started.

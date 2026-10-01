# XIAO ESP32-C3 + RA-02 + LAY37 E-stop transmitter

## RA-02 wiring

| RA-02 pin | XIAO label | ESP32-C3 GPIO |
|---|---:|---:|
| VCC / 3.3V | 3V3 | — |
| GND | GND | — |
| SCK | D8 | GPIO8 |
| MISO | D9 | GPIO9 |
| MOSI | D10 | GPIO10 |
| NSS / CS | D3 | GPIO5 |
| RESET | D2 | GPIO4 |
| DIO0 | D1 | GPIO3 |

Add a 10 kΩ pull-up from RA-02 `NSS/CS` to 3.3 V so the radio remains
deselected while the ESP32-C3 is booting. Connect a 433 MHz antenna before
transmitting.

Power the RA-02 from 3.3 V only. Place a 100 nF ceramic capacitor and a
10–100 µF bulk capacitor directly across the RA-02 VCC and GND pins.

## LAY37 normally-closed E-stop wiring

```text
XIAO 3V3 ── 4.7 kΩ ──┐
                      ├── XIAO D4 / GPIO6 ── LAY37 NC
XIAO GND ──────────────────────────────── LAY37 COM
```

Leave the LAY37 `NO` contact unused.

| Condition | GPIO6 | Packet state |
|---|---:|---:|
| Released, NC closed | LOW | `ESTOP,0,sequence` |
| Pressed, NC open | HIGH | `ESTOP,1,sequence` |
| Broken/disconnected switch wire | HIGH | `ESTOP,1,sequence` |

The external 4.7 kΩ resistor is recommended even though the firmware also
enables the ESP32-C3 internal pull-up.

## Firmware

Upload both files to the XIAO:

```bash
python3 -m mpremote connect /dev/ttyACM0 fs cp \
  bench/ra02_lora_chat_common.py :ra02_lora_chat_common.py

python3 -m mpremote connect /dev/ttyACM0 fs cp \
  bench/xiao_esp32c3_ra02_estop_transmitter.py :main.py

python3 -m mpremote connect /dev/ttyACM0 soft-reset
python3 -m mpremote connect /dev/ttyACM0 repl
```

Rediscover the port before flashing. Do not assume it will always be
`/dev/ttyACM0`, especially when the ESP32-S3 is connected at the same time.

## Packet behavior

The transmitter sends its actual state five times at startup, sends each
state transition five times, and repeats STOP while the switch remains
pressed or its NC wire remains open. It does not continuously transmit clear
packets.

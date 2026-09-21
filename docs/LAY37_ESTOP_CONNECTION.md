# LAY37 emergency-stop switch connection

The LAY37 is a normally-closed emergency-stop contact. Use `COM` and `NC`;
leave `NO` unused.

## Remote controller: ESP8266MOD board

```text
ESP8266 D0 / GPIO16 ───── LAY37 NC
ESP8266 G / GND     ───── LAY37 COM
ESP8266 D0 / GPIO16 ───── 4.7 kΩ ───── ESP8266 3V
```

```text
                 3V
                  |
                4.7 kΩ
                  |
ESP8266 D0/GPIO16+──── LAY37 NC
ESP8266 GND ─────────── LAY37 COM
```

Electrical state:

| Condition | GPIO16 | Meaning |
|---|---:|---|
| Switch released, NC closed | LOW | Healthy / permit operation |
| Switch pressed, NC open | HIGH | STOP |
| Wire disconnected | HIGH | STOP / fail-safe |

Configure GPIO16 as an input with a pull-up. GPIO16 is suitable for a
polling-based input; the firmware must treat anything other than a confirmed
LOW as STOP. Do not connect `NC` directly to ground without using the
LAY37 `COM` terminal.

## Optional vehicle-side input: ESP32-S3

If the LAY37 is physically located beside the ESP32-S3 instead, use an unused
input such as GPIO22:

```text
ESP32-S3 GPIO22 ───── LAY37 NC
ESP32-S3 GND    ───── LAY37 COM
GPIO22 ────────────── 4.7 kΩ ───── 3V3
```

Do not connect the same switch signal to both microcontrollers over a long
wire. The remote ESP8266 should send the emergency-stop state over LoRa, and
the ESP32-S3 should stop on STOP, missing heartbeat, invalid CRC, or timeout.

This is a signal-level bench connection only. Keep motors disabled and wheels
off the ground until the radio timeout and stop behavior are verified.

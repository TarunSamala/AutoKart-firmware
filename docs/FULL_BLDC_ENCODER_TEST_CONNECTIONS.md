# Full BLD750 + AS5047P encoder bench test

Firmware: `bench/full_bldc_encoder_test.py`

> Wheels off the ground, kart restrained, chain guarded, and an independent
> power disconnect within reach. The ESP32 emergency button is not an
> independent safety stop.

## ESP32-S3 to GP8630N

| GP8630N | ESP32-S3 / destination |
|---|---|
| `3.3V` | `3V3` |
| `GND` | ESP32 `GND` and BLD750 `COM` |
| `SDA` | GPIO7 — separate `I2C(1)` bus |
| `SCL` | GPIO21 — separate `I2C(1)` bus |
| `VOUT` | BLD750 `SV` |

GPIO1/GPIO2 remain available for the PCA9548A steering/brake encoder bus.
GPIO7 is used as I2C SDA here, so the unused AS5047P PWM output must remain
disconnected.

Do not use the GP8630N `VIN` pin as its signal output. The firmware selects
the 0–10 V output mode but limits the requested output to 5.0 V.

## ESP32-S3 to AS5047P using SPI

| AS5047P board | ESP32-S3 |
|---|---|
| `3.3V` | `3V3` |
| `GND` | `GND` |
| `CS` | GPIO16 |
| `CLK` | GPIO18 |
| `MOSI` | GPIO41 |
| `MISO` | GPIO42 |
| `5V` | Not connected |
| `U`, `V`, `W/PWM`, `I/PWM` | Not connected |

Mount the diametrically magnetized magnet concentrically on the 14-tooth BLDC
sprocket and place the stationary AS5047P sensor directly opposite it. Confirm
`MAGNET OK` through a complete manual revolution before enabling traction.

## ESP32-S3 to BLD750 through PN2222 stages

Do **not** connect a BLD750 digital input directly to an ESP32 GPIO.

| Function | ESP32 GPIO | PN2222 collector | BLD750 input |
|---|---:|---|---|
| Brake | GPIO13 | BRK signal | `BRK` |
| Enable | GPIO14 | EN signal | `EN` |
| Direction | GPIO15 | F/R signal | `F/R` |

For each channel:

```text
ESP32 GPIO ── 1 kΩ to 2.2 kΩ ── PN2222 base
ESP32 GND  ───────────────────── PN2222 emitter
PN2222 collector ─────────────── BLD750 input (BRK, EN or F/R)
10 kΩ base-to-emitter resistor keeps the transistor OFF at reset
BLD750 COM ───────────────────── common control ground
```

Tested firmware logic with the transistor stages:

| ESP32 output | Transistor | BLD input | Result |
|---|---|---|---|
| EN GPIO14 HIGH | ON | LOW | RUN |
| EN GPIO14 LOW | OFF | open/high | STOP |
| BRK GPIO13 HIGH | ON | LOW | brake released |
| BRK GPIO13 LOW | OFF | open/high | quick brake |
| F/R GPIO15 HIGH | ON | LOW | CCW / forward |
| F/R GPIO15 LOW | OFF | open/high | CW / reverse |

## Flash and run

```bash
cd /home/satvara/Github/AutoKart-firmware
PORT=/dev/ttyACM0

/home/satvara/python/bin/python3 -m mpremote connect "$PORT" \
  fs cp bench/full_bldc_encoder_test.py :main.py

/home/satvara/python/bin/python3 -m mpremote connect "$PORT" soft-reset
/home/satvara/python/bin/python3 -m mpremote connect "$PORT" repl
```

Connect a phone/laptop to:

```text
SSID: BLD750-Encoder-Test
Password: 12345678
URL: http://192.168.4.1
```

## Safe test sequence

1. Keep traction power isolated and rotate the sprocket manually.
2. Confirm `MAGNET OK`, correct angle movement and no increasing error count.
3. Press **ZERO ENCODER + DISTANCE** while stopped and braked.
4. Energize the traction system with the driven wheels raised and guarded.
5. Select `CCW / FORWARD` or `CW / REVERSE` while stopped.
6. Press **RUN**. It starts at 0 V.
7. Increase the slider in small steps: 0.5 V, 1.0 V, 1.5 V, and onward.
8. Compare motor RPM, axle RPM, turns and direction with physical motion.
9. Press **STOP + BRAKE** before changing direction.

Distance and speed are nominal values calculated from the 14T-to-44T
reduction and nominal 11-inch tyre diameter. They require loaded rolling-
circumference calibration before vehicle use.

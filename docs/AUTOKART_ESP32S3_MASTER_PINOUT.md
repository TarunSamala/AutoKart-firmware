# AutoKart final master pinout

Revision: E — 23 October 2026

Controller: YD ESP32-S3 N8R2, 44-pin terminal adapter

Status: final conflict-free pin allocation; physical integrated validation is
still required before vehicle operation.

This revision supersedes every isolated bench-test pinout where they differ.
It also freezes these choices:

- R86mini + NEMA34: steering actuator.
- TB6600 + NEMA23: brake actuator.
- BLD-750 + GP8630N: traction actuator.
- AS5600 #1: steering feedback.
- AS5600 #2: brake-position feedback.
- AS5047P over SPI: BLDC 14T sprocket encoder.
- Pico 2 + RA-02 + LAY37: remote E-stop transmitter.
- ESP32-S3 + RA-02: vehicle-side receiver.
- Raspberry Pi 4B ↔ ESP32-S3: native USB target for micro-ROS.

Any later GPIO change requires a new document revision, matching firmware
changes, a continuity check, and subsystem revalidation. Do not silently reuse
a frozen pin in an isolated test.

## 1. ESP32-S3 GPIO allocation

| GPIO | Final function | Direction | Connection/status |
|---:|---|---|---|
| 0 | Reserved | — | Boot strap; do not use |
| 1 | Encoder I2C(0) SDA | Bidirectional | PCA9548A upstream SDA only |
| 2 | Encoder I2C(0) SCL | Open-drain output | PCA9548A upstream SCL only |
| 3 | Reserved | — | Boot strap; do not use |
| 4 | Steering STEP/PUL | Output | R86mini opto interface |
| 5 | Steering DIR | Output | R86mini opto interface |
| 6 | Steering ENA | Output | R86mini opto interface; optional but allocated |
| 7 | DAC I2C(1) SDA | Bidirectional | GP8630N SDA; AS5047P PWM remains disconnected |
| 8 | LoRa RESET | Output | ESP32-S3 RA-02 RESET |
| 9 | LoRa NSS/CS | Output | ESP32-S3 RA-02 NSS |
| 10 | LoRa MISO | Input | ESP32-S3 RA-02 MISO |
| 11 | LoRa MOSI | Output | ESP32-S3 RA-02 MOSI |
| 12 | LoRa SCK | Output | ESP32-S3 RA-02 SCK |
| 13 | BLD-750 BRK | Output | Through PN2222 transistor stage |
| 14 | BLD-750 EN | Output | Through PN2222 transistor stage |
| 15 | BLD-750 F/R | Output | Through PN2222 transistor stage; never use for E-stop |
| 16 | AS5047P CSn | Output | BLDC sprocket encoder SPI chip select |
| 17 | LoRa DIO0 | Input | ESP32-S3 RA-02 DIO0 |
| 18 | AS5047P SCK | Output | SPI mode 1 clock |
| 19 | USB D− | USB | Reserved for native ESP32-S3 USB |
| 20 | USB D+ | USB | Reserved for native ESP32-S3 USB |
| 21 | DAC I2C(1) SCL | Open-drain output | GP8630N SCL |
| 38 | Brake STEP/PUL | Output | TB6600 opto interface |
| 39 | Brake DIR | Output | TB6600 opto interface |
| 40 | Brake ENA | Output | TB6600 opto interface |
| 41 | AS5047P MOSI | Output | SPI mode 1 MOSI |
| 42 | AS5047P MISO | Input | SPI mode 1 MISO, 3.3 V maximum |
| 45 | Reserved | — | Boot strap; do not use |
| 46 | Reserved | — | Boot strap; do not use |
| 47 | LoRa E-stop request | Output | To approved transistor/opto safety-interface input; not directly to a relay coil |
| 48 | Status LED | Output | Optional board/status LED only |

GPIO26–GPIO37 are intentionally unused because they may be occupied by
flash/PSRAM signals on ESP32-S3 variants. GPIO7 and GPIO21 are frozen for the
dedicated GP8630N bus.

## 2. Power and grounding rules

| Rail | Use |
|---|---|
| ESP32-S3 USB/5 V input | ESP32-S3 board power |
| ESP32-S3 3V3 | PCA9548A, both AS5600s, AS5047P 3.3 V input, ESP32-side RA-02 and GP8630N |
| Regulated 5 V control supply | R86mini/TB6600 opto-input `+` terminals where required |
| Driver/motor supplies | R86mini, TB6600 and BLD-750 power stages only |

- Never apply 5 V or motor-supply voltage to an ESP32 GPIO.
- Use the GP8630N `GND` output reference and BLD-750 `COM` as required by the
  verified control interface.
- R86mini and TB6600 opto inputs must use the tested interface, not direct
  5 V connection to ESP32 pins.
- Motor-current wiring must remain physically separated from encoder, I2C,
  SPI and USB wiring.
- Fit local decoupling at every RA-02: 100 nF ceramic plus 10–100 µF bulk.
- Fit the correct 433 MHz antenna before transmitting.

## 3. Frozen dual-I2C allocation

The encoder mux and traction DAC use separate ESP32-S3 I2C controllers. Do not
join these two SDA/SCL pairs.

| Bus | ESP32-S3 SDA | ESP32-S3 SCL | Device |
|---|---:|---:|---|
| I2C(0) | GPIO1 | GPIO2 | PCA9548A `SDA`/`SCL` |
| I2C(1) | GPIO7 | GPIO21 | GP8630N `SDA`/`SCL` |

Both devices use ESP32-S3 `3V3` and common `GND`. The GP8630N output ground
also references BLD-750 `COM` according to the verified control interface.

PCA9548A control pins:

| PCA9548A | Connection |
|---|---|
| RST | ESP32-S3 3V3; do **not** connect to `3V3_EN` |
| A0 | GND |
| A1 | GND |
| A2 | GND |

Expected addresses on the separate buses:

- I2C(0), GPIO1/GPIO2: PCA9548A `0x70`
- I2C(1), GPIO7/GPIO21: GP8630N `0x58` with default address switches

The GP8630N supports a 3.3–5 V operating supply; this design selects 3.3 V so
the I2C side remains in the ESP32-S3 logic domain. Its output mode/register
configuration and voltage accuracy must be corrected and revalidated because
the previous 5 V request produced only about 3.3 V.

### Steering AS5600 — PCA channel 0

| AS5600 | Connection |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | PCA SD0 |
| SCL | PCA SC0 |
| OUT | Not connected |
| DIR | Leave at module default unless direction is deliberately configured |

Expected address after selecting channel 0: `0x36`.

### Brake AS5600 — PCA channel 1

| AS5600 | Connection |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | PCA SD1 |
| SCL | PCA SC1 |
| OUT | Not connected |
| DIR | Leave at module default unless direction is deliberately configured |

Expected address after selecting channel 1: `0x36`. Enable only one PCA
channel at a time because both encoders use the same fixed address.

### GP8630N output to BLD-750

| GP8630N | BLD-750 |
|---|---|
| OUT | SV |
| Output GND | COM |

Do not treat I2C acknowledgement as proof of correct analog voltage.

## 4. AS5047P BLDC sprocket encoder — SPI only

The magnet is centred on the 14-tooth BLDC sprocket. The stationary AS5047P
faces the magnet perpendicular to the sprocket face.

| Meukron AS5047P | ESP32-S3 |
|---|---|
| 3.3V input | 3V3 |
| GND | GND |
| CS | GPIO16 |
| CLK | GPIO18 |
| MOSI | GPIO41 |
| MISO | GPIO42 |
| W/PWM | Not connected |
| I/PWM | Not connected |
| A/B/I outputs | Not connected |

Use SPI mode 1. Do not connect both the module's 5 V and 3.3 V supply inputs.
PWM testing failed and is not part of the final interface. The complete
14T/44T drivetrain calibration is documented in
[`BLDC_AS5047P_DRIVETRAIN_CALIBRATION.md`](BLDC_AS5047P_DRIVETRAIN_CALIBRATION.md).

## 5. R86mini and NEMA34 steering

Use the tested common-anode/sinking interface:

| R86mini input | Connection |
|---|---|
| PUL+ | Regulated 5 V control supply |
| PUL− | Transistor/opto sink controlled by ESP32 GPIO4 |
| DIR+ | Regulated 5 V control supply |
| DIR− | Transistor/opto sink controlled by ESP32 GPIO5 |
| ENA+ | Regulated 5 V control supply |
| ENA− | Transistor/opto sink controlled by ESP32 GPIO6; optional |

NEMA34 winding groups established during bench testing:

| R86mini motor terminal | Motor wires |
|---|---|
| A+ | Red + Blue |
| A− | Yellow + Black |
| B+ | Orange + Green |
| B− | Brown + White |

Driver power and motor-current settings must follow the motor/driver ratings.
Do not infer a safe current or microstep setting from this signal pinout.

## 6. TB6600 and NEMA23 brake

Use the same common-anode/sinking principle:

| TB6600 input | Connection |
|---|---|
| PUL+ | Regulated 5 V control supply |
| PUL− | Transistor/opto sink controlled by ESP32 GPIO38 |
| DIR+ | Regulated 5 V control supply |
| DIR− | Transistor/opto sink controlled by ESP32 GPIO39 |
| ENA+ | Regulated 5 V control supply |
| ENA− | Transistor/opto sink controlled by ESP32 GPIO40 |

Use the verified NEMA23 winding pairs for the installed brake motor. Confirm
direction and brake fail-safe behavior before attaching the final linkage.

## 7. BLD-750 digital controls

Each BLD-750 digital input uses its own PN2222 sinking stage:

```text
ESP32 GPIO ── base resistor ── PN2222 base
PN2222 emitter ─────────────── BLD-750 COM
PN2222 collector ───────────── BLD-750 control input
```

| BLD-750 | ESP32 control |
|---|---|
| BRK | GPIO13 through PN2222 |
| EN | GPIO14 through PN2222 |
| F/R | GPIO15 through PN2222 |
| SV | GP8630N OUT |
| COM | GP8630N output GND / transistor emitters |

The transistor inversion and BLD-750 terminal voltages must be captured in a
physical truth table. GPIO15 is exclusively F/R and must not be reused.

## 8. ESP32-S3 RA-02 receiver

| RA-02 | ESP32-S3 |
|---|---|
| VCC | 3V3 only |
| GND | GND |
| RESET | GPIO8 |
| NSS/CS | GPIO9 |
| MISO | GPIO10 |
| MOSI | GPIO11 |
| SCK | GPIO12 |
| DIO0 | GPIO17 |

Expected version register: `0x12`. Current working radio profile is 433 MHz,
SF7, BW125 and hardware CRC enabled.

GPIO47 is allocated as the ESP32 LoRa **stop-request output** to a separately
approved transistor/opto input. It must not drive a contactor/relay coil
directly. An ESP32-generated signal alone is not an independent safety cut;
the separate traction-isolation and fail-safe braking architecture still
requires validation.

## 9. Pico 2 RA-02 transmitter and LAY37

| RA-02 / LAY37 | Pico 2 |
|---|---|
| RA-02 VCC | 3V3(OUT), physical pin 36 |
| RA-02 GND | GND, physical pin 38 or another GND |
| RA-02 MISO | GP16, physical pin 21 |
| RA-02 NSS/CS | GP17, physical pin 22 |
| RA-02 SCK | GP18, physical pin 24 |
| RA-02 MOSI | GP19, physical pin 25 |
| RA-02 RESET | GP20, physical pin 26 |
| RA-02 DIO0 | GP21, physical pin 27 |
| LAY37 NC | GP22, physical pin 29 |
| LAY37 COM | GND |
| 4.7 kΩ pull-up | GP22 to 3V3(OUT) |

Healthy NC-closed reads LOW. Pressed, open contact or broken switch wire reads
HIGH/STOP. The transmitter selection is now Pico 2; XIAO ESP32-C3 is legacy.

## 10. Raspberry Pi 4B connections

### Pi ↔ ESP32-S3

Connect a Raspberry Pi USB host port to the ESP32-S3 native USB connector with
a data-capable USB cable. Do not add a separate UART link in the final pinout.
GPIO19/20 are reserved internally for native USB D−/D+.

Target production transport: native USB micro-ROS. The current serial REPL is
bench-proven, but micro-ROS remains to be implemented and validated.

### MPU-9250/6500 on Raspberry Pi I2C1

| MPU-9250/6500 | Raspberry Pi 4B |
|---|---|
| VCC | 3V3, physical pin 1 |
| GND | GND, physical pin 6 |
| SDA | GPIO2/SDA1, physical pin 3 |
| SCL | GPIO3/SCL1, physical pin 5 |
| ADD/AD0 | GND, physical pin 9; address `0x68` |
| INT | Not connected for the current polling test |

## 11. Final bus map

```text
ESP32-S3 I2C(0): SDA1 / SCL2 → PCA9548A 0x70
  ├─ channel 0 → steering AS5600 0x36
  └─ channel 1 → brake AS5600 0x36

ESP32-S3 I2C(1): SDA7 / SCL21 → GP8630N 0x58
  └─ OUT/GND → BLD-750 SV/COM

ESP32-S3 SPI, AS5047P: CS16 / SCK18 / MOSI41 / MISO42
ESP32-S3 SPI, RA-02:   CS9 / SCK12 / MOSI11 / MISO10 / RST8 / DIO0 17

Steering R86mini: STEP4 / DIR5 / ENA6
Brake TB6600:      STEP38 / DIR39 / ENA40
BLD-750:           BRK13 / EN14 / F-R15 / SV from GP8630N
LoRa stop request: GPIO47 → approved interface only
Pi link:           native USB, GPIO19/20 reserved
```

## 12. Mandatory checks before power

1. Continuity-check every final terminal against this document.
2. Confirm no shorts to adjacent GPIO, 3V3, 5V or GND.
3. Confirm PCA `0x70` on GPIO1/2, GP8630N `0x58` on GPIO7/21, each
   channel-isolated AS5600 `0x36`, RA-02 `0x12` and valid AS5047P SPI frames.
4. Verify GP8630N output voltage before connecting it to BLD-750 SV.
5. Measure the PN2222/BLD-750 truth table and all driver input polarities.
6. Confirm safe states during reset, boot, USB loss and control-power loss.
7. Keep the vehicle raised and restrained for the first powered actuator test.
8. Do not authorize autonomous operation until the eight validation tests and
   independent stopping hardware have passed.

## References

- [AutoKart eight remaining validation tests](AUTOKART_EIGHT_REMAINING_VALIDATION_TESTS.md)
- [BLDC AS5047P drivetrain calibration](BLDC_AS5047P_DRIVETRAIN_CALIBRATION.md)
- [DFRobot GP8630N documentation](https://wiki.dfrobot.com/dfr1230/)
- [Espressif ESP32-S3 GPIO documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html)

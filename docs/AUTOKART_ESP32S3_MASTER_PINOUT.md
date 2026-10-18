# AutoKart ESP32-S3 master pinout

Revision: C — 18 October 2026

This is the authoritative integrated pin allocation for the AutoKart
ESP32-S3 N8R2 controller. It supersedes the pin assignments in isolated bench
tests when several components are installed at the same time.

Assumed actuator assignment:

- R86mini + NEMA34: steering
- TB6600 + stepper: brake
- BLD750 + GP8630N DAC: traction
- AS5047P: BLDC/motor-shaft encoder
- AS5600 #1: steering-angle feedback
- AS5600 #2: brake-position feedback through TCA9548A channel isolation
- RA-02: LoRa emergency-stop receiver; transmitter MCU selection is pending
- Raspberry Pi 4B: high-level controller over native USB micro-ROS

There is no brake-position encoder in the current design. The LoRa E-stop is
being separated from normal actuator control; Pico 2 versus XIAO ESP32-C3 for
the remote transmitter remains an open hardware decision.

## Final GPIO allocation

| GPIO | Signal | Component | Direction at ESP32-S3 | Status |
|---:|---|---|---|---|
| 1 | I2C SDA | TCA9548A + GP8630N | Bidirectional | Integrated assignment |
| 2 | I2C SCL | TCA9548A + GP8630N | Bidirectional/open-drain | Integrated assignment |
| 4 | STEP / PUL | R86mini steering | Output | Bench-proven assignment |
| 5 | DIR | R86mini steering | Output | Bench-proven assignment |
| 6 | ENA | R86mini steering | Output | Reserved; optional |
| 7 | W/PWM bench signal | AS5047P | Input | Provisional test point; not production-frozen |
| 8 | RESET | RA-02 | Output | Frozen receiver assignment |
| 9 | NSS / CS | RA-02 | Output | Frozen receiver assignment |
| 10 | MISO | RA-02 | Input | Frozen receiver assignment |
| 11 | MOSI | RA-02 | Output | Frozen receiver assignment |
| 12 | SCK | RA-02 | Output | Frozen receiver assignment |
| 13 | BRK transistor | BLD750 | Output | Bench-proven assignment |
| 14 | EN transistor | BLD750 | Output | Bench-proven assignment |
| 15 | F/R transistor | BLD750 | Output | Bench-proven assignment |
| 16 | CSn | AS5047P SPI | Output | Integrated encoder assignment |
| 17 | DIO0 | RA-02 | Input | Frozen receiver assignment |
| 18 | CLK / SCK | AS5047P SPI | Output | Integrated encoder assignment |
| 38 | STEP / PUL | TB6600 brake | Output | New integrated assignment |
| 39 | DIR | TB6600 brake | Output | New integrated assignment |
| 40 | ENA | TB6600 brake | Output | New integrated assignment |
| 41 | MOSI | AS5047P SPI | Output | Integrated encoder assignment |
| 42 | MISO | AS5047P SPI | Input | Integrated encoder assignment |

GPIO21, GPIO47, and GPIO48 remain unallocated. GPIO19 and GPIO20 remain
reserved for native USB. Do not use GPIO0, GPIO3, GPIO45, or GPIO46 for this
design because they are boot-strapping pins. GPIO26 through GPIO37 are
intentionally avoided because they can be occupied by flash/PSRAM signals on
ESP32-S3 variants.

## Component connection sheets

### Dual AS5600 steering and brake encoders

Both AS5600 devices use fixed address `0x36`; use separate TCA9548A channels.

| TCA9548A upstream | ESP32-S3 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO1 |
| SCL | GPIO2 |
| A0 / A1 / A2 | GND (`0x70`) |

| Encoder | TCA9548A channel | Address |
|---|---:|---:|
| Steering AS5600 | 0 | `0x36` |
| Brake AS5600 | 1 | `0x36` |

Each AS5600 uses 3V3/GND and connects SDA/SCL to its channel's SDx/SCx pair.
OUT is unused for I2C operation. The GP8630N remains on the upstream bus.

### GP8630N DAC and BLD750 traction controller

| GP8630N | ESP32-S3 / BLD750 |
|---|---|
| VCC | Supply required by the installed GP8630N module |
| GND | ESP32-S3 GND and BLD750 COM |
| SDA | ESP32-S3 GPIO1 |
| SCL | ESP32-S3 GPIO2 |
| OUT | BLD750 SV |

I2C address: `0x58`. The AS5600 and GP8630N can share GPIO1/GPIO2 because
their addresses differ. Use one correctly sized set of I2C pull-ups to 3.3 V;
do not fit unnecessarily strong duplicate pull-ups on every board.

Wire the two modules in parallel, not one after the other:

```text
ESP32-S3 GPIO1 SDA -----+---- TCA9548A SDA
                        +---- GP8630N SDA

ESP32-S3 GPIO2 SCL -----+---- TCA9548A SCL
                        +---- GP8630N SCL

TCA9548A channel 0 SD0/SC0 ---- Steering AS5600 SDA/SCL
TCA9548A channel 1 SD1/SC1 ---- Brake AS5600 SDA/SCL

ESP32-S3 GND ------------------ GP8630N GND ---- BLD750 COM

GP8630N OUT ---------------------------------- BLD750 SV
```

An upstream scan should find TCA9548A `0x70` and GP8630N `0x58`. After selecting
channel 0 or 1, that channel should expose one AS5600 at `0x36`.

| BLD750 control | ESP32-S3 interface |
|---|---|
| BRK | GPIO13 through the tested transistor stage |
| EN | GPIO14 through the tested transistor stage |
| F/R | GPIO15 through the tested transistor stage |
| SV | GP8630N OUT |
| COM | GP8630N GND / control ground |

Do not connect a BLD750 control input or motor-supply voltage directly to an
ESP32-S3 GPIO.

### R86mini steering driver

| R86mini | ESP32-S3 interface |
|---|---|
| PUL | GPIO4 through the 5 V opto-input interface |
| DIR | GPIO5 through the 5 V opto-input interface |
| ENA | GPIO6 through the same interface, or leave disconnected |

The bench-proven common-anode interface supplies the driver `+` terminals
from 5 V and uses transistor stages to sink the `-` terminals. Do not feed 5 V
back into an ESP32-S3 GPIO.

Bench-proven NEMA34 winding groups:

| R86mini motor terminal | Motor wires |
|---|---|
| A+ | Red + Blue |
| A- | Yellow + Black |
| B+ | Orange + Green |
| B- | Brown + White |

### TB6600 brake driver

| TB6600 | ESP32-S3 interface |
|---|---|
| PUL / STEP | GPIO38 through a suitable opto-input interface |
| DIR | GPIO39 through a suitable opto-input interface |
| ENA | GPIO40 through a suitable opto-input interface |

Confirm the installed TB6600 input polarity before enabling motion. The final
brake fail-safe state must be verified physically; an ESP32 reset must not
release the brake unexpectedly.

### AS5047P BLDC encoder — SPI interface

| AS5047P | ESP32-S3 |
|---|---|
| Supply | Use either the module's 3.3V or 5V input as configured; never both |
| GND | GND |
| W/PWM | GPIO7 input |
| I/PWM | Not connected when PWM is mapped to W/PWM |

Prefer 3.3 V operation if the exact Meukron board supports it. Before joining
W/PWM to GPIO7, measure its HIGH level and confirm it does not exceed 3.3 V.

PWM is not mapped to an output in the AS5047P default configuration. Either
enable `PWMon=1` with `UVW_ABI=0` through SPI after every sensor power-up, or
permanently program the required settings into OTP only after the complete
configuration has been validated. Do not program OTP during initial testing.

Keep these pads/header available for volatile startup configuration and
diagnostics:

| AS5047P configuration SPI | ESP32-S3 |
|---|---|
| CSn | GPIO16 |
| CLK | GPIO18 |
| MOSI | GPIO41 |
| MISO | GPIO42 |

GPIO7 preserves the W/PWM bench test, but the integrated encoder interface is
continuous SPI angle sampling on GPIO16/18/41/42. This SPI bus does not
conflict with the RA-02 bus.

### RA-02 LoRa E-stop receiver — provisional monitoring interface

| RA-02 | ESP32-S3 |
|---|---|
| VCC | 3V3 only |
| GND | GND |
| RESET | GPIO8 |
| NSS / CS | GPIO9 |
| MISO | GPIO10 |
| MOSI | GPIO11 |
| SCK | GPIO12 |
| DIO0 | GPIO17 |

Fit a 100 nF ceramic capacitor and a 10–100 uF bulk capacitor close to RA-02
VCC/GND, and connect the correct 433 MHz antenna before transmitting.

These pins preserve the working ESP32-S3 receiver bench connection. They do
not make the E-stop independent: the production design must use a separately
validated isolation path that does not depend on ROS, Linux, normal VCU tasks,
or continued wireless communication. The transmitter controller is not frozen;
Pico 2 and XIAO ESP32-C3 remain candidates.

### Raspberry Pi 4B communication

The production communication target is native USB with a micro-ROS Agent on
the Raspberry Pi. GPIO19/GPIO20 remain reserved for the ESP32-S3 native USB
interface. GPIO21/GPIO47 are no longer reserved for Pi UART and remain
available until another function is frozen.

USB micro-ROS has not yet been implemented or validated in this repository.
The current MicroPython bench firmware is retained for component tests; the
production VCU is planned as ESP-IDF + FreeRTOS firmware.

## Bus summary

```text
TCA9548A (0x70) ------+
                       +-- I2C SDA GPIO1 / SCL GPIO2
GP8630N (0x58) -------+
  TCA channel 0 -------- Steering AS5600 (0x36)
  TCA channel 1 -------- Brake AS5600 (0x36)

RA-02 ------------------- SPI: SCK12 MOSI11 MISO10 CS9, RST8, DIO0 17
AS5047P production ------- SPI mode 1: SCK18 MOSI41 MISO42 CS16
AS5047P PWM bench point --- W/PWM GPIO7 (not production-frozen)
Raspberry Pi 4B ---------- Native USB micro-ROS: GPIO19/20 reserved
R86mini steering --------- STEP4 / DIR5 / ENA6
TB6600 brake ------------- STEP38 / DIR39 / ENA40
BLD750 digital controls -- BRK13 / EN14 / F-R15
```

## Integration status and required checks

The actuator, one AS5600 test setup, AS5047P SPI, GP8630N, and LoRa subsystems
have been bench-tested individually. The dual-AS5600 TCA9548A integration,
combined GPIO1/2 bus, mounted encoder behavior, and Raspberry Pi native USB
micro-ROS remain unvalidated. Any subsystem moved away from its original
bench-test GPIOs requires firmware migration and another powered validation.

Before moving the kart under power:

1. Verify every GPIO-to-terminal connection with continuity mode while power
   is disconnected.
2. Bring up one subsystem at a time, then run the complete pin-conflict test.
3. Confirm TCA9548A `0x70`, both channel-isolated AS5600 devices at `0x36`,
   GP8630N `0x58`, RA-02 `0x12`, and valid AS5047P SPI frames before commanding
   actuators.
4. Confirm the brake and traction controller enter a safe state during ESP32
   reset, power loss, and disconnected-signal tests.
5. Keep the LAY37 emergency-stop path independent of Raspberry Pi software.

## Hardware references

- [Espressif ESP32-S3 GPIO documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html)
- [Raspberry Pi 4 Model B datasheet and J8 header pinout](https://pip-assets.raspberrypi.com/categories/545-raspberry-pi-4-model-b/documents/RP-008341-DS-1-raspberry-pi-4-datasheet.pdf)

# ESP32-S3 encoder test connections

These are standalone bench tests. They may overlap with AutoKart actuator pins during bring-up, but disconnect the actuator hardware while testing.

## AS5600 — I2C

The AS5600 uses I2C address `0x36` and exposes raw/mapped angle registers.

```text
AS5600 VCC  -> ESP32-S3 3V3
AS5600 GND  -> ESP32-S3 GND
AS5600 SDA  -> ESP32-S3 GPIO8
AS5600 SCL  -> ESP32-S3 GPIO9
```

Use 4.7 kOhm pull-ups from SDA and SCL to 3.3 V if the module does not already include them. Place the magnet centered above the sensor IC.

## AS5600 PWM over MAX3485

The AS5600 `OUT` pin can provide PWM whose duty cycle represents the angle. It must be configured for PWM first; a MAX3485 does not convert an analog output into PWM.

Sensor-side MAX3485:

```text
AS5600 OUT  -> MAX3485 RX/RXO (TTL input)
MAX3485 EN  -> 3V3 (transmit mode)
MAX3485 VCC -> 3V3
MAX3485 GND -> sensor ground
```

Receiver-side MAX3485:

```text
MAX3485 EN       -> GND (receive mode)
MAX3485 TX/TXD/RO -> ESP32-S3 GPIO13
MAX3485 VCC      -> ESP32-S3 3V3
MAX3485 GND      -> ESP32-S3 GND
```

```text
Sensor MAX3485 A  -> CAT5e twisted pair -> Receiver MAX3485 A
Sensor MAX3485 B  -> CAT5e twisted pair -> Receiver MAX3485 B
```

Run `bench/as5600_pwm_rs485_receiver.py` on the ESP32-S3. The receiver measures the returned PWM frequency, duty cycle, and calculated angle. The AS5600 PWM frame encodes the angle in 4095 data clock periods with 128-clock high/low framing sections. [AS5600 datasheet](https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf)

## AS5047P — SPI

The AS5047P uses SPI mode 1 and the `ANGLECOM` register (`0x3FFF`).

```text
AS5047P VCC   -> ESP32-S3 3V3 input
AS5047P GND   -> ESP32-S3 GND
AS5047P CLK   -> ESP32-S3 GPIO13
AS5047P MOSI  -> ESP32-S3 GPIO14
AS5047P MISO  -> ESP32-S3 GPIO15
AS5047P CSn   -> ESP32-S3 GPIO16
```

The magnet must be the correct diametrically magnetized type and centered over the AS5047P IC. The test does not write configuration or zero-position registers.

## Test files

```text
bench/as5600_esp32s3_test.py
bench/as5600_brake_feedback_test.py
bench/as5047p_esp32s3_test.py
bench/as5600_pwm_rs485_receiver.py
```

## AS5600 brake-feedback bring-up

For the first brake test, use only the AS5600 and ESP32-S3. Do not connect
the brake motor or allow the firmware to command the TB6600. The feedback
test uses the same I2C pins as the GP8630N, but it is a standalone test and
expects the AS5600 at address `0x36`.

Flash `bench/as5600_brake_feedback_test.py` as `main.py`, then use:

```text
r       one reading
stream  continuous raw angle and magnet status
zero    capture the physically released brake position
limit   capture the physically applied brake position
save    save the calibration in the ESP32 filesystem
status  show angle, span, direction, and brake percentage
```

The calibration creates a released reference and an applied span. The
feedback percentage is calculated as:

```text
brake_percent = clamp((signed_angle - released_angle) / applied_span, 0, 1) * 100
```

The signed angle calculation handles the AS5600's 0/360 degree wraparound.
The applied span also establishes whether the brake increases or decreases
in encoder counts. A healthy reading requires the magnet-detected flag and
no weak/strong magnet warning.

### Planned closed-loop use

1. Read encoder feedback at a fixed rate, such as 50-100 Hz.
2. Compare commanded brake position with measured brake percentage.
3. Use a small proportional correction to generate TB6600 step pulses.
4. Stop stepping when the error is inside a deadband.
5. Trigger a safe brake state if the magnet disappears, the reading jumps,
   the encoder stops changing while the motor is moving, or the feedback is
   outside the calibrated range.

The current file intentionally stops at measurement and calibration. It does
not yet close the loop or move the brake automatically.

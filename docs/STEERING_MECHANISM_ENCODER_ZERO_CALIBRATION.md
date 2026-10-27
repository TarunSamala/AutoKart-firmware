# AutoKart steering mechanism and AS5600 zero calibration

**Revision:** 0.1 — 27 October 2026  
**Scope:** NEMA34 + R86mini steering actuator and AS5600 steering encoder  
**Status:** Encoder centre and motor-to-encoder ratios are provisional; the
current powered calibration is **not production-approved** because I2C errors
were recorded.

## 1. Purpose

This document defines how AutoKart:

1. establishes the true mechanical steering centre;
2. stores that centre as an absolute AS5600 count;
3. calculates signed displacement from the stored centre;
4. returns the steering mechanism to encoder zero under closed-loop control;
5. validates the left and right motor-to-encoder conversions.

This calibration relates **NEMA34 motor motion to AS5600 motion**. It does not
yet prove road-wheel angle. A physical road-wheel angle measurement is still
required before an encoder value may be published as calibrated road-wheel
degrees.

Do not program the AS5600 OTP zero. AutoKart uses a software zero so that the
centre can be audited, updated and associated with a calibration ID.

## 2. Tested bench configuration

| Item | Configuration |
|---|---|
| Steering motor | NEMA34 |
| Motor driver | R86mini |
| Driver resolution | 3200 pulses per motor revolution |
| Motor pulses per degree | 8.888889 pulses/degree |
| Steering encoder | AS5600, 12-bit absolute magnetic encoder |
| AS5600 resolution | 4096 counts/revolution |
| Encoder degrees per count | 0.087890625 degree/count |
| ESP32-S3 STEP | GPIO4 |
| ESP32-S3 DIR | GPIO5 |
| ESP32-S3 encoder SDA/SCL during this bench test | GPIO1/GPIO2, direct AS5600 at `0x36` |
| ENA during calibration | Disconnected and not initialized |

The integrated VCU may place the steering AS5600 on PCA9548A channel 0. If the
multiplexer is fitted, firmware must select channel 0 before communicating
with AS5600 address `0x36`. The direct-I2C bench script does not perform that
selection.

Only the steering AS5600 is covered here. The drivetrain AS5047P has a separate
role and must not be used as steering feedback.

## 3. Frozen direction convention

The observed steering convention is:

| Physical/motor movement | Motor command | GPIO5 | AS5600 response |
|---|---:|---:|---:|
| Right | Positive | LOW | Raw count decreases |
| Left | Negative | HIGH | Raw count increases |

Therefore, with right-positive road steering:

```text
positive road-steering angle = negative AS5600 count displacement
negative road-steering angle = positive AS5600 count displacement
```

This polarity must be checked after any motor wiring, driver, encoder, magnet,
coupler or linkage change.

## 4. Current measured calibration

The true mechanical centre captured on 27 October 2026 was:

```text
STEERING_CENTER_RAW = 1525 counts
```

The powered calibration commanded known motor movement at 3200 pulses/rev:

| Command | Pulses | AS5600 change | AS5600 angle | Encoder degree per motor degree |
|---:|---:|---:|---:|---:|
| +20.025 motor degrees, right | +178 | -39 counts | -3.428 degrees | 0.171173 |
| -20.025 motor degrees, left | -178 | +45 counts | +3.955 degrees | 0.197507 |
| +24.975 motor degrees, right | +222 | -49 counts | -4.307 degrees | 0.172438 |
| -24.975 motor degrees, left | -222 | +58 counts | +5.098 degrees | 0.204110 |

The 20/25-degree points were consistent on each individual side:

```text
right-side 20/25 scale mismatch = 0.74%
left-side 20/25 scale mismatch  = 3.29%
```

The centre returns were `0, 0, +1, 0` counts relative to the original centre.
This demonstrates repeatable return to encoder centre during that run.

The left and right conversion factors are different, so one averaged scale
must not be used for position commands:

| Side | Provisional encoder degree/motor degree | Motor degree/encoder degree | AS5600 counts/motor degree | Motor degree/AS5600 count |
|---|---:|---:|---:|---:|
| Right | 0.1718055 | 5.820535 | 1.954765 | 0.511570 |
| Left | 0.2008085 | 4.979869 | 2.284754 | 0.437684 |

These are provisional motor-to-encoder constants. They are not road-wheel
angle constants and must not be extrapolated to 90 degrees.

## 5. Software-zero calculation

AS5600 values wrap from 4095 back to 0. Always calculate the shortest signed
count displacement:

```python
AS5600_COUNTS = 4096
STEERING_CENTER_RAW = 1525

def wrapped_delta(raw, centre=STEERING_CENTER_RAW):
    return ((int(raw) - int(centre) + 2048) % 4096) - 2048
```

For the frozen right-positive convention:

```python
encoder_delta_counts = wrapped_delta(raw)
signed_encoder_degrees = -encoder_delta_counts * 360.0 / 4096.0
```

Examples:

| Raw count | Wrapped displacement | Interpretation |
|---:|---:|---|
| 1525 | 0 | Centre |
| 1486 | -39 | Right of centre |
| 1570 | +45 | Left of centre |

Recommended provisional centre band:

```text
abs(wrapped_delta(raw, 1525)) <= 2 counts
```

This is approximately the observed stationary/return resolution. It is not a
road-wheel angular tolerance.

## 6. Capturing the centre zero

Use this procedure after installing or changing the magnet, AS5600, shaft
coupling, gears, tie rods or steering linkage.

1. Raise and restrain the front wheels.
2. Isolate traction and keep the steering-driver power cut within reach.
3. Mechanically place both road wheels at the true centre using a physical
   alignment reference. Do not use the AS5600 display to decide centre.
4. Confirm the steering mechanism is unloaded and stationary.
5. Verify AS5600 `MAGNET OK` and stable I2C communication.
6. Collect at least 100 valid samples.
7. Reject invalid-magnet samples, I2C failures and implausible jumps.
8. Store the median raw count as `steering_center_raw`.
9. Save it with a calibration ID, timestamp, hardware identity and integrity
   check/CRC.
10. Reboot the ESP32 without moving the mechanism and verify that the measured
    count returns to the saved centre band.

Changing the saved number without physically centring the steering is not a
valid calibration.

## 7. Returning steering to encoder zero

Returning to zero must be an explicit authorized operation. Never move the
steering automatically merely because the ESP32, Jetson or dashboard booted.

At each control update:

1. Read and validate the AS5600 sample.
2. Calculate `delta = wrapped_delta(raw, steering_center_raw)`.
3. If `abs(delta) <= 2`, stop STEP pulses and report `CENTRED`.
4. If `delta < -2`, steering is right of centre; command motor movement left.
5. If `delta > +2`, steering is left of centre; command motor movement right.
6. Slow down as the count error approaches zero.
7. Stop immediately on stale/invalid feedback, wrong-direction movement,
   excessive travel, no progress, timeout, E-stop or operator stop.

With the tested motor convention, the correction polarity is:

```python
if delta_counts < -2:
    # Right of centre: GPIO5 HIGH moves back left.
    direction_level = 1
elif delta_counts > 2:
    # Left of centre: GPIO5 LOW moves back right.
    direction_level = 0
else:
    centred = True
```

Use closed-loop count feedback rather than issuing one large open-loop pulse
correction. The provisional side-specific conversion may estimate the initial
movement, but the AS5600 centre target must terminate the movement.

## 8. Calibration test procedure

Bench firmware:

```text
bench/esp32s3_nema34_as5600_20_25deg_calibration.py
```

Flash it from the repository:

```bash
cd /home/satvara/Github/AutoKart-firmware

ESP_PORT=/dev/serial/by-id/usb-Espressif_Systems_Espressif_Device_98c377ac9b780000-if00

/home/satvara/python/bin/python3 -m mpremote connect "$ESP_PORT" fs cp \
  bench/esp32s3_nema34_as5600_20_25deg_calibration.py :main.py

/home/satvara/python/bin/python3 -m mpremote connect "$ESP_PORT" repl
```

At the MicroPython prompt:

```python
import main
```

Type `RAISED-3200` only after confirming the front wheels are raised,
restrained and the R86mini is still configured for 3200 pulses/revolution.
Follow the prompts for centre, +20, centre, -20, centre, +25, centre, -25 and
final centre.

Retrieve the result:

```bash
/home/satvara/python/bin/python3 -m mpremote connect "$ESP_PORT" fs cp \
  :steering_motor_encoder_20_25deg_calibration.json \
  ./steering_motor_encoder_20_25deg_calibration.json
```

Isolated closed-loop firmware and its host test are:

```text
bench/esp32s3_steering_closed_loop_test.py
tools/steering_closed_loop_test.py
```

They remain wheels-off-ground commissioning tools, not production VCU
firmware.

## 9. Acceptance criteria

Repeat the complete calibration at least three times. All runs must satisfy:

| Check | Required result |
|---|---|
| AS5600 detection | `0x36` on the selected steering bus/channel |
| Magnet state | `MAGNET OK` for every accepted sample |
| Invalid magnet samples | 0 |
| Unrecovered I2C errors | 0 |
| Stationary peak-to-peak | At most 6 counts |
| Stationary standard deviation | At most 2 counts |
| Centre return | Preferably ±1 count; maximum provisional band ±2 counts |
| 20/25 scale mismatch on each side | At most 5% target; never above 10% |
| Direction | Right decreases raw count; left increases raw count |
| Wrong-direction/no-progress protection | Demonstrated |
| Reboot retention test | Raw centre remains within the centre band |

After these pass, physically measure actual left and right road-wheel angles
at multiple encoder positions. Build a left/right piecewise road-wheel map;
do not assume motor degrees, AS5600 degrees and road-wheel degrees are equal.

## 10. Current qualification and blocker

The latest run showed healthy magnetic strength:

```text
AGC approximately 41 to 47
MAG approximately 2060 to 2125
invalid magnet samples = 0 during the detailed run
```

It also repeatedly returned to centre count 1525. However, several stages had
I2C errors, and one stationary capture contained a 274-count spike. The
powered calibration therefore correctly ended with `Result: FAIL`.

The likely remaining fault domain is the I2C electrical path or interference:

- SDA/SCL connector or cable integrity;
- bus pull-ups and local decoupling;
- shared ground quality;
- routing near STEP/DIR or motor-current wiring;
- motor-driver electrical noise;
- cable movement at the mechanical centre.

Until three clean runs pass, use `1525` only as a bench-development zero. Do
not enable autonomous steering or claim production road-wheel calibration.

## 11. Recalibration triggers

Invalidate this calibration and repeat it after any of the following:

- magnet replacement or repositioning;
- AS5600 board replacement or movement;
- motor, driver or microstep-setting change;
- shaft coupler, steering gear, tie-rod or linkage adjustment;
- collision, hard-stop impact or mechanical slip;
- unexplained centre offset;
- repeated I2C, magnet-validity, no-progress or direction faults.


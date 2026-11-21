"""Powered Pico 2 NEMA34-to-AS5600 steering calibration over +/-20 degrees.

Sequence:
  software/mechanical center -> +20 motor degrees -> center
                             -> -20 motor degrees -> center

The test commands a known number of STEP pulses and measures how many native
AS5600 degrees were observed. It assumes a direct 1:1 motor-to-steering shaft
only for the provisional real-angle scale. Verify road-wheel angle separately.

Pico 2 wiring:
  AS5600 VCC -> 3V3(OUT), physical pin 36
  AS5600 GND -> GND, physical pin 38
  AS5600 SDA -> GP0, physical pin 1
  AS5600 SCL -> GP1, physical pin 2

  GP4 -> tested 5 V NPN interface -> driver PUL-
  GP5 -> tested 5 V NPN interface -> driver DIR-
  regulated 5 V -> driver PUL+ and DIR+
  Pico GND -> NPN interface signal GND
  ENA disconnected

Driver DIP setting required by this file: 3200 pulses/motor revolution.
"""

from machine import I2C, Pin
import math
import time

try:
    import ujson as json
except ImportError:
    import json


I2C_ID = 0
SDA_GPIO = 0
SCL_GPIO = 1
STEP_GPIO = 4
DIR_GPIO = 5
AS5600_ADDRESS = 0x36
I2C_FREQUENCY = 400_000

REG_STATUS = 0x0B
REG_ANGLE = 0x0E
REG_AGC = 0x1A
REG_MAGNITUDE = 0x1B
MAGNET_TOO_STRONG = 0x08
MAGNET_TOO_WEAK = 0x10
MAGNET_DETECTED = 0x20

COUNTS_PER_REVOLUTION = 4096
ENCODER_DEGREES_PER_COUNT = 360.0 / COUNTS_PER_REVOLUTION
PULSES_PER_MOTOR_REVOLUTION = 3200
PULSES_PER_MOTOR_DEGREE = PULSES_PER_MOTOR_REVOLUTION / 360.0
REQUESTED_TEST_DEGREES = 20.0
TEST_PULSES = round(REQUESTED_TEST_DEGREES * PULSES_PER_MOTOR_DEGREE)
ACTUAL_TEST_DEGREES = TEST_PULSES / PULSES_PER_MOTOR_DEGREE

# Verified convention from the existing R86mini steering test. If the motor
# moves physically in the opposite direction, stop and swap these two levels.
POSITIVE_DIRECTION_LEVEL = 1
NEGATIVE_DIRECTION_LEVEL = 0

STEP_ACTIVE_LEVEL = 1
STEP_INACTIVE_LEVEL = 0
STEP_HIGH_US = 30
START_PPS = 120
MAX_PPS = 500
RAMP_PULSES = 50
DIR_SETTLE_MS = 100
SETTLE_AFTER_MOVE_MS = 700

SAMPLE_INTERVAL_MS = 10
CAPTURE_DURATION_MS = 1500
MAX_STATIONARY_P2P_COUNTS = 6
MAX_STATIONARY_STDDEV_COUNTS = 2.0
MAX_CENTER_RETURN_MOTOR_DEG = 1.0
OUTPUT_FILE = "steering_motor_encoder_20deg_calibration.json"


def wrapped_delta(current, reference):
    return (
        (int(current) - int(reference) + COUNTS_PER_REVOLUTION // 2)
        % COUNTS_PER_REVOLUTION
    ) - COUNTS_PER_REVOLUTION // 2


def median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) & 1:
        return float(ordered[middle])
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def sign(value):
    return 1 if value > 0 else -1 if value < 0 else 0


def read_u16(register):
    data = i2c.readfrom_mem(AS5600_ADDRESS, register, 2)
    return (data[0] << 8) | data[1]


def magnet_state(status):
    if status & MAGNET_TOO_STRONG:
        return "MAGNET TOO STRONG"
    if status & MAGNET_TOO_WEAK:
        return "MAGNET TOO WEAK / NO MAGNET"
    if status & MAGNET_DETECTED:
        return "MAGNET OK"
    return "NO MAGNET"


def magnet_valid(status):
    return (
        bool(status & MAGNET_DETECTED)
        and not bool(status & MAGNET_TOO_WEAK)
        and not bool(status & MAGNET_TOO_STRONG)
    )


def read_sensor():
    status = i2c.readfrom_mem(AS5600_ADDRESS, REG_STATUS, 1)[0]
    angle = read_u16(REG_ANGLE) & 0x0FFF
    agc = i2c.readfrom_mem(AS5600_ADDRESS, REG_AGC, 1)[0]
    magnitude = read_u16(REG_MAGNITUDE) & 0x0FFF
    return status, angle, agc, magnitude


def pulse_rate(index, count):
    ramp = min(RAMP_PULSES, count // 2)
    if ramp <= 0:
        return START_PPS
    distance_from_end = count - index - 1
    ramp_position = min(index, distance_from_end, ramp)
    return START_PPS + (
        (MAX_PPS - START_PPS) * ramp_position // ramp
    )


def move_pulses(signed_pulses, label):
    count = abs(int(signed_pulses))
    if count == 0:
        return
    direction_level = (
        POSITIVE_DIRECTION_LEVEL if signed_pulses > 0
        else NEGATIVE_DIRECTION_LEVEL
    )
    direction_pin.value(direction_level)
    time.sleep_ms(DIR_SETTLE_MS)
    print(
        "MOVE {}: {:+d} pulses = {:+.3f} commanded motor deg".format(
            label,
            int(signed_pulses),
            signed_pulses / PULSES_PER_MOTOR_DEGREE,
        )
    )
    for index in range(count):
        rate = pulse_rate(index, count)
        period_us = max(STEP_HIGH_US + 1, 1_000_000 // max(1, rate))
        step_pin.value(STEP_ACTIVE_LEVEL)
        time.sleep_us(STEP_HIGH_US)
        step_pin.value(STEP_INACTIVE_LEVEL)
        time.sleep_us(period_us - STEP_HIGH_US)
    step_pin.value(STEP_INACTIVE_LEVEL)
    time.sleep_ms(SETTLE_AFTER_MOVE_MS)


def capture(name):
    samples = []
    invalid = 0
    errors = 0
    first = None
    agc_values = []
    magnitude_values = []
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < CAPTURE_DURATION_MS:
        try:
            status, angle, agc, magnitude = read_sensor()
            if not magnet_valid(status):
                invalid += 1
            else:
                if first is None:
                    first = angle
                samples.append(wrapped_delta(angle, first))
                agc_values.append(agc)
                magnitude_values.append(magnitude)
        except Exception:
            errors += 1
        time.sleep_ms(SAMPLE_INTERVAL_MS)

    if not samples:
        raise RuntimeError(name + ": no valid AS5600 samples")
    offset = median(samples)
    centred = [value - offset for value in samples]
    mean = sum(centred) / len(centred)
    variance = sum(
        (value - mean) * (value - mean) for value in centred
    ) / max(1, len(centred) - 1)
    result = {
        "name": name,
        "count": int(round(first + offset)) % COUNTS_PER_REVOLUTION,
        "valid_samples": len(samples),
        "invalid_magnet_samples": invalid,
        "i2c_errors": errors,
        "peak_to_peak_counts": max(centred) - min(centred),
        "standard_deviation_counts": math.sqrt(variance),
        "agc_median": median(agc_values),
        "magnitude_median": median(magnitude_values),
    }
    print(
        "{}: count={} valid={} invalid={} i2c={} p2p={:.1f} "
        "stddev={:.2f} AGC={:.1f} MAG={:.1f}".format(
            name,
            result["count"],
            result["valid_samples"],
            invalid,
            errors,
            result["peak_to_peak_counts"],
            result["standard_deviation_counts"],
            result["agc_median"],
            result["magnitude_median"],
        )
    )
    return result


def calculate(stages):
    center = stages["center_start"]["count"]
    positive_counts = wrapped_delta(stages["positive_20"]["count"], center)
    negative_counts = wrapped_delta(stages["negative_20"]["count"], center)
    failures = []
    warnings = []

    if abs(positive_counts) < 10 or abs(negative_counts) < 10:
        failures.append("encoder movement too small for a 20-degree motor command")
    if sign(positive_counts) == sign(negative_counts):
        failures.append("positive and negative motor commands did not produce opposite encoder changes")

    for stage in stages.values():
        if stage["invalid_magnet_samples"]:
            failures.append(stage["name"] + " has invalid magnet samples")
        if stage["i2c_errors"]:
            failures.append(stage["name"] + " has I2C errors")
        if stage["peak_to_peak_counts"] > MAX_STATIONARY_P2P_COUNTS:
            warnings.append(stage["name"] + " has high stationary peak-to-peak noise")
        if stage["standard_deviation_counts"] > MAX_STATIONARY_STDDEV_COUNTS:
            warnings.append(stage["name"] + " has high stationary standard deviation")

    if failures:
        return None, failures, warnings

    positive_encoder_deg = positive_counts * ENCODER_DEGREES_PER_COUNT
    negative_encoder_deg = negative_counts * ENCODER_DEGREES_PER_COUNT
    positive_encoder_per_motor = abs(positive_encoder_deg) / ACTUAL_TEST_DEGREES
    negative_encoder_per_motor = abs(negative_encoder_deg) / ACTUAL_TEST_DEGREES
    positive_motor_per_encoder = 1.0 / positive_encoder_per_motor
    negative_motor_per_encoder = 1.0 / negative_encoder_per_motor
    mismatch = (
        abs(positive_encoder_per_motor - negative_encoder_per_motor)
        / ((positive_encoder_per_motor + negative_encoder_per_motor) / 2.0)
        * 100.0
    )
    if mismatch > 10.0:
        warnings.append(
            "positive/negative scale differs by {:.1f}%".format(mismatch)
        )

    calibration = {
        "schema_version": 1,
        "test": "Pico2 NEMA34 STEP/DIR to AS5600 +/-20 degree calibration",
        "pulses_per_motor_revolution": PULSES_PER_MOTOR_REVOLUTION,
        "test_pulses": TEST_PULSES,
        "requested_test_motor_degrees": REQUESTED_TEST_DEGREES,
        "actual_test_motor_degrees": ACTUAL_TEST_DEGREES,
        "center_count": center,
        "positive_delta_counts": positive_counts,
        "negative_delta_counts": negative_counts,
        "positive_encoder_degrees": positive_encoder_deg,
        "negative_encoder_degrees": negative_encoder_deg,
        "positive_encoder_degrees_per_motor_degree": positive_encoder_per_motor,
        "negative_encoder_degrees_per_motor_degree": negative_encoder_per_motor,
        "positive_motor_degrees_per_encoder_degree": positive_motor_per_encoder,
        "negative_motor_degrees_per_encoder_degree": negative_motor_per_encoder,
        "positive_raw_count_sign": sign(positive_counts),
        "scale_mismatch_percent": mismatch,
        "provisional_real_angle_assumption": "motor-to-steering direct 1:1",
        "stages": stages,
    }

    center_errors = {}
    for key in ("center_after_positive", "center_final"):
        delta = wrapped_delta(stages[key]["count"], center)
        if delta * sign(positive_counts) >= 0:
            error = delta * ENCODER_DEGREES_PER_COUNT * positive_motor_per_encoder
        else:
            error = delta * ENCODER_DEGREES_PER_COUNT * negative_motor_per_encoder
        center_errors[key] = error
        if abs(error) > MAX_CENTER_RETURN_MOTOR_DEG:
            warnings.append(
                "{} return error is {:+.3f} motor deg".format(key, error)
            )
    calibration["center_return_error_motor_deg"] = center_errors
    calibration["warnings"] = warnings
    calibration["result"] = "PASS" if not warnings else "PASS_WITH_WARNINGS"
    return calibration, failures, warnings


def print_results(calibration):
    print()
    print("================ MOTOR/ENCODER RESULTS ================")
    print(
        "Motor command: {} pulses = {:.3f} motor degrees".format(
            calibration["test_pulses"], calibration["actual_test_motor_degrees"]
        )
    )
    print(
        "+20 command -> encoder delta {:+d} counts = {:+.3f} encoder deg".format(
            calibration["positive_delta_counts"],
            calibration["positive_encoder_degrees"],
        )
    )
    print(
        "-20 command -> encoder delta {:+d} counts = {:+.3f} encoder deg".format(
            calibration["negative_delta_counts"],
            calibration["negative_encoder_degrees"],
        )
    )
    print(
        "+ side: {:.6f} encoder-deg/motor-deg | "
        "{:.6f} motor-deg/encoder-deg".format(
            calibration["positive_encoder_degrees_per_motor_degree"],
            calibration["positive_motor_degrees_per_encoder_degree"],
        )
    )
    print(
        "- side: {:.6f} encoder-deg/motor-deg | "
        "{:.6f} motor-deg/encoder-deg".format(
            calibration["negative_encoder_degrees_per_motor_degree"],
            calibration["negative_motor_degrees_per_encoder_degree"],
        )
    )
    print("Scale mismatch: {:.2f}%".format(calibration["scale_mismatch_percent"]))
    for key, value in calibration["center_return_error_motor_deg"].items():
        print("{}: {:+.3f} motor deg".format(key, value))
    print("Result:", calibration["result"])
    for warning in calibration["warnings"]:
        print("WARNING:", warning)
    print("Saved:", OUTPUT_FILE)
    print()
    print("IMPORTANT: these are motor-to-encoder ratios. Treat them as real")
    print("steering degrees only after mechanically confirming the 1:1 linkage.")


step_pin = Pin(STEP_GPIO, Pin.OUT, value=STEP_INACTIVE_LEVEL)
direction_pin = Pin(DIR_GPIO, Pin.OUT, value=NEGATIVE_DIRECTION_LEVEL)
i2c = I2C(
    I2C_ID,
    sda=Pin(SDA_GPIO),
    scl=Pin(SCL_GPIO),
    freq=I2C_FREQUENCY,
)

print()
print("========================================================")
print(" PICO 2 NEMA34 + AS5600 +/-20 DEG CALIBRATION")
print("========================================================")
print("POWERED TEST: FRONT WHEELS MUST BE RAISED AND RESTRAINED.")
print("Keep the physical driver power cut within reach.")
print("STEP GP4 | DIR GP5 | AS5600 SDA GP0 SCL GP1")
print("Required driver setting: 3200 pulses/revolution")
print("Each test movement: {} pulses = {:.3f} motor degrees".format(
    TEST_PULSES, ACTUAL_TEST_DEGREES
))

devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])
if AS5600_ADDRESS not in devices:
    raise RuntimeError("AS5600 not detected at 0x36")
status, angle, agc, magnitude = read_sensor()
print("Magnet: {} | count {} | AGC {} | MAG {}".format(
    magnet_state(status), angle, agc, magnitude
))
if not magnet_valid(status):
    raise RuntimeError("MAGNET PREFLIGHT FAILED: " + magnet_state(status))

confirmation = input(
    "Type RAISED-3200 only after checking wheels and driver DIP switches: "
).strip()
if confirmation != "RAISED-3200":
    raise RuntimeError("Powered calibration cancelled")

print()
input("Mechanically center steering, then press Enter... ")
stages = {"center_start": capture("CENTER_START")}

try:
    input("Press Enter to command POSITIVE +20 motor degrees... ")
    move_pulses(TEST_PULSES, "POSITIVE")
    stages["positive_20"] = capture("POSITIVE_20")

    input("Press Enter to return to software CENTER... ")
    move_pulses(-TEST_PULSES, "RETURN_CENTER_FROM_POSITIVE")
    stages["center_after_positive"] = capture("CENTER_AFTER_POSITIVE")

    input("Press Enter to command NEGATIVE -20 motor degrees... ")
    move_pulses(-TEST_PULSES, "NEGATIVE")
    stages["negative_20"] = capture("NEGATIVE_20")

    input("Press Enter to return to final software CENTER... ")
    move_pulses(TEST_PULSES, "RETURN_CENTER_FROM_NEGATIVE")
    stages["center_final"] = capture("CENTER_FINAL")
finally:
    step_pin.value(STEP_INACTIVE_LEVEL)

calibration, failures, warnings = calculate(stages)
if calibration is None:
    print("CALIBRATION FAILED")
    for failure in failures:
        print("FAIL:", failure)
    for warning in warnings:
        print("WARNING:", warning)
    raise RuntimeError("Powered calibration rejected")

with open(OUTPUT_FILE, "w") as output:
    json.dump(calibration, output)
print_results(calibration)
print("Repeat this test three times before freezing controller constants.")

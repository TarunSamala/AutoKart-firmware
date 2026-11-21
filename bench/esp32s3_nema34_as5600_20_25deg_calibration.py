"""ESP32-S3 NEMA34-to-AS5600 open-loop steering calibration.

The NEMA34 is moved by known STEP counts, independently of AS5600 feedback:

    0 -> +20 -> 0 -> -20 -> 0 -> +25 -> 0 -> -25 -> 0 degrees

This establishes how many native AS5600 counts/degrees correspond to known
motor degrees after changing the magnet.  The 25-degree points validate the
scale measured at 20 degrees; they are not the intended steering limit.

ESP32-S3 wiring:
  AS5600 SDA -> GPIO1
  AS5600 SCL -> GPIO2
  STEP       -> GPIO4 through the tested 5 V NPN interface
  DIR        -> GPIO5 through the tested 5 V NPN interface
  ENA        -> disconnected (GPIO6 is not initialized)

Required driver setting: 3200 pulses per motor revolution.

WARNING: powered wheels-off-ground calibration only.  Keep the physical
driver power cut within reach.  If interrupted during movement, motor position
is no longer known; mechanically recenter before running the test again.
"""

from machine import I2C, Pin
import math
import time

try:
    import ujson as json
except ImportError:
    import json


I2C_ID = 0
SDA_GPIO = 1
SCL_GPIO = 2
STEP_GPIO = 4
DIR_GPIO = 5
AS5600_ADDRESS = 0x36
I2C_FREQUENCY = 100_000

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

# Existing physically tested vehicle convention: positive steering is RIGHT,
# and GPIO5 LOW commands physical right through the installed interface.
# If the first movement is not physically right, cut driver power immediately;
# do not continue with an unexpected wiring/driver convention.
POSITIVE_DIRECTION_LEVEL = 0
NEGATIVE_DIRECTION_LEVEL = 1

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
MIN_MOVEMENT_COUNTS = 10
MAX_STATIONARY_P2P_COUNTS = 6
MAX_STATIONARY_STDDEV_COUNTS = 2.0
MAX_CENTER_RETURN_MOTOR_DEG = 1.0
MAX_SCALE_MISMATCH_PERCENT = 10.0
OUTPUT_FILE = "steering_motor_encoder_20_25deg_calibration.json"

TARGETS = (20.0, 25.0)


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


def target_pulses(degrees):
    return int(round(float(degrees) * PULSES_PER_MOTOR_DEGREE))


def pulses_to_degrees(pulses):
    return float(pulses) / PULSES_PER_MOTOR_DEGREE


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


def move_relative(signed_pulses, label):
    count = abs(int(signed_pulses))
    if count == 0:
        return
    direction_pin.value(
        POSITIVE_DIRECTION_LEVEL
        if signed_pulses > 0
        else NEGATIVE_DIRECTION_LEVEL
    )
    time.sleep_ms(DIR_SETTLE_MS)
    print(
        "MOVE {}: {:+d} pulses = {:+.3f} motor deg".format(
            label, int(signed_pulses), pulses_to_degrees(signed_pulses)
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


def endpoint_result(stage, center_count, commanded_degrees, pulses):
    delta_counts = wrapped_delta(stage["count"], center_count)
    encoder_degrees = delta_counts * ENCODER_DEGREES_PER_COUNT
    actual_motor_degrees = abs(pulses_to_degrees(pulses))
    encoder_per_motor = abs(encoder_degrees) / actual_motor_degrees
    return {
        "requested_motor_degrees": commanded_degrees,
        "commanded_pulses": pulses,
        "actual_motor_degrees": pulses_to_degrees(pulses),
        "encoder_delta_counts": delta_counts,
        "encoder_degrees": encoder_degrees,
        "encoder_degrees_per_motor_degree": encoder_per_motor,
        "motor_degrees_per_encoder_degree": 1.0 / encoder_per_motor,
        "encoder_count_sign": sign(delta_counts),
    }


def percent_difference(a, b):
    average = (abs(a) + abs(b)) / 2.0
    if average == 0:
        return 0.0
    return abs(a - b) / average * 100.0


def calculate(stages):
    failures = []
    warnings = []
    center_count = stages["center_start"]["count"]

    for stage in stages.values():
        if stage["invalid_magnet_samples"]:
            failures.append(stage["name"] + " has invalid magnet samples")
        if stage["i2c_errors"]:
            failures.append(stage["name"] + " has I2C errors")
        if stage["peak_to_peak_counts"] > MAX_STATIONARY_P2P_COUNTS:
            warnings.append(stage["name"] + " has high stationary peak-to-peak noise")
        if stage["standard_deviation_counts"] > MAX_STATIONARY_STDDEV_COUNTS:
            warnings.append(stage["name"] + " has high stationary standard deviation")

    endpoints = {}
    for requested in TARGETS:
        label = str(int(requested))
        positive_pulses = target_pulses(requested)
        negative_pulses = -positive_pulses
        endpoints["positive_" + label] = endpoint_result(
            stages["positive_" + label],
            center_count,
            requested,
            positive_pulses,
        )
        endpoints["negative_" + label] = endpoint_result(
            stages["negative_" + label],
            center_count,
            -requested,
            negative_pulses,
        )

    for label in ("20", "25"):
        positive = endpoints["positive_" + label]
        negative = endpoints["negative_" + label]
        if abs(positive["encoder_delta_counts"]) < MIN_MOVEMENT_COUNTS:
            failures.append("positive " + label + " movement is too small")
        if abs(negative["encoder_delta_counts"]) < MIN_MOVEMENT_COUNTS:
            failures.append("negative " + label + " movement is too small")
        if positive["encoder_count_sign"] == negative["encoder_count_sign"]:
            failures.append(
                "+{} and -{} did not produce opposite encoder changes".format(
                    label, label
                )
            )
        mismatch = percent_difference(
            positive["encoder_degrees_per_motor_degree"],
            negative["encoder_degrees_per_motor_degree"],
        )
        if mismatch > MAX_SCALE_MISMATCH_PERCENT:
            warnings.append(
                "+/-{} scale differs by {:.2f}%".format(label, mismatch)
            )

    linearity = {}
    for side in ("positive", "negative"):
        scale_20 = endpoints[side + "_20"]["encoder_degrees_per_motor_degree"]
        scale_25 = endpoints[side + "_25"]["encoder_degrees_per_motor_degree"]
        mismatch = percent_difference(scale_20, scale_25)
        linearity[side + "_20_vs_25_scale_mismatch_percent"] = mismatch
        if mismatch > MAX_SCALE_MISMATCH_PERCENT:
            warnings.append(
                "{} 20/25 scale differs by {:.2f}%".format(side, mismatch)
            )

    all_scales = [
        endpoint["encoder_degrees_per_motor_degree"]
        for endpoint in endpoints.values()
    ]
    average_encoder_per_motor = sum(all_scales) / len(all_scales)
    average_motor_per_encoder = 1.0 / average_encoder_per_motor

    center_returns = {}
    center_keys = (
        "center_after_positive_20",
        "center_after_negative_20",
        "center_after_positive_25",
        "center_final",
    )
    for key in center_keys:
        delta_counts = wrapped_delta(stages[key]["count"], center_count)
        motor_error = (
            delta_counts
            * ENCODER_DEGREES_PER_COUNT
            * average_motor_per_encoder
        )
        center_returns[key] = {
            "delta_counts": delta_counts,
            "estimated_motor_degrees": motor_error,
        }
        if abs(motor_error) > MAX_CENTER_RETURN_MOTOR_DEG:
            warnings.append(
                "{} return error is {:+.3f} motor deg".format(
                    key, motor_error
                )
            )

    calibration = {
        "schema_version": 1,
        "test": "ESP32-S3 NEMA34 STEP/DIR to AS5600 20/25 degree calibration",
        "pulses_per_motor_revolution": PULSES_PER_MOTOR_REVOLUTION,
        "intended_operating_limit_motor_degrees": 20.0,
        "validation_point_motor_degrees": 25.0,
        "center_count": center_count,
        "endpoints": endpoints,
        "linearity": linearity,
        "average_encoder_degrees_per_motor_degree": average_encoder_per_motor,
        "average_motor_degrees_per_encoder_degree": average_motor_per_encoder,
        "center_returns": center_returns,
        "provisional_angle_scope": "motor shaft only; road-wheel angle not yet verified",
        "stages": stages,
        "warnings": warnings,
        "failures": failures,
        "result": "FAIL" if failures else (
            "PASS_WITH_WARNINGS" if warnings else "PASS"
        ),
    }
    return calibration


def print_results(calibration):
    print()
    print("================ CALIBRATION RESULTS =================")
    for label in ("20", "25"):
        for side in ("positive", "negative"):
            result = calibration["endpoints"][side + "_" + label]
            print(
                "{:+.0f} requested -> {:+d} pulses = {:+.3f} motor deg; "
                "encoder {:+d} counts = {:+.3f} deg; scale {:.6f} "
                "encoder-deg/motor-deg".format(
                    result["requested_motor_degrees"],
                    result["commanded_pulses"],
                    result["actual_motor_degrees"],
                    result["encoder_delta_counts"],
                    result["encoder_degrees"],
                    result["encoder_degrees_per_motor_degree"],
                )
            )
    print(
        "Average scale: {:.6f} encoder-deg/motor-deg = "
        "{:.6f} motor-deg/encoder-deg".format(
            calibration["average_encoder_degrees_per_motor_degree"],
            calibration["average_motor_degrees_per_encoder_degree"],
        )
    )
    for key, value in calibration["linearity"].items():
        print("{}: {:.2f}%".format(key, value))
    for key, value in calibration["center_returns"].items():
        print(
            "{}: {:+d} counts = {:+.3f} estimated motor deg".format(
                key,
                value["delta_counts"],
                value["estimated_motor_degrees"],
            )
        )
    print("Result:", calibration["result"])
    for failure in calibration["failures"]:
        print("FAIL:", failure)
    for warning in calibration["warnings"]:
        print("WARNING:", warning)
    print("Saved on ESP32:", OUTPUT_FILE)
    print()
    print("Do not extrapolate this result to 90 degrees.")
    print("This calibrates NEMA34 motor angle against AS5600 angle only.")
    print("Road-wheel angle still requires a physical angle measurement.")


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
print(" ESP32-S3 NEMA34 + AS5600 0/20/25 DEG CALIBRATION")
print("========================================================")
print("POWERED TEST: FRONT WHEELS MUST BE RAISED AND RESTRAINED.")
print("Keep the physical driver power cut within reach.")
print("STEP GPIO4 | DIR GPIO5 | AS5600 SDA GPIO1 SCL GPIO2")
print("ENA is untouched/disconnected.")
print("Required driver setting: 3200 pulses/revolution")
print("Direction convention: positive = physical RIGHT, negative = LEFT")
for requested in TARGETS:
    pulses = target_pulses(requested)
    print(
        "{:.0f} deg point: {} pulses = {:.3f} actual motor deg".format(
            requested, pulses, pulses_to_degrees(pulses)
        )
    )

devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])
if AS5600_ADDRESS not in devices:
    raise RuntimeError("AS5600 not detected at 0x36")
status, angle, agc, magnitude = read_sensor()
print(
    "Magnet: {} | count {} | AGC {} | MAG {}".format(
        magnet_state(status), angle, agc, magnitude
    )
)
if not magnet_valid(status):
    raise RuntimeError("MAGNET PREFLIGHT FAILED: " + magnet_state(status))

confirmation = input(
    "Type RAISED-3200 only after checking wheels and DIP switches: "
).strip()
if confirmation != "RAISED-3200":
    raise RuntimeError("Powered calibration cancelled")

input("Mechanically center steering, then press Enter... ")
stages = {"center_start": capture("CENTER_START")}
current_position_pulses = 0


def move_to(requested_degrees, stage_key, prompt):
    global current_position_pulses
    input(prompt)
    target = target_pulses(requested_degrees)
    move_relative(target - current_position_pulses, stage_key.upper())
    current_position_pulses = target
    stages[stage_key] = capture(stage_key.upper())


try:
    move_to(20, "positive_20", "Press Enter to command +20 motor degrees... ")
    move_to(0, "center_after_positive_20", "Press Enter to return to 0... ")
    move_to(-20, "negative_20", "Press Enter to command -20 motor degrees... ")
    move_to(0, "center_after_negative_20", "Press Enter to return to 0... ")
    move_to(25, "positive_25", "Press Enter to command +25 motor degrees... ")
    move_to(0, "center_after_positive_25", "Press Enter to return to 0... ")
    move_to(-25, "negative_25", "Press Enter to command -25 motor degrees... ")
    move_to(0, "center_final", "Press Enter to return to final 0... ")
finally:
    step_pin.value(STEP_INACTIVE_LEVEL)

calibration = calculate(stages)
with open(OUTPUT_FILE, "w") as output:
    json.dump(calibration, output)
print_results(calibration)
if calibration["failures"]:
    raise RuntimeError("Powered calibration rejected")
print("Repeat the complete test three times before freezing constants.")

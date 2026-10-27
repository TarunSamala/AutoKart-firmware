"""Read-only Pico 2 calibration for the AutoKart steering AS5600.

The operator places the real steering mechanism at known angles. This program
measures the corresponding AS5600 movement and computes a piecewise conversion
from encoder counts to real steering degrees over the approved -20..+20 deg
range. It never initializes or commands the NEMA34 steering motor.

Direct wiring:
  AS5600 VCC -> Pico 2 3V3(OUT), physical pin 36
  AS5600 GND -> Pico 2 GND, physical pin 38
  AS5600 SDA -> Pico 2 GP0, physical pin 1
  AS5600 SCL -> Pico 2 GP1, physical pin 2

Use a two-pole, diametrically magnetized magnet. Mechanically measure the real
steering angle; do not use the AS5600 display itself to choose +/-20 degrees.
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
I2C_FREQUENCY = 400_000
AS5600_ADDRESS = 0x36

REG_STATUS = 0x0B
REG_RAW_ANGLE = 0x0C
REG_ANGLE = 0x0E
REG_AGC = 0x1A
REG_MAGNITUDE = 0x1B

STATUS_MAGNET_TOO_STRONG = 0x08
STATUS_MAGNET_TOO_WEAK = 0x10
STATUS_MAGNET_DETECTED = 0x20

COUNTS_PER_REVOLUTION = 4096
NATIVE_DEGREES_PER_COUNT = 360.0 / COUNTS_PER_REVOLUTION
CALIBRATION_LIMIT_DEG = 20.0
SAMPLE_INTERVAL_MS = 10
CAPTURE_DURATION_MS = 2000
MIN_ENDPOINT_COUNTS = 40
MAX_STATIONARY_P2P_COUNTS = 6
MAX_STATIONARY_STDDEV_COUNTS = 2.0
MAX_CENTER_RETURN_ERROR_DEG = 1.0
CALIBRATION_FILE = "steering_as5600_calibration.json"


def wrapped_delta(current, reference):
    return (
        (int(current) - int(reference) + COUNTS_PER_REVOLUTION // 2)
        % COUNTS_PER_REVOLUTION
    ) - COUNTS_PER_REVOLUTION // 2


def median(values):
    ordered = sorted(values)
    count = len(ordered)
    middle = count // 2
    if count & 1:
        return float(ordered[middle])
    return (ordered[middle - 1] + ordered[middle]) / 2.0


def sign(value):
    return 1 if value > 0 else -1 if value < 0 else 0


def read_u16(register):
    data = i2c.readfrom_mem(AS5600_ADDRESS, register, 2)
    return (data[0] << 8) | data[1]


def magnet_state(status):
    if status & STATUS_MAGNET_TOO_STRONG:
        return "MAGNET TOO STRONG"
    if status & STATUS_MAGNET_TOO_WEAK:
        return "MAGNET TOO WEAK / NO MAGNET"
    if status & STATUS_MAGNET_DETECTED:
        return "MAGNET OK"
    return "NO MAGNET"


def magnet_valid(status):
    return (
        bool(status & STATUS_MAGNET_DETECTED)
        and not bool(status & STATUS_MAGNET_TOO_WEAK)
        and not bool(status & STATUS_MAGNET_TOO_STRONG)
    )


def read_sensor():
    status = i2c.readfrom_mem(AS5600_ADDRESS, REG_STATUS, 1)[0]
    raw = read_u16(REG_RAW_ANGLE) & 0x0FFF
    angle = read_u16(REG_ANGLE) & 0x0FFF
    agc = i2c.readfrom_mem(AS5600_ADDRESS, REG_AGC, 1)[0]
    magnitude = read_u16(REG_MAGNITUDE) & 0x0FFF
    return {
        "status": status,
        "raw": raw,
        "angle": angle,
        "agc": agc,
        "magnitude": magnitude,
    }


def wait_for_enter(message):
    print()
    print(message)
    input("Press Enter only when the steering is stationary... ")


def capture_stage(name):
    print("Capturing {} for {:.1f} seconds...".format(
        name, CAPTURE_DURATION_MS / 1000.0
    ))
    samples = []
    invalid = 0
    i2c_errors = 0
    agc_values = []
    magnitude_values = []
    first = None
    started = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), started) < CAPTURE_DURATION_MS:
        try:
            sample = read_sensor()
            if not magnet_valid(sample["status"]):
                invalid += 1
            else:
                if first is None:
                    first = sample["angle"]
                samples.append(wrapped_delta(sample["angle"], first))
                agc_values.append(sample["agc"])
                magnitude_values.append(sample["magnitude"])
        except Exception:
            i2c_errors += 1
        time.sleep_ms(SAMPLE_INTERVAL_MS)

    if not samples or first is None:
        raise RuntimeError(
            "{}: no valid AS5600 samples; invalid={} i2c_errors={}".format(
                name, invalid, i2c_errors
            )
        )

    centre_delta = median(samples)
    centred = [value - centre_delta for value in samples]
    mean = sum(centred) / len(centred)
    variance = (
        sum((value - mean) * (value - mean) for value in centred)
        / max(1, len(centred) - 1)
    )
    standard_deviation = math.sqrt(variance)
    peak_to_peak = max(centred) - min(centred)
    representative = int(round(first + centre_delta)) % COUNTS_PER_REVOLUTION
    total = len(samples) + invalid + i2c_errors

    result = {
        "name": name,
        "count": representative,
        "samples": len(samples),
        "total_attempts": total,
        "invalid_magnet_samples": invalid,
        "i2c_errors": i2c_errors,
        "peak_to_peak_counts": peak_to_peak,
        "standard_deviation_counts": standard_deviation,
        "agc_median": median(agc_values),
        "magnitude_median": median(magnitude_values),
    }
    print(
        "{}: count={} valid={}/{} p2p={:.1f} counts ({:.3f} encoder deg) "
        "stddev={:.2f} counts AGC={:.1f} MAG={:.1f}".format(
            name,
            representative,
            len(samples),
            total,
            peak_to_peak,
            peak_to_peak * NATIVE_DEGREES_PER_COUNT,
            standard_deviation,
            result["agc_median"],
            result["magnitude_median"],
        )
    )
    if invalid:
        print("WARNING: {} invalid magnet samples".format(invalid))
    if i2c_errors:
        print("WARNING: {} I2C errors".format(i2c_errors))
    return result


def real_degrees_from_count(raw_count, calibration):
    sensor_delta = wrapped_delta(raw_count, calibration["center_count"])
    normalized = sensor_delta * calibration["right_count_sign"]
    if normalized >= 0:
        return normalized / calibration["right_counts_per_real_degree"]
    return normalized / calibration["left_counts_per_real_degree"]


def calculate_calibration(stages):
    center = stages["center_start"]["count"]
    left_delta = wrapped_delta(stages["left_20"]["count"], center)
    right_delta = wrapped_delta(stages["right_20"]["count"], center)

    failures = []
    warnings = []
    if abs(left_delta) < MIN_ENDPOINT_COUNTS:
        failures.append("LEFT endpoint movement is too small")
    if abs(right_delta) < MIN_ENDPOINT_COUNTS:
        failures.append("RIGHT endpoint movement is too small")
    if sign(left_delta) == sign(right_delta):
        failures.append("LEFT and RIGHT encoder changes are not opposite")

    for item in stages.values():
        if item["invalid_magnet_samples"]:
            failures.append(item["name"] + " has invalid magnet samples")
        if item["i2c_errors"]:
            failures.append(item["name"] + " has I2C errors")
        if item["peak_to_peak_counts"] > MAX_STATIONARY_P2P_COUNTS:
            warnings.append(item["name"] + " stationary peak-to-peak is high")
        if item["standard_deviation_counts"] > MAX_STATIONARY_STDDEV_COUNTS:
            warnings.append(item["name"] + " stationary standard deviation is high")

    if failures:
        return None, failures, warnings

    right_count_sign = sign(right_delta)
    right_counts_per_degree = abs(right_delta) / CALIBRATION_LIMIT_DEG
    left_counts_per_degree = abs(left_delta) / CALIBRATION_LIMIT_DEG
    calibration = {
        "schema_version": 1,
        "sensor": "AS5600",
        "platform": "Pico 2",
        "approved_range_deg": [-CALIBRATION_LIMIT_DEG, CALIBRATION_LIMIT_DEG],
        "center_count": center,
        "left_20_count": stages["left_20"]["count"],
        "right_20_count": stages["right_20"]["count"],
        "left_delta_counts": left_delta,
        "right_delta_counts": right_delta,
        "right_count_sign": right_count_sign,
        "left_counts_per_real_degree": left_counts_per_degree,
        "right_counts_per_real_degree": right_counts_per_degree,
        "left_real_degrees_per_count": 1.0 / left_counts_per_degree,
        "right_real_degrees_per_count": 1.0 / right_counts_per_degree,
        "native_encoder_degrees_left_20": left_delta * NATIVE_DEGREES_PER_COUNT,
        "native_encoder_degrees_right_20": right_delta * NATIVE_DEGREES_PER_COUNT,
        "stages": stages,
    }
    left_encoder_deg_per_real_deg = (
        abs(calibration["native_encoder_degrees_left_20"])
        / CALIBRATION_LIMIT_DEG
    )
    right_encoder_deg_per_real_deg = (
        abs(calibration["native_encoder_degrees_right_20"])
        / CALIBRATION_LIMIT_DEG
    )
    calibration["left_encoder_degrees_per_real_degree"] = (
        left_encoder_deg_per_real_deg
    )
    calibration["right_encoder_degrees_per_real_degree"] = (
        right_encoder_deg_per_real_deg
    )
    calibration["left_real_degrees_per_encoder_degree"] = (
        1.0 / left_encoder_deg_per_real_deg
    )
    calibration["right_real_degrees_per_encoder_degree"] = (
        1.0 / right_encoder_deg_per_real_deg
    )
    scale_mismatch_percent = (
        abs(left_counts_per_degree - right_counts_per_degree)
        / ((left_counts_per_degree + right_counts_per_degree) / 2.0)
        * 100.0
    )
    calibration["left_right_scale_mismatch_percent"] = scale_mismatch_percent
    if scale_mismatch_percent > 10.0:
        warnings.append(
            "LEFT/RIGHT scale differs by {:.1f}%".format(scale_mismatch_percent)
        )

    center_errors = {}
    for key in ("center_after_left", "center_final"):
        error_deg = real_degrees_from_count(stages[key]["count"], calibration)
        center_errors[key] = error_deg
        if abs(error_deg) > MAX_CENTER_RETURN_ERROR_DEG:
            warnings.append(
                "{} return error is {:+.3f} real deg".format(key, error_deg)
            )
    calibration["center_return_errors_deg"] = center_errors
    calibration["warnings"] = warnings
    calibration["result"] = "PASS" if not warnings else "PASS_WITH_WARNINGS"
    return calibration, failures, warnings


def save_calibration(calibration):
    with open(CALIBRATION_FILE, "w") as output:
        json.dump(calibration, output)


def print_results(calibration):
    print()
    print("================ CALIBRATION RESULTS ================")
    print("Approved real steering range: -20.0 to +20.0 deg")
    print("Center raw count:", calibration["center_count"])
    print(
        "LEFT  -20 real deg -> delta {:+d} counts = {:+.3f} encoder deg".format(
            calibration["left_delta_counts"],
            calibration["native_encoder_degrees_left_20"],
        )
    )
    print(
        "RIGHT +20 real deg -> delta {:+d} counts = {:+.3f} encoder deg".format(
            calibration["right_delta_counts"],
            calibration["native_encoder_degrees_right_20"],
        )
    )
    print(
        "LEFT : {:.6f} counts/real-deg | {:.6f} real-deg/count".format(
            calibration["left_counts_per_real_degree"],
            calibration["left_real_degrees_per_count"],
        )
    )
    print(
        "RIGHT: {:.6f} counts/real-deg | {:.6f} real-deg/count".format(
            calibration["right_counts_per_real_degree"],
            calibration["right_real_degrees_per_count"],
        )
    )
    print(
        "LEFT ratio : {:.6f} encoder-deg / real-deg | "
        "{:.6f} real-deg / encoder-deg".format(
            calibration["left_encoder_degrees_per_real_degree"],
            calibration["left_real_degrees_per_encoder_degree"],
        )
    )
    print(
        "RIGHT ratio: {:.6f} encoder-deg / real-deg | "
        "{:.6f} real-deg / encoder-deg".format(
            calibration["right_encoder_degrees_per_real_degree"],
            calibration["right_real_degrees_per_encoder_degree"],
        )
    )
    print(
        "LEFT/RIGHT scale mismatch: {:.2f}%".format(
            calibration["left_right_scale_mismatch_percent"]
        )
    )
    print("Right-direction raw-count sign:", calibration["right_count_sign"])
    for name, error in calibration["center_return_errors_deg"].items():
        print("{} error: {:+.3f} real deg".format(name, error))
    print("Result:", calibration["result"])
    for warning in calibration["warnings"]:
        print("WARNING:", warning)
    print()
    print("Controller constants:")
    print("STEER_CENTER_RAW = {}".format(calibration["center_count"]))
    print("STEER_RIGHT_COUNT_SIGN = {}".format(calibration["right_count_sign"]))
    print(
        "STEER_LEFT_COUNTS_PER_REAL_DEG = {:.9f}".format(
            calibration["left_counts_per_real_degree"]
        )
    )
    print(
        "STEER_RIGHT_COUNTS_PER_REAL_DEG = {:.9f}".format(
            calibration["right_counts_per_real_degree"]
        )
    )
    print("STEER_MIN_REAL_DEG = -20.0")
    print("STEER_MAX_REAL_DEG = 20.0")
    print("Saved:", CALIBRATION_FILE)


print()
print("========================================================")
print(" PICO 2 AS5600 REAL STEERING ANGLE CALIBRATION")
print("========================================================")
print("READ ONLY: this program never commands the NEMA34 motor.")
print("Wiring: SDA GP0/pin 1 | SCL GP1/pin 2 | AS5600 0x36")
print("Required measured positions: CENTER, LEFT -20 deg, RIGHT +20 deg")

i2c = I2C(
    I2C_ID,
    sda=Pin(SDA_GPIO),
    scl=Pin(SCL_GPIO),
    freq=I2C_FREQUENCY,
)
devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])
if AS5600_ADDRESS not in devices:
    raise RuntimeError(
        "AS5600 not detected at 0x36; check 3V3, GND, GP0 SDA and GP1 SCL"
    )

initial = read_sensor()
print(
    "Initial magnet: {} | raw {} | AGC {} | magnitude {}".format(
        magnet_state(initial["status"]),
        initial["angle"],
        initial["agc"],
        initial["magnitude"],
    )
)
if not magnet_valid(initial["status"]):
    raise RuntimeError(
        "MAGNET PREFLIGHT FAILED: " + magnet_state(initial["status"])
    )

print()
print("Use a mechanical angle gauge/protractor for the real +/-20 deg positions.")
confirmation = input("Type CALIBRATE and press Enter to begin: ").strip()
if confirmation != "CALIBRATE":
    raise RuntimeError("Calibration cancelled: exact confirmation not received")

stages = {}
wait_for_enter("1/5: Place steering at the true mechanical CENTER (0 deg).")
stages["center_start"] = capture_stage("CENTER_START")

wait_for_enter("2/5: Place steering at real LEFT -20.0 deg and hold it.")
stages["left_20"] = capture_stage("LEFT_-20")

wait_for_enter("3/5: Return steering to the true mechanical CENTER (0 deg).")
stages["center_after_left"] = capture_stage("CENTER_AFTER_LEFT")

wait_for_enter("4/5: Place steering at real RIGHT +20.0 deg and hold it.")
stages["right_20"] = capture_stage("RIGHT_+20")

wait_for_enter("5/5: Return steering to the true mechanical CENTER (0 deg).")
stages["center_final"] = capture_stage("CENTER_FINAL")

calibration, failures, warnings = calculate_calibration(stages)
if calibration is None:
    print()
    print("CALIBRATION FAILED")
    for failure in failures:
        print("FAIL:", failure)
    for warning in warnings:
        print("WARNING:", warning)
    raise RuntimeError("Calibration rejected; fix magnet/mechanics and repeat")

save_calibration(calibration)
print_results(calibration)
print()
print("Do not use this calibration for vehicle motion until its repeatability")
print("has been confirmed by at least three complete calibration runs.")

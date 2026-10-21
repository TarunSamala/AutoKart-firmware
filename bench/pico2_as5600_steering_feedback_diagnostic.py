"""Pico 2 + one AS5600 steering-feedback filter and fault diagnostic.

Direct isolated wiring:
  AS5600 VCC -> Pico 2 3V3(OUT), physical pin 36
  AS5600 GND -> Pico 2 GND, physical pin 38
  AS5600 SDA -> Pico 2 GP0, physical pin 1
  AS5600 SCL -> Pico 2 GP1, physical pin 2

This program initializes no steering motor or actuator output. It detects I2C
errors, bad magnet states, impossible angle jumps/rates, stale samples,
calibrated travel-limit violations, and stationary encoder noise.
"""

from machine import I2C, Pin
import math
import time

from encoder_filters import AS5600AngleFilter, monotonic_us, wrapped_delta


I2C_ID = 0
SDA_GPIO = 0
SCL_GPIO = 1
I2C_FREQUENCY = 400_000
AS5600_ADDRESS = 0x36
ZERO_FILE = "steering_as5600_zero.txt"

SAMPLE_INTERVAL_MS = 5              # 200 Hz
DISPLAY_INTERVAL_MS = 100           # 10 Hz terminal display
SUMMARY_INTERVAL_MS = 1000
STALE_TIMEOUT_MS = 50

# Initial bench limits. Revalidate on the installed steering assembly.
MAX_STEERING_RATE_DPS = 1000.0
MAX_STEERING_ANGLE_DEG = 90.0
NOISE_TEST_SECONDS = 5
NOISE_P2P_WARNING_DEG = 0.50
NOISE_STDDEV_WARNING_DEG = 0.10

REG_STATUS = 0x0B
REG_RAW_ANGLE = 0x0C
REG_ANGLE = 0x0E
REG_AGC = 0x1A
REG_MAGNITUDE = 0x1B

STATUS_MAGNET_TOO_STRONG = 0x08
STATUS_MAGNET_TOO_WEAK = 0x10
STATUS_MAGNET_DETECTED = 0x20

COUNTS_PER_REVOLUTION = 4096
DEGREES_PER_COUNT = 360.0 / COUNTS_PER_REVOLUTION


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


def magnet_is_valid(status):
    return (
        bool(status & STATUS_MAGNET_DETECTED)
        and not bool(status & STATUS_MAGNET_TOO_WEAK)
        and not bool(status & STATUS_MAGNET_TOO_STRONG)
    )


def load_zero():
    try:
        with open(ZERO_FILE, "r") as file:
            value = int(file.read().strip())
        if 0 <= value < COUNTS_PER_REVOLUTION:
            return value
    except (OSError, ValueError):
        pass
    return None


def save_zero(raw_count):
    with open(ZERO_FILE, "w") as file:
        file.write(str(int(raw_count) & 0x0FFF))


def remove_zero():
    try:
        import os
        os.remove(ZERO_FILE)
    except OSError:
        pass


def elapsed_us(current, previous):
    return time.ticks_diff(current, previous)


i2c = I2C(
    I2C_ID,
    sda=Pin(SDA_GPIO),
    scl=Pin(SCL_GPIO),
    freq=I2C_FREQUENCY,
)

angle_filter = AS5600AngleFilter(
    time_constant_s=0.010,
    maximum_rate_dps=MAX_STEERING_RATE_DPS,
    direction=1,
)

saved_zero = load_zero()
if saved_zero is not None:
    angle_filter.set_zero_raw(saved_zero)

latest = None
previous_observed_raw = None
previous_observed_us = None
last_valid_us = None
last_fault_signature = None

counters = {
    "samples": 0,
    "valid": 0,
    "i2c_errors": 0,
    "magnet_errors": 0,
    "jump_errors": 0,
    "stale_errors": 0,
    "range_warnings": 0,
}


def acquire_sample():
    global latest
    global previous_observed_raw, previous_observed_us, last_valid_us

    timestamp_us = monotonic_us()
    counters["samples"] += 1

    try:
        status = i2c.readfrom_mem(AS5600_ADDRESS, REG_STATUS, 1)[0]
        raw = read_u16(REG_RAW_ANGLE) & 0x0FFF
        sensor_angle = read_u16(REG_ANGLE) & 0x0FFF
        magnitude = read_u16(REG_MAGNITUDE) & 0x0FFF
        agc = i2c.readfrom_mem(AS5600_ADDRESS, REG_AGC, 1)[0]
    except Exception as error:
        counters["i2c_errors"] += 1
        latest = {
            "valid": False,
            "faults": ("I2C ERROR",),
            "exception": str(error),
            "timestamp_us": timestamp_us,
        }
        return latest

    faults = []
    state = magnet_state(status)
    magnet_ok = magnet_is_valid(status)
    if not magnet_ok:
        counters["magnet_errors"] += 1
        faults.append(state)

    delta_deg = 0.0
    rate_dps = 0.0
    if previous_observed_raw is not None:
        sample_elapsed_us = elapsed_us(timestamp_us, previous_observed_us)
        if sample_elapsed_us > 0:
            delta_counts = wrapped_delta(
                sensor_angle,
                previous_observed_raw,
                COUNTS_PER_REVOLUTION,
            )
            delta_deg = delta_counts * DEGREES_PER_COUNT
            rate_dps = delta_deg / (sample_elapsed_us / 1_000_000.0)
            if abs(rate_dps) > MAX_STEERING_RATE_DPS:
                counters["jump_errors"] += 1
                faults.append("ANGLE JUMP / EXCESSIVE RATE")

    previous_observed_raw = sensor_angle
    previous_observed_us = timestamp_us

    result = angle_filter.update(
        sensor_angle,
        timestamp_us=timestamp_us,
        magnet_ok=magnet_ok,
    )
    if result is not None:
        last_valid_us = timestamp_us
        counters["valid"] += 1
        if (
            result["zero_valid"]
            and abs(result["angle_deg"]) > MAX_STEERING_ANGLE_DEG
        ):
            counters["range_warnings"] += 1
            faults.append("STEERING OUT OF RANGE")

    stale_ms = 0
    if last_valid_us is not None:
        stale_ms = elapsed_us(timestamp_us, last_valid_us) // 1000
        if stale_ms > STALE_TIMEOUT_MS:
            counters["stale_errors"] += 1
            faults.append("STALE FEEDBACK")

    latest = {
        "valid": result is not None,
        "status": status,
        "state": state,
        "raw": raw,
        "sensor_angle": sensor_angle,
        "absolute_deg": sensor_angle * DEGREES_PER_COUNT,
        "magnitude": magnitude,
        "agc": agc,
        "delta_deg": delta_deg,
        "rate_dps": rate_dps,
        "filtered": result,
        "stale_ms": stale_ms,
        "faults": tuple(faults),
        "timestamp_us": timestamp_us,
    }
    return latest


def print_sample(sample=None):
    sample = sample or latest
    if sample is None:
        print("No sample available")
        return
    if "exception" in sample:
        print("I2C ERROR:", sample["exception"])
        return

    result = sample["filtered"]
    filtered_text = "REJECTED"
    if result is not None:
        filtered_text = "{:+9.3f} deg".format(result["angle_deg"])
    fault_text = "OK" if not sample["faults"] else ", ".join(sample["faults"])
    print(
        "RAW {:4d} | ABS {:8.3f} deg | STEERING {} | "
        "RATE {:+8.1f} deg/s | MAG {:4d} AGC {:3d} {} | {}".format(
            sample["raw"],
            sample["absolute_deg"],
            filtered_text,
            sample["rate_dps"],
            sample["magnitude"],
            sample["agc"],
            sample["state"],
            fault_text,
        )
    )


def print_summary():
    total = counters["samples"]
    valid_percent = 0.0 if total == 0 else counters["valid"] * 100.0 / total
    print(
        "SUMMARY samples={} valid={:.1f}% i2c={} magnet={} jumps={} "
        "stale={} range={} filter_rejected={}".format(
            total,
            valid_percent,
            counters["i2c_errors"],
            counters["magnet_errors"],
            counters["jump_errors"],
            counters["stale_errors"],
            counters["range_warnings"],
            angle_filter.rejected_samples,
        )
    )


def reset_statistics():
    global previous_observed_raw, previous_observed_us, last_valid_us
    global last_fault_signature
    for key in counters:
        counters[key] = 0
    previous_observed_raw = None
    previous_observed_us = None
    last_valid_us = None
    last_fault_signature = None
    angle_filter.reset()
    if saved_zero is not None:
        angle_filter.set_zero_raw(saved_zero)
    print("Diagnostic counters and filter history reset")


def collect(duration_ms):
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < duration_ms:
        acquire_sample()
        time.sleep_ms(SAMPLE_INTERVAL_MS)


def monitor_errors():
    global last_fault_signature
    print("Monitoring faults at 200 Hz. Ctrl-C returns.")
    last_summary_ms = time.ticks_ms()
    while True:
        sample = acquire_sample()
        signature = sample["faults"]
        if signature != last_fault_signature:
            if signature:
                print("FAULT STATE CHANGED:", ", ".join(signature))
                print_sample(sample)
            elif last_fault_signature:
                print("FAULT CLEARED")
                print_sample(sample)
            last_fault_signature = signature

        now = time.ticks_ms()
        if time.ticks_diff(now, last_summary_ms) >= SUMMARY_INTERVAL_MS:
            print_summary()
            last_summary_ms = now
        time.sleep_ms(SAMPLE_INTERVAL_MS)


def stationary_noise_test():
    print("Keep the steering and magnet completely stationary for 5 seconds.")
    first_count = None
    count = 0
    mean = 0.0
    m2 = 0.0
    minimum = None
    maximum = None
    invalid = 0
    started = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), started) < NOISE_TEST_SECONDS * 1000:
        sample = acquire_sample()
        if "sensor_angle" not in sample or not magnet_is_valid(sample["status"]):
            invalid += 1
            time.sleep_ms(SAMPLE_INTERVAL_MS)
            continue

        if first_count is None:
            first_count = sample["sensor_angle"]
        value = wrapped_delta(
            sample["sensor_angle"], first_count, COUNTS_PER_REVOLUTION
        ) * DEGREES_PER_COUNT
        count += 1
        difference = value - mean
        mean += difference / count
        m2 += difference * (value - mean)
        minimum = value if minimum is None else min(minimum, value)
        maximum = value if maximum is None else max(maximum, value)
        time.sleep_ms(SAMPLE_INTERVAL_MS)

    if count < 2:
        print("NOISE TEST FAILED: insufficient valid samples; invalid={}".format(invalid))
        return

    standard_deviation = math.sqrt(m2 / (count - 1))
    peak_to_peak = maximum - minimum
    passed = (
        peak_to_peak <= NOISE_P2P_WARNING_DEG
        and standard_deviation <= NOISE_STDDEV_WARNING_DEG
        and invalid == 0
    )
    print(
        "NOISE {}: samples={} invalid={} peak-to-peak={:.4f} deg "
        "stddev={:.4f} deg mean_drift={:+.4f} deg".format(
            "PASS" if passed else "WARNING",
            count,
            invalid,
            peak_to_peak,
            standard_deviation,
            mean,
        )
    )
    if not passed:
        print("Check magnet alignment, air gap, shaft play, wiring and supply noise.")


def calibrate_zero():
    global saved_zero
    collect(200)
    if latest is None or "status" not in latest:
        print("ZERO REJECTED: no sensor reading")
    elif not magnet_is_valid(latest["status"]):
        print("ZERO REJECTED:", latest["state"])
    elif angle_filter.calibrate_zero():
        saved_zero = latest["sensor_angle"]
        save_zero(saved_zero)
        print("CENTER CALIBRATED: current position = 0.000 degrees")
        print("Saved zero count:", saved_zero)
    else:
        print("ZERO REJECTED: no valid filtered sample")


def show_status():
    print("SDA GP0/pin 1 | SCL GP1/pin 2 | AS5600 0x36")
    print("I2C devices:", [hex(device) for device in i2c.scan()])
    print("Saved center:", saved_zero if saved_zero is not None else "NOT CALIBRATED")
    print("Sample rate: 200 Hz | LPF: 10 ms | maximum rate: 1000 deg/s")
    print("Travel warning: +/-{:.1f} deg | stale timeout: {} ms".format(
        MAX_STEERING_ANGLE_DEG, STALE_TIMEOUT_MS
    ))
    print_sample()
    print_summary()


def show_help():
    print("Commands:")
    print("  read / r  - collect for 0.5 seconds and show a reading")
    print("  stream    - continuous readings at 10 Hz display")
    print("  monitor   - report fault transitions and 1-second summaries")
    print("  noise     - 5-second stationary noise/jitter test")
    print("  zero      - save current steering position as center 0 degrees")
    print("  clear     - remove saved center calibration")
    print("  reset     - reset filter history and diagnostic counters")
    print("  status    - show limits, reading and counters")
    print("  help      - show commands")
    print("  q         - stop")


print()
print("================================================")
print(" PICO 2 AS5600 STEERING FEEDBACK DIAGNOSTIC")
print("================================================")
print("ENCODER ONLY: NO STEERING MOTOR OUTPUTS INITIALIZED")

devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])
if AS5600_ADDRESS not in devices:
    raise RuntimeError("AS5600 not detected at 0x36")

collect(250)
print_sample()
show_help()

while True:
    try:
        command = input("\nSteering encoder > ").strip().lower()

        if command in ("r", "read"):
            collect(500)
            print_sample()

        elif command == "stream":
            print("Sampling at 200 Hz; displaying at 10 Hz. Ctrl-C returns.")
            try:
                while True:
                    collect(DISPLAY_INTERVAL_MS)
                    print_sample()
            except KeyboardInterrupt:
                print("Streaming stopped.")

        elif command == "monitor":
            try:
                monitor_errors()
            except KeyboardInterrupt:
                print("Fault monitor stopped.")

        elif command == "noise":
            stationary_noise_test()

        elif command == "zero":
            calibrate_zero()

        elif command == "clear":
            remove_zero()
            saved_zero = None
            angle_filter.zero_counts = None
            print("Saved steering center cleared")

        elif command == "reset":
            reset_statistics()

        elif command == "status":
            collect(250)
            show_status()

        elif command in ("help", "?"):
            show_help()

        elif command in ("q", "quit", "exit"):
            print("Steering feedback diagnostic stopped")
            break

        elif command:
            print("Unknown command. Type help.")

    except KeyboardInterrupt:
        print("\nUse q to stop or help for commands.")
    except Exception as error:
        print("STEERING ENCODER ERROR:", error)

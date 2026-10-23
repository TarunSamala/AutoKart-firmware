"""Pico 2 + AS5600 steering validation for the AutoKart RC.

Direct wiring:
  AS5600 VCC -> Pico 2 3V3(OUT), physical pin 36
  AS5600 GND -> Pico 2 GND, physical pin 38
  AS5600 SDA -> Pico 2 GP0, physical pin 1
  AS5600 SCL -> Pico 2 GP1, physical pin 2

Before booting, mechanically center the steering. Every boot treats the
current valid position as 0 degrees. The AutoKart convention is:
  LEFT   = -35 degrees
  CENTER =   0 degrees
  RIGHT  = +35 degrees

This is an encoder-only bench test. It does not drive the steering motor.
"""

from machine import I2C, Pin
import time


I2C_ID = 0
SDA_GPIO = 0
SCL_GPIO = 1
I2C_FREQUENCY = 100_000
AS5600_ADDRESS = 0x36

REG_STATUS = 0x0B
REG_RAW_ANGLE = 0x0C
REG_AGC = 0x1A
REG_MAGNITUDE = 0x1B

MAGNET_DETECTED = 0x20
MAGNET_TOO_WEAK = 0x10
MAGNET_TOO_STRONG = 0x08

COUNTS_PER_REV = 4096
DEGREES_PER_COUNT = 360.0 / COUNTS_PER_REV

TARGET_DEG = 35.0
TARGET_TOLERANCE_DEG = 3.0
CENTER_TOLERANCE_DEG = 2.0
TRAVEL_WARNING_DEG = 40.0

SAMPLE_INTERVAL_MS = 10       # 100 Hz acquisition
DISPLAY_INTERVAL_MS = 100     # 10 Hz terminal output
MAX_RATE_DPS = 1000.0
FILTER_TIME_CONSTANT_S = 0.025
I2C_RETRIES = 3

# Existing RC convention: increasing AS5600 angle should mean RIGHT/positive.
# Type "invert" at the prompt if your physical mounting reports the opposite.
direction = 1


def wrapped_delta(current, previous):
    return ((int(current) - int(previous) + 2048) % 4096) - 2048


def read_register(register, length=1):
    last_error = None
    for attempt in range(I2C_RETRIES):
        try:
            return i2c.readfrom_mem(AS5600_ADDRESS, register, length)
        except OSError as error:
            last_error = error
            time.sleep_ms(5 * (attempt + 1))
    raise OSError(
        "register 0x{:02X} failed after {} attempts: {}".format(
            register, I2C_RETRIES, last_error
        )
    )


def read_u16(register):
    data = read_register(register, 2)
    return (data[0] << 8) | data[1]


def magnet_text(status):
    if status & MAGNET_TOO_STRONG:
        return "MAGNET TOO STRONG"
    if status & MAGNET_TOO_WEAK:
        return "MAGNET TOO WEAK"
    if status & MAGNET_DETECTED:
        return "MAGNET OK"
    return "NO MAGNET"


def magnet_ok(status):
    return (
        bool(status & MAGNET_DETECTED)
        and not bool(status & MAGNET_TOO_WEAK)
        and not bool(status & MAGNET_TOO_STRONG)
    )


i2c = I2C(
    I2C_ID,
    sda=Pin(SDA_GPIO),
    scl=Pin(SCL_GPIO),
    freq=I2C_FREQUENCY,
)

zero_raw = None
last_raw = None
last_sample_us = None
unwrapped_counts = None
filtered_counts = None
median_window = []
latest = None
valid_samples = 0
rejected_samples = 0
i2c_errors = 0


def reset_filter():
    global last_raw, last_sample_us, unwrapped_counts, filtered_counts
    global median_window, latest
    last_raw = None
    last_sample_us = None
    unwrapped_counts = None
    filtered_counts = None
    median_window = []
    latest = None


def read_sensor():
    status = read_register(REG_STATUS)[0]
    raw = read_u16(REG_RAW_ANGLE) & 0x0FFF
    magnitude = read_u16(REG_MAGNITUDE) & 0x0FFF
    agc = read_register(REG_AGC)[0]
    return status, raw, magnitude, agc


def acquire_sample():
    global last_raw, last_sample_us, unwrapped_counts, filtered_counts
    global median_window, latest, valid_samples, rejected_samples, i2c_errors

    now_us = time.ticks_us()
    try:
        status, raw, magnitude, agc = read_sensor()
    except OSError as error:
        i2c_errors += 1
        latest = {"valid": False, "error": str(error)}
        return latest

    if not magnet_ok(status):
        rejected_samples += 1
        latest = {
            "valid": False,
            "raw": raw,
            "magnet": magnet_text(status),
            "magnitude": magnitude,
            "agc": agc,
        }
        return latest

    if last_raw is None:
        last_raw = raw
        last_sample_us = now_us
        unwrapped_counts = float(raw)
        filtered_counts = float(raw)
        median_window = [float(raw)]
    else:
        elapsed_us = time.ticks_diff(now_us, last_sample_us)
        if elapsed_us <= 0:
            rejected_samples += 1
            return latest

        delta = wrapped_delta(raw, last_raw)
        rate_dps = abs(delta * DEGREES_PER_COUNT / (elapsed_us / 1_000_000.0))
        if rate_dps > MAX_RATE_DPS:
            rejected_samples += 1
            latest = {
                "valid": False,
                "raw": raw,
                "magnet": "ANGLE JUMP REJECTED",
                "magnitude": magnitude,
                "agc": agc,
            }
            return latest

        last_raw = raw
        last_sample_us = now_us
        unwrapped_counts += delta
        median_window.append(unwrapped_counts)
        if len(median_window) > 3:
            median_window.pop(0)
        median_counts = sorted(median_window)[len(median_window) // 2]
        elapsed_s = elapsed_us / 1_000_000.0
        alpha = elapsed_s / (FILTER_TIME_CONSTANT_S + elapsed_s)
        filtered_counts += alpha * (median_counts - filtered_counts)

    valid_samples += 1
    angle = None
    if zero_raw is not None:
        angle = (filtered_counts - zero_raw) * DEGREES_PER_COUNT * direction

    latest = {
        "valid": True,
        "raw": raw,
        "angle": angle,
        "magnet": magnet_text(status),
        "magnitude": magnitude,
        "agc": agc,
    }
    return latest


def collect(duration_ms):
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < duration_ms:
        acquire_sample()
        time.sleep_ms(SAMPLE_INTERVAL_MS)


def calibrate_center():
    global zero_raw
    reset_filter()
    values = []
    started = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), started) < 600:
        sample = acquire_sample()
        if sample and sample.get("valid") and filtered_counts is not None:
            values.append(filtered_counts)
        time.sleep_ms(SAMPLE_INTERVAL_MS)

    if len(values) < 20:
        print("ZERO FAILED: insufficient valid samples; keep steering still")
        return False

    zero_raw = sorted(values)[len(values) // 2]
    print("CENTER ZEROED: current steering = 0.00 deg")
    print("Zero count: {:.1f} | samples: {}".format(zero_raw, len(values)))
    return True


def position_label(angle):
    if abs(angle) <= CENTER_TOLERANCE_DEG:
        return "CENTER PASS"
    if abs(angle + TARGET_DEG) <= TARGET_TOLERANCE_DEG:
        return "LEFT -35 PASS"
    if abs(angle - TARGET_DEG) <= TARGET_TOLERANCE_DEG:
        return "RIGHT +35 PASS"
    if angle < -TRAVEL_WARNING_DEG:
        return "LEFT OVER-TRAVEL"
    if angle > TRAVEL_WARNING_DEG:
        return "RIGHT OVER-TRAVEL"
    return "MOVING"


def print_sample(sample=None):
    sample = sample or latest
    if not sample:
        print("NO SAMPLE")
        return
    if not sample.get("valid"):
        print(
            "INVALID | {} | raw {} | I2C errors {} | rejected {}".format(
                sample.get("error", sample.get("magnet", "UNKNOWN")),
                sample.get("raw", "--"), i2c_errors, rejected_samples
            )
        )
        return
    angle = sample.get("angle")
    if angle is None:
        angle_text = "NOT ZEROED"
        label = "CALIBRATE CENTER"
    else:
        angle_text = "{:+7.2f} deg".format(angle)
        label = position_label(angle)
    print(
        "STEERING {} | {:18s} | RAW {:4d} | {} | MAG {:4d} AGC {:3d}".format(
            angle_text, label, sample["raw"], sample["magnet"],
            sample["magnitude"], sample["agc"]
        )
    )


def stream():
    print("Streaming at 10 Hz. Ctrl-C returns to commands.")
    try:
        while True:
            collect(DISPLAY_INTERVAL_MS)
            print_sample()
    except KeyboardInterrupt:
        print("Streaming stopped")


def validation_test():
    seen = {"center": False, "left": False, "right": False}
    minimum = 999.0
    maximum = -999.0
    print("Operate the RC: CENTER -> LEFT 35 -> CENTER -> RIGHT 35 -> CENTER")
    print("Ctrl-C when finished to print the validation result.")
    try:
        while True:
            collect(DISPLAY_INTERVAL_MS)
            sample = latest
            if sample and sample.get("valid") and sample.get("angle") is not None:
                angle = sample["angle"]
                minimum = min(minimum, angle)
                maximum = max(maximum, angle)
                if abs(angle) <= CENTER_TOLERANCE_DEG:
                    seen["center"] = True
                if abs(angle + TARGET_DEG) <= TARGET_TOLERANCE_DEG:
                    seen["left"] = True
                if abs(angle - TARGET_DEG) <= TARGET_TOLERANCE_DEG:
                    seen["right"] = True
            print_sample(sample)
    except KeyboardInterrupt:
        print("\nRC STEERING VALIDATION")
        print("  CENTER 0 deg :", "PASS" if seen["center"] else "NOT REACHED")
        print("  LEFT -35 deg :", "PASS" if seen["left"] else "NOT REACHED")
        print("  RIGHT +35 deg:", "PASS" if seen["right"] else "NOT REACHED")
        if minimum < 900:
            print("  Observed range: {:+.2f} to {:+.2f} deg".format(minimum, maximum))
        passed = seen["center"] and seen["left"] and seen["right"]
        print("  RESULT:", "PASS" if passed else "INCOMPLETE / FAIL")
        print("  I2C errors: {} | rejected samples: {}".format(
            i2c_errors, rejected_samples
        ))


def show_help():
    print("Commands:")
    print("  test    - validate center, left -35 and right +35 using the RC")
    print("  stream  - continuously show angle and position")
    print("  read    - show one filtered reading")
    print("  zero    - make the current physical position 0 degrees")
    print("  invert  - reverse left/right encoder sign")
    print("  status  - show wiring, direction and counters")
    print("  help    - show commands")
    print("  q       - stop")


print()
print("================================================")
print(" PICO 2 AS5600 RC STEERING +/-35 DEG VALIDATION")
print("================================================")
print("GP0 SDA/pin 1 | GP1 SCL/pin 2 | AS5600 0x36")
print("CENTER STEERING BEFORE STARTUP")

devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])
if AS5600_ADDRESS not in devices:
    raise RuntimeError("AS5600 not detected at 0x36; check 3V3, GND, GP0 and GP1")

if not calibrate_center():
    print("Fix the encoder/magnet and type zero to retry.")
else:
    collect(150)
    print_sample()

show_help()

while True:
    try:
        command = input("\nSteering encoder > ").strip().lower()
        if command == "test":
            validation_test()
        elif command == "stream":
            stream()
        elif command in ("read", "r"):
            collect(300)
            print_sample()
        elif command == "zero":
            calibrate_center()
            collect(100)
            print_sample()
        elif command == "invert":
            direction *= -1
            print("Direction inverted. LEFT must now read negative.")
            collect(100)
            print_sample()
        elif command == "status":
            print("GP0 SDA | GP1 SCL | 100 kHz | address 0x36")
            print("Direction:", direction, "(LEFT negative, RIGHT positive)")
            print("Target: +/-35 deg | tolerance: +/-3 deg")
            print("Valid: {} | rejected: {} | I2C errors: {}".format(
                valid_samples, rejected_samples, i2c_errors
            ))
            print_sample()
        elif command in ("help", "?"):
            show_help()
        elif command in ("q", "quit", "exit"):
            print("Steering encoder test stopped")
            break
        elif command:
            print("Unknown command. Type help.")
    except KeyboardInterrupt:
        print("\nUse q to stop or help for commands.")
    except Exception as error:
        print("STEERING ENCODER ERROR:", error)

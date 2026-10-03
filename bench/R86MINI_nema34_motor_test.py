"""ESP32-S3 + R86mini + NEMA34 open-loop steering test.

The steering shaft is direct drive (1:1). At the configured 3200 pulses per
motor revolution, 90 steering degrees equals 800 pulses.

IMPORTANT: This test has no encoder or limit switches. Mechanically center the
steering before boot/reset; the program then treats that position as 0 degrees.

Motor/driver wiring verified experimentally:
  R86mini A+ = RED + BLUE
  R86mini A- = YELLOW + BLACK
  R86mini B+ = ORANGE + GREEN
  R86mini B- = BROWN + WHITE

Control interface:
  GPIO4 -> tested 5 V NPN interface -> PUL-
  GPIO5 -> tested 5 V NPN interface -> DIR-
  5 V -> PUL+ and DIR+
  ENA+ and ENA- disconnected
  ESP32 GND and interface/driver signal GND must be common

R86mini pulse setting:
  3200 pulses/revolution: SW5 OFF, SW6 OFF, SW7 ON, SW8 ON
"""

from machine import Pin
import time


FIRMWARE_NAME = "r86mini-nema34-steering-test"
FIRMWARE_VERSION = "2.1.0"

STEP_GPIO = 4
DIR_GPIO = 5

STEP = Pin(STEP_GPIO, Pin.OUT, value=0)
DIR = Pin(DIR_GPIO, Pin.OUT, value=0)

ACTIVE = 1
INACTIVE = 0

# Verified convention from the earlier steering firmware.
RIGHT_DIRECTION = 1
LEFT_DIRECTION = 0

PULSES_PER_REVOLUTION = 3200
STEERING_RATIO = 1.0
PULSES_PER_DEGREE = (
    PULSES_PER_REVOLUTION * STEERING_RATIO / 360.0
)

MIN_ANGLE_DEG = -90.0
MAX_ANGLE_DEG = 90.0

MIN_PPS = 100
MAX_PPS = 2500
START_PPS = 200
MAX_RAMP_STEPS = 160
STEP_HIGH_US = 50
DIR_SETTLE_MS = 100

SWEEP_STEP_DEG = 2
SWEEP_DELAY_MS = 1000

pulse_rate_pps = 1200
current_position_pulses = 0


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def degrees_to_pulses(degrees):
    return round(float(degrees) * PULSES_PER_DEGREE)


def pulses_to_degrees(pulses):
    return float(pulses) / PULSES_PER_DEGREE


def pulse_steps(count):
    """Move a known number of pulses with symmetric acceleration."""
    ramp_steps = min(MAX_RAMP_STEPS, count // 2)
    start_rate = min(START_PPS, pulse_rate_pps)

    for index in range(count):
        if ramp_steps:
            distance_from_end = count - index - 1
            ramp_position = min(index, distance_from_end, ramp_steps)
            rate = start_rate + (
                (pulse_rate_pps - start_rate)
                * ramp_position
                // ramp_steps
            )
        else:
            rate = start_rate

        period_us = 1_000_000 // max(1, rate)
        low_us = max(1, period_us - STEP_HIGH_US)

        STEP.value(ACTIVE)
        time.sleep_us(STEP_HIGH_US)
        STEP.value(INACTIVE)
        time.sleep_us(low_us)


def move_to_angle(requested_degrees):
    """Move from the estimated position to an absolute steering angle."""
    global current_position_pulses

    requested_degrees = float(requested_degrees)
    target_degrees = clamp(
        requested_degrees,
        MIN_ANGLE_DEG,
        MAX_ANGLE_DEG,
    )
    target_pulses = degrees_to_pulses(target_degrees)
    delta_pulses = target_pulses - current_position_pulses

    if target_degrees != requested_degrees:
        print(
            "ANGLE CLAMPED:",
            requested_degrees,
            "->",
            target_degrees,
            "degrees",
        )

    if delta_pulses == 0:
        print("ALREADY AT", round(target_degrees, 2), "degrees")
        return

    direction_name = "RIGHT" if delta_pulses > 0 else "LEFT"
    DIR.value(RIGHT_DIRECTION if delta_pulses > 0 else LEFT_DIRECTION)
    time.sleep_ms(DIR_SETTLE_MS)

    print(
        "MOVE:", direction_name,
        "| from:", round(pulses_to_degrees(current_position_pulses), 2),
        "deg | to:", round(target_degrees, 2),
        "deg | pulses:", abs(delta_pulses),
    )

    pulse_steps(abs(delta_pulses))
    current_position_pulses = target_pulses

    print(
        "DONE | estimated steering:",
        round(pulses_to_degrees(current_position_pulses), 2),
        "degrees",
    )


def set_speed(requested_pps):
    global pulse_rate_pps

    requested_pps = int(requested_pps)
    pulse_rate_pps = int(clamp(requested_pps, MIN_PPS, MAX_PPS))

    if pulse_rate_pps != requested_pps:
        print("SPEED CLAMPED:", requested_pps, "->", pulse_rate_pps)

    print("STEERING SPEED:", pulse_rate_pps, "pulses/second")


def calibrate_center_here():
    """Assign the current physical steering position as software center."""
    global current_position_pulses
    current_position_pulses = 0
    print()
    print("CENTER CALIBRATED")
    print("Current physical steering position is now software 0 degrees.")
    print("No motor movement was commanded.")
    print()


def sweep_targets(direction):
    """Generate 2-degree targets and include the exact configured limit."""
    angle = SWEEP_STEP_DEG

    while angle < MAX_ANGLE_DEG:
        yield direction * angle
        angle += SWEEP_STEP_DEG

    yield direction * MAX_ANGLE_DEG


def continuous_steering_test():
    """Continuously sweep center -> right limit -> center -> left limit."""
    print()
    print("CONTINUOUS STEERING TEST STARTED")
    print("Each target changes by 2 degrees with a 1 second pause.")
    print("Sequence: 0 -> +90 -> 0 -> -90 -> 0 -> repeat")
    print("Press Ctrl+C to stop the sweep and return to the command prompt.")
    print()

    try:
        move_to_angle(0)
        time.sleep_ms(SWEEP_DELAY_MS)

        while True:
            for target in sweep_targets(1):
                move_to_angle(target)
                time.sleep_ms(SWEEP_DELAY_MS)

            move_to_angle(0)
            time.sleep_ms(SWEEP_DELAY_MS)

            for target in sweep_targets(-1):
                move_to_angle(target)
                time.sleep_ms(SWEEP_DELAY_MS)

            move_to_angle(0)
            time.sleep_ms(SWEEP_DELAY_MS)

    except KeyboardInterrupt:
        STEP.value(INACTIVE)
        print("\nCONTINUOUS SWEEP STOPPED")
        print(
            "Estimated position:",
            round(pulses_to_degrees(current_position_pulses), 2),
            "degrees",
        )
        print("Use 'center' to return to 0 degrees.")


def status():
    print()
    print("========================================")
    print("R86mini + NEMA34 STEERING TEST")
    print("========================================")
    print("Firmware          :", FIRMWARE_NAME, FIRMWARE_VERSION)
    print("STEP / DIR GPIO   :", STEP_GPIO, "/", DIR_GPIO)
    print("Pulses/rev        :", PULSES_PER_REVOLUTION)
    print("Steering ratio    : 1:1")
    print("Pulses/degree     :", round(PULSES_PER_DEGREE, 4))
    print("90 degree pulses  :", degrees_to_pulses(90))
    print("Software limits   : -90 to +90 degrees")
    print(
        "Estimated position:",
        round(pulses_to_degrees(current_position_pulses), 2),
        "degrees",
    )
    print("Maximum pulse rate:", pulse_rate_pps, "pps")
    print("Position feedback : NONE (open-loop)")
    print("ENA               : disconnected")
    print("========================================")
    print()


def help_text():
    print()
    print("Commands:")
    print("  left       -> move to -90 degrees")
    print("  left 30    -> move to -30 degrees")
    print("  center     -> move to 0 degrees")
    print("  right      -> move to +90 degrees")
    print("  right 30   -> move to +30 degrees")
    print("  angle 20   -> move to any angle from -90 to +90")
    print("  sweep      -> continuously test 0, +2...+90, 0, -2...-90")
    print("  speed 1200 -> set 100 to 2500 pulses/second")
    print("  calibrate  -> assign current physical position as center / 0 degrees")
    print("  zero       -> alias for calibrate")
    print("  status     -> show configuration and estimated position")
    print("  help       -> show commands")
    print("  q          -> quit")
    print()


STEP.value(INACTIVE)

print()
print("========================================")
print(" R86mini + NEMA34 1:1 STEERING TEST")
print("========================================")
print("CENTER THE STEERING PHYSICALLY BEFORE USING LEFT/RIGHT.")
print("-90 deg =", degrees_to_pulses(-90), "pulses")
print("  0 deg = 0 pulses")
print("+90 deg =", degrees_to_pulses(90), "pulses")
help_text()

try:
    while True:
        parts = input("nema34-steering> ").strip().lower().split()

        if not parts:
            continue

        command = parts[0]

        if command == "left":
            try:
                degrees = (
                    abs(float(parts[1]))
                    if len(parts) > 1
                    else abs(MIN_ANGLE_DEG)
                )
                move_to_angle(-degrees)
            except ValueError:
                print("Example: left or left 30")

        elif command in ("center", "centre"):
            move_to_angle(0)

        elif command == "right":
            try:
                degrees = (
                    abs(float(parts[1]))
                    if len(parts) > 1
                    else MAX_ANGLE_DEG
                )
                move_to_angle(degrees)
            except ValueError:
                print("Example: right or right 30")

        elif command == "angle":
            try:
                move_to_angle(float(parts[1]))
            except (ValueError, IndexError):
                print("Example: angle 20 or angle -30")

        elif command == "sweep":
            continuous_steering_test()

        elif command == "speed":
            try:
                set_speed(parts[1])
            except (ValueError, IndexError):
                print("Example: speed 1200")

        elif command in ("calibrate", "zero"):
            calibrate_center_here()

        elif command == "status":
            status()

        elif command == "help":
            help_text()

        elif command == "q":
            break

        else:
            print(
                "Commands: left [deg] | center | right [deg] | angle deg | "
                "sweep | speed pps | calibrate | status | help | q"
            )

except KeyboardInterrupt:
    print("\nSTOPPED BY USER")
    print("WARNING: software position may now be inaccurate; re-center and reset.")

finally:
    STEP.value(INACTIVE)
    print("SAFE EXIT: STEP LOW")

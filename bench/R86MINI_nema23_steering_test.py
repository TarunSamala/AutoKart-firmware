"""Isolated AutoKart steering test: ESP32-S3 + R86mini + NEMA23.

Motor: JK57HS112-3004-03, 1.8 degree, four-wire bipolar
Control convention: 0 degrees is center, range is -45 to +45 degrees.

This adapts the steering logic from steering_brake_test.py. It deliberately
does not control the brake and does not use ENA. Leave ENA+ and ENA-
disconnected during this diagnostic.

R86mini settings expected by this file:
  PUL+DIR external pulse mode
  3200 pulses/revolution
  SW5=OFF, SW6=OFF, SW7=ON, SW8=ON

Initial current setting for the 3 A-peak motor:
  2.40 A peak: SW1=ON, SW2=ON, SW3=ON
  Half-current at standstill: SW4=OFF

Signal interface:
  GPIO4 and GPIO5 drive 5 V R86mini inputs through NPN transistor stages.
  GPIO HIGH turns its transistor/input ON.

Open-loop warning:
  Physically center the steering before startup. The software position is
  only an estimate and becomes wrong if the motor misses steps.
"""

from machine import Pin
import time


FIRMWARE_NAME = "r86mini-nema23-steering-test"
FIRMWARE_VERSION = "1.1.0"

# ESP32-S3 pins. ENA is intentionally not connected or controlled.
STEP_PIN_NUMBER = 4
DIR_PIN_NUMBER = 5

STEP = Pin(STEP_PIN_NUMBER, Pin.OUT, value=0)
DIR = Pin(DIR_PIN_NUMBER, Pin.OUT, value=0)

# NPN interface logic: GPIO HIGH activates the R86mini optocoupler.
SIGNAL_ACTIVE = 1
SIGNAL_INACTIVE = 0

# Change RIGHT_DIR to 0 only if the physical direction is reversed.
RIGHT_DIR = 1
LEFT_DIR = 1 - RIGHT_DIR

# The R86mini must be set to 3200 pulses per motor revolution.
PULSES_PER_MOTOR_REV = 3200

# Keep the current steering_brake_test.py calibration for this first test.
# Change this only when the physical steering reduction is confirmed.
STEERING_GEAR_RATIO = 1.0
PULSES_PER_STEERING_DEG = (
    PULSES_PER_MOTOR_REV * STEERING_GEAR_RATIO / 360.0
)

STEERING_MIN_DEG = -45.0
STEERING_MAX_DEG = 45.0

# Adjustable pulse rate. The old steering_brake_test.py used 200 pps, which
# is only 3.75 motor RPM at 3200 pulses/revolution. Start at 800 pps and allow
# conservative bench adjustment from 100 to 1500 pps.
MIN_PULSE_RATE_HZ = 100
MAX_PULSE_RATE_HZ = 1500
pulse_rate_hz = 800

# Keep the active pulse comfortably longer than the driver's minimum while
# changing speed through the inactive interval.
STEP_HIGH_US = 50
DIR_SETTLE_MS = 100

# Software position assumes the mechanism is physically centered at startup.
steering_position_pulses = 0


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def current_angle():
    return steering_position_pulses / PULSES_PER_STEERING_DEG


def pulse_steps(count):
    period_us = 1_000_000 // pulse_rate_hz
    step_low_us = max(1, period_us - STEP_HIGH_US)

    for _ in range(count):
        STEP.value(SIGNAL_ACTIVE)
        time.sleep_us(STEP_HIGH_US)
        STEP.value(SIGNAL_INACTIVE)
        time.sleep_us(step_low_us)


def set_speed(requested_rate):
    global pulse_rate_hz

    requested_rate = int(requested_rate)
    pulse_rate_hz = int(
        clamp(requested_rate, MIN_PULSE_RATE_HZ, MAX_PULSE_RATE_HZ)
    )

    if pulse_rate_hz != requested_rate:
        print("SPEED CLAMPED:", requested_rate, "->", pulse_rate_hz, "pps")

    motor_rpm = pulse_rate_hz * 60 / PULSES_PER_MOTOR_REV
    steering_rpm = motor_rpm / STEERING_GEAR_RATIO

    print(
        "SPEED:", pulse_rate_hz, "pps | motor:",
        round(motor_rpm, 2), "RPM | steering output:",
        round(steering_rpm, 2), "RPM"
    )


def steer(target_angle):
    global steering_position_pulses

    requested_angle = float(target_angle)
    target_angle = clamp(
        requested_angle,
        STEERING_MIN_DEG,
        STEERING_MAX_DEG,
    )

    if target_angle != requested_angle:
        print("CLAMPED:", requested_angle, "->", target_angle, "deg")

    target_pulses = round(target_angle * PULSES_PER_STEERING_DEG)
    movement = target_pulses - steering_position_pulses

    if movement == 0:
        print("Already at", round(current_angle(), 2), "deg")
        return

    if movement > 0:
        DIR.value(RIGHT_DIR)
        direction = "RIGHT / CW"
    else:
        DIR.value(LEFT_DIR)
        direction = "LEFT / CCW"

    time.sleep_ms(DIR_SETTLE_MS)

    print(
        "STEERING:",
        round(target_angle, 2),
        "deg |",
        direction,
        "| pulses:",
        abs(movement),
    )

    pulse_steps(abs(movement))
    steering_position_pulses = target_pulses

    print("POSITION:", round(current_angle(), 2), "deg")


def status():
    print()
    print("========================================")
    print("R86mini + NEMA23 STEERING STATUS")
    print("========================================")
    print("Firmware          :", FIRMWARE_NAME, FIRMWARE_VERSION)
    print("Motor             : JK57HS112-3004-03")
    print("STEP / DIR GPIO   :", STEP_PIN_NUMBER, "/", DIR_PIN_NUMBER)
    print("ENA               : disconnected")
    print("Driver pulses/rev :", PULSES_PER_MOTOR_REV)
    print("Gear ratio        :", STEERING_GEAR_RATIO)
    print("Pulses/degree     :", round(PULSES_PER_STEERING_DEG, 4))
    print("Pulse rate        :", pulse_rate_hz, "pps")
    print(
        "Motor speed       :",
        round(pulse_rate_hz * 60 / PULSES_PER_MOTOR_REV, 2),
        "RPM",
    )
    print("Software angle    :", round(current_angle(), 2), "deg")
    print("Software pulses   :", steering_position_pulses)
    print("Limits            :", STEERING_MIN_DEG, "to", STEERING_MAX_DEG)
    print("========================================")
    print()


def help_text():
    print()
    print("Commands:")
    print("  s 5     -> move to +5 degrees")
    print("  s -5    -> move to -5 degrees")
    print("  s 45    -> move to full right")
    print("  s -45   -> move to full left")
    print("  center  -> return to software center")
    print("  speed 800  -> set pulse rate (100 to 1500 pps)")
    print("  status  -> show configuration and position")
    print("  help    -> show commands")
    print("  q       -> quit and force STEP inactive")
    print()


STEP.value(SIGNAL_INACTIVE)

print()
print("================================================")
print(" AUTOKART R86mini + NEMA23 STEERING TEST")
print("================================================")
print("Physically center and unload/jack steering first.")
print("R86mini: PUL+DIR, 3200 pulses/rev, ENA disconnected.")
print("Initial current: 2.40 A peak; do not change DIP live.")
print("Initial speed:", pulse_rate_hz, "pps")
print("Range: -45 deg to +45 deg, 0 deg center.")
help_text()

try:
    while True:
        command = input("steer> ").strip()

        if not command:
            continue

        if command.lower() == "center":
            steer(0.0)

        elif command.lower() == "status":
            status()

        elif command.lower() == "help":
            help_text()

        elif command.lower().startswith("speed "):
            try:
                set_speed(command.split()[1])
            except (ValueError, IndexError):
                print("Example: speed 800")

        elif command.lower() == "q":
            break

        elif command.lower().startswith("s "):
            try:
                steer(float(command.split()[1]))
            except (ValueError, IndexError):
                print("Example: s 5 or s -5")

        else:
            print("Commands: s angle | speed pps | center | status | help | q")

except KeyboardInterrupt:
    print("\nSTOPPED BY USER")

finally:
    STEP.value(SIGNAL_INACTIVE)
    print("STEP forced inactive; R86mini ENA remains disconnected")

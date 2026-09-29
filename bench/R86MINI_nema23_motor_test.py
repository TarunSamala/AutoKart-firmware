"""Isolated ESP32-S3 + R86mini + JK57HS112-3004-03 motor test.

This test does not contain steering angles, gearing, brake logic, or vehicle
limits. It commands the motor shaft directly in complete revolutions.

Required R86mini configuration:
  - External PUL+DIR mode
  - 3200 pulses/revolution
  - SW5=OFF, SW6=OFF, SW7=ON, SW8=ON
  - Initial current: SW1=ON, SW2=ON, SW3=ON (2.40 A peak)
  - SW4=OFF (reduced standstill current)

Signal interface:
  GPIO4 -> NPN transistor stage -> PUL-
  GPIO5 -> NPN transistor stage -> DIR-
  5 V -> PUL+ and DIR+
  ENA+ and ENA- disconnected
"""

from machine import Pin
import time


FIRMWARE_NAME = "r86mini-nema23-motor-test"
FIRMWARE_VERSION = "1.0.0"

STEP_PIN_NUMBER = 4
DIR_PIN_NUMBER = 5

STEP = Pin(STEP_PIN_NUMBER, Pin.OUT, value=0)
DIR = Pin(DIR_PIN_NUMBER, Pin.OUT, value=0)

SIGNAL_ACTIVE = 1
SIGNAL_INACTIVE = 0

# Change these if the printed direction is opposite to physical rotation.
CW_DIRECTION = 1
CCW_DIRECTION = 1 - CW_DIRECTION

PULSES_PER_REVOLUTION = 3200

MIN_PULSE_RATE_HZ = 100
MAX_PULSE_RATE_HZ = 2500
START_PULSE_RATE_HZ = 300
MAX_RAMP_STEPS = 300
STEP_HIGH_US = 50
DIR_SETTLE_MS = 100

pulse_rate_hz = 1500
total_position_pulses = 0


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def pulse_steps(count):
    ramp_steps = min(MAX_RAMP_STEPS, count // 2)
    start_rate = min(START_PULSE_RATE_HZ, pulse_rate_hz)

    for step_index in range(count):
        if ramp_steps:
            steps_from_end = count - step_index - 1
            ramp_position = min(step_index, steps_from_end, ramp_steps)
            rate = start_rate + (
                (pulse_rate_hz - start_rate)
                * ramp_position
                // ramp_steps
            )
        else:
            rate = start_rate

        period_us = 1_000_000 // max(1, rate)
        step_low_us = max(1, period_us - STEP_HIGH_US)

        STEP.value(SIGNAL_ACTIVE)
        time.sleep_us(STEP_HIGH_US)
        STEP.value(SIGNAL_INACTIVE)
        time.sleep_us(step_low_us)


def rotate(direction_level, direction_name, revolutions=1.0):
    global total_position_pulses

    revolutions = float(revolutions)
    if revolutions <= 0:
        print("Revolutions must be greater than zero")
        return

    pulses = round(revolutions * PULSES_PER_REVOLUTION)

    DIR.value(direction_level)
    time.sleep_ms(DIR_SETTLE_MS)

    print(
        direction_name,
        "| revolutions:", round(revolutions, 3),
        "| pulses:", pulses,
        "| max rate:", pulse_rate_hz, "pps",
    )

    pulse_steps(pulses)

    if direction_level == CW_DIRECTION:
        total_position_pulses += pulses
    else:
        total_position_pulses -= pulses

    print(
        "DONE | net motor position:",
        round(total_position_pulses / PULSES_PER_REVOLUTION, 3),
        "revolutions",
    )


def set_speed(requested_rate):
    global pulse_rate_hz

    requested_rate = int(requested_rate)
    pulse_rate_hz = int(
        clamp(requested_rate, MIN_PULSE_RATE_HZ, MAX_PULSE_RATE_HZ)
    )

    if pulse_rate_hz != requested_rate:
        print("SPEED CLAMPED:", requested_rate, "->", pulse_rate_hz, "pps")

    print(
        "SPEED:", pulse_rate_hz, "pps |",
        round(pulse_rate_hz * 60 / PULSES_PER_REVOLUTION, 2),
        "motor RPM at maximum rate",
    )


def status():
    print()
    print("========================================")
    print("R86mini + NEMA23 MOTOR TEST")
    print("========================================")
    print("Firmware          :", FIRMWARE_NAME, FIRMWARE_VERSION)
    print("Motor             : JK57HS112-3004-03")
    print("STEP / DIR GPIO   :", STEP_PIN_NUMBER, "/", DIR_PIN_NUMBER)
    print("ENA               : disconnected")
    print("Pulses/revolution :", PULSES_PER_REVOLUTION)
    print("Maximum pulse rate:", pulse_rate_hz, "pps")
    print(
        "Maximum motor RPM :",
        round(pulse_rate_hz * 60 / PULSES_PER_REVOLUTION, 2),
    )
    print(
        "Net motor position:",
        round(total_position_pulses / PULSES_PER_REVOLUTION, 3),
        "revolutions",
    )
    print("========================================")
    print()


def help_text():
    print()
    print("Commands:")
    print("  cw       -> one clockwise revolution (360 degrees)")
    print("  ccw      -> one counter-clockwise revolution (360 degrees)")
    print("  cw 2     -> two clockwise revolutions")
    print("  ccw 0.5  -> half a counter-clockwise revolution")
    print("  speed 1500 -> set maximum pulse rate (100 to 2500 pps)")
    print("  status   -> show configuration")
    print("  help     -> show commands")
    print("  q        -> quit and force STEP inactive")
    print()


STEP.value(SIGNAL_INACTIVE)

print()
print("========================================")
print(" R86mini + NEMA23 360-DEGREE MOTOR TEST")
print("========================================")
print("No steering ratio is used.")
print("One revolution =", PULSES_PER_REVOLUTION, "pulses")
print("Initial speed =", pulse_rate_hz, "pps")
print("Keep the motor mechanically unloaded for the first test.")
help_text()

try:
    while True:
        command = input("motor> ").strip().lower()

        if not command:
            continue

        parts = command.split()
        action = parts[0]

        if action == "cw":
            try:
                revolutions = float(parts[1]) if len(parts) > 1 else 1.0
                rotate(CW_DIRECTION, "CW", revolutions)
            except ValueError:
                print("Example: cw or cw 2")

        elif action == "ccw":
            try:
                revolutions = float(parts[1]) if len(parts) > 1 else 1.0
                rotate(CCW_DIRECTION, "CCW", revolutions)
            except ValueError:
                print("Example: ccw or ccw 0.5")

        elif action == "speed":
            try:
                set_speed(parts[1])
            except (ValueError, IndexError):
                print("Example: speed 1500")

        elif action == "status":
            status()

        elif action == "help":
            help_text()

        elif action == "q":
            break

        else:
            print("Commands: cw [turns] | ccw [turns] | speed pps | status | help | q")

except KeyboardInterrupt:
    print("\nSTOPPED BY USER")

finally:
    STEP.value(SIGNAL_INACTIVE)
    print("STEP forced inactive; R86mini ENA remains disconnected")

"""Isolated ESP32-S3 + R86mini + JK86HS155-4208 NEMA34 test.

Motor wiring: parallel
  A+ = RED + BLACK
  A- = YELLOW + BLUE
  B+ = WHITE + GREEN
  B- = ORANGE + BROWN

R86mini:
  External PUL+DIR mode
  3200 pulses/revolution: SW5 OFF, SW6 OFF, SW7 ON, SW8 ON
  ENA+ and ENA- disconnected

Control:
  GPIO4 -> tested 5 V NPN interface -> PUL-
  GPIO5 -> tested 5 V NPN interface -> DIR-
"""

from machine import Pin
import time


STEP_GPIO = 4
DIR_GPIO = 5

STEP = Pin(STEP_GPIO, Pin.OUT, value=0)
DIR = Pin(DIR_GPIO, Pin.OUT, value=0)

ACTIVE = 1
INACTIVE = 0
CW_DIRECTION = 1
CCW_DIRECTION = 0

PULSES_PER_REVOLUTION = 3200
MIN_PPS = 100
MAX_PPS = 2500
START_PPS = 200
MAX_RAMP_STEPS = 400
STEP_HIGH_US = 50
DIR_SETTLE_MS = 100

pulse_rate_pps = 500
net_position_pulses = 0


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def pulse_steps(count):
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


def rotate(direction_level, direction_name, revolutions=1.0):
    global net_position_pulses

    revolutions = float(revolutions)
    if revolutions <= 0:
        print("Revolutions must be greater than zero")
        return

    pulses = round(revolutions * PULSES_PER_REVOLUTION)
    DIR.value(direction_level)
    time.sleep_ms(DIR_SETTLE_MS)

    print(
        direction_name,
        "| turns:", round(revolutions, 3),
        "| pulses:", pulses,
        "| max speed:", pulse_rate_pps, "pps",
    )

    pulse_steps(pulses)

    if direction_level == CW_DIRECTION:
        net_position_pulses += pulses
    else:
        net_position_pulses -= pulses

    print(
        "DONE | net position:",
        round(net_position_pulses / PULSES_PER_REVOLUTION, 3),
        "turns",
    )


def set_speed(requested_pps):
    global pulse_rate_pps

    requested_pps = int(requested_pps)
    pulse_rate_pps = int(clamp(requested_pps, MIN_PPS, MAX_PPS))

    if pulse_rate_pps != requested_pps:
        print("SPEED CLAMPED:", requested_pps, "->", pulse_rate_pps)

    print(
        "SPEED:", pulse_rate_pps, "pps =",
        round(pulse_rate_pps * 60 / PULSES_PER_REVOLUTION, 2),
        "motor RPM",
    )


def status():
    print()
    print("R86mini + JK86HS155-4208 NEMA34")
    print("Wiring          : parallel")
    print("Pulses/rev      :", PULSES_PER_REVOLUTION)
    print("Pulse rate      :", pulse_rate_pps, "pps")
    print(
        "Motor RPM       :",
        round(pulse_rate_pps * 60 / PULSES_PER_REVOLUTION, 2),
    )
    print(
        "Net position    :",
        round(net_position_pulses / PULSES_PER_REVOLUTION, 3),
        "turns",
    )
    print("ENA             : disconnected")
    print()


def help_text():
    print()
    print("Commands:")
    print("  cw       -> one clockwise revolution")
    print("  ccw      -> one counter-clockwise revolution")
    print("  cw 2     -> two clockwise revolutions")
    print("  ccw 0.5  -> half counter-clockwise revolution")
    print("  speed 500  -> set 100 to 2500 pulses/second")
    print("  status   -> show settings")
    print("  help     -> show commands")
    print("  q        -> quit")
    print()


STEP.value(INACTIVE)

print()
print("========================================")
print(" R86mini + NEMA34 PARALLEL MOTOR TEST")
print("========================================")
print("One motor revolution =", PULSES_PER_REVOLUTION, "pulses")
print("Initial speed =", pulse_rate_pps, "pps")
print("Motor must be unloaded and securely clamped.")
help_text()

try:
    while True:
        parts = input("nema34> ").strip().lower().split()

        if not parts:
            continue

        command = parts[0]

        if command in ("cw", "ccw"):
            try:
                turns = float(parts[1]) if len(parts) > 1 else 1.0
                rotate(
                    CW_DIRECTION if command == "cw" else CCW_DIRECTION,
                    command.upper(),
                    turns,
                )
            except ValueError:
                print("Example: cw or ccw 0.5")

        elif command == "speed":
            try:
                set_speed(parts[1])
            except (ValueError, IndexError):
                print("Example: speed 500")

        elif command == "status":
            status()

        elif command == "help":
            help_text()

        elif command == "q":
            break

        else:
            print("Commands: cw [turns] | ccw [turns] | speed pps | status | help | q")

except KeyboardInterrupt:
    print("\nSTOPPED BY USER")

finally:
    STEP.value(INACTIVE)
    print("SAFE EXIT: STEP LOW")

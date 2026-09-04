import time
from machine import Pin

# AUTOKART STEERING-ONLY TEST
# ESP32-S3 + TB6600 + NEMA23
# 17T motor gear -> 51T steering gear = 3:1
# OPEN LOOP: physically center steering before running.

STEP_PIN = Pin(4, Pin.OUT, value=0)
DIR_PIN  = Pin(5, Pin.OUT, value=0)
EN_PIN   = Pin(6, Pin.OUT, value=1)

TB_ENABLE = 0
TB_DISABLE = 1

# RIGHT = clockwise. If physical direction is reversed, change 1 -> 0.
RIGHT_DIR = 1
LEFT_DIR = 0

PULSES_PER_MOTOR_REV = 3200
GEAR_RATIO = 51 / 17
PULSES_PER_DEG = (PULSES_PER_MOTOR_REV * GEAR_RATIO) / 360.0

LEFT_LIMIT_DEG = -90.0
RIGHT_LIMIT_DEG = 90.0

# Conservative test motion
START_PERIOD_US = 1800
FAST_PERIOD_US = 800
ACCEL_US_PER_STEP = 5
STEP_HIGH_US = 10

current_steps = 0

def current_angle():
    return current_steps / PULSES_PER_DEG

def clamp_angle(angle):
    return max(LEFT_LIMIT_DEG, min(RIGHT_LIMIT_DEG, angle))

def enable_driver():
    EN_PIN.value(TB_ENABLE)

def disable_driver():
    EN_PIN.value(TB_DISABLE)

def pulse_step():
    STEP_PIN.value(1)
    time.sleep_us(STEP_HIGH_US)
    STEP_PIN.value(0)

def move_to_angle(target_angle):
    global current_steps

    target_angle = clamp_angle(float(target_angle))
    target_steps = round(target_angle * PULSES_PER_DEG)
    delta_steps = target_steps - current_steps

    if delta_steps == 0:
        print("Already at target")
        return

    if delta_steps > 0:
        DIR_PIN.value(RIGHT_DIR)
        direction = 1
        direction_name = "RIGHT"
    else:
        DIR_PIN.value(LEFT_DIR)
        direction = -1
        direction_name = "LEFT"

    total_steps = abs(delta_steps)
    period_us = START_PERIOD_US

    print()
    print("Moving", direction_name, "to", target_angle, "deg")
    print("Steps:", total_steps)

    for _ in range(total_steps):
        pulse_step()
        time.sleep_us(period_us)
        current_steps += direction

        if period_us > FAST_PERIOD_US:
            period_us = max(
                FAST_PERIOD_US,
                period_us - ACCEL_US_PER_STEP
            )

    print("Software position:", round(current_angle(), 2), "deg")

def move_relative(delta_angle):
    move_to_angle(current_angle() + float(delta_angle))

def print_status():
    print()
    print("==============================")
    print("STEERING TEST STATUS")
    print("==============================")
    print("Gear ratio      :", GEAR_RATIO)
    print("Pulses / degree :", round(PULSES_PER_DEG, 4))
    print("Software angle  :", round(current_angle(), 2))
    print("Software steps  :", current_steps)
    print("==============================")

enable_driver()

print()
print("========================================")
print("     AUTOKART STEERING TEST")
print("========================================")
print("17T -> 51T")
print("Gear ratio = 3:1")
print("Pulses/degree =", round(PULSES_PER_DEG, 4))
print()
print("PHYSICALLY CENTER STEERING FIRST")
print()
print("Commands:")
print("  r 5   -> RIGHT 5 deg")
print("  l 5   -> LEFT 5 deg")
print("  r 15  -> RIGHT 15 deg")
print("  l 15  -> LEFT 15 deg")
print("  R     -> FULL RIGHT")
print("  L     -> FULL LEFT")
print("  c     -> CENTER")
print("  s     -> STATUS")
print("  q     -> QUIT")
print("========================================")

try:
    while True:
        command = input("steer> ").strip()

        if not command:
            continue

        if command == "R":
            move_to_angle(RIGHT_LIMIT_DEG)

        elif command == "L":
            move_to_angle(LEFT_LIMIT_DEG)

        elif command.lower() == "c":
            move_to_angle(0)

        elif command.lower() == "s":
            print_status()

        elif command.lower() == "q":
            disable_driver()
            print("TB6600 disabled")
            break

        elif command.lower().startswith("r "):
            try:
                deg = float(command.split()[1])
                move_relative(abs(deg))
            except:
                print("Example: r 5")

        elif command.lower().startswith("l "):
            try:
                deg = float(command.split()[1])
                move_relative(-abs(deg))
            except:
                print("Example: l 5")

        else:
            print("Unknown command")

except KeyboardInterrupt:
    print()
    print("STOPPED")

finally:
    STEP_PIN.value(0)
    disable_driver()
    print("TB6600 disabled")

import time
from machine import Pin

# ============================================================
# AUTOKART STEERING BENCH TEST
# ESP32-S3 + Rtelligent R86mini + JK86HS155-4208 NEMA34
#
# IMPORTANT:
# - MicroPython
# - R86mini set to PUL+DIR mode
# - Driver microstep setting = 3200 pulses/rev:
#       SW5=OFF, SW6=OFF, SW7=ON, SW8=ON
# - Steering reduction = 17T motor -> 34T steering = 2:1
# - Software assumes steering is PHYSICALLY CENTERED at startup.
# - Open-loop position only. Do NOT use this as final vehicle steering
#   without encoder feedback and independent E-stop.
#
# SIGNAL INTERFACE:
# R86mini manual specifies 5-24 V control inputs.
# Do NOT connect the R86mini control input directly assuming 3.3 V is valid.
# Use a proper 3.3 V -> 5 V interface / transistor stage.
#
# Logic below assumes:
#   GPIO HIGH = R86mini optocoupler input ACTIVE
#   GPIO LOW  = R86mini optocoupler input INACTIVE
#
# For ENA, the R86mini default behavior is:
#   ENA optocoupler OFF -> driver enabled
#   ENA optocoupler ON  -> driver disabled
# ============================================================

FIRMWARE_NAME = "autokart-r86mini-nema34-steering-test"
FIRMWARE_VERSION = "2.0.0"

# ------------------------------------------------------------
# ESP32-S3 GPIO
# ------------------------------------------------------------
STEP_PIN = Pin(4, Pin.OUT, value=0)
DIR_PIN  = Pin(5, Pin.OUT, value=0)
ENA_PIN  = Pin(6, Pin.OUT, value=1)   # ACTIVE = driver disabled

# ------------------------------------------------------------
# DRIVER LOGIC
# ------------------------------------------------------------
SIGNAL_ACTIVE = 1
SIGNAL_INACTIVE = 0

# R86mini default ENA logic:
# optocoupler ON = motor current cut / driver offline
R86_DISABLE = SIGNAL_ACTIVE
R86_ENABLE  = SIGNAL_INACTIVE

# Change these two ONLY if physical steering direction is reversed.
RIGHT_DIR = SIGNAL_ACTIVE
LEFT_DIR  = SIGNAL_INACTIVE

# ------------------------------------------------------------
# MOTOR / DRIVER / MECHANISM
# ------------------------------------------------------------
# R86mini DIP setting:
# SW5 OFF, SW6 OFF, SW7 ON, SW8 ON = 3200 pulses / motor revolution
PULSES_PER_MOTOR_REV = 3200

MOTOR_GEAR_TEETH = 17
STEERING_GEAR_TEETH = 34
GEAR_RATIO = STEERING_GEAR_TEETH / MOTOR_GEAR_TEETH  # 2.0

PULSES_PER_STEERING_DEG = (
    PULSES_PER_MOTOR_REV * GEAR_RATIO / 360.0
)

# ------------------------------------------------------------
# VERIFIED MECHANICAL SOFTWARE LIMITS
# ------------------------------------------------------------
LEFT_LIMIT_DEG = -20.0
RIGHT_LIMIT_DEG = 20.0

# ------------------------------------------------------------
# MOTION PROFILE
# Slower + smooth acceleration/deceleration gives much better
# resistance to missed steps than instantly commanding high speed.
# ------------------------------------------------------------
START_PERIOD_US = 3000      # start slow
FAST_PERIOD_US = 1200       # minimum pulse period
RAMP_CHANGE_US = 10         # period change per step
STEP_HIGH_US = 10           # pulse HIGH width
DIR_SETUP_US = 30           # settle after direction change
ENABLE_SETTLE_MS = 100      # driver settle after enabling

# Software-only position estimate.
# It is valid ONLY if no steps are missed.
current_steps = 0
driver_enabled = False


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def current_angle():
    return current_steps / PULSES_PER_STEERING_DEG


def enable_driver():
    global driver_enabled
    STEP_PIN.value(SIGNAL_INACTIVE)
    ENA_PIN.value(R86_ENABLE)
    time.sleep_ms(ENABLE_SETTLE_MS)
    driver_enabled = True
    print("R86mini ENABLED")


def disable_driver():
    global driver_enabled
    STEP_PIN.value(SIGNAL_INACTIVE)
    ENA_PIN.value(R86_DISABLE)
    driver_enabled = False
    print("R86mini DISABLED")


def set_direction(direction_level):
    DIR_PIN.value(direction_level)
    time.sleep_us(DIR_SETUP_US)


def step_once(period_us):
    """
    One complete step pulse.
    period_us is the approximate total interval between rising edges.
    """
    period_us = max(int(period_us), STEP_HIGH_US + 5)

    STEP_PIN.value(SIGNAL_ACTIVE)
    time.sleep_us(STEP_HIGH_US)
    STEP_PIN.value(SIGNAL_INACTIVE)
    time.sleep_us(period_us - STEP_HIGH_US)


def motion_period(step_index, total_steps):
    """
    Symmetric trapezoidal/triangular speed profile.
    Slow at the start and end, faster in the middle.
    """
    if total_steps <= 1:
        return START_PERIOD_US

    max_ramp_steps = max(
        1,
        (START_PERIOD_US - FAST_PERIOD_US) // RAMP_CHANGE_US
    )

    ramp_steps = min(max_ramp_steps, total_steps // 2)

    if ramp_steps <= 0:
        return START_PERIOD_US

    # Acceleration region
    if step_index < ramp_steps:
        period = START_PERIOD_US - (step_index * RAMP_CHANGE_US)
        return max(FAST_PERIOD_US, period)

    # Deceleration region
    steps_remaining = total_steps - step_index - 1
    if steps_remaining < ramp_steps:
        period = START_PERIOD_US - (steps_remaining * RAMP_CHANGE_US)
        return max(FAST_PERIOD_US, period)

    # Cruise region
    return FAST_PERIOD_US


def move_to_angle(target_angle):
    global current_steps

    if not driver_enabled:
        print("REFUSED: driver is disabled. Use 'e' first.")
        return

    target_angle = clamp(
        float(target_angle),
        LEFT_LIMIT_DEG,
        RIGHT_LIMIT_DEG
    )

    target_steps = round(
        target_angle * PULSES_PER_STEERING_DEG
    )

    delta_steps = target_steps - current_steps

    if delta_steps == 0:
        print("Already at target:", round(current_angle(), 2), "deg")
        return

    if delta_steps > 0:
        set_direction(RIGHT_DIR)
        direction_sign = 1
        direction_name = "RIGHT"
    else:
        set_direction(LEFT_DIR)
        direction_sign = -1
        direction_name = "LEFT"

    total_steps = abs(delta_steps)

    print()
    print(
        "MOVE:",
        direction_name,
        "target =", round(target_angle, 2),
        "deg | steps =", total_steps
    )

    try:
        for i in range(total_steps):
            period_us = motion_period(i, total_steps)
            step_once(period_us)
            current_steps += direction_sign

    except KeyboardInterrupt:
        STEP_PIN.value(SIGNAL_INACTIVE)
        print()
        print("MOTION INTERRUPTED")
        print("Estimated position:", round(current_angle(), 2), "deg")
        raise

    print(
        "DONE | estimated steering angle:",
        round(current_angle(), 2),
        "deg"
    )


def move_relative(delta_angle):
    move_to_angle(current_angle() + float(delta_angle))


def print_status():
    print()
    print("========================================")
    print("AUTOKART R86mini / NEMA34 STATUS")
    print("========================================")
    print("Firmware            :", FIRMWARE_NAME, FIRMWARE_VERSION)
    print("Motor               : JK86HS155-4208")
    print("Driver              : Rtelligent R86mini")
    print("Driver enabled      :", driver_enabled)
    print("Pulse setting       :", PULSES_PER_MOTOR_REV, "pulses/motor rev")
    print(
        "Gear reduction      :",
        MOTOR_GEAR_TEETH,
        "T ->",
        STEERING_GEAR_TEETH,
        "T =",
        round(GEAR_RATIO, 3),
        ":1"
    )
    print(
        "Pulses/steering deg :",
        round(PULSES_PER_STEERING_DEG, 4)
    )
    print(
        "Software position   :",
        round(current_angle(), 2),
        "deg"
    )
    print("Software steps      :", current_steps)
    print(
        "Limits              :",
        LEFT_LIMIT_DEG,
        "to",
        RIGHT_LIMIT_DEG,
        "deg"
    )
    print(
        "Motion period       :",
        START_PERIOD_US,
        "->",
        FAST_PERIOD_US,
        "us"
    )
    print("========================================")


def print_help():
    print()
    print("Commands:")
    print("  e       -> enable R86mini")
    print("  d       -> disable R86mini")
    print("  r 5     -> move RIGHT by 5 deg")
    print("  l 5     -> move LEFT by 5 deg")
    print("  R       -> move to +20 deg")
    print("  L       -> move to -20 deg")
    print("  c       -> return to software center")
    print("  s       -> status")
    print("  h       -> help")
    print("  q       -> disable driver and quit")


# ============================================================
# STARTUP
# ============================================================

# Start disabled.
disable_driver()

print()
print("================================================")
print(" AUTOKART NEMA34 + R86mini STEERING BENCH TEST")
print("================================================")
print("Motor  : JK86HS155-4208")
print("Driver : Rtelligent R86mini")
print("Ratio  : 17T -> 34T = 2:1")
print("Driver : 3200 pulses/rev")
print(
    "Scale  :",
    round(PULSES_PER_STEERING_DEG, 4),
    "pulses/steering-degree"
)
print()
print("WARNING:")
print("1. JACK UP / DISCONNECT STEERING LOAD FOR FIRST TEST.")
print("2. PHYSICALLY CENTER STEERING BEFORE STARTING.")
print("3. THIS POSITION IS OPEN-LOOP; MISSED STEPS ARE NOT DETECTED.")
print("4. KEEP A HARDWARE POWER/E-STOP AVAILABLE.")
print()
print_help()

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
            move_to_angle(0.0)

        elif command.lower() == "e":
            enable_driver()

        elif command.lower() == "d":
            disable_driver()

        elif command.lower() == "s":
            print_status()

        elif command.lower() == "h":
            print_help()

        elif command.lower() == "q":
            break

        elif command.lower().startswith("r "):
            try:
                deg = float(command.split()[1])
                move_relative(abs(deg))
            except (ValueError, IndexError):
                print("Example: r 5")

        elif command.lower().startswith("l "):
            try:
                deg = float(command.split()[1])
                move_relative(-abs(deg))
            except (ValueError, IndexError):
                print("Example: l 5")

        else:
            print("Unknown command. Use 'h'.")

except KeyboardInterrupt:
    print()
    print("STOPPED BY USER")

finally:
    STEP_PIN.value(SIGNAL_INACTIVE)
    disable_driver()
    print("SAFE EXIT")

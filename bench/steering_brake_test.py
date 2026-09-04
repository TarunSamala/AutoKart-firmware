from machine import Pin
import time

# ============================================================
# GPIO
# ============================================================

# Steering TB6600
STEER_STEP = Pin(4, Pin.OUT, value=0)
STEER_DIR  = Pin(5, Pin.OUT, value=0)
STEER_EN   = Pin(6, Pin.OUT, value=0)

# Brake TB6600
BRAKE_STEP = Pin(10, Pin.OUT, value=0)
BRAKE_DIR  = Pin(11, Pin.OUT, value=0)
BRAKE_EN   = Pin(12, Pin.OUT, value=0)


# ============================================================
# MOTOR / DRIVER SETTINGS
# ============================================================

# Assuming:
# 1.8 degree stepper
# TB6600 = 16x microstep = 3200 pulses/revolution

PULSES_PER_REV = 3200

MOTOR_PULSES_PER_DEG = PULSES_PER_REV / 360.0
# = 8.8889 pulses per MOTOR degree


# ============================================================
# IMPORTANT - GEAR RATIOS
# ============================================================

# steering output angle:
#
# motor gear -> steering shaft gear
#
# Example:
# motor pinion = 15 teeth
# steering gear = 45 teeth
#
# ratio = 45 / 15 = 3.0

STEER_GEAR_RATIO = 1.0       # <-- CHANGE THIS

# If brake shaft is direct:
BRAKE_GEAR_RATIO = 1.0       # <-- CHANGE if geared


STEER_PULSES_PER_DEG = (
    MOTOR_PULSES_PER_DEG * STEER_GEAR_RATIO
)

BRAKE_PULSES_PER_DEG = (
    MOTOR_PULSES_PER_DEG * BRAKE_GEAR_RATIO
)


# ============================================================
# LIMITS
# ============================================================

STEER_MIN = -45.0
STEER_MAX = 45.0

BRAKE_MIN = 0.0
BRAKE_MAX = 30.0


# ============================================================
# DIRECTION
#
# You said:
#
# Steering RIGHT = CLOCKWISE
# Steering LEFT  = ANTICLOCKWISE
#
# Brake APPLY = CLOCKWISE
#
# Test this at very low movement first.
# If backwards, swap 1 <-> 0.
# ============================================================

STEER_CW = 1
BRAKE_CW = 1


# ============================================================
# SPEED
#
# SLOW intentionally because mechanism is mounted/sensitive.
#
# 2500us HIGH + 2500us LOW
# ~200 pulses/sec
# ============================================================

STEP_HALF_PERIOD_US = 2500


# ============================================================
# SOFTWARE POSITION
#
# CRITICAL:
# At power-up physically put:
#
# Steering = CENTER
# Brake    = RELEASED
#
# Otherwise these positions are WRONG.
# ============================================================

steer_position_pulses = 0
brake_position_pulses = 0


# ============================================================
# ENABLE
#
# TB6600 clones differ in EN polarity.
#
# If motors do not energize, change:
# ENABLE_LEVEL = 1
# ============================================================

ENABLE_LEVEL = 0
DISABLE_LEVEL = 1

STEER_EN.value(ENABLE_LEVEL)
BRAKE_EN.value(ENABLE_LEVEL)


# ============================================================
# STEP FUNCTION
# ============================================================

def pulse_motor(step_pin, pulses):

    for _ in range(abs(pulses)):

        step_pin.value(1)
        time.sleep_us(STEP_HALF_PERIOD_US)

        step_pin.value(0)
        time.sleep_us(STEP_HALF_PERIOD_US)


# ============================================================
# STEERING
# ============================================================

def steer(angle):

    global steer_position_pulses

    angle = float(angle)

    # Safety clamp
    angle = max(
        STEER_MIN,
        min(STEER_MAX, angle)
    )

    target = round(
        angle * STEER_PULSES_PER_DEG
    )

    movement = target - steer_position_pulses

    if movement == 0:
        print("Steering already at", angle, "deg")
        return


    if movement > 0:

        # RIGHT = CLOCKWISE
        STEER_DIR.value(STEER_CW)

        direction = "RIGHT / CW"

    else:

        # LEFT = ANTICLOCKWISE
        STEER_DIR.value(1 - STEER_CW)

        direction = "LEFT / CCW"


    print(
        "STEERING:",
        round(angle, 1),
        "deg",
        direction,
        "| pulses:",
        abs(movement)
    )

    pulse_motor(
        STEER_STEP,
        movement
    )

    steer_position_pulses = target

    print(
        "STEERING POSITION =",
        round(
            steer_position_pulses /
            STEER_PULSES_PER_DEG,
            1
        ),
        "deg"
    )


# ============================================================
# BRAKE
# ============================================================

def brake(angle):

    global brake_position_pulses

    angle = float(angle)

    angle = max(
        BRAKE_MIN,
        min(BRAKE_MAX, angle)
    )

    target = round(
        angle * BRAKE_PULSES_PER_DEG
    )

    movement = target - brake_position_pulses

    if movement == 0:
        print("Brake already at", angle, "deg")
        return


    if movement > 0:

        # APPLY = CLOCKWISE
        BRAKE_DIR.value(BRAKE_CW)

        direction = "APPLY / CW"

    else:

        # RELEASE = CCW
        BRAKE_DIR.value(1 - BRAKE_CW)

        direction = "RELEASE / CCW"


    print(
        "BRAKE:",
        round(angle, 1),
        "deg",
        direction,
        "| pulses:",
        abs(movement)
    )

    pulse_motor(
        BRAKE_STEP,
        movement
    )

    brake_position_pulses = target

    print(
        "BRAKE POSITION =",
        round(
            brake_position_pulses /
            BRAKE_PULSES_PER_DEG,
            1
        ),
        "deg"
    )


# ============================================================
# STATUS
# ============================================================

def status():

    steer_deg = (
        steer_position_pulses /
        STEER_PULSES_PER_DEG
    )

    brake_deg = (
        brake_position_pulses /
        BRAKE_PULSES_PER_DEG
    )

    print()
    print("============================")
    print("STEERING:", round(steer_deg, 1), "deg")
    print("BRAKE   :", round(brake_deg, 1), "deg")
    print("============================")
    print()


# ============================================================
# TERMINAL
# ============================================================

print()
print("================================")
print(" AUTOKART STEERING + BRAKE TEST")
print("================================")
print()
print("STEERING:")
print(" s -45 ... s 45")
print(" center")
print()
print("BRAKE:")
print(" b 0 ... b 30")
print(" release")
print()
print("status")
print()
print("IMPORTANT:")
print("Start physically with steering CENTER")
print("and brake RELEASED.")
print("================================")


while True:

    try:

        cmd = input("\nKART > ").strip().lower()

        # Steering
        if cmd.startswith("s "):

            value = float(
                cmd.split()[1]
            )

            steer(value)


        elif cmd == "center":

            steer(0)


        # Brake
        elif cmd.startswith("b "):

            value = float(
                cmd.split()[1]
            )

            brake(value)


        elif cmd == "release":

            brake(0)


        elif cmd == "status":

            status()


        else:

            print(
                "Commands: s angle | center | "
                "b angle | release | status"
            )


    except KeyboardInterrupt:

        print("\nSTOPPED")
        break


    except Exception as e:

        print("ERROR:", e)    
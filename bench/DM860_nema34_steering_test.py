"""ESP32-S3 + DM860 + JK86HS155-4208 NEMA34 steering bench test.

Frozen AutoKart control pins (through the tested 5 V NPN sink interface):
  GPIO4 -> NPN interface -> DM860 PUL-
  GPIO5 -> NPN interface -> DM860 DIR-
  5 V                         -> DM860 PUL+ and DIR+
  DM860 ENA+ / ENA-           -> leave disconnected for initial testing
  ESP32 GND                   -> NPN interface ground

Verified NEMA34 parallel winding groups:
  DM860 A+ = RED + BLUE
  DM860 A- = YELLOW + BLACK
  DM860 B+ = ORANGE + GREEN
  DM860 B- = BROWN + WHITE

DM860 switch baseline for this motor:
  SW1 OFF, SW2 ON,  SW3 OFF = 5.83 A peak (verify against driver label)
  SW4 OFF                    = half current while idle
  SW5 OFF, SW6 OFF, SW7 ON, SW8 ON = 3200 pulses/revolution
  If the unit has SW9/SW10: SW9 OFF = PUL/DIR, SW10 OFF = self-test disabled

This is an OPEN-LOOP driver test. Position is calculated from pulses sent; it
cannot detect a stall, skipped step, slipping linkage, wrong center or a hard
stop. Lift/unload the steering and keep an independent power disconnect ready.
"""

from machine import Pin
import math
import time


FIRMWARE_NAME = "dm860-nema34-steering-test"
FIRMWARE_VERSION = "1.0.0"

STEP_GPIO = 4
DIR_GPIO = 5

PULSES_PER_REVOLUTION = 3200
STEERING_RATIO = 1.0  # motor revolutions / steering-shaft revolution
MIN_ANGLE_DEG = -45.0
MAX_ANGLE_DEG = 45.0

START_PPS = 150
MAX_PPS = 1200
ACCEL_PPS2 = 900
STEP_ACTIVE_US = 20
DIR_SETTLE_US = 100

# This is the GPIO level presented to the NPN interface, not the voltage at
# the DM860 DIR- terminal. Use `direction invert` if left/right is reversed.
RIGHT_GPIO_LEVEL = 1

MIN_ALLOWED_PPS = 50
MAX_ALLOWED_PPS = 20_000
MIN_PULSE_US = 3
MAX_PULSE_US = 200


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


step_pin = Pin(STEP_GPIO, Pin.OUT, value=0)
dir_pin = Pin(DIR_GPIO, Pin.OUT, value=0)

armed = False
position_steps = 0
total_pulses = 0

settings = {
    "ppr": int(PULSES_PER_REVOLUTION),
    "ratio": float(STEERING_RATIO),
    "min_angle": float(MIN_ANGLE_DEG),
    "max_angle": float(MAX_ANGLE_DEG),
    "start_pps": int(START_PPS),
    "max_pps": int(MAX_PPS),
    "accel_pps2": int(ACCEL_PPS2),
    "pulse_us": int(STEP_ACTIVE_US),
    "right_level": int(RIGHT_GPIO_LEVEL),
}


def pulses_per_degree():
    return settings["ppr"] * settings["ratio"] / 360.0


def steps_to_degrees(steps):
    return float(steps) / pulses_per_degree()


def degrees_to_steps(degrees):
    return int(round(float(degrees) * pulses_per_degree()))


def current_degrees():
    return steps_to_degrees(position_steps)


def arm():
    global armed
    step_pin.value(0)
    armed = True
    print("ARMED: motion commands accepted; DM860 ENA remains disconnected")


def disarm():
    global armed
    step_pin.value(0)
    armed = False
    print("DISARMED: STEP output blocked")


def motion_speed_pps(completed, remaining):
    """Symmetric acceleration/deceleration profile."""
    start = settings["start_pps"]
    accel = settings["accel_pps2"]
    accelerating = math.sqrt(start * start + 2.0 * accel * completed)
    decelerating = math.sqrt(start * start + 2.0 * accel * remaining)
    return int(min(settings["max_pps"], accelerating, decelerating))


def move_steps(delta_steps, label="MOVE"):
    global armed, position_steps, total_pulses

    delta_steps = int(delta_steps)
    count = abs(delta_steps)
    if count == 0:
        print(label + ": already at target")
        return True
    if not armed:
        print("REJECTED: type arm first")
        return False

    sign = 1 if delta_steps > 0 else -1
    direction_level = (
        settings["right_level"] if sign > 0 else 1 - settings["right_level"]
    )
    dir_pin.value(direction_level)
    time.sleep_us(DIR_SETTLE_US)

    target_angle = steps_to_degrees(position_steps + delta_steps)
    print(
        "{}: {:.2f} -> {:.2f} deg | {} pulses".format(
            label, current_degrees(), target_angle, count
        )
    )

    moved = 0
    try:
        while moved < count:
            remaining = count - moved
            pps = motion_speed_pps(moved, remaining)
            period_us = max(
                settings["pulse_us"] + MIN_PULSE_US,
                int(1_000_000 / max(1, pps)),
            )

            # GPIO HIGH turns on the tested NPN sink interface. The complete
            # active/inactive cycle works with either DM860 pulse-edge setting.
            step_pin.value(1)
            time.sleep_us(settings["pulse_us"])
            step_pin.value(0)
            time.sleep_us(period_us - settings["pulse_us"])

            position_steps += sign
            total_pulses += 1
            moved += 1

    except KeyboardInterrupt:
        step_pin.value(0)
        armed = False
        print("ABORTED after", moved, "pulses")
        print("Position estimate is not trusted; physically center, then zero.")
        return False

    step_pin.value(0)
    print("ARRIVED (open-loop estimate):", round(current_degrees(), 2), "deg")
    return True


def goto_angle(target_degrees, label="GOTO"):
    requested = float(target_degrees)
    target = clamp(requested, settings["min_angle"], settings["max_angle"])
    if target != requested:
        print("CLAMPED:", requested, "->", target, "deg")
    return move_steps(degrees_to_steps(target) - position_steps, label)


def calibrate_center():
    global position_steps
    if armed:
        print("REJECTED: disarm, physically center steering, then run zero")
        return
    position_steps = 0
    print("CENTER CALIBRATED: current physical position = 0.00 deg")
    print("No movement occurred.")


def run_sequence(angle, cycles):
    angle = abs(float(angle))
    angle = min(angle, abs(settings["min_angle"]), settings["max_angle"])
    cycles = max(1, min(20, int(cycles)))

    for cycle in range(cycles):
        print("TEST CYCLE", cycle + 1, "/", cycles)
        for target, label in (
            (angle, "RIGHT"),
            (0, "CENTER"),
            (-angle, "LEFT"),
            (0, "CENTER"),
        ):
            if not goto_angle(target, label):
                return
            time.sleep_ms(500)


def print_status():
    motor_rpm = settings["max_pps"] * 60.0 / settings["ppr"]
    steering_speed = settings["max_pps"] / pulses_per_degree()
    print("\n--- DM860 + NEMA34 STEERING STATUS ---")
    print("Firmware           :", FIRMWARE_NAME, FIRMWARE_VERSION)
    print("State              :", "ARMED" if armed else "DISARMED")
    print("STEP / DIR GPIO    :", STEP_GPIO, "/", DIR_GPIO)
    print("Position estimate  : {:.3f} deg / {} pulses".format(
        current_degrees(), position_steps
    ))
    print("Software limits    : {:.1f} to {:.1f} deg".format(
        settings["min_angle"], settings["max_angle"]
    ))
    print("Pulses/revolution  :", settings["ppr"], "(must match SW5-SW8)")
    print("Steering ratio     :", settings["ratio"], "motor rev / shaft rev")
    print("Pulses/degree      :", round(pulses_per_degree(), 5))
    print("Start / max speed  :", settings["start_pps"], "/", settings["max_pps"], "pps")
    print("Acceleration       :", settings["accel_pps2"], "pps^2")
    print("Pulse active width :", settings["pulse_us"], "us")
    print("Max motor speed    :", round(motor_rpm, 2), "RPM")
    print("Max shaft speed    :", round(steering_speed, 2), "deg/s")
    print("Right GPIO level   :", settings["right_level"])
    print("Total sent pulses  :", total_pulses)
    print("DM860 ENA          : physically disconnected")
    print("Feedback           : NONE; OPEN-LOOP BENCH TEST")


def print_help():
    print("""
Commands:
  status                    show configuration and position estimate
  arm / disarm              allow or block STEP pulses
  zero                      assign current physical position as 0 deg (disarmed)
  left [deg]                absolute left angle; default 35
  right [deg]               absolute right angle; default 35
  center                    return to software 0 deg
  goto <signed_deg>         absolute target, e.g. goto -20
  jog <signed_deg>          relative motion, e.g. jog 2 or jog -2
  turns <signed_turns>      motor-only turn test, e.g. turns 1
  test [deg]                center/right/center/left/center; default 10 deg
  sweep [deg] [cycles]      repeated test; default 35 deg, 1 cycle
  speed <pps>               maximum pulse rate, 50..20000
  start <pps>               starting pulse rate, 50..max speed
  accel <pps_per_s2>        acceleration/deceleration
  ppr <pulses_per_rev>      must match physical DM860 SW5-SW8
  ratio <motor/shaft>       1 for current 1:1 steering
  limits <left> <right>     e.g. limits -35 35
  pulse <microseconds>      active pulse width, 3..200 us
  direction invert          reverse electrical left/right
  q                         disarm and exit

Ctrl-C during movement aborts and disarms. Runtime settings reset on reboot.
""")


def set_number(parts, key, minimum, maximum, cast=int):
    if len(parts) != 2:
        print("VALUE REQUIRED")
        return
    value = cast(parts[1])
    settings[key] = cast(clamp(value, minimum, maximum))
    print(key, "->", settings[key])


print("\n========================================")
print(" DM860 + NEMA34 STEERING BENCH TEST")
print("========================================")
print("Boot state: DISARMED; no STEP pulses are emitted.")
print("Lift steering, verify wiring/DIP settings, physically center, then zero.")
print_help()
print_status()


try:
    while True:
        try:
            parts = input("\nDM860 steering > ").strip().lower().split()
            if not parts:
                continue
            command = parts[0]

            if command in ("help", "h", "?"):
                print_help()
            elif command in ("status", "s"):
                print_status()
            elif command == "arm":
                arm()
            elif command == "disarm":
                disarm()
            elif command in ("zero", "calibrate"):
                calibrate_center()
            elif command == "left":
                goto_angle(-abs(float(parts[1]) if len(parts) > 1 else 35), "LEFT")
            elif command == "right":
                goto_angle(abs(float(parts[1]) if len(parts) > 1 else 35), "RIGHT")
            elif command == "center":
                goto_angle(0, "CENTER")
            elif command == "goto" and len(parts) == 2:
                goto_angle(float(parts[1]))
            elif command == "jog" and len(parts) == 2:
                goto_angle(current_degrees() + float(parts[1]), "JOG")
            elif command == "turns" and len(parts) == 2:
                move_steps(round(float(parts[1]) * settings["ppr"]), "MOTOR TURNS")
            elif command == "test":
                run_sequence(float(parts[1]) if len(parts) > 1 else 10, 1)
            elif command == "sweep":
                run_sequence(
                    float(parts[1]) if len(parts) > 1 else 35,
                    int(parts[2]) if len(parts) > 2 else 1,
                )
            elif command == "speed":
                set_number(parts, "max_pps", MIN_ALLOWED_PPS, MAX_ALLOWED_PPS)
                settings["start_pps"] = min(settings["start_pps"], settings["max_pps"])
            elif command == "start":
                set_number(parts, "start_pps", MIN_ALLOWED_PPS, settings["max_pps"])
            elif command == "accel":
                set_number(parts, "accel_pps2", 50, 100_000)
            elif command == "ppr":
                set_number(parts, "ppr", 200, 51_200)
                print("Physically center, disarm, then run zero.")
            elif command == "ratio":
                set_number(parts, "ratio", 0.1, 100.0, float)
                print("Physically center, disarm, then run zero.")
            elif command == "limits" and len(parts) == 3:
                left = float(parts[1])
                right = float(parts[2])
                if left >= 0 or right <= 0 or left >= right:
                    print("REJECTED: use a negative left and positive right limit")
                else:
                    settings["min_angle"] = left
                    settings["max_angle"] = right
                    print("LIMITS ->", left, "to", right, "deg")
            elif command == "pulse":
                set_number(parts, "pulse_us", MIN_PULSE_US, MAX_PULSE_US)
            elif command == "direction" and len(parts) == 2 and parts[1] == "invert":
                settings["right_level"] = 1 - settings["right_level"]
                print("RIGHT GPIO LEVEL ->", settings["right_level"])
            elif command in ("q", "quit", "exit"):
                break
            else:
                print("Unknown/incomplete command. Type help.")
        except ValueError:
            print("INVALID NUMBER")
        except KeyboardInterrupt:
            step_pin.value(0)
            armed = False
            print("STOPPED AND DISARMED; re-center if motion was interrupted")
finally:
    disarm()
    print("DM860 steering test stopped")

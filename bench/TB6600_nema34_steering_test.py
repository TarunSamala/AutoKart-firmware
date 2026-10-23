"""Standalone ESP32-S3 + TB6600 + NEMA34 steering bench test.

Frozen control pins:
  GPIO4 -> tested 5 V NPN interface -> TB6600 PUL-
  GPIO5 -> tested 5 V NPN interface -> TB6600 DIR-
  GPIO6 -> tested 5 V NPN interface -> TB6600 ENA- (optional)
  Regulated 5 V -> PUL+, DIR+ and ENA+
  ESP32 GND -> NPN emitter/interface ground

Verified NEMA34 winding groups:
  TB6600 A+ = RED + BLUE
  TB6600 A- = YELLOW + BLACK
  TB6600 B+ = ORANGE + GREEN
  TB6600 B- = BROWN + WHITE

This is open-loop test firmware. It cannot detect a stall, skipped step,
mechanical slip or a hard stop. Raise/unload the steering before testing.
Press Ctrl-C during movement for an immediate software abort.

The TB6600 current and microstep DIP switches are physical settings. Set the
firmware PPR to match the pulse/revolution value printed on the exact driver.
Do not infer safe phase current from this program.
"""

from machine import Pin
import math
import time


FIRMWARE_NAME = "tb6600-nema34-steering-test"
FIRMWARE_VERSION = "1.0.0"

STEP_GPIO = 4
DIR_GPIO = 5
ENA_GPIO = 6

# Runtime defaults. Use the commands below to tune these without reflashing.
PULSES_PER_REVOLUTION = 3200
STEERING_RATIO = 1.0       # motor revolutions / steering-shaft revolution
MIN_ANGLE_DEG = -45.0
MAX_ANGLE_DEG = 45.0
START_PPS = 150
MAX_PPS = 1200
ACCEL_PPS2 = 900
STEP_HIGH_US = 20
DIR_SETTLE_MS = 10

# GPIO level at the NPN base that requests a positive/right motor direction.
RIGHT_GPIO_LEVEL = 1

# Set False if ENA is physically disconnected. Some TB6600 variants invert
# ENA behavior; use `ena invert`, then verify while steering is unloaded.
USE_ENABLE = True
ENABLE_RUN_GPIO_LEVEL = 1

MIN_ALLOWED_PPS = 50
MAX_ALLOWED_PPS = 20_000
MIN_STEP_HIGH_US = 3
MAX_STEP_HIGH_US = 200


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


step_pin = Pin(STEP_GPIO, Pin.OUT, value=0)
dir_pin = Pin(DIR_GPIO, Pin.OUT, value=0)
ena_pin = Pin(ENA_GPIO, Pin.OUT, value=1 - ENABLE_RUN_GPIO_LEVEL)

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
    "pulse_high_us": int(STEP_HIGH_US),
    "right_level": int(RIGHT_GPIO_LEVEL),
    "use_enable": bool(USE_ENABLE),
    "enable_run_level": int(ENABLE_RUN_GPIO_LEVEL),
}


def pulses_per_degree():
    return settings["ppr"] * settings["ratio"] / 360.0


def steps_to_degrees(steps):
    return float(steps) / pulses_per_degree()


def degrees_to_steps(degrees):
    return int(round(float(degrees) * pulses_per_degree()))


def current_degrees():
    return steps_to_degrees(position_steps)


def set_enable_output(run):
    if not settings["use_enable"]:
        return
    level = settings["enable_run_level"] if run else 1 - settings["enable_run_level"]
    ena_pin.value(level)


def arm():
    global armed
    step_pin.value(0)
    set_enable_output(True)
    time.sleep_ms(20)
    armed = True
    print("ARMED: steering commands enabled")


def disarm():
    global armed
    step_pin.value(0)
    set_enable_output(False)
    armed = False
    print("DISARMED: STEP inactive")


def motion_speed_pps(progress, remaining):
    """Symmetric acceleration/deceleration profile in pulses per second."""
    start = settings["start_pps"]
    accel = settings["accel_pps2"]
    accelerating = math.sqrt(start * start + 2.0 * accel * progress)
    decelerating = math.sqrt(start * start + 2.0 * accel * remaining)
    return int(min(settings["max_pps"], accelerating, decelerating))


def move_steps(delta_steps, label="MOVE"):
    global position_steps, total_pulses

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
    time.sleep_ms(DIR_SETTLE_MS)

    start_angle = current_degrees()
    target_angle = steps_to_degrees(position_steps + delta_steps)
    print(
        "{}: {:.2f} -> {:.2f} deg | {} pulses".format(
            label, start_angle, target_angle, count
        )
    )

    moved = 0
    try:
        while moved < count:
            remaining = count - moved
            pps = motion_speed_pps(moved, remaining)
            period_us = max(
                settings["pulse_high_us"] + 3,
                int(1_000_000 / max(1, pps)),
            )
            step_pin.value(1)
            time.sleep_us(settings["pulse_high_us"])
            step_pin.value(0)
            low_us = period_us - settings["pulse_high_us"]
            if low_us > 0:
                time.sleep_us(low_us)
            position_steps += sign
            total_pulses += 1
            moved += 1
    except KeyboardInterrupt:
        step_pin.value(0)
        disarm()
        print("ABORTED after", moved, "pulses at", round(current_degrees(), 2), "deg")
        return False

    step_pin.value(0)
    print("ARRIVED:", round(current_degrees(), 2), "deg")
    return True


def goto_angle(target_degrees, label="GOTO"):
    target = clamp(
        float(target_degrees), settings["min_angle"], settings["max_angle"]
    )
    target_steps = degrees_to_steps(target)
    return move_steps(target_steps - position_steps, label)


def calibrate_center():
    global position_steps
    if armed:
        print("REJECTED: disarm and physically center steering first")
        return
    position_steps = 0
    print("CENTER CALIBRATED: current physical position = 0.00 deg")


def print_status():
    motor_rpm = settings["max_pps"] * 60.0 / settings["ppr"]
    steering_speed = settings["max_pps"] * 360.0 / (
        settings["ppr"] * settings["ratio"]
    )
    print("\n--- TB6600 NEMA34 STEERING STATUS ---")
    print("State             :", "ARMED" if armed else "DISARMED")
    print("Position estimate : {:.3f} deg / {} steps".format(current_degrees(), position_steps))
    print("Limits            : {:.1f} to {:.1f} deg".format(settings["min_angle"], settings["max_angle"]))
    print("Pulses/revolution :", settings["ppr"], "(must match TB6600 DIP table)")
    print("Steering ratio    :", settings["ratio"], "motor rev / shaft rev")
    print("Pulses/degree     :", round(pulses_per_degree(), 5))
    print("Start / max speed :", settings["start_pps"], "/", settings["max_pps"], "pps")
    print("Acceleration      :", settings["accel_pps2"], "pps^2")
    print("Pulse HIGH width  :", settings["pulse_high_us"], "us")
    print("Max motor speed   :", round(motor_rpm, 2), "RPM")
    print("Max shaft speed   :", round(steering_speed, 2), "deg/s")
    print("Right GPIO level  :", settings["right_level"])
    print("ENA used/run level:", settings["use_enable"], "/", settings["enable_run_level"])
    print("Total sent pulses :", total_pulses)
    print("Feedback          : OPEN LOOP; no encoder validation in this file")


def print_help():
    print("""
Commands:
  status                    show all runtime settings
  arm / disarm              allow or block motion
  zero                      assign current physical position as 0 deg (disarmed)
  left [deg]                absolute left angle, e.g. left 35
  right [deg]               absolute right angle, e.g. right 35
  center                    return to software 0 deg
  goto <signed_deg>         absolute target, e.g. goto -20
  jog <signed_deg>          relative movement, e.g. jog 2 or jog -2
  turns <signed_turns>      motor-turn test, e.g. turns 1 or turns -1
  test [deg]                center, right, center, left, center (default 10 deg)
  sweep [deg] [cycles]      repeated left/right test (default 35 deg, 1 cycle)
  speed <pps>               maximum pulse rate, 50..20000
  start <pps>               starting pulse rate, 50..max speed
  accel <pps_per_s2>        acceleration/deceleration
  ppr <pulses_per_rev>      match the TB6600 microstep DIP setting
  ratio <motor/shaft>       1 for direct drive
  limits <left> <right>     e.g. limits -35 35
  pulse <microseconds>      STEP HIGH width, 3..200 us
  direction invert          reverse left/right electrical direction
  ena on / ena off          use or ignore GPIO6 ENA
  ena invert                reverse assumed ENA polarity
  q                         disarm and exit

Ctrl-C during movement aborts and disarms the driver.
Settings are runtime-only and reset after ESP32 reboot.
""")


def set_number(parts, key, minimum, maximum, cast=int):
    if len(parts) != 2:
        print("VALUE REQUIRED")
        return
    value = cast(parts[1])
    settings[key] = cast(clamp(value, minimum, maximum))
    print(key, "->", settings[key])


def run_sequence(angle, cycles):
    angle = abs(float(angle))
    angle = min(angle, abs(settings["min_angle"]), settings["max_angle"])
    cycles = max(1, min(20, int(cycles)))
    for cycle in range(cycles):
        print("SEQUENCE CYCLE", cycle + 1, "/", cycles)
        if not goto_angle(angle, "RIGHT"):
            return
        time.sleep_ms(500)
        if not goto_angle(0, "CENTER"):
            return
        time.sleep_ms(500)
        if not goto_angle(-angle, "LEFT"):
            return
        time.sleep_ms(500)
        if not goto_angle(0, "CENTER"):
            return


print("\n========================================")
print(" TB6600 + NEMA34 STEERING TEST")
print("========================================")
print("STEP GPIO4 | DIR GPIO5 | ENA GPIO6")
print("Boot state: DISARMED")
print("Raise/unload steering and verify DIP current before ARM.")
print_help()
print_status()


try:
    while True:
        try:
            parts = input("\nTB6600 steering > ").strip().lower().split()
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
                requested = float(parts[1])
                goto_angle(current_degrees() + requested, "JOG")
            elif command == "turns" and len(parts) == 2:
                move_steps(round(float(parts[1]) * settings["ppr"]), "MOTOR TURNS")
            elif command == "test":
                test_angle = float(parts[1]) if len(parts) > 1 else 10.0
                run_sequence(test_angle, 1)
            elif command == "sweep":
                sweep_angle = float(parts[1]) if len(parts) > 1 else 35.0
                cycles = int(parts[2]) if len(parts) > 2 else 1
                run_sequence(sweep_angle, cycles)
            elif command == "speed":
                set_number(parts, "max_pps", MIN_ALLOWED_PPS, MAX_ALLOWED_PPS)
                settings["start_pps"] = min(settings["start_pps"], settings["max_pps"])
            elif command == "start":
                set_number(parts, "start_pps", MIN_ALLOWED_PPS, settings["max_pps"])
            elif command == "accel":
                set_number(parts, "accel_pps2", 50, 100_000)
            elif command == "ppr":
                set_number(parts, "ppr", 200, 51_200)
                print("Recenter physically, disarm, then run zero.")
            elif command == "ratio":
                set_number(parts, "ratio", 0.1, 100.0, float)
                print("Recenter physically, disarm, then run zero.")
            elif command == "pulse":
                set_number(parts, "pulse_high_us", MIN_STEP_HIGH_US, MAX_STEP_HIGH_US)
            elif command == "limits" and len(parts) == 3:
                left = float(parts[1])
                right = float(parts[2])
                if left >= 0 or right <= 0 or left >= right:
                    print("REJECTED: expected negative left and positive right")
                else:
                    settings["min_angle"] = left
                    settings["max_angle"] = right
                    print("LIMITS ->", left, "to", right, "deg")
            elif command == "direction" and len(parts) == 2 and parts[1] == "invert":
                settings["right_level"] = 1 - settings["right_level"]
                print("RIGHT GPIO LEVEL ->", settings["right_level"])
            elif command == "ena" and len(parts) == 2:
                if parts[1] == "on":
                    settings["use_enable"] = True
                    set_enable_output(armed)
                    print("ENA GPIO6 ENABLED")
                elif parts[1] == "off":
                    settings["use_enable"] = False
                    print("ENA GPIO6 IGNORED; disconnect ENA wiring if required")
                elif parts[1] == "invert":
                    was_armed = armed
                    disarm()
                    settings["enable_run_level"] = 1 - settings["enable_run_level"]
                    set_enable_output(False)
                    print("ENA RUN GPIO LEVEL ->", settings["enable_run_level"])
                    if was_armed:
                        print("Driver remains DISARMED; type arm after verification")
                else:
                    print("Use: ena on | ena off | ena invert")
            elif command in ("q", "quit", "exit"):
                break
            else:
                print("Unknown/incomplete command. Type help.")
        except ValueError:
            print("INVALID NUMBER")
        except KeyboardInterrupt:
            step_pin.value(0)
            disarm()
            print("STOPPED AND DISARMED")
finally:
    disarm()
    print("TB6600 steering test stopped")

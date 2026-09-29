"""Controlled NEMA23 maximum-speed bench test using ESP32-S3 hardware PWM.

Hardware:
  Motor  : JK57HS112-3004-03
  Driver : Rtelligent R86mini in external PUL+DIR mode
  STEP   : GPIO4 through the tested 5 V NPN interface
  DIR    : GPIO5 through the tested 5 V NPN interface
  ENA    : disconnected

R86mini pulse setting:
  3200 pulses/revolution: SW5=OFF, SW6=OFF, SW7=ON, SW8=ON

This is a speed test, not a position test. Hardware PWM creates a continuous
pulse train while the program ramps to the requested speed, holds it briefly,
and ramps back to zero.
"""

from machine import Pin, PWM
import time


FIRMWARE_NAME = "r86mini-nema23-speed-test"
FIRMWARE_VERSION = "1.0.0"

STEP_GPIO = 4
DIR_GPIO = 5

STEP = Pin(STEP_GPIO, Pin.OUT, value=0)
DIR = Pin(DIR_GPIO, Pin.OUT, value=0)

CW_DIRECTION = 1
CCW_DIRECTION = 1 - CW_DIRECTION

PULSES_PER_REVOLUTION = 3200

# Controlled test envelope. At 3200 pulses/rev, 32000 pps = 600 RPM.
MIN_RUNNING_PPS = 200
MAX_TEST_PPS = 32000

# 250 pps every 100 ms = 2500 pps/s = 46.875 RPM/s acceleration.
RAMP_INCREMENT_PPS = 250
RAMP_INTERVAL_MS = 100

DEFAULT_HOLD_SECONDS = 3.0


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def pps_to_rpm(pps):
    return pps * 60.0 / PULSES_PER_REVOLUTION


def stop_pwm(pwm):
    if pwm is not None:
        try:
            pwm.duty_u16(0)
        except Exception:
            pass
        try:
            pwm.deinit()
        except Exception:
            pass

    Pin(STEP_GPIO, Pin.OUT, value=0)


def run_speed_test(direction_level, direction_name, target_pps, hold_seconds):
    target_pps = int(
        clamp(int(target_pps), MIN_RUNNING_PPS, MAX_TEST_PPS)
    )
    hold_seconds = max(0.0, float(hold_seconds))

    print()
    print("DIRECTION :", direction_name)
    print("TARGET    :", target_pps, "pps")
    print("MOTOR RPM :", round(pps_to_rpm(target_pps), 2))
    print("HOLD      :", round(hold_seconds, 2), "seconds")
    print("Ctrl-C stops PWM immediately.")

    DIR.value(direction_level)
    time.sleep_ms(100)

    pwm = None
    current_pps = MIN_RUNNING_PPS

    try:
        pwm = PWM(
            Pin(STEP_GPIO),
            freq=current_pps,
            duty_u16=32768,
        )

        print("RAMP UP")
        while current_pps < target_pps:
            current_pps = min(
                target_pps,
                current_pps + RAMP_INCREMENT_PPS,
            )
            pwm.freq(current_pps)

            # Print at useful milestones without flooding the terminal.
            if current_pps == target_pps or current_pps % 2500 == 0:
                print(
                    " ", current_pps, "pps =",
                    round(pps_to_rpm(current_pps), 1), "RPM",
                )

            time.sleep_ms(RAMP_INTERVAL_MS)

        print("HOLDING", round(pps_to_rpm(target_pps), 1), "RPM")

        hold_end = time.ticks_add(
            time.ticks_ms(),
            int(hold_seconds * 1000),
        )
        while time.ticks_diff(hold_end, time.ticks_ms()) > 0:
            time.sleep_ms(50)

        print("RAMP DOWN")
        while current_pps > MIN_RUNNING_PPS:
            current_pps = max(
                MIN_RUNNING_PPS,
                current_pps - RAMP_INCREMENT_PPS,
            )
            pwm.freq(current_pps)
            time.sleep_ms(RAMP_INTERVAL_MS)

        print("TEST COMPLETE")

    except KeyboardInterrupt:
        print("\nEMERGENCY SOFTWARE STOP")

    finally:
        stop_pwm(pwm)
        print("STEP PWM OFF")


def status():
    print()
    print("========================================")
    print("R86mini + NEMA23 SPEED TEST")
    print("========================================")
    print("Firmware        :", FIRMWARE_NAME, FIRMWARE_VERSION)
    print("Motor           : JK57HS112-3004-03")
    print("STEP / DIR GPIO :", STEP_GPIO, "/", DIR_GPIO)
    print("ENA             : disconnected")
    print("Pulses/rev      :", PULSES_PER_REVOLUTION)
    print("Maximum test    :", MAX_TEST_PPS, "pps")
    print("Maximum test RPM:", round(pps_to_rpm(MAX_TEST_PPS), 2))
    print("Ramp rate       :", 2500, "pps/s")
    print("========================================")
    print()


def help_text():
    print()
    print("Commands:")
    print("  cw 5000       -> ramp CW to 93.75 RPM; hold 3 seconds")
    print("  ccw 5000      -> ramp CCW to 93.75 RPM; hold 3 seconds")
    print("  cw 10000 5    -> ramp CW to 187.5 RPM; hold 5 seconds")
    print("  ccw 20000 2   -> ramp CCW to 375 RPM; hold 2 seconds")
    print("  cw 32000 2    -> maximum staged test: 600 RPM")
    print("  status        -> show configuration")
    print("  help          -> show commands")
    print("  q             -> quit")
    print()


print()
print("========================================")
print(" R86mini + NEMA23 CONTROLLED SPEED TEST")
print("========================================")
print("Motor must be unloaded and securely clamped.")
print("Remove loose keys, gears and couplers from the shaft.")
print("Use a physical driver-power disconnect for emergencies.")
print("Begin at 5000 pps; do not begin with the maximum test.")
help_text()

try:
    while True:
        command = input("speed-test> ").strip().lower()

        if not command:
            continue

        parts = command.split()
        action = parts[0]

        if action in ("cw", "ccw"):
            try:
                target_pps = int(parts[1])
                hold_seconds = (
                    float(parts[2])
                    if len(parts) > 2
                    else DEFAULT_HOLD_SECONDS
                )
            except (ValueError, IndexError):
                print("Example: cw 5000 or ccw 10000 3")
                continue

            direction_level = (
                CW_DIRECTION if action == "cw" else CCW_DIRECTION
            )
            run_speed_test(
                direction_level,
                action.upper(),
                target_pps,
                hold_seconds,
            )

        elif action == "status":
            status()

        elif action == "help":
            help_text()

        elif action == "q":
            break

        else:
            print("Commands: cw pps [hold] | ccw pps [hold] | status | help | q")

except KeyboardInterrupt:
    print("\nSTOPPED BY USER")

finally:
    stop_pwm(None)
    print("SAFE EXIT: STEP LOW")

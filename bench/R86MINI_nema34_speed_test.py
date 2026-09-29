"""Controlled high-speed test for the verified AutoKart NEMA34 wiring.

Motor/driver wiring verified experimentally by the user:
  R86mini A+ = RED + BLUE
  R86mini A- = YELLOW + BLACK
  R86mini B+ = ORANGE + GREEN
  R86mini B- = BROWN + WHITE

Control interface:
  GPIO4 -> tested 5 V NPN interface -> PUL-
  GPIO5 -> tested 5 V NPN interface -> DIR-
  5 V -> PUL+ and DIR+
  ENA+ and ENA- disconnected

R86mini pulse setting:
  3200 pulses/revolution: SW5 OFF, SW6 OFF, SW7 ON, SW8 ON

This test uses hardware PWM for stable high-rate pulses. Every command ramps
up, holds for the requested duration, ramps down, and stops. It does not
automatically advance to the next speed because the software cannot detect a
stalled open-loop stepper motor.
"""

from machine import Pin, PWM
import time


FIRMWARE_NAME = "r86mini-nema34-speed-test"
FIRMWARE_VERSION = "1.0.0"

STEP_GPIO = 4
DIR_GPIO = 5

STEP = Pin(STEP_GPIO, Pin.OUT, value=0)
DIR = Pin(DIR_GPIO, Pin.OUT, value=0)

CW_DIRECTION = 1
CCW_DIRECTION = 0

PULSES_PER_REVOLUTION = 3200

# 24000 pps = 450 RPM at 3200 pulses/revolution.
MIN_RUNNING_PPS = 200
MAX_TEST_PPS = 24000

# Slow ramp for the higher-inertia 5.3 kg NEMA34 rotor.
# 100 pps per 100 ms = 1000 pps/s = 18.75 RPM/s.
RAMP_INCREMENT_PPS = 100
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


def run_test(direction_level, direction_name, requested_pps, hold_seconds):
    target_pps = int(
        clamp(int(requested_pps), MIN_RUNNING_PPS, MAX_TEST_PPS)
    )
    hold_seconds = max(0.0, float(hold_seconds))

    if target_pps != int(requested_pps):
        print("TARGET CLAMPED:", requested_pps, "->", target_pps, "pps")

    print()
    print("DIRECTION :", direction_name)
    print("TARGET    :", target_pps, "pps")
    print("MOTOR RPM :", round(pps_to_rpm(target_pps), 2))
    print("HOLD      :", round(hold_seconds, 2), "seconds")
    print("Ctrl-C stops PWM; keep a physical power cut-off available.")

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
        print("\nSOFTWARE EMERGENCY STOP")

    finally:
        stop_pwm(pwm)
        print("STEP PWM OFF")


def status():
    print()
    print("========================================")
    print("R86mini + NEMA34 SPEED TEST")
    print("========================================")
    print("Firmware         :", FIRMWARE_NAME, FIRMWARE_VERSION)
    print("Pulses/rev       :", PULSES_PER_REVOLUTION)
    print("Maximum test     :", MAX_TEST_PPS, "pps")
    print("Maximum test RPM :", round(pps_to_rpm(MAX_TEST_PPS), 2))
    print("Ramp             : 1000 pps/s")
    print("ENA              : disconnected")
    print("========================================")
    print()


def help_text():
    print()
    print("Commands: direction, pulse-rate, optional hold time")
    print("  cw 1000 3      -> 18.75 RPM")
    print("  cw 2500 3      -> 46.88 RPM")
    print("  cw 5000 3      -> 93.75 RPM")
    print("  cw 7500 3      -> 140.63 RPM")
    print("  cw 10000 3     -> 187.50 RPM")
    print("  cw 15000 3     -> 281.25 RPM")
    print("  cw 20000 2     -> 375.00 RPM")
    print("  cw 24000 2     -> 450.00 RPM software cap")
    print("  ccw ...        -> same test counter-clockwise")
    print("  status         -> show configuration")
    print("  help           -> show commands")
    print("  q              -> quit")
    print()


print()
print("========================================")
print(" R86mini + NEMA34 STAGED SPEED TEST")
print("========================================")
print("Clamp the unloaded motor securely.")
print("Remove loose keys, gears and couplers from the shaft.")
print("Test one stage at a time and listen for loss of synchronism.")
help_text()

try:
    while True:
        parts = input("nema34-speed> ").strip().lower().split()

        if not parts:
            continue

        command = parts[0]

        if command in ("cw", "ccw"):
            try:
                target_pps = int(parts[1])
                hold_seconds = (
                    float(parts[2])
                    if len(parts) > 2
                    else DEFAULT_HOLD_SECONDS
                )
            except (ValueError, IndexError):
                print("Example: cw 2500 3 or ccw 5000 2")
                continue

            run_test(
                CW_DIRECTION if command == "cw" else CCW_DIRECTION,
                command.upper(),
                target_pps,
                hold_seconds,
            )

        elif command == "status":
            status()

        elif command == "help":
            help_text()

        elif command == "q":
            break

        else:
            print("Commands: cw pps [hold] | ccw pps [hold] | status | help | q")

except KeyboardInterrupt:
    print("\nSTOPPED BY USER")

finally:
    stop_pwm(None)
    print("SAFE EXIT: STEP LOW")

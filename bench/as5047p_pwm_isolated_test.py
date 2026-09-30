"""Standalone ESP32-S3 AS5047P PWM-output test.

This file is independent of braking, steering, TB6600 drivers, and motors.
It measures the AS5047P PWM output and converts the pulse width to the
12-bit angle data described in the AS5047P datasheet.

Suggested isolated wiring:
  AS5047P W/PWM (or module PWM) -> ESP32-S3 GPIO4
  AS5047P GND                  -> ESP32-S3 GND
  AS5047P supply               -> the voltage required by the breakout board

Important:
  - The AS5047P PWM interface must already be enabled/configured.
  - The PWM output may be on W/PWM (sensor pin 8) or I/PWM (sensor pin 14),
    depending on the UVW_ABI setting. This test does not configure the sensor
    over SPI; it only measures the selected PWM output.
  - Do not connect a 5 V signal directly to an ESP32-S3 GPIO.

Commands:
  r        capture and print one PWM frame
  stream   continuously print PWM data
  help     show commands
  q        stop the test

The AS5047P PWM frame is nominally 4119 clock periods. The angle data is
represented by 0..4095 data clocks, with a typical PWM clock of 444 ns.
"""

from machine import Pin, time_pulse_us
import time


PWM_PIN = 4
TIMEOUT_US = 100000
PWM_CLOCK_US = 0.444
INIT_CLOCKS = 12
DATA_CLOCKS = 4095
FRAME_CLOCKS = 4119


def capture_frame():
    pin = Pin(PWM_PIN, Pin.IN)

    # The AS5047P frame starts with a high section. Capture one high pulse
    # followed by its low section; this avoids needing SPI or an interrupt.
    high_us = time_pulse_us(pin, 1, TIMEOUT_US)
    low_us = time_pulse_us(pin, 0, TIMEOUT_US)

    if high_us < 0 or low_us < 0:
        raise RuntimeError(
            "PWM edge timeout (high={}, low={})".format(high_us, low_us)
        )

    period_us = high_us + low_us
    frequency_hz = 1000000.0 / period_us if period_us else 0.0
    duty_percent = 100.0 * high_us / period_us if period_us else 0.0

    # The initial 12 clocks are high. The following data section contains
    # the absolute angle. Timing is approximate, so clamp to the valid range.
    data_clocks = int(round(high_us / PWM_CLOCK_US)) - INIT_CLOCKS
    data_clocks = max(0, min(DATA_CLOCKS, data_clocks))
    angle_deg = data_clocks * 360.0 / DATA_CLOCKS

    return high_us, low_us, period_us, frequency_hz, duty_percent, data_clocks, angle_deg


def print_frame():
    values = capture_frame()
    high_us, low_us, period_us, frequency_hz, duty_percent, raw, angle_deg = values
    print(
        "PWM high: {:6d} us | low: {:6d} us | period: {:7d} us | "
        "freq: {:7.1f} Hz | duty: {:6.2f}%".format(
            high_us, low_us, period_us, frequency_hz, duty_percent
        )
    )
    print(
        "AS5047P PWM RAW: {:4d}/4095 | ANGLE: {:8.3f} deg"
        .format(raw, angle_deg)
    )


def print_help():
    print("r | stream | help | q")
    print("PWM input pin: GPIO{}".format(PWM_PIN))


print("\nESP32-S3 STANDALONE AS5047P PWM TEST")
print("No braking, steering, TB6600, SPI, or motor control is used.")
print("PWM input: ESP32-S3 GPIO{}".format(PWM_PIN))
print("The AS5047P PWM output must already be enabled.")
print_help()

while True:
    try:
        command = input("\nAS5047P PWM > ").strip().lower()
        if command == "r":
            print_frame()
        elif command == "stream":
            print("Streaming. Press Ctrl-C to return to the command prompt.")
            try:
                while True:
                    print_frame()
                    time.sleep_ms(100)
            except KeyboardInterrupt:
                print("\nStream stopped.")
        elif command == "help":
            print_help()
        elif command == "q":
            print("AS5047P PWM test stopped.")
            break
        elif command:
            print("Unknown command. Type help.")
    except KeyboardInterrupt:
        print("\nType help for commands, or q to stop.")
    except Exception as error:
        print("PWM read error:", error)

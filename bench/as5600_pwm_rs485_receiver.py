"""ESP32-S3 receiver test for AS5600 PWM transported over RS485.

Temporary receiver wiring:
  MAX3485 VCC -> ESP32-S3 3V3
  MAX3485 GND -> ESP32-S3 GND
  MAX3485 EN  -> GND (receive mode)
  MAX3485 TX/TXD/RO -> ESP32-S3 GPIO13

The AS5600 OUT must already be configured for PWM. This test measures the
PWM returned by the MAX3485; it does not use I2C or configure the AS5600.
"""

from machine import Pin, time_pulse_us
import time


PWM_PIN = 13
TIMEOUT_US = 200000
pwm_input = Pin(PWM_PIN, Pin.IN)


def read_pwm():
    high_us = time_pulse_us(pwm_input, 1, TIMEOUT_US)
    if high_us < 0:
        return None

    low_us = time_pulse_us(pwm_input, 0, TIMEOUT_US)
    if low_us < 0:
        return None

    period_us = high_us + low_us
    duty = high_us / period_us

    # AS5600 PWM frame: 128 header clocks + 4095 angle clocks + 128 low.
    data_clocks = (duty * 4351.0) - 128.0
    data_clocks = max(0.0, min(4095.0, data_clocks))
    angle_deg = data_clocks * 360.0 / 4095.0

    return period_us, duty * 100.0, angle_deg


print("\nESP32-S3 AS5600 PWM OVER RS485 RECEIVER TEST")
print("MAX3485 TX/TXD/RO -> GPIO", PWM_PIN)
print("MAX3485 EN = LOW (receive mode)")
print("Waiting for AS5600 PWM...\n")

while True:
    result = read_pwm()

    if result is None:
        print("NO PWM SIGNAL - check AS5600 PWM mode, A/B, EN, and magnet")
    else:
        period_us, duty_percent, angle_deg = result
        frequency_hz = 1000000.0 / period_us
        print(
            "FREQ: {:7.2f} Hz | DUTY: {:6.2f}% | ANGLE: {:7.2f} deg"
            .format(frequency_hz, duty_percent, angle_deg)
        )

    time.sleep_ms(100)

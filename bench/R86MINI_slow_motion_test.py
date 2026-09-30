"""ESP32-S3 + R86mini slow one-revolution CW/CCW test.

This diagnostic uses only STEP/PUL and DIR. Leave ENA+ and ENA- disconnected.

Required driver setup for this exact pulse count:
  PUL+DIR mode
  400 pulses/revolution
  SW5=ON, SW6=ON, SW7=ON, SW8=ON

GPIO assignment:
  GPIO4 -> PUL signal interface
  GPIO5 -> DIR signal interface

The code outputs 400 pulses at 100 pulses/second:
  400 pulses = one motor revolution
  100 pulses/s = approximately 15 motor RPM

If an NPN low-side 5 V optocoupler interface is used, GPIO HIGH turns the
transistor and optocoupler ON, so SIGNAL_ACTIVE=1 is correct.
"""

from machine import Pin
import time


STEP_PIN = 4
DIR_PIN = 5

STEP = Pin(STEP_PIN, Pin.OUT, value=0)
DIR = Pin(DIR_PIN, Pin.OUT, value=0)

SIGNAL_ACTIVE = 1
SIGNAL_INACTIVE = 0

PULSES_PER_REV = 400
STEP_HIGH_US = 500
STEP_LOW_US = 9500
DIR_SETTLE_MS = 500
PAUSE_MS = 2000


def move_one_revolution(direction_level, direction_name):
    DIR.value(direction_level)
    time.sleep_ms(DIR_SETTLE_MS)

    print(direction_name, "-", PULSES_PER_REV, "pulses")

    for _ in range(PULSES_PER_REV):
        STEP.value(SIGNAL_ACTIVE)
        time.sleep_us(STEP_HIGH_US)

        STEP.value(SIGNAL_INACTIVE)
        time.sleep_us(STEP_LOW_US)


print("\nR86mini SLOW STEP/DIR MOTION TEST")
print("GPIO4=STEP/PUL, GPIO5=DIR, ENA disconnected")
print("Driver must be in PUL+DIR mode at 400 pulses/revolution.")
print("Expected motion: one revolution CW, then one revolution CCW.")
print("Starting in 3 seconds. Press Ctrl-C to stop.\n")
time.sleep(3)

try:
    while True:
        move_one_revolution(1, "CW")
        time.sleep_ms(PAUSE_MS)

        move_one_revolution(0, "CCW")
        time.sleep_ms(PAUSE_MS)

except KeyboardInterrupt:
    print("\nMotion test stopped")

finally:
    STEP.value(SIGNAL_INACTIVE)
    print("STEP/PUL forced inactive")

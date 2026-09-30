"""ESP32-S3 GPIO4 / R86mini PUL input static-level test.

Test 1:
  Disconnect GPIO4 from the driver and measure GPIO4 to ESP32 GND.

Test 2:
  Reconnect the chosen signal interface and measure PUL+ to PUL-.

Expected level changes every two seconds:
  HIGH: approximately 3.3 V at the ESP32 GPIO
  LOW:  approximately 0 V at the ESP32 GPIO

ENA is deliberately not used by this test.
"""

from machine import Pin
import time


PUL_PIN = 4
PUL = Pin(PUL_PIN, Pin.OUT, value=0)

print("\nR86mini PUL STATIC LEVEL TEST")
print("GPIO{} changes HIGH/LOW every 2 seconds.".format(PUL_PIN))
print("Press Ctrl-C to stop.\n")

try:
    while True:
        PUL.value(1)
        print("HIGH")
        time.sleep(2)

        PUL.value(0)
        print("LOW")
        time.sleep(2)

except KeyboardInterrupt:
    print("\nPUL test stopped")

finally:
    PUL.value(0)
    print("GPIO{} forced LOW".format(PUL_PIN))

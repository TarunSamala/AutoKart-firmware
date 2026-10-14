from machine import Pin
import time

PUL = Pin(4, Pin.OUT)

while True:
    PUL.value(1)
    print("HIGH")
    time.sleep(2)

    PUL.value(0)
    print("LOW")
    time.sleep(2)
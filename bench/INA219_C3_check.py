from machine import Pin, I2C
import time

# XIAO ESP32-C3
# D4 = GPIO6 = SDA
# D5 = GPIO7 = SCL

i2c = I2C(
    0,
    sda=Pin(6),
    scl=Pin(7),
    freq=100000
)

print("Scanning I2C bus...")

devices = i2c.scan()

if devices:
    print("I2C devices found:")
    for device in devices:
        print("  Address:", hex(device))
else:
    print("No I2C devices found.")
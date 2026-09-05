"""ESP32-S3 direct AS5600 I2C encoder test.

Temporary test wiring:
  AS5600 VCC -> ESP32-S3 3V3
  AS5600 GND -> ESP32-S3 GND
  AS5600 SDA -> ESP32-S3 GPIO8
  AS5600 SCL -> ESP32-S3 GPIO9

The AS5600 uses I2C address 0x36. No calibration registers are written.
"""

from machine import Pin, I2C
import time


SDA_PIN = 8
SCL_PIN = 9
I2C_FREQ = 100000
AS5600_ADDR = 0x36

REG_STATUS = 0x0B
REG_RAW_ANGLE = 0x0C
REG_ANGLE = 0x0E
REG_MAGNITUDE = 0x1B


def read_u16(i2c, register):
    data = i2c.readfrom_mem(AS5600_ADDR, register, 2)
    return (data[0] << 8) | data[1]


print("\nESP32-S3 AS5600 DIRECT I2C TEST")
print("SDA = GPIO", SDA_PIN, " SCL = GPIO", SCL_PIN)

i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)
devices = i2c.scan()
print("I2C devices:", [hex(x) for x in devices])

if AS5600_ADDR not in devices:
    print("AS5600 NOT FOUND at 0x36")
    print("Check VCC, GND, SDA/SCL, pull-ups, and the magnet position.")
    raise RuntimeError("AS5600 not detected")

print("AS5600 found at 0x36")
print("Rotate the magnet; angle should change from 0 to 360 degrees.\n")

while True:
    status = i2c.readfrom_mem(AS5600_ADDR, REG_STATUS, 1)[0]
    raw = read_u16(i2c, REG_RAW_ANGLE) & 0x0FFF
    angle = read_u16(i2c, REG_ANGLE) & 0x0FFF
    magnitude = read_u16(i2c, REG_MAGNITUDE) & 0x0FFF

    magnet_detected = bool(status & 0x20)
    magnet_too_strong = bool(status & 0x08)
    magnet_too_weak = bool(status & 0x10)

    print(
        "RAW: {:4d} ({:7.2f} deg) | "
        "ANGLE: {:4d} ({:7.2f} deg) | MAG: {:4d} | "
        "MD:{} ML:{} MH:{}".format(
            raw, raw * 360.0 / 4096.0,
            angle, angle * 360.0 / 4096.0,
            magnitude,
            int(magnet_detected),
            int(magnet_too_weak),
            int(magnet_too_strong),
        )
    )
    time.sleep_ms(100)


"""ESP32-S3 direct MPU-9250 / MPU-6500 I2C smoke test.

Wiring:
  ESP32-S3 GPIO8  -> IMU SDA
  ESP32-S3 GPIO9  -> IMU SCL
  ESP32-S3 3V3    -> IMU VCC
  ESP32-S3 GND    -> IMU GND

The pins can be changed below if your test wiring is different.
This test does not use RS485.
"""

from machine import Pin, I2C
import time


SDA_PIN = 8
SCL_PIN = 9
I2C_FREQ = 400000

MPU_ADDRS = (0x68, 0x69)
WHO_AM_I = 0x75
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B


def s16(high, low):
    value = (high << 8) | low
    return value - 65536 if value & 0x8000 else value


def write8(i2c, address, register, value):
    i2c.writeto_mem(address, register, bytes([value]))


def detect_mpu(i2c):
    for address in MPU_ADDRS:
        try:
            identity = i2c.readfrom_mem(address, WHO_AM_I, 1)[0]
        except OSError:
            continue

        if identity == 0x71:
            return address, identity, "MPU-9250"
        if identity == 0x70:
            return address, identity, "MPU-6500"

        print("I2C device at", hex(address),
              "but unexpected WHO_AM_I =", hex(identity))

    return None, None, None


def read_motion(i2c, address):
    data = i2c.readfrom_mem(address, ACCEL_XOUT_H, 14)

    ax = s16(data[0], data[1]) / 16384.0
    ay = s16(data[2], data[3]) / 16384.0
    az = s16(data[4], data[5]) / 16384.0
    temperature_c = (s16(data[6], data[7]) / 333.87) + 21.0
    gx = s16(data[8], data[9]) / 131.0
    gy = s16(data[10], data[11]) / 131.0
    gz = s16(data[12], data[13]) / 131.0

    return ax, ay, az, temperature_c, gx, gy, gz


print("\nESP32-S3 MPU-9250/6500 DIRECT I2C TEST")
print("SDA = GPIO", SDA_PIN, " SCL = GPIO", SCL_PIN)

i2c = I2C(
    0,
    sda=Pin(SDA_PIN),
    scl=Pin(SCL_PIN),
    freq=I2C_FREQ,
)

devices = i2c.scan()
print("I2C devices:", [hex(x) for x in devices])

address, identity, model = detect_mpu(i2c)

if address is None:
    print("MPU NOT FOUND")
    print("Check VCC, GND, SDA/SCL, pull-ups, and AD0 address.")
    raise RuntimeError("MPU-9250/6500 not detected")

print(model, "found at", hex(address),
      "WHO_AM_I =", hex(identity))

write8(i2c, address, PWR_MGMT_1, 0x01)
time.sleep_ms(100)

# ±2 g accelerometer and ±250 degrees/s gyroscope.
write8(i2c, address, ACCEL_CONFIG, 0x00)
write8(i2c, address, GYRO_CONFIG, 0x00)

print("Streaming accel [g], gyro [deg/s], temperature [C]. CTRL+C to stop.\n")

try:
    while True:
        ax, ay, az, temp, gx, gy, gz = read_motion(i2c, address)
        print(
            "A: {: .3f} {: .3f} {: .3f} g | "
            "G: {: .2f} {: .2f} {: .2f} deg/s | "
            "T: {: .2f} C".format(
                ax, ay, az, gx, gy, gz, temp
            )
        )
        time.sleep_ms(100)

except KeyboardInterrupt:
    print("\nIMU test stopped")

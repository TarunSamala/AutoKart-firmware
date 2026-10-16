#!/usr/bin/env python3
"""Raspberry Pi 4B direct MPU-9250/MPU-6500 I2C smoke test.

Connections (Raspberry Pi physical header numbers):
  Pin 1  / 3V3       -> MPU VCC
  Pin 6  / GND       -> MPU GND and ADD/AD0
  Pin 3  / GPIO2 SDA -> MPU SDA
  Pin 5  / GPIO3 SCL -> MPU SCL
  3V3                -> MPU NCS/CS, if the board exposes it
  MPU INT            -> not connected for this polling test

ADD/AD0 to GND selects 0x68. Connecting it to 3V3 selects 0x69.
"""

import argparse
import time

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


WHO_AM_I = 0x75
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B

IDENTITIES = {
    0x70: "MPU-6500",
    0x71: "MPU-9250",
    0x73: "MPU-9255",
}


def signed16(high, low):
    value = (high << 8) | low
    return value - 65536 if value & 0x8000 else value


def detect(bus):
    for address in (0x68, 0x69):
        try:
            identity = bus.read_byte_data(address, WHO_AM_I)
        except OSError:
            continue

        return address, identity, IDENTITIES.get(
            identity,
            "unknown MPU-compatible device",
        )

    return None, None, None


def configure(bus, address):
    # Wake sensor and select a stable clock source.
    bus.write_byte_data(address, PWR_MGMT_1, 0x01)
    time.sleep(0.1)

    # Accelerometer +/-2 g and gyroscope +/-250 degrees/s.
    bus.write_byte_data(address, ACCEL_CONFIG, 0x00)
    bus.write_byte_data(address, GYRO_CONFIG, 0x00)


def read_motion(bus, address):
    data = bus.read_i2c_block_data(address, ACCEL_XOUT_H, 14)

    ax = signed16(data[0], data[1]) / 16384.0
    ay = signed16(data[2], data[3]) / 16384.0
    az = signed16(data[4], data[5]) / 16384.0
    temperature_c = signed16(data[6], data[7]) / 333.87 + 21.0
    gx = signed16(data[8], data[9]) / 131.0
    gy = signed16(data[10], data[11]) / 131.0
    gz = signed16(data[12], data[13]) / 131.0

    return ax, ay, az, temperature_c, gx, gy, gz


def main():
    parser = argparse.ArgumentParser(
        description="Test an MPU-9250/6500 on Raspberry Pi I2C",
    )
    parser.add_argument(
        "--bus",
        type=int,
        default=1,
        help="Linux I2C bus number (default: 1)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="seconds between readings (default: 0.1)",
    )
    args = parser.parse_args()

    print("Raspberry Pi MPU-9250/6500 I2C test")
    print("Using /dev/i2c-{}".format(args.bus))

    bus = SMBus(args.bus)
    try:
        address, identity, model = detect(bus)

        if address is None:
            raise SystemExit(
                "MPU not detected at 0x68 or 0x69. "
                "Check power, ground, SDA/SCL, ADD/AD0, and NCS/CS."
            )

        print(
            "{} found at 0x{:02X}; WHO_AM_I=0x{:02X}".format(
                model,
                address,
                identity,
            )
        )

        if identity not in IDENTITIES:
            print("WARNING: unexpected WHO_AM_I; readings may be unreliable")

        configure(bus, address)
        print("Streaming readings. Press Ctrl-C to stop.\n")

        try:
            while True:
                ax, ay, az, temperature, gx, gy, gz = read_motion(
                    bus,
                    address,
                )
                print(
                    "A: {:+7.3f} {:+7.3f} {:+7.3f} g | "
                    "G: {:+8.2f} {:+8.2f} {:+8.2f} deg/s | "
                    "T: {:6.2f} C".format(
                        ax,
                        ay,
                        az,
                        gx,
                        gy,
                        gz,
                        temperature,
                    )
                )
                time.sleep(max(0.01, args.interval))
        except KeyboardInterrupt:
            print("\nIMU test stopped")
    finally:
        close = getattr(bus, "close", None)
        if close is not None:
            close()


if __name__ == "__main__":
    main()

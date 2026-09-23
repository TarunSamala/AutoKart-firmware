"""Standalone ESP32-S3 AS5600 angle and raw-data test.

This test is independent of braking, steering, TB6600 drivers, and motors.
It only reads the AS5600 over I2C and calculates the shortest rotation to a
requested absolute angle.

Wiring:
  AS5600 VCC -> ESP32-S3 3V3
  AS5600 GND -> ESP32-S3 GND
  AS5600 SDA -> ESP32-S3 GPIO8
  AS5600 SCL -> ESP32-S3 GPIO9

Commands:
  r             show one raw and angle reading
  angle 90      calculate rotation needed to reach 90 degrees
  90            shorthand for angle 90
  raw 2048      calculate rotation needed to reach raw count 2048
  stream        continuously print raw data
  help          show commands
  q             stop the test

The AS5600 is an absolute 12-bit sensor:
  raw range:     0..4095
  angle range:   0..359.91 degrees
  counts/degree: 4096 / 360 = 11.3778

The result is a measurement/calculation only. The ESP32-S3 does not rotate
anything automatically.
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
COUNTS_PER_REV = 4096


def read_u16(register):
    data = i2c.readfrom_mem(AS5600_ADDR, register, 2)
    return ((data[0] << 8) | data[1]) & 0x0FFF


def raw_to_degrees(raw):
    return raw * 360.0 / COUNTS_PER_REV


def degrees_to_raw(degrees):
    degrees = float(degrees) % 360.0
    return int(round(degrees * COUNTS_PER_REV / 360.0)) % COUNTS_PER_REV


def shortest_raw_delta(current_raw, target_raw):
    """Signed shortest movement from current to target, in raw counts."""
    return ((target_raw - current_raw + 2048) % COUNTS_PER_REV) - 2048


def magnet_status(status):
    if not (status & 0x20):
        return "NO MAGNET"
    if status & 0x10:
        return "MAGNET TOO WEAK"
    if status & 0x08:
        return "MAGNET TOO STRONG"
    return "MAGNET OK"


def read_data():
    status = i2c.readfrom_mem(AS5600_ADDR, REG_STATUS, 1)[0]
    raw = read_u16(REG_RAW_ANGLE)
    angle = read_u16(REG_ANGLE)
    magnitude = read_u16(REG_MAGNITUDE)
    return status, raw, angle, magnitude


def print_data():
    status, raw, angle, magnitude = read_data()
    print(
        "RAW: {:4d}/{:4d} | RAW ANGLE: {:8.3f} deg | "
        "FILTERED ANGLE: {:8.3f} deg | MAG: {:4d} | {}"
        .format(
            raw, COUNTS_PER_REV - 1,
            raw_to_degrees(raw),
            raw_to_degrees(angle),
            magnitude,
            magnet_status(status),
        )
    )


def calculate_target(target_degrees):
    status, current_raw, angle, magnitude = read_data()
    target_raw = degrees_to_raw(target_degrees)
    delta_raw = shortest_raw_delta(current_raw, target_raw)
    delta_degrees = delta_raw * 360.0 / COUNTS_PER_REV

    print("\nTARGET CALCULATION")
    print("Current raw:  {} ({:.3f} deg)".format(current_raw, raw_to_degrees(current_raw)))
    print("Target:       {:.3f} deg".format(float(target_degrees) % 360.0))
    print("Target raw:   {}".format(target_raw))
    print("Magnet:       {}".format(magnet_status(status)))

    if delta_raw == 0:
        print("Already at target: move 0 raw counts / 0.000 degrees.")
    elif delta_raw > 0:
        print(
            "Rotate CLOCKWISE by {} raw counts / {:.3f} degrees."
            .format(delta_raw, delta_degrees)
        )
    else:
        print(
            "Rotate COUNTER-CLOCKWISE by {} raw counts / {:.3f} degrees."
            .format(abs(delta_raw), abs(delta_degrees))
        )


def calculate_raw_target(target_raw):
    target_raw = int(target_raw) % COUNTS_PER_REV
    status, current_raw, angle, magnitude = read_data()
    delta_raw = shortest_raw_delta(current_raw, target_raw)
    delta_degrees = delta_raw * 360.0 / COUNTS_PER_REV

    print("\nRAW TARGET CALCULATION")
    print("Current raw: {} ({:.3f} deg)".format(current_raw, raw_to_degrees(current_raw)))
    print("Target raw:  {} ({:.3f} deg)".format(target_raw, raw_to_degrees(target_raw)))
    print("Magnet:      {}".format(magnet_status(status)))

    if delta_raw == 0:
        print("Already at target: move 0 raw counts / 0.000 degrees.")
    elif delta_raw > 0:
        print("Rotate CLOCKWISE by {} raw counts / {:.3f} degrees.".format(delta_raw, delta_degrees))
    else:
        print("Rotate COUNTER-CLOCKWISE by {} raw counts / {:.3f} degrees.".format(abs(delta_raw), abs(delta_degrees)))


def print_help():
    print("r | angle <0..360> | <0..360> | raw <0..4095> | stream | help | q")


print("\nESP32-S3 STANDALONE AS5600 TEST")
print("No braking, steering, TB6600, or motor control is used.")
print("I2C: SDA GPIO{}  SCL GPIO{}  address 0x{:02X}".format(SDA_PIN, SCL_PIN, AS5600_ADDR))

i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)
devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])
if AS5600_ADDR not in devices:
    raise RuntimeError("AS5600 not detected at 0x36")

print_data()
print_help()

while True:
    try:
        command = input("\nAS5600 > ").strip().lower()

        if command == "r":
            print_data()
        elif command == "stream":
            print("Streaming. Press Ctrl-C to return to the command prompt.")
            try:
                while True:
                    print_data()
                    time.sleep_ms(100)
            except KeyboardInterrupt:
                print("\nStream stopped.")
        elif command.startswith("angle "):
            calculate_target(float(command.split()[1]))
        elif command.startswith("raw "):
            calculate_raw_target(int(command.split()[1]))
        elif command == "help":
            print_help()
        elif command == "q":
            print("AS5600 test stopped.")
            break
        elif command:
            try:
                calculate_target(float(command))
            except ValueError:
                print("Unknown command. Type help.")
    except KeyboardInterrupt:
        print("\nType help for commands, or q to stop.")
    except Exception as error:
        print("Command/read error:", error)

"""ESP32-S3 AS5600 brake-feedback calibration and readout test.

This file deliberately does not move the brake motor. It only reads the
AS5600 mounted on the brake shaft so that the mechanical feedback range can
be measured safely before closed-loop control is added.

Wiring:
  AS5600 VCC -> ESP32-S3 3V3
  AS5600 GND -> ESP32-S3 GND
  AS5600 SDA -> ESP32-S3 GPIO8
  AS5600 SCL -> ESP32-S3 GPIO9

Commands after startup:
  r       read one sample
  stream  print feedback continuously
  zero    capture the physically released-brake position
  limit   capture the physically applied-brake position
  status  show the calibration and current brake percentage
  save    save the zero/span to the ESP32 filesystem
  clear   erase the saved calibration
  help    show commands
  q       stop this test

Calibration sequence:
  1. Put the brake at RELEASED and run: zero
  2. Move the brake to the intended APPLIED limit and run: limit
  3. Run: save

The reported brake percentage is feedback only. It is not yet connected to
the TB6600 brake controller or the BLDC safety logic.
"""

from machine import Pin, I2C
import time


SDA_PIN = 8
SCL_PIN = 9
I2C_FREQ = 100000
AS5600_ADDR = 0x36
CAL_FILE = "as5600_brake_cal.txt"

REG_STATUS = 0x0B
REG_RAW_ANGLE = 0x0C
REG_ANGLE = 0x0E
REG_MAGNITUDE = 0x1B

MIN_SPAN_DEG = 5.0


def read_u16(i2c, register):
    data = i2c.readfrom_mem(AS5600_ADDR, register, 2)
    return ((data[0] << 8) | data[1]) & 0x0FFF


def read_sample():
    status = i2c.readfrom_mem(AS5600_ADDR, REG_STATUS, 1)[0]
    raw = read_u16(i2c, REG_RAW_ANGLE)
    angle = read_u16(i2c, REG_ANGLE)
    magnitude = read_u16(i2c, REG_MAGNITUDE)
    return status, raw, angle, magnitude


def degrees(counts):
    return counts * 360.0 / 4096.0


def signed_delta(raw, reference):
    """Return the shortest signed AS5600 movement from reference."""
    return ((raw - reference + 2048) % 4096) - 2048


def magnet_state(status):
    detected = bool(status & 0x20)
    too_weak = bool(status & 0x10)
    too_strong = bool(status & 0x08)
    if not detected:
        return "NO MAGNET"
    if too_weak:
        return "MAGNET WEAK"
    if too_strong:
        return "MAGNET STRONG"
    return "MAGNET OK"


def load_calibration():
    try:
        with open(CAL_FILE, "r") as file:
            values = [float(value) for value in file.read().strip().split(",")]
        if len(values) != 2:
            return None, None
        return values[0], values[1]
    except Exception:
        return None, None


def save_calibration():
    if zero_raw is None or span_deg is None:
        print("Nothing to save: run zero and limit first.")
        return
    with open(CAL_FILE, "w") as file:
        file.write("{:.3f},{:.3f}\n".format(zero_raw, span_deg))
    print("Saved calibration to", CAL_FILE)


def clear_calibration():
    global zero_raw, span_deg
    try:
        import os
        os.remove(CAL_FILE)
    except Exception:
        pass
    zero_raw = None
    span_deg = None
    print("Calibration cleared.")


def feedback(raw):
    if zero_raw is None:
        return None, None
    relative = degrees(signed_delta(raw, int(zero_raw)))
    if span_deg is None or abs(span_deg) < MIN_SPAN_DEG:
        return relative, None
    percent = (relative / span_deg) * 100.0
    percent = max(0.0, min(100.0, percent))
    return relative, percent


def print_sample():
    status, raw, angle, magnitude = read_sample()
    relative, percent = feedback(raw)
    line = (
        "RAW {:4d} ({:7.2f} deg) | ANGLE {:4d} ({:7.2f} deg) | "
        "MAG {:4d} | {}"
    ).format(raw, degrees(raw), angle, degrees(angle), magnitude, magnet_state(status))
    print(line)
    if relative is not None:
        if percent is None:
            print("  RELEASED reference: {:7.2f} deg | span not set".format(relative))
        else:
            print(
                "  relative: {:7.2f} deg | brake feedback: {:6.1f}%"
                .format(relative, percent)
            )


def print_status():
    print("\n--- AS5600 BRAKE FEEDBACK STATUS ---")
    if zero_raw is None:
        print("Released zero: NOT SET")
    else:
        print("Released zero: raw {:.0f} ({:.2f} deg)".format(zero_raw, degrees(zero_raw)))
    if span_deg is None:
        print("Applied span:  NOT SET")
    else:
        print("Applied span:  {:.2f} deg".format(span_deg))
        print("Direction:     {}".format("increasing" if span_deg > 0 else "decreasing"))
    print_sample()


def capture_zero():
    global zero_raw, span_deg
    print("Place the brake in the physically RELEASED position.")
    input("Press Enter to capture released zero...")
    status, raw, angle, magnitude = read_sample()
    if not (status & 0x20) or (status & 0x18):
        print("WARNING: magnet status is not healthy:", magnet_state(status))
    zero_raw = raw
    span_deg = None
    print("Released zero captured: raw {} ({:.2f} deg)".format(raw, degrees(raw)))


def capture_limit():
    global span_deg
    if zero_raw is None:
        print("Run zero first.")
        return
    print("Move the brake to the intended fully APPLIED calibration limit.")
    input("Press Enter to capture applied limit...")
    status, raw, angle, magnitude = read_sample()
    if not (status & 0x20) or (status & 0x18):
        print("WARNING: magnet status is not healthy:", magnet_state(status))
    span_deg = degrees(signed_delta(raw, int(zero_raw)))
    if abs(span_deg) < MIN_SPAN_DEG:
        span_deg = None
        print("Span is too small. Check magnet coupling and brake movement.")
        return
    print("Applied limit captured: raw {} ({:.2f} deg from release)".format(raw, span_deg))
    print("Direction:", "increasing" if span_deg > 0 else "decreasing")


def print_help():
    print("r, stream, zero, limit, status, save, clear, help, q")


print("\nESP32-S3 AS5600 BRAKE FEEDBACK TEST")
print("I2C SDA GPIO{}  SCL GPIO{}  address 0x{:02X}".format(SDA_PIN, SCL_PIN, AS5600_ADDR))
i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)
devices = i2c.scan()
print("I2C devices:", [hex(device) for device in devices])
if AS5600_ADDR not in devices:
    raise RuntimeError("AS5600 not detected at 0x36")

zero_raw, span_deg = load_calibration()
print("AS5600 detected:", magnet_state(read_sample()[0]))
print_status()
print_help()

while True:
    try:
        command = input("\nBRAKE ENCODER > ").strip().lower()
        if command == "r":
            print_sample()
        elif command == "stream":
            print("Streaming. Press Ctrl-C to return to the command prompt.")
            try:
                while True:
                    print_sample()
                    time.sleep_ms(100)
            except KeyboardInterrupt:
                print("\nStream stopped.")
        elif command == "zero":
            capture_zero()
        elif command == "limit":
            capture_limit()
        elif command == "status":
            print_status()
        elif command == "save":
            save_calibration()
        elif command == "clear":
            clear_calibration()
        elif command == "help":
            print_help()
        elif command == "q":
            print("AS5600 brake feedback test stopped.")
            break
        elif command:
            print("Unknown command. Type help.")
    except KeyboardInterrupt:
        print("\nUse q to stop, or type a command.")
    except Exception as error:
        print("Read/command error:", error)

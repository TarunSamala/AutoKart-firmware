#!/usr/bin/env python3
"""Guided host test for esp32s3_steering_closed_loop_test.py."""

import argparse
import glob
import time

import serial


def find_port(requested):
    if requested != "auto":
        return requested
    candidates = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(glob.glob("/dev/ttyACM*"))
    if not candidates:
        raise SystemExit("No ESP32-S3 serial port found")
    return candidates[0]


def send(port, command):
    port.reset_input_buffer()
    port.write((command + "\n").encode("ascii"))
    port.flush()
    print("TX:", command)


def wait_for(port, token, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", "ignore").strip()
        if not line:
            continue
        if line.startswith("@STEER"):
            print("RX:", line)
        if token in line:
            return line
        if "@STEER,FAULT" in line or ",NAK," in line:
            raise RuntimeError(line)
    raise TimeoutError("Timed out waiting for {}".format(token))


def settled_angle(port, target, settle_seconds=1.0):
    time.sleep(settle_seconds)
    send(port, "STATUS")
    line = wait_for(port, "@STEER,STATUS", 2.0)
    fields = line.split(",")
    if len(fields) < 9:
        raise RuntimeError("Malformed status: " + line)
    angle = float(fields[3])
    error = target - angle
    print("SETTLED: target={:.2f} angle={:.2f} error={:+.2f}".format(target, angle, error))
    if abs(error) > 1.0:
        raise RuntimeError("Settled steering error exceeds 1.0 deg")
    return angle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    port_name = find_port(args.port)

    print("POWERED STEERING TEST - FRONT WHEELS MUST BE RAISED")
    print("Keep the physical driver power cut within reach.")
    answer = input("Type RAISED to continue: ").strip()
    if answer != "RAISED":
        raise SystemExit("Cancelled; no command sent")

    with serial.Serial(port_name, args.baud, timeout=0.15, exclusive=True) as port:
        time.sleep(1.0)
        port.reset_input_buffer()
        send(port, "DISARM")
        wait_for(port, "ACK,DISARM", 2.0)
        input("Place steering at the true mechanical centre, then press Enter...")
        send(port, "ZERO")
        wait_for(port, "ACK,ZERO", 2.0)
        send(port, "ARM")
        wait_for(port, "ACK,ARM", 2.0)

        try:
            for label, target in (("LEFT", -25), ("CENTRE", 0), ("RIGHT", 25), ("FINAL CENTRE", 0)):
                input("Ready for {} {} deg; press Enter...".format(label, target))
                send(port, "TARGET {}".format(target))
                wait_for(port, "TARGET_REACHED", 8.0)
                settled_angle(port, target)
        finally:
            send(port, "DISARM")
            try:
                wait_for(port, "ACK,DISARM", 2.0)
            except Exception:
                pass

    print("TEST COMPLETE: controller disarmed")


if __name__ == "__main__":
    main()

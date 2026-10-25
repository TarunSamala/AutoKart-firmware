#!/usr/bin/env python3
"""Interactive read-only steering telemetry test for the ESP32-S3 VCU."""

import argparse
import glob
import statistics
import time

import serial


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def find_port(requested):
    if requested != "auto":
        return requested
    candidates = (
        sorted(glob.glob("/dev/serial/by-id/*"))
        + sorted(glob.glob("/dev/ttyACM*"))
        + sorted(glob.glob("/dev/ttyUSB*"))
    )
    if not candidates:
        raise SystemExit("No serial device found. Connect the ESP32-S3 and retry.")
    return candidates[0]


def parse_control_feedback(raw_line):
    line = raw_line.decode("ascii", "ignore").strip()
    if "$" not in line:
        return None
    line = "$" + line.rsplit("$", 1)[1]
    if not line.startswith("$CONTROL_FEEDBACK,") or "*" not in line:
        return None

    body, supplied_crc = line[1:].rsplit("*", 1)
    try:
        if crc16_ccitt_false(body.encode("ascii")) != int(supplied_crc, 16):
            return None
    except ValueError:
        return None

    fields = body.split(",")
    if len(fields) < 22:
        return None
    try:
        validity = int(fields[20])
        if not validity & 0x02:
            return None
        return int(fields[11]) / 100.0, validity
    except (ValueError, IndexError):
        return None


def collect_angle(port, duration):
    samples = []
    validity_values = set()
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        parsed = parse_control_feedback(port.readline())
        if parsed is None:
            continue
        angle, validity = parsed
        samples.append(angle)
        validity_values.add(validity)

    if not samples:
        return None

    ordered = sorted(samples)
    trim = len(ordered) // 10 if len(ordered) >= 10 else 0
    filtered = ordered[trim:-trim] if trim else ordered
    return {
        "mean": round(statistics.fmean(filtered), 3),
        "minimum": round(min(filtered), 3),
        "maximum": round(max(filtered), 3),
        "validity": sorted(validity_values),
        "samples": len(samples),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto", help="serial port path or auto")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=1.5, help="measurement time per position")
    args = parser.parse_args()

    port_name = find_port(args.port)
    stages = (
        ("CENTRE_START", "Place steering at CENTRE"),
        ("LEFT", "Move steering fully LEFT and hold it"),
        ("CENTRE_AFTER_LEFT", "Return steering to CENTRE"),
        ("RIGHT", "Move steering fully RIGHT and hold it"),
        ("CENTRE_FINAL", "Return steering to CENTRE"),
    )

    print("READ-ONLY TEST: no actuator commands will be transmitted")
    print("Opening:", port_name)
    results = {}
    with serial.Serial(port_name, args.baud, timeout=0.2, exclusive=True) as port:
        port.reset_input_buffer()
        for name, instruction in stages:
            input("\n{}; then press Enter...".format(instruction))
            port.reset_input_buffer()
            print("Measuring for {:.1f} seconds; keep steering still".format(args.duration))
            result = collect_angle(port, args.duration)
            results[name] = result
            print(name, result)

    print("\n=== FINAL RESULTS ===")
    for name, _ in stages:
        print(name, results[name])
    print("\nExpected convention: LEFT negative, RIGHT positive, CENTRE near 0 deg.")
    return 0 if all(value is not None for value in results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())

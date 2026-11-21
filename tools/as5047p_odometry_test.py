#!/usr/bin/env python3
"""Read-only AS5047P one-turn and drivetrain conversion test."""

import argparse
import glob
import select
import sys
import time

import serial


COUNTS_PER_MOTOR_REV = 16384.0
DRIVE_TEETH = 14.0
AXLE_TEETH = 44.0
NOMINAL_TYRE_DIAMETER_M = 11.0 * 0.0254
NOMINAL_TYRE_CIRCUMFERENCE_M = 3.141592653589793 * NOMINAL_TYRE_DIAMETER_M
COUNT_TOLERANCE = 0.03
STATIONARY_COUNT_LIMIT = 24


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
    candidates = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(glob.glob("/dev/ttyACM*"))
    if not candidates:
        raise SystemExit("No ESP32-S3 serial port found")
    return candidates[0]


def parse_motion(raw_line):
    line = raw_line.decode("ascii", "ignore").strip()
    if "$" not in line:
        return None
    line = "$" + line.rsplit("$", 1)[1]
    if not line.startswith("$MOTION,") or "*" not in line:
        return None
    body, supplied_crc = line[1:].rsplit("*", 1)
    try:
        if crc16_ccitt_false(body.encode("ascii")) != int(supplied_crc, 16):
            return None
        fields = body.split(",")
        if len(fields) < 9:
            return None
        return {
            "sequence": int(fields[2]),
            "time_us": int(fields[3]),
            "count": int(fields[4]),
            "delta": int(fields[5]),
            "raw": int(fields[6]),
            "validity": int(fields[7]),
        }
    except (ValueError, IndexError):
        return None


def collect_for(port, seconds):
    frames = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        frame = parse_motion(port.readline())
        if frame is not None:
            frames.append(frame)
    return frames


def collect_until_enter(port):
    print("Rotate now; press Enter only after exactly one marked revolution.")
    frames = []
    while True:
        readable, _, _ = select.select([port, sys.stdin], [], [], 0.2)
        if port in readable:
            frame = parse_motion(port.readline())
            if frame is not None:
                frames.append(frame)
        if sys.stdin in readable:
            sys.stdin.readline()
            # Capture settling/closure samples after the operator stops.
            frames.extend(collect_for(port, 0.5))
            return frames


def summarize(name, frames):
    if len(frames) < 2:
        raise RuntimeError(name + ": insufficient valid MOTION frames")
    valid = [frame for frame in frames if frame["validity"] & 0x01]
    if len(valid) < 2:
        raise RuntimeError(name + ": AS5047P validity bit was not present")
    start = valid[0]
    end = valid[-1]
    count_change = end["count"] - start["count"]
    elapsed_s = max(1e-6, (end["time_us"] - start["time_us"]) / 1_000_000.0)
    motor_revs = count_change / COUNTS_PER_MOTOR_REV
    axle_revs = motor_revs * DRIVE_TEETH / AXLE_TEETH
    nominal_distance = axle_revs * NOMINAL_TYRE_CIRCUMFERENCE_M
    peak_rpm = 0.0
    for previous, current in zip(valid, valid[1:]):
        dt = (current["time_us"] - previous["time_us"]) / 1_000_000.0
        if dt > 0:
            rpm = current["delta"] / COUNTS_PER_MOTOR_REV / dt * 60.0
            peak_rpm = rpm if abs(rpm) > abs(peak_rpm) else peak_rpm
    result = {
        "name": name,
        "frames": len(valid),
        "count_change": count_change,
        "motor_revs": motor_revs,
        "axle_revs": axle_revs,
        "nominal_distance_m": nominal_distance,
        "peak_motor_rpm": peak_rpm,
        "start_raw": start["raw"],
        "end_raw": end["raw"],
        "elapsed_s": elapsed_s,
    }
    print(
        "{}: frames={} counts={:+d} motor_rev={:+.5f} axle_rev={:+.5f} "
        "nominal_distance={:+.4f} m peak_rpm={:+.1f} raw={} -> {}".format(
            name, result["frames"], result["count_change"], result["motor_revs"],
            result["axle_revs"], result["nominal_distance_m"],
            result["peak_motor_rpm"], result["start_raw"], result["end_raw"]
        )
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    port_name = find_port(args.port)

    print("READ-ONLY ODOMETRY TEST: no actuator commands are transmitted")
    print("Traction power must remain isolated. Turn only the marked 14T sprocket by hand.")
    print("Nominal conversion uses 14T:44T and an uncalibrated 11-inch tyre diameter.")
    if input("Type ISOLATED to continue: ").strip() != "ISOLATED":
        raise SystemExit("Cancelled")

    with serial.Serial(port_name, args.baud, timeout=0.1, exclusive=True) as port:
        time.sleep(0.5)
        port.reset_input_buffer()
        print("Keep the sprocket stationary for 2 seconds...")
        stationary_before = summarize("STATIONARY_BEFORE", collect_for(port, 2.0))

        input("Mark the 14T sprocket. Press Enter to begin one physical FORWARD revolution (CCW viewed from the encoder/magnet side)...")
        port.reset_input_buffer()
        forward = summarize("FORWARD_ONE_TURN", collect_until_enter(port))

        input("Press Enter to begin one physical REVERSE revolution back to the original mark...")
        port.reset_input_buffer()
        reverse = summarize("REVERSE_ONE_TURN", collect_until_enter(port))

        print("Keep the sprocket stationary for 2 seconds...")
        stationary_after = summarize("STATIONARY_AFTER", collect_for(port, 2.0))

    failures = []
    for stationary in (stationary_before, stationary_after):
        if abs(stationary["count_change"]) > STATIONARY_COUNT_LIMIT:
            failures.append(stationary["name"] + " count creep exceeded 24 counts")
    for movement in (forward, reverse):
        error = abs(abs(movement["count_change"]) - COUNTS_PER_MOTOR_REV) / COUNTS_PER_MOTOR_REV
        if error > COUNT_TOLERANCE:
            failures.append(movement["name"] + " was not within 3% of one encoder revolution")
    if forward["count_change"] * reverse["count_change"] >= 0:
        failures.append("forward and reverse count signs were not opposite")
    if forward["count_change"] <= 0:
        failures.append("physical forward was not published as positive odometry")
    if reverse["count_change"] >= 0:
        failures.append("physical reverse was not published as negative odometry")
    net_counts = forward["count_change"] + reverse["count_change"]
    if abs(net_counts) > COUNTS_PER_MOTOR_REV * COUNT_TOLERANCE:
        failures.append("forward plus reverse did not return within 3% of the starting count")

    print("\nNET COUNTS AFTER FORWARD + REVERSE:", net_counts)
    if failures:
        print("RESULT: FAIL_DIAGNOSTIC")
        for failure in failures:
            print(" -", failure)
        return 1
    print("RESULT: PASS_DIAGNOSTIC")
    print("Direction convention confirmed: forward positive, reverse negative.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

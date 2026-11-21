#!/usr/bin/env python3
"""Read-only ESP32-S3 VCU serial telemetry acceptance test."""

import argparse
import glob
import json
from pathlib import Path
import time

import serial


EXPECTED_MINIMUM_HZ = {
    "MOTION": 40.0,
    "CONTROL_FEEDBACK": 16.0,
    "HELLO": 0.5,
}


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
    candidates = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    if not candidates:
        raise SystemExit("No serial port found")
    return candidates[0]


def parse_frame(raw_line):
    line = raw_line.decode("ascii", "ignore").strip()
    if "$" not in line or "*" not in line:
        return None, "NON_FRAME"
    line = "$" + line.rsplit("$", 1)[1]
    try:
        body, supplied = line[1:].rsplit("*", 1)
        expected = crc16_ccitt_false(body.encode("ascii"))
        if expected != int(supplied, 16):
            return None, "CRC"
    except (ValueError, UnicodeError):
        return None, "MALFORMED"
    fields = body.split(",")
    if not fields:
        return None, "MALFORMED"
    return fields, None


def rate_hz(arrivals):
    if len(arrivals) < 2:
        return 0.0
    elapsed = arrivals[-1] - arrivals[0]
    return (len(arrivals) - 1) / elapsed if elapsed > 0 else 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", default="log/vcu_serial_validation_latest.json")
    args = parser.parse_args()
    port_name = find_port(args.port)

    print("READ-ONLY VCU SERIAL VALIDATION: no commands are transmitted")
    print("Port:", port_name)
    print("The program will announce each timed action:")
    print("  0-8 s: steering left, centre, right, centre")
    print("  8-18 s: raised rear wheel/14T sprocket forward, then reverse")
    print("  18-20 s: return steering to centre and stop drivetrain")
    print("  20-30 s: touch nothing; mandatory stationary window")
    if input("Type READY to begin: ").strip() != "READY":
        raise SystemExit("Cancelled")

    arrivals = {name: [] for name in EXPECTED_MINIMUM_HZ}
    type_counts = {}
    boot_ids = set()
    sequences = []
    timestamps = []
    validity_values = set()
    fault_values = set()
    steering = []
    encoder_counts = []
    encoder_raw = []
    calibrations = set()
    crc_errors = 0
    malformed_frames = 0
    non_frames = 0
    final_stationary_counts = []
    final_stationary_steering = []

    with serial.Serial(port_name, args.baud, timeout=0.1, exclusive=True) as port:
        time.sleep(0.5)
        port.reset_input_buffer()
        started = time.monotonic()
        deadline = started + args.duration
        next_report = started + 5.0
        announced_phase = -1
        while time.monotonic() < deadline:
            now = time.monotonic()
            elapsed = now - started
            if elapsed < 8.0:
                phase = 0
            elif elapsed < 18.0:
                phase = 1
            elif elapsed < 20.0:
                phase = 2
            else:
                phase = 3
            if phase != announced_phase:
                announced_phase = phase
                messages = (
                    "NOW: move steering LEFT -> CENTRE -> RIGHT -> CENTRE",
                    "NOW: rotate raised drivetrain FORWARD, then REVERSE",
                    "NOW: return steering to CENTRE and STOP drivetrain",
                    "STOP ALL MOVEMENT NOW - remain stationary until completion",
                )
                print("\n" + messages[phase])
            fields, error = parse_frame(port.readline())
            if error == "CRC":
                crc_errors += 1
                continue
            if error == "MALFORMED":
                malformed_frames += 1
                continue
            if error == "NON_FRAME":
                non_frames += 1
                continue

            frame_type = fields[0]
            type_counts[frame_type] = type_counts.get(frame_type, 0) + 1
            if frame_type in arrivals:
                arrivals[frame_type].append(now)
            if len(fields) > 1:
                boot_ids.add(fields[1])

            try:
                if frame_type == "MOTION" and len(fields) >= 9:
                    sequences.append(int(fields[2]))
                    timestamps.append(int(fields[3]))
                    count = int(fields[4])
                    encoder_counts.append(count)
                    encoder_raw.append(int(fields[6]))
                    validity_values.add(int(fields[7]))
                    calibrations.add(fields[8])
                    if now >= deadline - 5.0:
                        final_stationary_counts.append(count)
                elif frame_type == "CONTROL_FEEDBACK" and len(fields) >= 22:
                    sequences.append(int(fields[2]))
                    timestamps.append(int(fields[3]))
                    steer = int(fields[11]) / 100.0
                    steering.append(steer)
                    validity_values.add(int(fields[20]))
                    fault_values.add(int(fields[21]))
                    if now >= deadline - 5.0:
                        final_stationary_steering.append(steer)
                elif frame_type == "HELLO" and len(fields) >= 6:
                    calibrations.add(fields[5])
            except (ValueError, IndexError):
                malformed_frames += 1

            if now >= next_report:
                remaining = max(0, int(round(deadline - now)))
                print("Capture active: {} seconds remaining".format(remaining))
                next_report += 5.0

    sequence_gaps = 0
    sequence_duplicates = 0
    sequence_reversals = 0
    for previous, current in zip(sequences, sequences[1:]):
        difference = current - previous
        if difference == 0:
            sequence_duplicates += 1
        elif difference < 0:
            sequence_reversals += 1
        elif difference > 1:
            sequence_gaps += difference - 1
    timestamp_reversals = sum(current < previous for previous, current in zip(timestamps, timestamps[1:]))

    rates = {name: round(rate_hz(values), 3) for name, values in arrivals.items()}
    steering_range = (min(steering), max(steering)) if steering else None
    count_range = (min(encoder_counts), max(encoder_counts)) if encoder_counts else None
    final_count_span = (
        max(final_stationary_counts) - min(final_stationary_counts)
        if final_stationary_counts else None
    )
    final_steering_span = (
        max(final_stationary_steering) - min(final_stationary_steering)
        if final_stationary_steering else None
    )

    failures = []
    for name, minimum in EXPECTED_MINIMUM_HZ.items():
        if rates[name] < minimum:
            failures.append("{} rate {:.2f} Hz below {:.2f} Hz".format(name, rates[name], minimum))
    if crc_errors:
        failures.append("{} CRC errors".format(crc_errors))
    if malformed_frames:
        failures.append("{} malformed telemetry frames".format(malformed_frames))
    if sequence_gaps or sequence_duplicates or sequence_reversals:
        failures.append("sequence continuity failure")
    if timestamp_reversals:
        failures.append("non-monotonic VCU timestamps")
    if len(boot_ids) != 1:
        failures.append("expected one stable boot_id; observed {}".format(len(boot_ids)))
    if not validity_values or any((value & 0x03) != 0x03 for value in validity_values):
        failures.append("encoder/steering validity was not continuously 3")
    if fault_values and any(value != 0 for value in fault_values):
        failures.append("non-zero fault bits observed: {}".format(sorted(fault_values)))
    if not steering_range or steering_range[1] - steering_range[0] < 5.0:
        failures.append("less than 5 degrees of steering movement observed")
    if not count_range or count_range[1] - count_range[0] < 200:
        failures.append("less than 200 odometry counts of movement observed")
    if final_count_span is None or final_count_span > 24:
        failures.append("final stationary odometry span exceeded 24 counts")
    if final_steering_span is None or final_steering_span > 0.5:
        failures.append("final stationary steering span exceeded 0.5 degrees")

    result = {
        "port": port_name,
        "duration_s": args.duration,
        "type_counts": type_counts,
        "rates_hz": rates,
        "crc_errors": crc_errors,
        "malformed_frames": malformed_frames,
        "non_frame_reads": non_frames,
        "sequence_gaps": sequence_gaps,
        "sequence_duplicates": sequence_duplicates,
        "sequence_reversals": sequence_reversals,
        "timestamp_reversals": timestamp_reversals,
        "boot_ids": sorted(boot_ids),
        "validity_values": sorted(validity_values),
        "fault_values": sorted(fault_values),
        "calibration_ids": sorted(calibrations),
        "steering_range_deg": steering_range,
        "encoder_count_range": count_range,
        "final_stationary_count_span": final_count_span,
        "final_stationary_steering_span_deg": final_steering_span,
        "failures": failures,
        "result": "PASS_DIAGNOSTIC" if not failures else "FAIL_DIAGNOSTIC",
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(result, indent=2))
    print("Saved:", output.resolve())
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())

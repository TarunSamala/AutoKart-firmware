#!/usr/bin/env python3
"""Read-only validation that an ESP32-S3 reset creates a new VCU epoch."""

import argparse
import glob
import json
from pathlib import Path
import time

import serial


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def ports():
    return (
        sorted(glob.glob("/dev/serial/by-id/*Espressif*"))
        + sorted(glob.glob("/dev/serial/by-id/*"))
        + sorted(glob.glob("/dev/ttyACM*"))
        + sorted(glob.glob("/dev/ttyUSB*"))
    )


def resolve_port(requested, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if requested != "auto":
            if Path(requested).exists():
                return requested
        else:
            found = ports()
            if found:
                return found[0]
        time.sleep(0.1)
    raise RuntimeError("VCU serial port did not appear within {:.1f} seconds".format(timeout))


def parse(raw_line):
    line = raw_line.decode("ascii", "ignore").strip()
    if "$" not in line or "*" not in line:
        return None
    line = "$" + line.rsplit("$", 1)[1]
    try:
        body, supplied = line[1:].rsplit("*", 1)
        if crc16_ccitt_false(body.encode("ascii")) != int(supplied, 16):
            return None
        return body.split(",")
    except (ValueError, UnicodeError):
        return None


def capture_epoch(requested_port, timeout=8.0, discard_backlog=False):
    identity = None
    first_motion = None
    first_control = None
    deadline = time.monotonic() + timeout
    port = None
    port_name = None
    while time.monotonic() < deadline and port is None:
        try:
            port_name = resolve_port(requested_port, timeout=1.0)
            port = serial.Serial(
                port_name, 115200, timeout=0.1, exclusive=True
            )
        except (OSError, serial.SerialException):
            port = None
            time.sleep(0.2)
    if port is None:
        raise RuntimeError("VCU serial port did not become accessible")
    with port:
        if discard_backlog:
            # A USB-UART bridge may retain complete pre-reset frames while the
            # host port is closed. They do not belong to the new VCU epoch.
            time.sleep(0.25)
            port.reset_input_buffer()
        while time.monotonic() < deadline:
            fields = parse(port.readline())
            if not fields:
                continue
            if fields[0] == "HELLO" and len(fields) >= 6:
                identity = {
                    "boot_id": int(fields[1]),
                    "firmware_id": fields[2],
                    "protocol_version": int(fields[3]),
                    "capability_bits": int(fields[4]),
                    "calibration_id": fields[5],
                }
            elif fields[0] == "MOTION" and len(fields) >= 9 and first_motion is None:
                first_motion = {
                    "boot_id": int(fields[1]),
                    "sequence": int(fields[2]),
                    "sample_time_us": int(fields[3]),
                    "encoder_count": int(fields[4]),
                    "delta_count": int(fields[5]),
                    "raw": int(fields[6]),
                    "validity_bits": int(fields[7]),
                    "calibration_id": fields[8],
                }
            elif fields[0] == "CONTROL_FEEDBACK" and len(fields) >= 22 and first_control is None:
                first_control = {
                    "boot_id": int(fields[1]),
                    "sequence": int(fields[2]),
                    "sample_time_us": int(fields[3]),
                    "steering_deg": int(fields[11]) / 100.0,
                    "validity_bits": int(fields[20]),
                    "fault_bits": int(fields[21]),
                }
            if identity and first_motion and first_control:
                if first_motion["validity_bits"] == 3 and first_control["validity_bits"] == 3:
                    return {
                        "port": port_name,
                        "identity": identity,
                        "motion": first_motion,
                        "control": first_control,
                    }
    raise RuntimeError("Did not receive a complete valid HELLO/MOTION/CONTROL epoch")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--output", default="log/vcu_reboot_epoch_latest.json")
    args = parser.parse_args()

    print("READ-ONLY RESET TEST: no actuator commands are transmitted")
    print("Keep traction isolated and steering mechanically centred.")
    before = capture_epoch(args.port)
    print("BEFORE boot_id={} sequence={} time_us={} count={}".format(
        before["identity"]["boot_id"], before["motion"]["sequence"],
        before["motion"]["sample_time_us"], before["motion"]["encoder_count"]
    ))

    input("Press the ESP32 physical RST button once, wait two seconds, then press Enter here...")
    after = capture_epoch("auto", discard_backlog=True)
    print("AFTER  boot_id={} sequence={} time_us={} count={}".format(
        after["identity"]["boot_id"], after["motion"]["sequence"],
        after["motion"]["sample_time_us"], after["motion"]["encoder_count"]
    ))

    failures = []
    if before["identity"]["boot_id"] == after["identity"]["boot_id"]:
        failures.append("boot_id did not change across physical reset")
    if after["motion"]["boot_id"] != after["identity"]["boot_id"]:
        failures.append("post-reset MOTION boot_id did not match HELLO")
    if after["control"]["boot_id"] != after["identity"]["boot_id"]:
        failures.append("post-reset CONTROL_FEEDBACK boot_id did not match HELLO")
    if after["motion"]["calibration_id"] != after["identity"]["calibration_id"]:
        failures.append("post-reset calibration identity mismatch")
    if abs(after["motion"]["encoder_count"]) > 24:
        failures.append("post-reset odometry did not restart near zero")
    if after["motion"]["sample_time_us"] > 10_000_000:
        failures.append("post-reset monotonic timestamp did not restart near zero")
    if after["motion"]["sequence"] > 1000:
        failures.append("post-reset sequence did not restart near zero")
    if after["motion"]["validity_bits"] != 3 or after["control"]["validity_bits"] != 3:
        failures.append("post-reset sensor validity did not recover to 3")
    if after["control"]["fault_bits"] != 0:
        failures.append("post-reset fault bits were non-zero")

    result = {
        "before": before,
        "after": after,
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

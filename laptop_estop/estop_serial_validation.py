#!/usr/bin/env python3
"""Guided laptop USB-serial validation of the LoRa-to-BLD750 E-stop path."""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path
import sys
import time

import serial


def find_port(requested: str) -> str:
    if requested != "auto":
        return requested

    stable = sorted(glob.glob("/dev/serial/by-id/*Espressif*"))
    if len(stable) == 1:
        return stable[0]
    if len(stable) > 1:
        raise SystemExit(
            "Multiple Espressif ports found; pass --port explicitly:\n  "
            + "\n  ".join(stable)
        )

    fallback = sorted(glob.glob("/dev/ttyACM*")) + sorted(
        glob.glob("/dev/ttyUSB*")
    )
    if len(fallback) == 1:
        return fallback[0]
    if not fallback:
        raise SystemExit("No ESP32 serial port found")
    raise SystemExit(
        "No unique Espressif by-id port; pass --port explicitly:\n  "
        + "\n  ".join(fallback)
    )


class Controller:
    def __init__(self, port: serial.Serial):
        self.port = port
        self.request_number = 0
        self.recent_lines: list[str] = []

    def request(self, command: str, argument: str = "", timeout: float = 2.0) -> str:
        self.request_number += 1
        request_id = "laptop-{}".format(self.request_number)
        frame = "@AKCMD {} {}".format(request_id, command)
        if argument:
            frame += " " + argument
        self.port.write((frame + "\n").encode("ascii"))
        self.port.flush()

        prefix = "@AKR {} ".format(request_id)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.port.readline().decode("utf-8", "ignore").strip()
            if not line:
                continue
            if not line.startswith(prefix):
                self.recent_lines.append(line)
                self.recent_lines = self.recent_lines[-12:]
                if (
                    "LORA" in line
                    or "WHEEL LOCKED" in line
                    or "Traceback" in line
                    or "Error" in line
                    or ">>>" in line
                    or "OFFLINE" in line
                ):
                    print("ESP32:", line)
                continue
            remainder = line[len(prefix):]
            outcome, _, payload = remainder.partition(" ")
            if outcome != "OK":
                raise RuntimeError(
                    "ESP32 rejected {}: {}".format(command, payload or outcome)
                )
            return payload
        detail = " | ".join(self.recent_lines[-8:]) or "NO SERIAL TEXT"
        raise TimeoutError(
            "ESP32 serial response timeout for {} | RX: {}".format(
                command, detail
            )
        )

    def status(self) -> dict:
        return json.loads(self.request("STATUS"))

    def lock(self) -> None:
        self.request("LOCK")


def concise(status: dict) -> dict:
    fields = (
        "armed",
        "running",
        "brake_released",
        "wheel_locked",
        "voltage",
        "target_voltage",
        "gp_online",
        "lora_ready",
        "lora_interlock_active",
        "remote_estop",
        "remote_clear_seen",
        "lora_last_packet",
        "lora_age_ms",
        "lora_link_timeout",
        "lora_clear_confirmations",
        "lora_clear_required",
        "lora_radio_errors",
        "mechanical_brake_enabled",
        "mechanical_brake_ready",
        "mechanical_brake_error",
        "mechanical_brake_state",
        "mechanical_brake_steps",
        "mechanical_brake_degrees",
        "mechanical_brake_apply_degrees",
    )
    return {field: status.get(field) for field in fields}


def clear_ready(status: dict) -> bool:
    return bool(
        status.get("gp_online")
        and status.get("lora_ready")
        and status.get("mechanical_brake_ready")
        and not status.get("lora_link_timeout")
        and status.get("remote_clear_seen")
        and not status.get("remote_estop")
        and not status.get("lora_interlock_active")
        and int(status.get("lora_clear_confirmations", 0))
        >= int(status.get("lora_clear_required", 1))
        and str(status.get("lora_last_packet", "")).startswith("ESTOP,0,")
    )


def traction_commanded(status: dict, expected_voltage: float) -> bool:
    return bool(
        status.get("armed")
        and status.get("running")
        and status.get("brake_released")
        and not status.get("wheel_locked")
        and not status.get("remote_estop")
        and abs(float(status.get("target_voltage", -1.0)) - expected_voltage)
        <= 0.01
        and abs(float(status.get("voltage", -1.0)) - expected_voltage) <= 0.02
    )


def stop_packet_received(status: dict) -> bool:
    return bool(
        status.get("remote_estop")
        and status.get("lora_interlock_active")
        and not status.get("lora_link_timeout")
        and str(status.get("lora_last_packet", "")).startswith("ESTOP,1,")
    )


def fully_locked(status: dict) -> bool:
    return bool(
        status.get("wheel_locked")
        and not status.get("armed")
        and not status.get("running")
        and not status.get("brake_released")
        and abs(float(status.get("voltage", 999.0))) <= 0.01
        and abs(float(status.get("target_voltage", 999.0))) <= 0.01
        and status.get("mechanical_brake_enabled")
        and status.get("mechanical_brake_ready")
        and status.get("mechanical_brake_state") == "APPLIED"
    )


def wait_for(
    controller: Controller,
    label: str,
    predicate,
    timeout: float,
    interval: float = 0.10,
) -> tuple[dict, float]:
    deadline = time.monotonic() + timeout
    last_print = 0.0
    last_status = None
    while time.monotonic() < deadline:
        status = controller.status()
        last_status = status
        observed = time.monotonic()
        if predicate(status):
            return status, observed
        if observed - last_print >= 2.0:
            print(label + ":", json.dumps(concise(status), sort_keys=True))
            last_print = observed
        time.sleep(interval)
    raise RuntimeError(
        "Timed out waiting for {}. Last status: {}".format(
            label, json.dumps(concise(last_status or {}), sort_keys=True)
        )
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument(
        "--test-voltage",
        type=float,
        choices=(0.4, 1.0),
        default=0.4,
        help="BLD750 SV command for the wheels-off test (default: 0.4 V)",
    )
    parser.add_argument("--report", default="estop-laptop-validation-latest.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port_name = find_port(args.port)
    report = {
        "port": port_name,
        "result": "INCOMPLETE",
        "test_voltage": args.test_voltage,
        "events": [],
    }

    print("AUTOKART LAPTOP USB-SERIAL LORA E-STOP VALIDATION")
    print("ESP32 port:", port_name)
    print("This test commands the BLDC forward at {:.2f} V.".format(
        args.test_voltage
    ))
    print("Rear wheels must be raised, restrained and guarded.")
    print("Keep the independent physical power cut within reach.")
    accepted_confirmation = "WHEELS-OFF-{:.1f}V".format(args.test_voltage)
    confirmation = input(
        "Type {} to continue: ".format(accepted_confirmation)
    ).strip().upper()
    if confirmation != accepted_confirmation:
        print("Cancelled; no motor command was transmitted.")
        return 2

    controller = None
    port = None
    result_code = 1
    try:
        port = serial.Serial(
            port_name,
            args.baud,
            timeout=0.10,
            write_timeout=1.0,
            exclusive=True,
        )
        if port is not None:
            # Opening native USB can coincide with a board reset. Allow the
            # fail-closed firmware to initialise LoRa and its optional Wi-Fi
            # fallback before sending the first framed command.
            time.sleep(3.0)
            port.reset_input_buffer()
            controller = Controller(port)

            print("\nRelease the LAY37 switch and power the Pico transmitter.")
            clear_before, _ = wait_for(
                controller, "confirmed CLEAR heartbeat", clear_ready, args.timeout
            )
            report["events"].append(
                {"event": "CLEAR_BEFORE", "status": concise(clear_before)}
            )
            print("CLEAR VERIFIED; manual start is now permitted.")

            controller.request("TEST_START")
            if args.test_voltage == 1.0:
                controller.request("GEAR", "1")
            running, _ = wait_for(
                controller,
                "BLDC traction outputs commanded at {:.2f} V".format(
                    args.test_voltage
                ),
                lambda status: traction_commanded(status, args.test_voltage),
                10.0,
            )
            report["events"].append(
                {"event": "TRACTION_COMMANDED", "status": concise(running)}
            )
            print(
                "BLDC OUTPUTS COMMANDED AT {:.2f} V; THIS IS NOT YET "
                "PHYSICAL MOTION PROOF".format(args.test_voltage)
            )
            motion = input(
                "Visually confirm the raised drive wheel is rotating. "
                "Type MOTOR-SPINNING: "
            ).strip().upper()
            if motion != "MOTOR-SPINNING":
                raise RuntimeError(
                    "BLDC physical rotation was not confirmed at {:.2f} V"
                    .format(args.test_voltage)
                )
            report["events"].append(
                {
                    "event": "PHYSICAL_MOTION_CONFIRMED",
                    "test_voltage": args.test_voltage,
                }
            )
            print("PHYSICAL BLDC ROTATION CONFIRMED")

            print("\nPRESS AND HOLD THE LAY37 E-STOP NOW.")
            stop_status, stop_observed = wait_for(
                controller, "ESTOP,1 packet", stop_packet_received, args.timeout
            )
            report["events"].append(
                {"event": "STOP_PACKET", "status": concise(stop_status)}
            )

            locked, lock_observed = wait_for(
                controller, "full BLD750 lock", fully_locked, 3.0, interval=0.05
            )
            report["events"].append(
                {"event": "FULL_LOCK", "status": concise(locked)}
            )
            report["host_observed_packet_to_lock_ms"] = round(
                (lock_observed - stop_observed) * 1000.0, 1
            )
            print(
                "E-STOP VERIFIED: BLD BRK APPLIED, EN STOP, DAC 0.00 V, "
                "MECHANICAL BRAKE APPLIED"
            )

            input("\nRelease the LAY37 E-stop, then press Enter... ")
            clear_after, _ = wait_for(
                controller, "confirmed CLEAR after stop", clear_ready, args.timeout
            )
            report["events"].append(
                {"event": "CLEAR_AFTER", "status": concise(clear_after)}
            )

            stationary_deadline = time.monotonic() + 2.0
            while time.monotonic() < stationary_deadline:
                final = controller.status()
                if not fully_locked(final):
                    raise RuntimeError("BLDC restarted after CLEAR")
                time.sleep(0.10)
            report["events"].append(
                {"event": "FINAL_LOCKED", "status": concise(final)}
            )

            report["result"] = "PASS_DIAGNOSTIC"
            result_code = 0
            print("\nRESULT: PASS_DIAGNOSTIC")
            print("STOP locked the BLDC and CLEAR did not restart it.")
    except (OSError, ValueError, RuntimeError, TimeoutError, serial.SerialException) as error:
        report["result"] = "FAIL_DIAGNOSTIC"
        report["error"] = str(error)
        print("\nRESULT: FAIL_DIAGNOSTIC")
        print(error)
    finally:
        if controller is not None:
            try:
                controller.lock()
                print("FINAL COMMAND: BLDC wheel lock asserted")
            except Exception as error:
                print("WARNING: final serial lock acknowledgement unavailable:", error)
                print("USE THE PHYSICAL POWER CUT NOW")
        if port is not None:
            try:
                port.close()
            except Exception:
                pass

        report_path = Path(args.report).expanduser()
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print("Saved:", report_path.resolve())

    return result_code


if __name__ == "__main__":
    sys.exit(main())

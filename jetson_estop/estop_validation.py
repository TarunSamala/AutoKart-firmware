#!/usr/bin/env python3
"""Read-only guided validation of the Jetson dashboard LoRa E-stop path."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from urllib.error import URLError
from urllib.request import Request, urlopen


SAFE_FIELDS = (
    "lora_ready",
    "lora_required",
    "lora_interlock_active",
    "remote_estop",
    "remote_clear_seen",
    "lora_last_packet",
    "lora_age_ms",
    "lora_link_timeout",
    "lora_timeout_ms",
    "lora_clear_confirmations",
    "lora_clear_required",
    "lora_radio_errors",
    "armed",
    "running",
    "wheel_locked",
    "brake_released",
    "voltage",
    "target_voltage",
    "mechanical_brake_enabled",
    "mechanical_brake_state",
    "mechanical_brake_steps",
    "mechanical_brake_degrees",
    "mechanical_brake_apply_degrees",
)


def fetch_status(base_url: str) -> dict:
    request = Request(base_url.rstrip("/") + "/api/status")
    with urlopen(request, timeout=2.0) as response:
        return json.load(response)


def controller(status: dict) -> dict:
    value = status.get("controller")
    return value if isinstance(value, dict) else {}


def snapshot(status: dict) -> dict:
    ctrl = controller(status)
    result = {name: ctrl.get(name) for name in SAFE_FIELDS}
    result["controller_online"] = status.get("controller_online")
    result["camera_ok"] = status.get("camera_ok")
    return result


def is_clear(status: dict) -> bool:
    ctrl = controller(status)
    return bool(
        status.get("controller_online")
        and ctrl.get("lora_ready")
        and ctrl.get("remote_clear_seen")
        and not ctrl.get("remote_estop")
        and not ctrl.get("lora_interlock_active")
        and not ctrl.get("lora_link_timeout")
        and int(ctrl.get("lora_clear_confirmations", 0))
        >= int(ctrl.get("lora_clear_required", 1))
        and isinstance(ctrl.get("lora_last_packet"), str)
        and ctrl["lora_last_packet"].startswith("ESTOP,0,")
    )


def is_running(status: dict) -> bool:
    ctrl = controller(status)
    return bool(
        ctrl.get("armed")
        and ctrl.get("running")
        and not ctrl.get("wheel_locked")
        and ctrl.get("brake_released")
        and float(ctrl.get("target_voltage", 0.0)) > 0.0
    )


def is_interlocked(status: dict) -> bool:
    ctrl = controller(status)
    return bool(ctrl.get("lora_interlock_active") and ctrl.get("remote_estop"))


def is_fully_locked(status: dict) -> bool:
    ctrl = controller(status)
    return bool(
        ctrl.get("wheel_locked")
        and not ctrl.get("armed")
        and not ctrl.get("running")
        and not ctrl.get("brake_released")
        and abs(float(ctrl.get("voltage", 999.0))) <= 0.01
        and abs(float(ctrl.get("target_voltage", 999.0))) <= 0.01
        and ctrl.get("mechanical_brake_enabled")
        and ctrl.get("mechanical_brake_state") == "APPLIED"
    )


def wait_for(base_url: str, label: str, predicate, timeout: float) -> tuple[dict, float]:
    deadline = time.monotonic() + timeout
    last_print = 0.0
    last_error = None
    while time.monotonic() < deadline:
        try:
            status = fetch_status(base_url)
            if predicate(status):
                return status, time.monotonic()
            if time.monotonic() - last_print >= 2.0:
                print(label + ":", json.dumps(snapshot(status), sort_keys=True))
                last_print = time.monotonic()
        except (OSError, URLError, ValueError, json.JSONDecodeError) as error:
            last_error = error
            if time.monotonic() - last_print >= 2.0:
                print(label + ": dashboard unavailable:", error)
                last_print = time.monotonic()
        time.sleep(0.1)
    detail = "" if last_error is None else ": {}".format(last_error)
    raise RuntimeError("Timed out waiting for {}{}".format(label, detail))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8781")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument(
        "--report",
        default="estop-validation-latest.json",
        help="JSON report path",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = {
        "dashboard_url": args.url,
        "result": "INCOMPLETE",
        "events": [],
    }

    print("READ-ONLY JETSON LORA E-STOP VALIDATION")
    print("This tool does not transmit START, ARM, STOP, or brake commands.")
    print("Rear wheels must be raised/restrained; keep physical power cut ready.")
    if input("Type WHEELS-OFF to continue: ").strip() != "WHEELS-OFF":
        print("Cancelled; no commands were transmitted.")
        return 2

    try:
        print("\nRelease the LAY37 E-stop and power the Pico transmitter.")
        clear, _ = wait_for(args.url, "CLEAR heartbeat", is_clear, args.timeout)
        report["events"].append({"event": "CLEAR_BEFORE", "status": snapshot(clear)})
        print("CLEAR HEARTBEAT VERIFIED")

        print("\nUse the dashboard to click START at 0.40 V.")
        running, _ = wait_for(args.url, "motor running", is_running, args.timeout)
        report["events"].append({"event": "RUNNING", "status": snapshot(running)})
        print("RUNNING STATE OBSERVED")

        print("\nPRESS AND HOLD THE LAY37 E-STOP NOW.")
        stopped, detected_at = wait_for(
            args.url, "LoRa E-stop interlock", is_interlocked, args.timeout
        )
        report["events"].append({"event": "INTERLOCK", "status": snapshot(stopped)})

        locked, locked_at = wait_for(args.url, "full wheel lock", is_fully_locked, 5.0)
        report["events"].append({"event": "LOCKED", "status": snapshot(locked)})
        report["host_observed_interlock_to_lock_ms"] = round(
            (locked_at - detected_at) * 1000.0, 1
        )
        print("E-STOP LOCK VERIFIED")

        print("\nRelease the LAY37 E-stop. The motor must remain locked.")
        clear_after, _ = wait_for(args.url, "CLEAR after stop", is_clear, args.timeout)
        report["events"].append(
            {"event": "CLEAR_AFTER", "status": snapshot(clear_after)}
        )
        time.sleep(1.0)
        final = fetch_status(args.url)
        report["events"].append({"event": "FINAL", "status": snapshot(final)})
        if not is_fully_locked(final):
            raise RuntimeError("Motor restarted or did not remain fully locked after CLEAR")

        report["result"] = "PASS_DIAGNOSTIC"
        print("\nRESULT: PASS_DIAGNOSTIC")
        print("STOP locked the BLDC; CLEAR did not restart it.")
        return_code = 0
    except (RuntimeError, OSError, URLError, ValueError, json.JSONDecodeError) as error:
        report["result"] = "FAIL_DIAGNOSTIC"
        report["error"] = str(error)
        print("\nRESULT: FAIL_DIAGNOSTIC")
        print(error)
        return_code = 1

    report_path = Path(args.report).expanduser()
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("Saved:", report_path.resolve())
    return return_code


if __name__ == "__main__":
    sys.exit(main())

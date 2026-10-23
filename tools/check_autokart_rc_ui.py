"""Static and behavioral contract checks for the full AutoKart RC firmware."""

import ast
import json
import re
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "bench/autokart_rc_simple_encoders.py"
source = FIRMWARE.read_text(encoding="utf-8")
compile(source, str(FIRMWARE), "exec")
tree = ast.parse(source)
html = next(
    ast.literal_eval(node.value)
    for node in tree.body
    if isinstance(node, ast.Assign)
    and any(isinstance(target, ast.Name) and target.id == "HTML" for target in node.targets)
)

ids = re.findall(r'id="([^"]+)"', html)
assert len(ids) == len(set(ids)), "Duplicate HTML IDs found"

required_ids = {
    "armed", "armBtn", "disarmBtn", "forward", "reverse",
    "leftBtn", "centerBtn", "rightBtn", "brakeBtn", "offBtn",
    "emergencyBtn", "selectedGear", "driveState", "voltage",
    "targetVoltage", "steerNow", "steerTarget", "steerPosition",
    "steerDirection", "zeroEncodersBtn", "encoderZeroState",
    "brakeState", "brakeAngle", "brakeTarget", "brakeMagnet",
    "brakeValidation", "brakeError", "brakeTime", "steerDetail",
    "steerErrors", "bldcEnc", "bldcDetail", "bldcRpm", "axleRpm",
    "fullTurns", "motorRevs", "distance", "displacement", "speed",
    "gpState", "message",
}
assert not required_ids - set(ids), "Missing UI IDs: {}".format(
    sorted(required_ids - set(ids))
)
assert len(re.findall(r"<button\b", html)) == 16, "Expected 16 controls"

for route, handler in {
    "/status": 'path.startswith("/status")',
    "/arm": 'path.startswith("/arm")',
    "/disarm": 'path.startswith("/disarm")',
    "/zero_encoders": 'path.startswith("/zero_encoders")',
    "/steer_hold": 'path.startswith("/steer_hold")',
    "/steer": 'path.startswith("/steer")',
    "/drive": 'path.startswith("/drive")',
    "/gear": 'path.startswith("/gear")',
    "/brake": 'path.startswith("/brake")',
}.items():
    assert route in html, "UI does not reference {}".format(route)
    assert handler in source, "Firmware does not handle {}".format(route)

hold_handler = source.index('path.startswith("/steer_hold")')
steer_handler = source.index('path.startswith("/steer")', hold_handler + 1)
assert hold_handler < steer_handler
assert "setPointerCapture" in html
assert "lostpointercapture" in html
assert "setInterval(()=>send('/steer_hold?d='+dir,true),180)" in html
assert "send('/steer_hold?d=STOP')" in html
assert "navigator.sendBeacon('/disarm')" in html
assert "SLOWING THEN " in source
assert "DIRECTION_SETTLE_MS" in source
assert "brake_encoder_magnet" in source
assert "1, brake_encoder, BRAKE_ENCODER_APPLY_SIGN" in source
assert "brake_transition_ms" in source
assert "brake_validation" in source

script = html.split("<script>", 1)[1].split("</script>", 1)[0]
node = subprocess.run(
    ["node", "--check"], input=script, text=True,
    capture_output=True, check=False,
)
assert node.returncode == 0, "Embedded JavaScript syntax failed: {}".format(
    node.stderr
)


class FakeTime:
    def __init__(self):
        self.us = 0

    def ticks_us(self):
        return self.us

    def ticks_ms(self):
        return self.us // 1000

    @staticmethod
    def ticks_diff(current, previous):
        return current - previous

    def sleep_us(self, amount):
        self.us += amount

    def sleep_ms(self, amount):
        self.us += amount * 1000

    def advance_ms(self, amount):
        self.us += amount * 1000


class FakePin:
    OUT = 1

    def __init__(self, number, mode=None, value=0):
        self.number = number
        self.level = value

    def value(self, new_value=None):
        if new_value is not None:
            self.level = new_value
        return self.level


class FakeI2C:
    steer_raw = 0x0800
    brake_raw = 0x0400

    def __init__(self, *args, **kwargs):
        self.writes = []
        self.selected_channel = 0

    @staticmethod
    def scan():
        return [0x58, 0x70]

    def writeto(self, address, data):
        self.writes.append((address, None, bytes(data)))
        if address == 0x70 and data:
            mask = data[0]
            for channel in range(8):
                if mask == (1 << channel):
                    self.selected_channel = channel
                    break

    def writeto_mem(self, address, register, data):
        self.writes.append((address, register, bytes(data)))

    def readfrom_mem(self, address, register, length):
        if address == 0x36 and register == 0x0B:
            return bytes((0x20,))
        if address == 0x36 and register == 0x0E:
            raw = (
                FakeI2C.steer_raw if self.selected_channel == 0
                else FakeI2C.brake_raw
            )
            return bytes(((raw >> 8) & 0xFF, raw & 0xFF))
        return bytes(length)


class FakeSPI:
    MSB = 0

    def __init__(self, *args, **kwargs):
        pass

    @staticmethod
    def write_readinto(transmit, receive):
        receive[0] = 0
        receive[1] = 0


prefix = source.split("\nap = network.WLAN", 1)[0]
prefix = prefix.replace("import network\n", "")
prefix = prefix.replace("import socket\n", "")
prefix = prefix.replace("import time\n", "")
prefix = prefix.replace("from machine import I2C, Pin, SPI\n", "")
fake_time = FakeTime()
env = {"I2C": FakeI2C, "Pin": FakePin, "SPI": FakeSPI, "time": fake_time}
exec(compile(prefix, str(FIRMWARE), "exec"), env)


def run_brake_until_settled(limit=700):
    for _ in range(limit):
        fake_time.advance_ms(6)
        env["brake"].update()
        angle = max(0.0, min(45.0, env["brake"].degrees()))
        FakeI2C.brake_raw = (
            0x0400 + round(angle / 360.0 * 4096.0)
        ) & 0x0FFF
        env["as5600_update"]()
        env["update_brake_state"]()
        if env["brake_state"] in ("APPLIED", "RELEASED"):
            return
    raise AssertionError("Brake transition did not settle")


run_brake_until_settled()
assert env["brake_state"] == "APPLIED"
assert env["brake_validation"] == "PASS"
assert abs(env["brake_encoder_reading"]()["angle"] - 45.0) < 0.01
assert env["brake_encoder_reading"]()["magnet"] == "OK"
assert env["brake_encoder_reading"]()["online"] is True

env["arm_controller"]()
run_brake_until_settled()
assert env["armed"] is True
assert env["brake_state"] == "RELEASED"
assert env["brake_validation"] == "PASS"
assert abs(env["brake_encoder_reading"]()["angle"]) < 0.01

# Steering initializes at center and reports signed left/right motion.
env["as5600_update"]()
assert env["steer_encoder"]["initialized"] is True
assert env["steer_encoder"]["angle"] == 0.0
FakeI2C.steer_raw = 0x0900
env["as5600_update"]()
assert env["steer_encoder"]["direction"] == "RIGHT"
assert abs(env["steer_encoder"]["angle"] - 22.5) < 0.01
FakeI2C.steer_raw = 0x0700
env["as5600_update"]()
assert env["steer_encoder"]["direction"] == "LEFT"
assert abs(env["steer_encoder"]["angle"] + 22.5) < 0.01
FakeI2C.steer_raw = 0x0800
env["as5600_update"]()

# One complete BLDC encoder revolution gives one turn and nominal travel.
angle_raw = [0]
def fake_as5047_read(register):
    return (angle_raw[0] if register == 0x3FFF else (1 << 8), False, False)
env["as5047_read"] = fake_as5047_read
env["as5047_update"]()
for _ in range(8):
    fake_time.advance_ms(5)
    angle_raw[0] = (angle_raw[0] + 2048) % 16384
    env["as5047_update"]()
assert env["bldc_encoder"]["initialized"] is True
assert env["bldc_encoder"]["full_turns"] == 1
assert abs(env["bldc_encoder"]["distance_m"] - 0.279288) < 0.001
assert env["bldc_encoder"]["rpm"] > 0

env["selected_gear"] = 3
env["request_drive"]("FORWARD")
for _ in range(60):
    fake_time.advance_ms(40)
    env["update_traction"]()
assert env["drive_state"] == "RUNNING FORWARD"
assert abs(env["throttle_command"] - 3.0) < 0.01

env["request_drive"]("REVERSE")
direction_change_voltage = None
for _ in range(120):
    fake_time.advance_ms(40)
    previous_direction = env["direction"]
    env["update_traction"]()
    if env["direction"] != previous_direction:
        direction_change_voltage = env["throttle_command"]
assert env["direction"] == "REVERSE"
assert env["drive_state"] == "RUNNING REVERSE"
assert abs(env["throttle_command"] - 3.0) < 0.01
assert direction_change_voltage == 0.0, direction_change_voltage

env["steer_hold"]("LEFT")
for _ in range(50):
    fake_time.advance_ms(6)
    env["steering"].update()
    env["last_steer_hold_ms"] = fake_time.ticks_ms()
env["steer_hold"]("STOP")
assert env["steering"].target_steps == env["steering"].current_steps
assert env["steering"].degrees() < 0
env["steer_hold"]("RIGHT")
fake_time.advance_ms(500)
env["update_steer_hold"]()
assert env["steer_hold_direction"] == "STOP"
assert env["steering"].target_steps == env["steering"].current_steps

env["apply_brakes"]()
assert env["throttle_command"] == 0.0
assert env["bld_en"].value() == 0
run_brake_until_settled()
assert env["brake_state"] == "APPLIED"
assert env["brake_validation"] == "PASS"

status = json.loads(env["json_status"]())
for field in (
    "drive_state", "selected_gear", "target_voltage", "steer_hold",
    "brake_encoder_magnet", "brake_encoder_angle", "brake_encoder_target",
    "brake_encoder_initialized", "brake_encoder_errors",
    "brake_validation", "brake_transition_ms", "steer_direction",
    "steer_initialized", "bldc_initialized", "bldc_rpm", "axle_rpm",
    "motor_revolutions", "full_turns", "distance_m", "displacement_m",
    "speed_kmh", "encoders_initialized",
):
    assert field in status, "Status field missing: {}".format(field)

print("PASS: firmware Python and embedded JavaScript syntax")
print("PASS: 16 responsive controls mapped to backend routes")
print("PASS: hold-left/right press, keepalive, release and timeout contract")
print("PASS: steering encoder zero plus signed left/right degrees")
print("PASS: brake AS5600 reports physical 0/45 deg, timing and validation")
print("PASS: BLDC RPM, turns, distance and displacement calculations")
print("PASS: arm/release and disarm/apply brake transitions")
print("PASS: forward ramp and controlled forward-to-reverse transition")
print("PASS: brake immediately cuts BLDC enable and DAC command")
print("PASS: live status JSON contains every UI feedback field")

"""Static UI/firmware contract check for the simple AutoKart RC controller."""

import ast
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "bench/autokart_rc_simple_encoders.py"
source = FIRMWARE.read_text(encoding="utf-8")
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
    "leftBtn", "centerBtn", "rightBtn", "brakeBtn", "offBtn", "emergencyBtn",
    "driveState", "gear", "voltage", "steerTarget", "brakeState",
    "brakeAngle", "steerDetail", "steerErrors", "bldcEnc",
    "bldcDetail", "gpState", "message",
}
missing_ids = required_ids - set(ids)
assert not missing_ids, "Missing UI IDs: {}".format(sorted(missing_ids))

required_routes = {
    "/status": 'path.startswith("/status")',
    "/arm": 'path.startswith("/arm")',
    "/disarm": 'path.startswith("/disarm")',
    "/steer": 'path.startswith("/steer")',
    "/direction": 'path.startswith("/direction")',
    "/gear": 'path.startswith("/gear")',
    "/brake": 'path.startswith("/brake")',
}
for route, handler in required_routes.items():
    assert route in html, "UI does not reference {}".format(route)
    assert handler in source, "Firmware does not handle {}".format(route)

expected_actions = {
    "ARM": "/arm",
    "DISARM": "/disarm",
    "UP": "/direction?d=FORWARD",
    "DOWN": "/direction?d=REVERSE",
    "LEFT": "/steer?v=-45",
    "CENTER": "/steer?v=0",
    "RIGHT": "/steer?v=45",
    "BRAKE_APPLY": "/brake?s=apply",
    "BRAKE_RELEASE": "/brake?s=release",
}
for name, action in expected_actions.items():
    assert action in html, "{} action missing: {}".format(name, action)

for gear in range(6):
    if gear == 0:
        assert "gear(0)" in html, "Throttle-off action missing"
    else:
        assert "gear({})".format(gear) in html, "Gear {} missing".format(gear)

assert "navigator.sendBeacon('/disarm')" in html, "Page-exit disarm missing"
assert "bb.disabled=!s.armed" in html, "Disarmed brake lockout missing"
assert "s.gear!==0" in html, "Direction-change gear lockout missing"
assert "s.brake_state!=='RELEASED'" in html, "Gear brake interlock missing"

traction_stop_body = source.split("def traction_stop():", 1)[1].split(
    "\ndef apply_brakes():", 1
)[0]
assert traction_stop_body.index("bld_en.value(0)") < traction_stop_body.index(
    "write_voltage(0.0)"
), "Traction stop must cut BLDC EN before touching the DAC"
gear_handler = source.split('elif path.startswith("/gear"):', 1)[1].split(
    'elif path.startswith("/brake"):', 1
)[0]
assert 'elif requested == 0:' in gear_handler
assert gear_handler.index('elif requested == 0:') < gear_handler.index(
    'elif not gp_present:'
), "Throttle Off must remain available when the DAC is offline"

print("PASS: 15 controls mapped to firmware routes")
print("PASS: unique IDs and required status fields")
print("PASS: disarm, direction, brake, and gear UX interlocks present")
print("PASS: stop cuts BLDC EN before DAC access")
print("PASS: throttle-off remains available with DAC offline")

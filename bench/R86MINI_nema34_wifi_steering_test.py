"""Standalone Wi-Fi steering controller for ESP32-S3 + R86mini + NEMA34.

Flash this file as ``main.py``. The ESP32-S3 creates its own Wi-Fi access
point and serves a control page, so no laptop is required after installation.

IMPORTANT: Steering is open-loop. Physically center the steering before boot,
then use CALIBRATE whenever the mechanical center and software center differ.
CALIBRATE does not move the motor; it assigns the current position as 0 deg.

Verified steering wiring:
  GPIO4 -> tested 5 V NPN interface -> R86mini PUL-
  GPIO5 -> tested 5 V NPN interface -> R86mini DIR-
  5 V -> R86mini PUL+ and DIR+
  ENA+ and ENA- disconnected
  ESP32 GND and interface/driver signal GND common

Verified NEMA34 winding connections:
  R86mini A+ = RED + BLUE
  R86mini A- = YELLOW + BLACK
  R86mini B+ = ORANGE + GREEN
  R86mini B- = BROWN + WHITE

R86mini pulse setting:
  3200 pulses/revolution: SW5 OFF, SW6 OFF, SW7 ON, SW8 ON
"""

import network
import socket
import time
from machine import Pin


FIRMWARE_NAME = "r86mini-nema34-wifi-steering"
FIRMWARE_VERSION = "1.0.0"

SSID = "AutoKart-Steering"
PASSWORD = "12345678"

STEP_GPIO = 4
DIR_GPIO = 5

PULSES_PER_REVOLUTION = 3200
STEERING_RATIO = 1.0
PULSES_PER_DEGREE = PULSES_PER_REVOLUTION * STEERING_RATIO / 360.0

MIN_ANGLE_DEG = -90.0
MAX_ANGLE_DEG = 90.0

RIGHT_DIRECTION = 1
LEFT_DIRECTION = 0

MIN_PPS = 100
MAX_PPS = 2500
DEFAULT_PPS = 1200
START_PPS = 200
STEP_HIGH_US = 50

SWEEP_STEP_DEG = 2
SWEEP_DELAY_MS = 1000


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class SteeringAxis:
    """Non-blocking open-loop step/dir steering axis."""

    def __init__(self):
        self.step_pin = Pin(STEP_GPIO, Pin.OUT, value=0)
        self.dir_pin = Pin(DIR_GPIO, Pin.OUT, value=0)
        self.current_steps = 0
        self.target_steps = 0
        self.maximum_pps = DEFAULT_PPS
        self.current_pps = START_PPS
        self.last_step_us = time.ticks_us()
        self.last_direction = 0

    def degrees_to_steps(self, degrees):
        return round(float(degrees) * PULSES_PER_DEGREE)

    def steps_to_degrees(self, steps):
        return float(steps) / PULSES_PER_DEGREE

    def current_degrees(self):
        return self.steps_to_degrees(self.current_steps)

    def target_degrees(self):
        return self.steps_to_degrees(self.target_steps)

    def set_target(self, degrees):
        degrees = clamp(float(degrees), MIN_ANGLE_DEG, MAX_ANGLE_DEG)
        self.target_steps = self.degrees_to_steps(degrees)
        print("STEERING TARGET ->", round(degrees, 1), "deg")

    def set_speed(self, pps):
        self.maximum_pps = int(clamp(int(pps), MIN_PPS, MAX_PPS))
        self.current_pps = min(self.current_pps, self.maximum_pps)
        print("STEERING SPEED ->", self.maximum_pps, "pps")

    def hold(self):
        self.target_steps = self.current_steps
        self.current_pps = START_PPS
        self.last_direction = 0
        self.step_pin.value(0)
        print("STEERING HOLD ->", round(self.current_degrees(), 1), "deg")

    def calibrate_center(self):
        self.hold()
        self.current_steps = 0
        self.target_steps = 0
        print("CENTER CALIBRATED -> current physical position is 0 deg")

    def at_target(self):
        return self.current_steps == self.target_steps

    def update(self):
        if self.at_target():
            self.current_pps = START_PPS
            self.last_direction = 0
            return

        direction = 1 if self.target_steps > self.current_steps else -1

        if direction != self.last_direction:
            self.current_pps = min(START_PPS, self.maximum_pps)
            self.last_direction = direction
            self.dir_pin.value(
                RIGHT_DIRECTION if direction > 0 else LEFT_DIRECTION
            )

        now = time.ticks_us()
        period_us = 1_000_000 // max(1, self.current_pps)

        if time.ticks_diff(now, self.last_step_us) < period_us:
            return

        self.step_pin.value(1)
        time.sleep_us(STEP_HIGH_US)
        self.step_pin.value(0)

        self.current_steps += direction
        self.last_step_us = now

        # Gentle acceleration matching the working serial test's slow start.
        self.current_pps = min(self.maximum_pps, self.current_pps + 5)


steering = SteeringAxis()

sweep_active = False
sweep_index = 0
sweep_arrival_ms = None


def make_sweep_targets():
    targets = []

    angle = SWEEP_STEP_DEG
    while angle < MAX_ANGLE_DEG:
        targets.append(float(angle))
        angle += SWEEP_STEP_DEG
    targets.append(MAX_ANGLE_DEG)
    targets.append(0.0)

    angle = SWEEP_STEP_DEG
    while angle < abs(MIN_ANGLE_DEG):
        targets.append(float(-angle))
        angle += SWEEP_STEP_DEG
    targets.append(MIN_ANGLE_DEG)
    targets.append(0.0)

    return targets


SWEEP_TARGETS = make_sweep_targets()


def start_sweep():
    global sweep_active, sweep_index, sweep_arrival_ms
    sweep_active = True
    sweep_index = 0
    sweep_arrival_ms = None
    steering.set_target(0)
    print("SWEEP START -> 0, +2...+90, 0, -2...-90, repeat")


def stop_sweep():
    global sweep_active, sweep_arrival_ms
    sweep_active = False
    sweep_arrival_ms = None
    steering.hold()
    print("SWEEP STOPPED")


def update_sweep():
    global sweep_index, sweep_arrival_ms

    if not sweep_active or not steering.at_target():
        return

    now = time.ticks_ms()

    if sweep_arrival_ms is None:
        sweep_arrival_ms = now
        return

    if time.ticks_diff(now, sweep_arrival_ms) < SWEEP_DELAY_MS:
        return

    steering.set_target(SWEEP_TARGETS[sweep_index])
    sweep_index = (sweep_index + 1) % len(SWEEP_TARGETS)
    sweep_arrival_ms = None


HTML = """<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AutoKart Steering</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#101216;color:#f5f7fa;font-family:Arial,sans-serif;text-align:center}
.wrap{max-width:680px;margin:auto;padding:18px}.card{background:#1b2028;border:1px solid #333b48;border-radius:16px;padding:17px;margin:12px 0}
h1{margin:4px}.sub{color:#98a2b3;margin:7px 0 18px}.angle{font-size:52px;font-weight:bold;margin:8px}.target{color:#aab4c3}
.row{display:flex;gap:9px;justify-content:center;flex-wrap:wrap}button{border:0;border-radius:11px;padding:15px;min-width:29%;font-size:16px;font-weight:bold;color:white;touch-action:manipulation}
.blue{background:#2679d8}.gray{background:#606b78}.green{background:#219653}.orange{background:#d67a20}.red{background:#d9363e}.wide{width:95%}
input[type=number]{width:120px;padding:12px;border-radius:9px;border:1px solid #4c5665;background:#11151b;color:white;font-size:20px;text-align:center}
input[type=range]{width:94%;accent-color:#2f91ff}.status{color:#5ce19b;font-weight:bold}.warning{font-size:13px;color:#f0b45d;line-height:1.5}
</style></head><body><div class="wrap">
<h1>AutoKart Steering</h1><div class="sub">R86mini + NEMA34 | 1:1 ratio</div>
<div class="card"><div id="status" class="status">READY</div><div class="angle"><span id="position">0.0</span>&deg;</div><div class="target">Target: <span id="target">0.0</span>&deg;</div></div>
<div class="card"><div>ANGLE</div><p><input id="degrees" type="number" min="0" max="90" step="1" value="45"> degrees</p>
<div class="row"><button class="blue" onclick="move('left')">LEFT</button><button class="gray" onclick="command('/center')">CENTER</button><button class="blue" onclick="move('right')">RIGHT</button></div></div>
<div class="card"><div>SPEED: <span id="speedText">1200</span> pps</div><input id="speed" type="range" min="100" max="2500" step="50" value="1200" oninput="speedChanged()"></div>
<div class="card"><div class="row"><button class="green" onclick="command('/sweep?state=start')">START SWEEP</button><button class="orange" onclick="command('/sweep?state=stop')">STOP SWEEP</button></div>
<div class="row"><button class="red wide" onclick="command('/hold')">STOP / HOLD POSITION</button></div></div>
<div class="card"><button class="orange wide" onclick="calibrate()">CALIBRATE CURRENT POSITION AS CENTER</button></div>
<p class="warning">Open-loop test controller. Keep steering clear of people and hard stops.<br>CALIBRATE causes no movement; it assigns the current position as 0&deg;.</p>
</div><script>
function command(path){return fetch(path,{cache:'no-store'}).catch(()=>{});}
function degrees(){return Math.max(0,Math.min(90,Number(document.getElementById('degrees').value)||0));}
function move(side){command('/'+side+'?deg='+degrees());}
function speedChanged(){let value=document.getElementById('speed').value;document.getElementById('speedText').textContent=value;command('/speed?pps='+value);}
function calibrate(){if(confirm('Is the steering physically centered now?'))command('/calibrate');}
setInterval(()=>{command('/status').then(r=>r&&r.text()).then(t=>{if(!t)return;let p=t.split('|');document.getElementById('position').textContent=Number(p[0]).toFixed(1);document.getElementById('target').textContent=Number(p[1]).toFixed(1);document.getElementById('status').textContent=p[3]==='1'?'SWEEP RUNNING':'READY';});},300);
</script></body></html>"""


def query_number(path, key, default):
    marker = key + "="
    try:
        return float(path.split(marker, 1)[1].split("&", 1)[0])
    except (IndexError, ValueError):
        return default


def query_text(path, key, default):
    marker = key + "="
    try:
        return path.split(marker, 1)[1].split("&", 1)[0]
    except IndexError:
        return default


def send_response(client, body, content_type="text/plain", status="200 OK"):
    if isinstance(body, str):
        body = body.encode()

    header = (
        "HTTP/1.1 " + status + "\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Content-Length: " + str(len(body)) + "\r\n"
        "Connection: close\r\n\r\n"
    )
    client.send(header.encode())
    client.send(body)


ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=SSID, password=PASSWORD)

while not ap.active():
    time.sleep_ms(100)

ip_address = ap.ifconfig()[0]

server = socket.socket()
try:
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except OSError:
    pass

server.bind(("0.0.0.0", 80))
server.listen(2)
server.settimeout(0)

print()
print("========================================")
print(" NEMA34 WIFI STEERING READY")
print("========================================")
print("Wi-Fi    :", SSID)
print("Password :", PASSWORD)
print("Open     : http://" + ip_address)
print("Limits   : -90 to +90 degrees")
print("Speed    :", steering.maximum_pps, "pps")
print("Boot zero: current physical position assumed 0 degrees")
print("========================================")


try:
    while True:
        steering.update()
        update_sweep()

        try:
            client, remote = server.accept()
        except OSError:
            time.sleep_us(100)
            continue

        try:
            request = client.recv(1024)
            if not request:
                continue

            first_line = request.decode("utf-8", "ignore").split("\r\n", 1)[0]
            parts = first_line.split(" ")

            if len(parts) < 2:
                send_response(client, "BAD REQUEST", status="400 Bad Request")
                continue

            path = parts[1]

            if path == "/":
                send_response(client, HTML, "text/html")

            elif path.startswith("/left"):
                if sweep_active:
                    stop_sweep()
                degrees = abs(query_number(path, "deg", 90.0))
                steering.set_target(-degrees)
                send_response(client, "OK")

            elif path.startswith("/right"):
                if sweep_active:
                    stop_sweep()
                degrees = abs(query_number(path, "deg", 90.0))
                steering.set_target(degrees)
                send_response(client, "OK")

            elif path.startswith("/center"):
                if sweep_active:
                    stop_sweep()
                steering.set_target(0)
                send_response(client, "OK")

            elif path.startswith("/calibrate"):
                if sweep_active:
                    stop_sweep()
                steering.calibrate_center()
                send_response(client, "CENTER CALIBRATED")

            elif path.startswith("/hold"):
                if sweep_active:
                    stop_sweep()
                else:
                    steering.hold()
                send_response(client, "HOLDING")

            elif path.startswith("/speed"):
                steering.set_speed(query_number(path, "pps", DEFAULT_PPS))
                send_response(client, "OK")

            elif path.startswith("/sweep"):
                state = query_text(path, "state", "stop")
                if state == "start":
                    start_sweep()
                else:
                    stop_sweep()
                send_response(client, "OK")

            elif path.startswith("/status"):
                body = (
                    str(round(steering.current_degrees(), 2))
                    + "|" + str(round(steering.target_degrees(), 2))
                    + "|" + str(steering.maximum_pps)
                    + "|" + ("1" if sweep_active else "0")
                )
                send_response(client, body)

            else:
                send_response(client, "NOT FOUND", status="404 Not Found")

        except Exception as error:
            print("HTTP ERROR:", error)
            try:
                send_response(client, "ERROR", status="500 Internal Server Error")
            except Exception:
                pass
        finally:
            try:
                client.close()
            except Exception:
                pass

except KeyboardInterrupt:
    print("CTRL+C")

finally:
    steering.hold()
    steering.step_pin.value(0)
    try:
        server.close()
    except Exception:
        pass

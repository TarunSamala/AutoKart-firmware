"""New AutoKart RC controller for ESP32-S3.

Combines:
  - R86mini + NEMA34 direct-drive steering: -45, 0, +45 degrees
  - GP8630N 0-5 V speed command for BLD-750
  - PN2222A transistor control for BLD-750 BRK, EN and F/R
  - Wi-Fi access-point web remote

This is an open-loop bench controller. Mechanically center steering before
boot/reset. Keep the driven wheels off the ground during validation.

Steering wiring (tested NPN interface):
  ESP32-S3 GPIO4 -> NPN interface -> R86mini PUL-
  ESP32-S3 GPIO5 -> NPN interface -> R86mini DIR-
  5 V -> R86mini PUL+ and DIR+
  R86mini ENA+ and ENA- disconnected

Verified NEMA34 parallel winding pairs:
  R86mini A+ = RED + BLUE
  R86mini A- = YELLOW + BLACK
  R86mini B+ = ORANGE + GREEN
  R86mini B- = BROWN + WHITE

R86mini pulse setting:
  3200 pulses/revolution: SW5 OFF, SW6 OFF, SW7 ON, SW8 ON

Traction wiring:
  GPIO8  -> GP8630N SDA
  GPIO9  -> GP8630N SCL
  GPIO13 -> BRK transistor
  GPIO14 -> EN transistor
  GPIO15 -> F/R transistor
"""

import network
import socket
import time
from machine import I2C, Pin


FIRMWARE_NAME = "New-AutoKartRC"
FIRMWARE_VERSION = "1.0.0"

# Wi-Fi access point.
SSID = "New-AutoKartRC"
PASSWORD = "12345678"
COMMAND_TIMEOUT_MS = 1800

# R86mini/NEMA34 steering.
STEER_STEP_GPIO = 4
STEER_DIR_GPIO = 5
PULSES_PER_REVOLUTION = 3200
STEERING_RATIO = 1.0
STEER_LIMIT_DEG = 45.0
STEER_PULSES_PER_DEG = (
    PULSES_PER_REVOLUTION * STEERING_RATIO / 360.0
)
STEER_RIGHT_DIR = 1
STEER_STEP_HIGH_US = 50
STEER_START_PERIOD_US = 2500  # 400 pps
STEER_FAST_PERIOD_US = 800    # 1250 pps
STEER_ACCEL_US_PER_STEP = 8

# GP8630N DAC.
SDA_GPIO = 8
SCL_GPIO = 9
GP_ADDR = 0x58
REG_MODE = 0x01
REG_DAC = 0x02
MODE_0_10V = 0x1C
MAX_VOLTAGE = 5.0
THROTTLE_UPDATE_MS = 20
THROTTLE_RISE_STEP = 0.04
THROTTLE_FALL_STEP = 0.12

# BLD-750 transistor controls. These levels match full_bldc_test.py.
BRK_GPIO = 13
EN_GPIO = 14
FR_GPIO = 15


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class SteeringAxis:
    """Non-blocking open-loop steering position estimator."""

    def __init__(self):
        self.step_pin = Pin(STEER_STEP_GPIO, Pin.OUT, value=0)
        self.dir_pin = Pin(STEER_DIR_GPIO, Pin.OUT, value=0)
        self.current_steps = 0
        self.target_steps = 0
        self.current_period_us = STEER_START_PERIOD_US
        self.last_step_us = time.ticks_us()
        self.last_direction = 0

    def set_target(self, degrees):
        degrees = clamp(float(degrees), -STEER_LIMIT_DEG, STEER_LIMIT_DEG)
        self.target_steps = round(degrees * STEER_PULSES_PER_DEG)
        print("STEERING TARGET ->", round(degrees, 1), "deg")

    def hold(self):
        self.target_steps = self.current_steps
        self.current_period_us = STEER_START_PERIOD_US
        self.last_direction = 0

    def current_deg(self):
        return self.current_steps / STEER_PULSES_PER_DEG

    def target_deg(self):
        return self.target_steps / STEER_PULSES_PER_DEG

    def update(self):
        if self.current_steps == self.target_steps:
            self.current_period_us = STEER_START_PERIOD_US
            self.last_direction = 0
            return

        direction = 1 if self.target_steps > self.current_steps else -1

        if direction != self.last_direction:
            self.current_period_us = STEER_START_PERIOD_US
            self.last_direction = direction
            self.dir_pin.value(
                STEER_RIGHT_DIR if direction > 0 else 1 - STEER_RIGHT_DIR
            )

        now = time.ticks_us()
        if time.ticks_diff(now, self.last_step_us) < self.current_period_us:
            return

        self.step_pin.value(1)
        time.sleep_us(STEER_STEP_HIGH_US)
        self.step_pin.value(0)

        self.current_steps += direction
        self.last_step_us = now
        self.current_period_us = max(
            STEER_FAST_PERIOD_US,
            self.current_period_us - STEER_ACCEL_US_PER_STEP,
        )


# Create digital outputs first and immediately enter the safest traction state.
en_pin = Pin(EN_GPIO, Pin.OUT, value=0)       # transistor off -> EN stop
brk_pin = Pin(BRK_GPIO, Pin.OUT, value=0)    # transistor off -> brake applied
fr_pin = Pin(FR_GPIO, Pin.OUT, value=0)      # CW
steering = SteeringAxis()

armed = False
brake_released = False
direction = "CW"
current_voltage = 0.0
target_voltage = 0.0
last_throttle_update_ms = time.ticks_ms()
last_contact_ms = time.ticks_ms()


def enable_run():
    global armed
    en_pin.value(1)
    armed = True
    print("EN -> RUN")


def enable_stop():
    global armed
    en_pin.value(0)
    armed = False
    print("EN -> STOP")


def brake_release():
    global brake_released
    brk_pin.value(1)
    brake_released = True
    print("BRK -> RELEASED")


def brake_apply():
    global brake_released
    brk_pin.value(0)
    brake_released = False
    print("BRK -> APPLIED")


def set_direction(name):
    global direction
    direction = "CCW" if name == "CCW" else "CW"
    fr_pin.value(1 if direction == "CCW" else 0)
    print("DIRECTION ->", direction)


# Initialize and verify the DAC before permitting traction.
i2c = I2C(
    0,
    sda=Pin(SDA_GPIO),
    scl=Pin(SCL_GPIO),
    freq=100000,
)

devices = i2c.scan()
print("I2C:", [hex(device) for device in devices])

if GP_ADDR not in devices:
    brake_apply()
    enable_stop()
    raise RuntimeError("GP8630N NOT FOUND AT 0x58")

i2c.writeto_mem(GP_ADDR, REG_MODE, bytes([MODE_0_10V]))
time.sleep_ms(100)


def write_voltage(voltage):
    global current_voltage
    voltage = clamp(float(voltage), 0.0, MAX_VOLTAGE)
    dac_value = int((voltage / 10.0) * 65535)
    i2c.writeto_mem(
        GP_ADDR,
        REG_DAC,
        bytes([dac_value & 0xFF, (dac_value >> 8) & 0xFF]),
    )
    current_voltage = voltage


def throttle_update():
    global last_throttle_update_ms

    now = time.ticks_ms()
    if time.ticks_diff(now, last_throttle_update_ms) < THROTTLE_UPDATE_MS:
        return

    last_throttle_update_ms = now

    if current_voltage < target_voltage:
        write_voltage(min(current_voltage + THROTTLE_RISE_STEP, target_voltage))
    elif current_voltage > target_voltage:
        write_voltage(max(current_voltage - THROTTLE_FALL_STEP, target_voltage))


def safe_stop(reason="SAFE STOP"):
    global target_voltage

    target_voltage = 0.0
    try:
        write_voltage(0.0)
    except Exception as error:
        print("DAC ZERO FAILED:", error)

    brake_apply()
    enable_stop()
    steering.hold()
    print("!!!", reason, "!!!")


write_voltage(0.0)
brake_apply()
enable_stop()
set_direction("CW")


HTML = """<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>New AutoKart RC</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#101216;color:#f4f6f8;font-family:Arial,sans-serif;text-align:center}
.wrap{max-width:680px;margin:auto;padding:18px}.card{background:#1a1e25;border:1px solid #303744;border-radius:16px;padding:16px;margin:12px 0}
h1{margin:4px 0;font-size:28px}.sub{color:#98a2b3;margin:6px 0 18px}.state{font-size:22px;font-weight:bold;color:#ff6767}
.angle{font-size:42px;font-weight:bold;margin:8px}.row{display:flex;gap:10px;justify-content:center;flex-wrap:wrap}
button{border:0;border-radius:12px;padding:16px 18px;min-width:29%;font-size:17px;font-weight:bold;color:white;touch-action:manipulation}
.left,.right{background:#2878cf}.center{background:#606a78}.run{background:#219653}.stop{background:#d9363e}.dir{background:#9b5de5}
.panic{width:100%;background:#ff1f2d;font-size:22px}.slider{width:96%;accent-color:#36a3ff}.volts{font-size:38px;font-weight:bold}
.note{font-size:13px;color:#f0b45d;line-height:1.45}.selected{outline:3px solid white}
</style></head><body><div class="wrap">
<h1>New AutoKart RC</h1><div class="sub">NEMA34 steering + BLD-750 traction</div>
<div class="card"><div id="state" class="state">DISARMED</div></div>
<div class="card"><div>STEERING (1:1)</div><div class="angle"><span id="angle">0</span>&deg;</div>
<div class="row"><button class="left" onclick="steer(-45)">-45 LEFT</button><button class="center" onclick="steer(0)">0 CENTER</button><button class="right" onclick="steer(45)">+45 RIGHT</button></div></div>
<div class="card"><div>TRACTION COMMAND</div><div class="volts"><span id="volts">0.00</span> V</div>
<input id="speed" class="slider" type="range" min="0" max="500" value="0" oninput="speedChanged()">
<div class="row"><button class="run" onclick="arm()">ARM / RUN</button><button class="stop" onclick="stopAll()">STOP + BRAKE</button></div>
<div class="row"><button id="cw" class="dir selected" onclick="setDir('CW')">CW</button><button id="ccw" class="dir" onclick="setDir('CCW')">CCW</button></div></div>
<button class="panic" onclick="stopAll()">SAFE STOP</button>
<p class="note">BENCH TEST ONLY - WHEELS OFF GROUND<br>Center steering mechanically before ESP32 boot. Position is open-loop.</p>
</div><script>
let armed=false, pending=false, sending=false;
function get(path){return fetch(path,{cache:'no-store'}).catch(()=>{});}
function setState(text,ok){let e=document.getElementById('state');e.textContent=text;e.style.color=ok?'#54d98c':'#ff6767';}
function steer(deg){get('/steer?deg='+deg).then(r=>{if(r&&r.ok)document.getElementById('angle').textContent=deg;});}
function arm(){get('/arm').then(r=>{if(r&&r.ok){armed=true;setState('ARMED / RUN',true);sendSpeed();}});}
function stopAll(){armed=false;document.getElementById('speed').value=0;document.getElementById('volts').textContent='0.00';get('/stop');setState('STOPPED / BRAKE APPLIED',false);}
function speedChanged(){let s=document.getElementById('speed');document.getElementById('volts').textContent=(s.value/100).toFixed(2);pending=true;sendSpeed();}
function sendSpeed(){if(!armed||sending||!pending)return;pending=false;sending=true;let v=document.getElementById('speed').value/100;get('/speed?v='+v).finally(()=>{sending=false;if(pending)setTimeout(sendSpeed,40);});}
function setDir(d){if(Number(document.getElementById('speed').value)!==0){alert('Set speed to 0 before changing direction');return;}get('/direction?d='+d).then(r=>{if(r&&r.ok){document.getElementById('cw').classList.toggle('selected',d==='CW');document.getElementById('ccw').classList.toggle('selected',d==='CCW');}});}
setInterval(()=>{get('/heartbeat').then(r=>r&&r.text()).then(t=>{if(!t)return;let p=t.split('|');armed=p[0]==='ARMED';setState(armed?'ARMED / RUN':'DISARMED',armed);document.getElementById('angle').textContent=Number(p[1]).toFixed(1);if(!armed){document.getElementById('speed').value=0;document.getElementById('volts').textContent='0.00';}});},500);
window.addEventListener('beforeunload',()=>{navigator.sendBeacon('/stop');});
</script></body></html>"""


def query_value(path, key, default=0.0):
    marker = key + "="
    try:
        value = path.split(marker, 1)[1].split("&", 1)[0]
        return float(value)
    except (IndexError, ValueError):
        return default


def query_text(path, key, default=""):
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
print(" NEW AUTOKART RC READY")
print("========================================")
print("Firmware :", FIRMWARE_NAME, FIRMWARE_VERSION)
print("Wi-Fi    :", SSID)
print("Password :", PASSWORD)
print("Open     : http://" + ip_address)
print("Steering : -45 / 0 / +45 deg, direct 1:1")
print("45 deg   :", round(45 * STEER_PULSES_PER_DEG), "pulses")
print("Throttle : 0-", MAX_VOLTAGE, "V")
print("BLDC     : BRK13 EN14 F/R15")
print("CENTER STEERING PHYSICALLY BEFORE ARMING")
print("========================================")


try:
    while True:
        steering.update()
        throttle_update()

        now = time.ticks_ms()
        if armed and time.ticks_diff(now, last_contact_ms) > COMMAND_TIMEOUT_MS:
            safe_stop("WEB CONTROL TIMEOUT")

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

            elif path.startswith("/arm"):
                target_voltage = 0.0
                write_voltage(0.0)
                brake_release()
                enable_run()
                last_contact_ms = time.ticks_ms()
                send_response(client, "OK")

            elif path.startswith("/stop"):
                safe_stop("REMOTE STOP")
                send_response(client, "OK")

            elif path.startswith("/heartbeat"):
                if armed:
                    last_contact_ms = time.ticks_ms()
                body = (
                    ("ARMED" if armed else "DISARMED")
                    + "|" + str(round(steering.current_deg(), 1))
                    + "|" + direction
                )
                send_response(client, body)

            elif path.startswith("/steer"):
                if not armed:
                    send_response(client, "ARM FIRST", status="409 Conflict")
                    continue

                requested_angle = query_value(path, "deg", 0.0)
                if requested_angle not in (-45.0, 0.0, 45.0):
                    send_response(client, "USE -45, 0 OR 45", status="400 Bad Request")
                    continue

                steering.set_target(requested_angle)
                last_contact_ms = time.ticks_ms()
                send_response(client, "OK")

            elif path.startswith("/speed"):
                requested_voltage = clamp(
                    query_value(path, "v", 0.0),
                    0.0,
                    MAX_VOLTAGE,
                )

                if armed and brake_released:
                    target_voltage = requested_voltage
                    last_contact_ms = time.ticks_ms()
                    print("TARGET SV ->", round(target_voltage, 2), "V")
                    send_response(client, "OK")
                else:
                    target_voltage = 0.0
                    send_response(client, "ARM FIRST", status="409 Conflict")

            elif path.startswith("/direction"):
                requested_direction = query_text(path, "d", "CW")

                if target_voltage > 0.01 or current_voltage > 0.05:
                    send_response(
                        client,
                        "SET SPEED TO ZERO FIRST",
                        status="409 Conflict",
                    )
                    continue

                if requested_direction not in ("CW", "CCW"):
                    send_response(client, "BAD DIRECTION", status="400 Bad Request")
                    continue

                set_direction(requested_direction)
                if armed:
                    last_contact_ms = time.ticks_ms()
                send_response(client, "OK")

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
    safe_stop("CONTROLLER EXIT")
    steering.step_pin.value(0)
    try:
        server.close()
    except Exception:
        pass

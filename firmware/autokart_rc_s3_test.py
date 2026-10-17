"""AutoKart ESP32-S3 responsive Wi-Fi RC bench test.

Controls:
  - R86mini + NEMA34 open-loop steering on GPIO4/GPIO5
  - TB6600 + brake stepper on GPIO38/GPIO39/GPIO40
  - GP8630N throttle command on I2C GPIO1/GPIO2, limited to 3.0 V
  - BLD750 BRK/EN/F-R through the tested transistor stages on GPIO13/14/15

The steering UI defaults to +/-30 degrees and has an explicit +/-60-degree
mode. Steering and brake positions are open-loop estimates. This firmware is
for wheels-off-ground bench testing; it is not production VCU firmware.

Before every boot:
  1. Mechanically center steering; firmware treats that position as 0 degrees.
  2. Put the brake mechanism at RELEASED; firmware treats that as 0 degrees.
  3. Keep the driven wheels off the ground and the area clear.

Final integrated pin allocation:
  Steering R86mini: STEP GPIO4, DIR GPIO5 (ENA disconnected/optional)
  Brake TB6600:     STEP GPIO38, DIR GPIO39, ENA GPIO40
  GP8630N:          SDA GPIO1, SCL GPIO2, address 0x58
  BLD750 stages:    BRK GPIO13, EN GPIO14, F/R GPIO15
"""

import network
import socket
import time
from machine import I2C, Pin


FIRMWARE_NAME = "AutoKart-RC-S3-Test"
FIRMWARE_VERSION = "1.0.0"

SSID = "AutoKart-RC"
PASSWORD = "12345678"
COMMAND_TIMEOUT_MS = 1600

# Steering: R86mini + NEMA34, configurable pulses/rev, 1:1 gearing.
STEER_STEP_GPIO = 4
STEER_DIR_GPIO = 5
STEER_RIGHT_DIR = 1
STEER_PULSES_PER_REV = 400
STEER_GEAR_RATIO = 1.0
STEER_PULSES_PER_DEG = (
    STEER_PULSES_PER_REV * STEER_GEAR_RATIO / 360.0
)
STEER_DEFAULT_LIMIT_DEG = 30.0
STEER_EXTENDED_LIMIT_DEG = 60.0

# Brake: TB6600. This is an open-loop commanded angle, not brake feedback.
BRAKE_STEP_GPIO = 38
BRAKE_DIR_GPIO = 39
BRAKE_EN_GPIO = 40
BRAKE_APPLY_DIR = 1
BRAKE_ENABLE_LEVEL = 0
BRAKE_DISABLE_LEVEL = 1
BRAKE_PULSES_PER_REV = 3200
BRAKE_GEAR_RATIO = 1.0
BRAKE_PULSES_PER_DEG = (
    BRAKE_PULSES_PER_REV * BRAKE_GEAR_RATIO / 360.0
)
BRAKE_APPLY_DEG = 15.0

# Non-blocking step timing.
STEP_HIGH_US = 20
STEER_START_PERIOD_US = 1400
STEER_DEFAULT_PPS = 1500
STEER_FAST_PERIOD_US = 1_000_000 // STEER_DEFAULT_PPS
STEER_ACCEL_US_PER_STEP = 12
BRAKE_START_PERIOD_US = 1800
BRAKE_FAST_PERIOD_US = 900
BRAKE_ACCEL_US_PER_STEP = 10

# GP8630N DAC on the final shared I2C bus.
SDA_GPIO = 1
SCL_GPIO = 2
GP_ADDR = 0x58
REG_MODE = 0x01
REG_DAC = 0x02
MODE_0_10V = 0x1C
THROTTLE_MAX_V = 3.0
THROTTLE_UPDATE_MS = 20
THROTTLE_RISE_STEP_V = 0.03
THROTTLE_FALL_STEP_V = 0.15

# Tested PN2222A stages for the BLD750.
BLD_BRK_GPIO = 13
BLD_EN_GPIO = 14
BLD_FR_GPIO = 15


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


class StepperAxis:
    """Non-blocking open-loop STEP/DIR position estimator."""

    def __init__(
        self,
        step_pin,
        dir_pin,
        pulses_per_degree,
        minimum_degrees,
        maximum_degrees,
        positive_direction,
        start_period_us,
        fast_period_us,
        acceleration_us,
    ):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.pulses_per_degree = pulses_per_degree
        self.minimum_degrees = minimum_degrees
        self.maximum_degrees = maximum_degrees
        self.positive_direction = positive_direction
        self.start_period_us = start_period_us
        self.fast_period_us = fast_period_us
        self.acceleration_us = acceleration_us
        self.current_steps = 0
        self.target_steps = 0
        self.current_period_us = start_period_us
        self.last_step_us = time.ticks_us()
        self.last_direction = 0

    def set_limits(self, minimum_degrees, maximum_degrees):
        self.minimum_degrees = float(minimum_degrees)
        self.maximum_degrees = float(maximum_degrees)
        self.set_target(self.target_degrees())

    def set_pulses_per_degree(self, pulses_per_degree):
        self.pulses_per_degree = float(pulses_per_degree)

    def set_speed_pps(self, pulses_per_second):
        pulses_per_second = int(pulses_per_second)
        self.fast_period_us = max(
            STEP_HIGH_US + 20,
            1_000_000 // pulses_per_second,
        )
        self.start_period_us = max(
            STEER_START_PERIOD_US,
            self.fast_period_us,
        )
        self.current_period_us = self.start_period_us

    def set_target(self, degrees):
        degrees = clamp(
            float(degrees),
            self.minimum_degrees,
            self.maximum_degrees,
        )
        self.target_steps = round(degrees * self.pulses_per_degree)

    def hold(self):
        self.target_steps = self.current_steps
        self.current_period_us = self.start_period_us
        self.last_direction = 0

    def calibrate_zero(self):
        self.current_steps = 0
        self.target_steps = 0
        self.current_period_us = self.start_period_us
        self.last_direction = 0

    def current_degrees(self):
        return self.current_steps / self.pulses_per_degree

    def target_degrees(self):
        return self.target_steps / self.pulses_per_degree

    def at_target(self):
        return self.current_steps == self.target_steps

    def update(self):
        if self.at_target():
            self.current_period_us = self.start_period_us
            self.last_direction = 0
            return

        direction = 1 if self.target_steps > self.current_steps else -1

        if direction != self.last_direction:
            self.current_period_us = self.start_period_us
            self.last_direction = direction
            self.dir_pin.value(
                self.positive_direction
                if direction > 0
                else 1 - self.positive_direction
            )

        now = time.ticks_us()
        if time.ticks_diff(now, self.last_step_us) < self.current_period_us:
            return

        self.step_pin.value(1)
        time.sleep_us(STEP_HIGH_US)
        self.step_pin.value(0)
        self.current_steps += direction
        self.last_step_us = now
        self.current_period_us = max(
            self.fast_period_us,
            self.current_period_us - self.acceleration_us,
        )


# Initialize outputs in a non-driving state before Wi-Fi or I2C startup.
bld_en = Pin(BLD_EN_GPIO, Pin.OUT, value=0)     # transistor off -> STOP
bld_brk = Pin(BLD_BRK_GPIO, Pin.OUT, value=0)  # transistor off -> BRAKE
bld_fr = Pin(BLD_FR_GPIO, Pin.OUT, value=0)    # tested CW state

steer_step = Pin(STEER_STEP_GPIO, Pin.OUT, value=0)
steer_dir = Pin(STEER_DIR_GPIO, Pin.OUT, value=0)
brake_step = Pin(BRAKE_STEP_GPIO, Pin.OUT, value=0)
brake_dir = Pin(BRAKE_DIR_GPIO, Pin.OUT, value=0)
brake_enable = Pin(
    BRAKE_EN_GPIO,
    Pin.OUT,
    value=BRAKE_ENABLE_LEVEL,
)

steering_limit_degrees = STEER_DEFAULT_LIMIT_DEG
steering_pulses_per_rev = STEER_PULSES_PER_REV
steering_speed_pps = STEER_DEFAULT_PPS
steering = StepperAxis(
    steer_step,
    steer_dir,
    STEER_PULSES_PER_DEG,
    -steering_limit_degrees,
    steering_limit_degrees,
    STEER_RIGHT_DIR,
    STEER_START_PERIOD_US,
    STEER_FAST_PERIOD_US,
    STEER_ACCEL_US_PER_STEP,
)
brake = StepperAxis(
    brake_step,
    brake_dir,
    BRAKE_PULSES_PER_DEG,
    0.0,
    BRAKE_APPLY_DEG,
    BRAKE_APPLY_DIR,
    BRAKE_START_PERIOD_US,
    BRAKE_FAST_PERIOD_US,
    BRAKE_ACCEL_US_PER_STEP,
)


def bld_enable_run():
    bld_en.value(1)
    print("BLD EN -> RUN")


def bld_enable_stop():
    bld_en.value(0)
    print("BLD EN -> STOP")


def bld_brake_release():
    bld_brk.value(1)
    print("BLD BRK -> RELEASED")


def bld_brake_apply():
    bld_brk.value(0)
    print("BLD BRK -> APPLIED")


direction = "CW"


def set_bld_direction(requested):
    global direction
    direction = "CCW" if requested == "CCW" else "CW"
    bld_fr.value(1 if direction == "CCW" else 0)
    print("BLD DIRECTION ->", direction)


i2c = I2C(
    0,
    sda=Pin(SDA_GPIO),
    scl=Pin(SCL_GPIO),
    freq=100000,
)

devices = i2c.scan()
print("I2C:", [hex(device) for device in devices])

if GP_ADDR not in devices:
    bld_brake_apply()
    bld_enable_stop()
    raise RuntimeError("GP8630N NOT FOUND AT 0x58")

i2c.writeto_mem(GP_ADDR, REG_MODE, bytes((MODE_0_10V,)))
time.sleep_ms(100)

throttle_current = 0.0
throttle_target = 0.0
last_throttle_update_ms = time.ticks_ms()


def set_dac_voltage(voltage):
    global throttle_current
    voltage = clamp(float(voltage), 0.0, THROTTLE_MAX_V)
    value = int((voltage / 10.0) * 65535)
    i2c.writeto_mem(
        GP_ADDR,
        REG_DAC,
        bytes((value & 0xFF, (value >> 8) & 0xFF)),
    )
    throttle_current = voltage


def throttle_update():
    global last_throttle_update_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_throttle_update_ms) < THROTTLE_UPDATE_MS:
        return

    last_throttle_update_ms = now
    if throttle_current < throttle_target:
        set_dac_voltage(
            min(throttle_current + THROTTLE_RISE_STEP_V, throttle_target)
        )
    elif throttle_current > throttle_target:
        set_dac_voltage(
            max(throttle_current - THROTTLE_FALL_STEP_V, throttle_target)
        )


armed = False
brake_state = "APPLYING"
last_contact_ms = time.ticks_ms()


def throttle_zero_immediate():
    global throttle_target
    throttle_target = 0.0
    try:
        set_dac_voltage(0.0)
        return True
    except Exception as error:
        print("DAC ZERO FAILED:", error)
        return False


def command_brake_apply():
    global brake_state
    # Hardware stop/brake commands must not depend on a successful I2C write.
    bld_brake_apply()
    bld_enable_stop()
    brake_enable.value(BRAKE_ENABLE_LEVEL)
    brake.set_target(BRAKE_APPLY_DEG)
    brake_state = "APPLYING"
    throttle_zero_immediate()
    print("PHYSICAL BRAKE -> APPLY")


def command_brake_release():
    global brake_state
    bld_brake_apply()
    bld_enable_stop()

    if not throttle_zero_immediate():
        command_brake_apply()
        return False

    brake_enable.value(BRAKE_ENABLE_LEVEL)
    brake.set_target(0.0)
    brake_state = "RELEASING"
    print("PHYSICAL BRAKE -> RELEASE")
    return True


def update_brake_transition():
    global brake_state
    if not brake.at_target():
        return

    if brake_state == "APPLYING":
        brake_state = "APPLIED"
    elif brake_state == "RELEASING":
        brake_state = "RELEASED"
        if armed:
            bld_brake_release()
            bld_enable_run()


def safe_stop(reason):
    global armed
    armed = False
    steering.hold()
    command_brake_apply()
    print("!!!", reason, "!!!")


set_dac_voltage(0.0)
set_bld_direction("CW")
bld_brake_apply()
bld_enable_stop()
brake.set_target(BRAKE_APPLY_DEG)


HTML = r"""<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>AutoKart RC</title>
<style>
:root{--bg:#090c12;--panel:#141a24;--line:#273143;--text:#f4f7fb;--muted:#9eabc0;--blue:#2583ff;--green:#19b66a;--red:#ec3547;--orange:#ee912f}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(150deg,#07101b,#11141c 55%,#090b10);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,sans-serif;touch-action:manipulation}
.wrap{width:min(760px,100%);margin:auto;padding:14px}.head{display:flex;justify-content:space-between;align-items:center;gap:12px;margin:4px 2px 14px}.title{font-size:clamp(23px,6vw,34px);font-weight:850}.tag{color:var(--muted);font-size:12px}.status{padding:9px 13px;border-radius:999px;background:#33151a;color:#ff7c88;font-weight:800;font-size:13px;white-space:nowrap}.status.on{background:#0e3627;color:#67e9aa}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.card{background:rgba(20,26,36,.96);border:1px solid var(--line);border-radius:18px;padding:16px;box-shadow:0 12px 28px #0005}.wide{grid-column:1/-1}.label{color:var(--muted);font-size:12px;font-weight:800;letter-spacing:.12em}.value{font-size:clamp(30px,9vw,48px);font-weight:850;margin:6px 0}.small{font-size:14px;color:var(--muted)}
.row{display:flex;gap:9px;flex-wrap:wrap}.row>*{flex:1}button{border:0;border-radius:13px;min-height:52px;padding:12px;color:white;font-size:16px;font-weight:800;background:#384357;cursor:pointer;touch-action:manipulation}button:active{transform:scale(.97)}.blue{background:var(--blue)}.green{background:var(--green)}.red{background:var(--red)}.orange{background:var(--orange)}.selected{outline:3px solid #fff8;outline-offset:1px}
input[type=range]{width:100%;height:42px;accent-color:var(--blue)}.ticks{display:flex;justify-content:space-between;color:var(--muted);font-size:12px}.warn{color:#ffbd6a;font-size:12px;line-height:1.45;margin-top:12px}.panic{width:100%;min-height:66px;font-size:20px;margin-top:12px;background:var(--red)}
.config{display:grid;grid-template-columns:1fr 1fr;gap:9px;margin-top:9px}.config label{color:var(--muted);font-size:12px;text-align:left}.config input,.config select{display:block;width:100%;margin-top:5px;padding:11px;border:1px solid var(--line);border-radius:10px;background:#0c1119;color:white;font-size:16px}.message{min-height:24px;margin:10px 0 0;padding:7px 10px;border-radius:9px;color:#9eabc0;font-size:13px}.message.ok{background:#0e3627;color:#67e9aa}.message.error{background:#3b171c;color:#ff8892}
@media(max-width:620px){.grid{grid-template-columns:1fr}.wide{grid-column:auto}.card{padding:14px}.wrap{padding:10px}.head{align-items:flex-start}.row button{min-width:29%}}
</style></head><body><main class="wrap">
<header class="head"><div><div class="title">AutoKart RC</div><div class="tag">ESP32-S3 bench controller · max throttle 3.0 V</div></div><div id="status" class="status">DISARMED</div></header>
<section class="grid">
<div class="card wide"><div class="label">STEERING</div><div class="value"><span id="steerNow">0.0</span>&deg;</div><div class="small">Target <span id="steerTarget">0.0</span>&deg; · Limit &plusmn;<span id="limitText">30</span>&deg;</div>
<div class="row" style="margin-top:12px"><button id="limit30" class="selected" onclick="setLimit(30)">&plusmn;30&deg;</button><button id="limit60" onclick="setLimit(60)">&plusmn;60&deg;</button></div>
<div class="row" style="margin-top:9px"><button class="blue" onclick="steerSide(-1)">&#9664; LEFT</button><button onclick="steer(0)">CENTER</button><button class="blue" onclick="steerSide(1)">RIGHT &#9654;</button></div>
<button style="width:100%;margin-top:9px" onclick="calibrateCenter()">CALIBRATE CURRENT POSITION AS CENTER</button>
<div class="config"><label>Steering pulses/revolution<input id="steerPpr" type="number" min="200" max="20000" step="100" value="400"></label><label>Steering pulse rate<select id="steerPps" onchange="setSteerSpeed()"><option value="500">500 pps</option><option value="1000">1000 pps</option><option value="1500" selected>1500 pps</option><option value="2000">2000 pps</option><option value="2500">2500 pps</option></select></label></div>
<button style="width:100%;margin-top:9px" onclick="setSteerPulses()">APPLY PULSES/REV</button>
<div id="message" class="message">Ready</div></div>

<div class="card"><div class="label">BRAKE</div><div id="brakeState" class="value" style="font-size:30px">APPLIED</div><div class="row"><button class="red" onclick="brakeOn()">APPLY</button><button class="green" onclick="brakeOff()">RELEASE</button></div><div class="warn">Open-loop commanded position. There is no brake encoder.</div></div>

<div class="card"><div class="label">DRIVE</div><div class="value"><span id="volts">0.00</span> V</div><div class="row"><button class="green" onclick="arm()">ARM</button><button class="red" onclick="disarm()">DISARM</button></div><div class="row" style="margin-top:9px"><button id="cw" class="selected" onclick="setDirection('CW')">CW</button><button id="ccw" onclick="setDirection('CCW')">CCW</button></div></div>

<div class="card wide"><div class="label">THROTTLE COMMAND</div><input id="throttle" type="range" min="0" max="300" step="1" value="0"><div class="ticks"><span>0 V</span><span>1.5 V</span><span>3.0 V</span></div><div class="small">Applied output: <span id="actualVolts">0.00</span> V</div></div>
</section>
<button class="panic" onclick="emergencyStop()">SAFE STOP + BRAKE</button>
<p class="warn">WHEELS-OFF-GROUND TEST ONLY. Mechanically center steering and release the physical brake before boot. Steering and brake positions are open-loop estimates.</p>
</main><script>
let armed=false, steerLimit=30, pendingThrottle=0, throttleTimer=null;
const throttle=document.getElementById('throttle');
function request(path){return fetch(path,{cache:'no-store'}).catch(()=>null)}
function setStatus(text,on){const e=document.getElementById('status');e.textContent=text;e.classList.toggle('on',on)}
function showMessage(text,ok){const e=document.getElementById('message');e.textContent=text;e.className='message '+(ok?'ok':'error')}
async function action(path){const r=await request(path);if(!r){showMessage('No response from ESP32',false);return false}const text=await r.text();showMessage((r.ok?'OK: ':'ERROR: ')+text,r.ok);return r.ok}
function setLimit(value){steerLimit=value;document.getElementById('limitText').textContent=value;document.getElementById('limit30').classList.toggle('selected',value===30);document.getElementById('limit60').classList.toggle('selected',value===60);request('/steer_limit?deg='+value)}
function steer(value){request('/steer?deg='+value)}
function steerSide(sign){steer(sign*steerLimit)}
async function calibrateCenter(){if(!confirm('Steering is physically centered. Set this position to 0 degrees without movement?'))return;await action('/calibrate_center')}
async function setSteerPulses(){const value=Math.round(Number(document.getElementById('steerPpr').value));await action('/steer_pulses?ppr='+value)}
async function setSteerSpeed(){const value=Math.round(Number(document.getElementById('steerPps').value));await action('/steer_speed?pps='+value)}
function arm(){throttle.value=0;pendingThrottle=0;document.getElementById('volts').textContent='0.00';request('/arm')}
function disarm(){throttle.value=0;pendingThrottle=0;document.getElementById('volts').textContent='0.00';request('/disarm')}
function brakeOn(){throttle.value=0;pendingThrottle=0;document.getElementById('volts').textContent='0.00';request('/brake?state=apply')}
function brakeOff(){request('/brake?state=release')}
function emergencyStop(){throttle.value=0;pendingThrottle=0;document.getElementById('volts').textContent='0.00';request('/stop')}
function setDirection(value){request('/direction?d='+value).then(r=>{if(r&&r.ok){document.getElementById('cw').classList.toggle('selected',value==='CW');document.getElementById('ccw').classList.toggle('selected',value==='CCW')}})}
function sendThrottle(){throttleTimer=null;if(!armed)return;request('/throttle?v='+pendingThrottle.toFixed(2))}
throttle.addEventListener('input',()=>{pendingThrottle=Number(throttle.value)/100;document.getElementById('volts').textContent=pendingThrottle.toFixed(2);if(throttleTimer===null)throttleTimer=setTimeout(sendThrottle,45)});
['change','pointerup','touchend'].forEach(name=>throttle.addEventListener(name,()=>{pendingThrottle=Number(throttle.value)/100;sendThrottle()}));
function update(){request('/status').then(r=>r&&r.ok?r.text():null).then(text=>{if(!text)return;const p=text.split('|');armed=p[0]==='ARMED';const liveLimit=Number(p[3]);if(liveLimit===30||liveLimit===60){steerLimit=liveLimit;document.getElementById('limitText').textContent=liveLimit;document.getElementById('limit30').classList.toggle('selected',liveLimit===30);document.getElementById('limit60').classList.toggle('selected',liveLimit===60)}setStatus(armed?'ARMED':'DISARMED',armed);document.getElementById('steerNow').textContent=Number(p[1]).toFixed(1);document.getElementById('steerTarget').textContent=Number(p[2]).toFixed(1);document.getElementById('brakeState').textContent=p[4];document.getElementById('actualVolts').textContent=Number(p[5]).toFixed(2);const pprInput=document.getElementById('steerPpr');if(p.length>8&&document.activeElement!==pprInput)pprInput.value=p[8];if(p.length>9)document.getElementById('steerPps').value=p[9];if(!armed||p[4]!=='RELEASED'){throttle.value=0;pendingThrottle=0;document.getElementById('volts').textContent='0.00'}document.getElementById('cw').classList.toggle('selected',p[7]==='CW');document.getElementById('ccw').classList.toggle('selected',p[7]==='CCW')})}
setInterval(update,300);update();
window.addEventListener('pagehide',()=>navigator.sendBeacon('/stop'));
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


def send_all(sock, data):
    view = memoryview(data)
    while view:
        sent = sock.send(view)
        if sent is None:
            return
        view = view[sent:]


def response(client, body, content_type="text/plain", status="200 OK"):
    if isinstance(body, str):
        body = body.encode()
    header = (
        "HTTP/1.1 " + status + "\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Content-Length: " + str(len(body)) + "\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n"
    ).encode()
    send_all(client, header)
    send_all(client, body)


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
server.listen(3)
server.settimeout(0)

print()
print("========================================")
print(" AUTOKART RC S3 TEST READY")
print("========================================")
print("Firmware :", FIRMWARE_NAME, FIRMWARE_VERSION)
print("Wi-Fi    :", SSID)
print("Password :", PASSWORD)
print("Open     : http://" + ip_address)
print("Steering : STEP4 DIR5, default +/-30, optional +/-60")
print("Brake    : STEP38 DIR39 ENA40, apply", BRAKE_APPLY_DEG, "deg")
print("Throttle : GP8630N SDA1 SCL2, maximum", THROTTLE_MAX_V, "V")
print("BLD750   : BRK13 EN14 F/R15")
print("BOOT ASSUMPTION: STEERING CENTERED, BRAKE RELEASED")
print("========================================")


try:
    while True:
        steering.update()
        brake.update()
        throttle_update()
        update_brake_transition()

        now = time.ticks_ms()
        if armed and time.ticks_diff(now, last_contact_ms) > COMMAND_TIMEOUT_MS:
            safe_stop("WEB WATCHDOG TIMEOUT")

        try:
            client, remote = server.accept()
        except OSError:
            time.sleep_us(100)
            continue

        try:
            request = client.recv(1200)
            if not request:
                continue

            first_line = request.decode("utf-8", "ignore").split("\r\n", 1)[0]
            parts = first_line.split(" ")
            if len(parts) < 2:
                response(client, "BAD REQUEST", status="400 Bad Request")
                continue

            path = parts[1]

            if path == "/":
                response(client, HTML, "text/html")

            elif path.startswith("/status"):
                if armed:
                    last_contact_ms = time.ticks_ms()
                body = (
                    "{}|{:.2f}|{:.2f}|{:.0f}|{}|{:.2f}|{:.2f}|{}|{}|{}"
                ).format(
                    "ARMED" if armed else "DISARMED",
                    steering.current_degrees(),
                    steering.target_degrees(),
                    steering_limit_degrees,
                    brake_state,
                    throttle_current,
                    throttle_target,
                    direction,
                    steering_pulses_per_rev,
                    steering_speed_pps,
                )
                response(client, body)

            elif path.startswith("/arm"):
                armed = True
                last_contact_ms = time.ticks_ms()
                if command_brake_release():
                    response(client, "RELEASING BRAKE")
                else:
                    armed = False
                    response(client, "DAC FAULT", status="503 Service Unavailable")

            elif path.startswith("/disarm"):
                safe_stop("REMOTE DISARM")
                response(client, "DISARMED")

            elif path.startswith("/stop"):
                safe_stop("REMOTE SAFE STOP")
                response(client, "STOPPED")

            elif path.startswith("/steer_limit"):
                requested_limit = query_number(
                    path,
                    "deg",
                    STEER_DEFAULT_LIMIT_DEG,
                )
                if requested_limit not in (
                    STEER_DEFAULT_LIMIT_DEG,
                    STEER_EXTENDED_LIMIT_DEG,
                ):
                    response(client, "USE 30 OR 60", status="400 Bad Request")
                    continue
                steering_limit_degrees = requested_limit
                steering.set_limits(-requested_limit, requested_limit)
                response(client, "OK")

            elif path.startswith("/steer_pulses"):
                if armed:
                    response(client, "DISARM FIRST", status="409 Conflict")
                    continue
                if not steering.at_target() or steering.current_steps != 0:
                    response(
                        client,
                        "CALIBRATE CENTER FIRST",
                        status="409 Conflict",
                    )
                    continue
                requested_ppr = round(
                    query_number(path, "ppr", steering_pulses_per_rev)
                )
                if requested_ppr < 200 or requested_ppr > 20000:
                    response(
                        client,
                        "PULSES/REV RANGE IS 200-20000",
                        status="400 Bad Request",
                    )
                    continue
                steering_pulses_per_rev = requested_ppr
                steering.set_pulses_per_degree(
                    requested_ppr * STEER_GEAR_RATIO / 360.0
                )
                print("STEERING PULSES/REV ->", steering_pulses_per_rev)
                response(client, "PULSES/REV = " + str(steering_pulses_per_rev))

            elif path.startswith("/steer_speed"):
                requested_pps = round(
                    query_number(path, "pps", steering_speed_pps)
                )
                if requested_pps < 200 or requested_pps > 3000:
                    response(
                        client,
                        "PULSE RATE RANGE IS 200-3000 PPS",
                        status="400 Bad Request",
                    )
                    continue
                steering_speed_pps = requested_pps
                steering.set_speed_pps(requested_pps)
                print("STEERING SPEED ->", steering_speed_pps, "pps")
                response(client, "SPEED = " + str(steering_speed_pps) + " PPS")

            elif path.startswith("/steer"):
                if not armed:
                    response(client, "ARM FIRST", status="409 Conflict")
                    continue
                angle = query_number(path, "deg", 0.0)
                steering.set_target(angle)
                last_contact_ms = time.ticks_ms()
                print("STEERING TARGET ->", steering.target_degrees(), "deg")
                response(client, "OK")

            elif path.startswith("/calibrate_center"):
                if armed:
                    response(client, "DISARM FIRST", status="409 Conflict")
                    continue
                if brake_state != "APPLIED":
                    response(
                        client,
                        "WAIT FOR BRAKE APPLIED",
                        status="409 Conflict",
                    )
                    continue
                steering.calibrate_zero()
                print("STEERING CENTER CALIBRATED; NO MOVEMENT")
                response(client, "CENTER = 0; NO MOVEMENT")

            elif path.startswith("/brake"):
                if not armed:
                    response(client, "ARM FIRST", status="409 Conflict")
                    continue
                requested_state = query_text(path, "state", "apply")
                if requested_state == "apply":
                    command_brake_apply()
                elif requested_state == "release":
                    if not command_brake_release():
                        response(
                            client,
                            "DAC FAULT",
                            status="503 Service Unavailable",
                        )
                        continue
                else:
                    response(client, "BAD BRAKE STATE", status="400 Bad Request")
                    continue
                last_contact_ms = time.ticks_ms()
                response(client, "OK")

            elif path.startswith("/throttle"):
                requested_voltage = clamp(
                    query_number(path, "v", 0.0),
                    0.0,
                    THROTTLE_MAX_V,
                )
                if armed and brake_state == "RELEASED":
                    throttle_target = requested_voltage
                    last_contact_ms = time.ticks_ms()
                    print("THROTTLE TARGET ->", round(throttle_target, 2), "V")
                    response(client, "OK")
                else:
                    throttle_target = 0.0
                    response(client, "BRAKE NOT RELEASED", status="409 Conflict")

            elif path.startswith("/direction"):
                requested_direction = query_text(path, "d", "CW")
                if requested_direction not in ("CW", "CCW"):
                    response(client, "BAD DIRECTION", status="400 Bad Request")
                    continue
                if armed or throttle_current > 0.01 or throttle_target > 0.01:
                    response(client, "DISARM BEFORE DIRECTION CHANGE", status="409 Conflict")
                    continue
                set_bld_direction(requested_direction)
                response(client, "OK")

            else:
                response(client, "NOT FOUND", status="404 Not Found")

        except Exception as error:
            print("HTTP ERROR:", error)
            try:
                response(client, "ERROR", status="500 Internal Server Error")
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
    deadline = time.ticks_add(time.ticks_ms(), 1500)
    while not brake.at_target() and time.ticks_diff(deadline, time.ticks_ms()) > 0:
        brake.update()
    brake_enable.value(BRAKE_DISABLE_LEVEL)
    steer_step.value(0)
    brake_step.value(0)
    try:
        server.close()
    except Exception:
        pass

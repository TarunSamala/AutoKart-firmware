"""AutoKart full RC bench test without encoders.

ESP32-S3 hardware:
  Steering R86mini: STEP GPIO4, DIR GPIO5
  Brake TB6600: STEP GPIO38, DIR GPIO39, ENA GPIO40
  GP8630N: SDA GPIO1, SCL GPIO2, address 0x58
  BLD750 transistor stages: BRK GPIO13, EN GPIO14, F/R GPIO15

Steering and brake positions are open-loop estimates. Before boot, place the
steering at mechanical center and the brake mechanism at its released/home
position. Use only with driven wheels lifted clear of the ground.
"""

import network
import socket
import time
from machine import I2C, Pin


SSID = "AutoKart-RC-Test"
PASSWORD = "12345678"
COMMAND_TIMEOUT_MS = 1600

STEER_STEP_GPIO = 4
STEER_DIR_GPIO = 5
STEER_RIGHT_DIR = 1
STEER_PULSES_PER_REV = 3200
STEER_PULSES_PER_DEG = STEER_PULSES_PER_REV / 360.0
STEER_LIMIT_DEG = 30.0
STEER_START_PPS = 100
STEER_MAX_PPS = 350

BRAKE_STEP_GPIO = 38
BRAKE_DIR_GPIO = 39
BRAKE_EN_GPIO = 40
# GPIO level used by the tested interface for clockwise brake application.
BRAKE_APPLY_DIR = 1
BRAKE_ENABLE_LEVEL = 0
BRAKE_DISABLE_LEVEL = 1
BRAKE_PULSES_PER_REV = 3200
BRAKE_PULSES_PER_DEG = BRAKE_PULSES_PER_REV / 360.0
BRAKE_APPLY_DEG = 30.0

SDA_GPIO = 1
SCL_GPIO = 2
GP_ADDR = 0x58
GP_MODE_REG = 0x01
GP_DAC_REG = 0x02
GP_MODE_0_10V = 0x1C
THROTTLE_MAX_V = 3.0
THROTTLE_RISE_VPS = 1.5
THROTTLE_FALL_VPS = 7.5
THROTTLE_UPDATE_MS = 20

BLD_BRK_GPIO = 13
BLD_EN_GPIO = 14
BLD_FR_GPIO = 15

STEP_HIGH_US = 20


def clamp(value, low, high):
    return max(low, min(high, value))


class StepperAxis:
    def __init__(self, step_gpio, dir_gpio, pulses_per_degree,
                 positive_direction, start_period_us, fast_period_us):
        self.step = Pin(step_gpio, Pin.OUT, value=0)
        self.direction = Pin(dir_gpio, Pin.OUT, value=0)
        self.pulses_per_degree = pulses_per_degree
        self.positive_direction = positive_direction
        self.start_period_us = start_period_us
        self.fast_period_us = fast_period_us
        self.current_period_us = start_period_us
        self.current_steps = 0
        self.target_steps = 0
        self.last_step_us = time.ticks_us()
        self.last_direction = 0

    def degrees(self):
        return self.current_steps / self.pulses_per_degree

    def target_degrees(self):
        return self.target_steps / self.pulses_per_degree

    def set_target(self, degrees):
        self.target_steps = round(float(degrees) * self.pulses_per_degree)

    def hold(self):
        self.target_steps = self.current_steps

    def calibrate_zero(self):
        self.current_steps = 0
        self.target_steps = 0

    def set_pulses_per_revolution(self, pulses_per_revolution):
        self.pulses_per_degree = float(pulses_per_revolution) / 360.0

    def set_speed(self, start_pps, maximum_pps):
        self.start_period_us = 1_000_000 // int(start_pps)
        self.fast_period_us = 1_000_000 // int(maximum_pps)
        self.current_period_us = self.start_period_us

    def at_target(self):
        return self.current_steps == self.target_steps

    def update(self):
        if self.at_target():
            self.current_period_us = self.start_period_us
            self.last_direction = 0
            return

        move_direction = 1 if self.target_steps > self.current_steps else -1
        if move_direction != self.last_direction:
            self.last_direction = move_direction
            self.current_period_us = self.start_period_us
            self.direction.value(
                self.positive_direction
                if move_direction > 0
                else 1 - self.positive_direction
            )

        now = time.ticks_us()
        if time.ticks_diff(now, self.last_step_us) < self.current_period_us:
            return
        self.step.value(1)
        time.sleep_us(STEP_HIGH_US)
        self.step.value(0)
        self.current_steps += move_direction
        self.last_step_us = now
        self.current_period_us = max(
            self.fast_period_us,
            self.current_period_us - 12,
        )


# Safe output states are established before I2C or Wi-Fi starts.
bld_en = Pin(BLD_EN_GPIO, Pin.OUT, value=0)       # STOP
bld_brk = Pin(BLD_BRK_GPIO, Pin.OUT, value=0)    # BRAKE APPLIED
bld_fr = Pin(BLD_FR_GPIO, Pin.OUT, value=0)      # CW
brake_enable = Pin(BRAKE_EN_GPIO, Pin.OUT, value=BRAKE_ENABLE_LEVEL)

steering = StepperAxis(
    STEER_STEP_GPIO,
    STEER_DIR_GPIO,
    STEER_PULSES_PER_DEG,
    STEER_RIGHT_DIR,
    1_000_000 // STEER_START_PPS,
    1_000_000 // STEER_MAX_PPS,
)
brake = StepperAxis(
    BRAKE_STEP_GPIO,
    BRAKE_DIR_GPIO,
    BRAKE_PULSES_PER_DEG,
    BRAKE_APPLY_DIR,
    1800,
    900,
)

i2c = I2C(0, sda=Pin(SDA_GPIO), scl=Pin(SCL_GPIO), freq=100000)
devices = i2c.scan()
print("I2C:", [hex(device) for device in devices])
if GP_ADDR not in devices:
    raise RuntimeError("GP8630N NOT FOUND AT 0x58")
i2c.writeto_mem(GP_ADDR, GP_MODE_REG, bytes((GP_MODE_0_10V,)))

armed = False
direction = "CW"
steering_limit = STEER_LIMIT_DEG
steering_ppr = STEER_PULSES_PER_REV
steering_start_pps = STEER_START_PPS
steering_max_pps = STEER_MAX_PPS
brake_state = "APPLYING"
throttle_current = 0.0
throttle_target = 0.0
last_throttle_ms = time.ticks_ms()
last_contact_ms = time.ticks_ms()


def write_voltage(voltage):
    global throttle_current
    voltage = clamp(float(voltage), 0.0, THROTTLE_MAX_V)
    raw = int(voltage / 10.0 * 65535)
    i2c.writeto_mem(GP_ADDR, GP_DAC_REG, bytes((raw & 0xFF, raw >> 8)))
    throttle_current = voltage


def throttle_zero():
    global throttle_target
    throttle_target = 0.0
    write_voltage(0.0)


def throttle_update():
    global last_throttle_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_throttle_ms) < THROTTLE_UPDATE_MS:
        return
    last_throttle_ms = now
    if throttle_current < throttle_target:
        step = THROTTLE_RISE_VPS * THROTTLE_UPDATE_MS / 1000.0
        write_voltage(min(throttle_current + step, throttle_target))
    elif throttle_current > throttle_target:
        step = THROTTLE_FALL_VPS * THROTTLE_UPDATE_MS / 1000.0
        write_voltage(max(throttle_current - step, throttle_target))


def apply_brake():
    global brake_state
    throttle_zero()
    bld_en.value(0)
    bld_brk.value(0)
    brake_enable.value(BRAKE_ENABLE_LEVEL)
    brake.set_target(BRAKE_APPLY_DEG)
    brake_state = "APPLYING"
    print("BRAKE -> APPLYING | BLDC -> STOP")


def release_brake():
    global brake_state
    throttle_zero()
    bld_en.value(0)
    bld_brk.value(0)
    brake_enable.value(BRAKE_ENABLE_LEVEL)
    brake.set_target(0.0)
    brake_state = "RELEASING"
    print("BRAKE -> RELEASING")


def update_brake():
    global brake_state
    if not brake.at_target():
        return
    if brake_state == "APPLYING":
        brake_state = "APPLIED"
    elif brake_state == "RELEASING":
        brake_state = "RELEASED"
        if armed:
            bld_brk.value(1)
            bld_en.value(1)
            print("BLDC -> RUN")


def safe_stop(reason):
    global armed
    armed = False
    steering.hold()
    apply_brake()
    print("SAFE STOP:", reason)


write_voltage(0.0)
apply_brake()


HTML = r"""<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>AutoKart RC</title><style>
:root{--bg:#090c12;--card:#151b25;--line:#2a3548;--text:#f4f7fb;--muted:#a4afc1;--blue:#2684ff;--green:#18ad66;--red:#e93849}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui;touch-action:manipulation}.wrap{max-width:760px;margin:auto;padding:12px}.head{display:flex;justify-content:space-between;align-items:center;gap:10px}.state{padding:8px 12px;border-radius:20px;background:#47191f;color:#ff8c96;font-weight:800;white-space:nowrap}.state.on{background:#12422e;color:#75edb3}.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:12px}.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:15px;min-width:0}.wide{grid-column:1/-1}.label{font-size:12px;color:var(--muted);font-weight:800;letter-spacing:.1em}.value{font-size:38px;font-weight:850;margin:7px 0}.row{display:flex;flex-wrap:wrap;gap:8px;margin-top:9px}.row>*{flex:1 1 120px;min-width:0}button{min-height:48px;border:0;border-radius:11px;background:#39465c;color:white;font-size:15px;font-weight:800;cursor:pointer;touch-action:manipulation}button:active{transform:scale(.98)}button:disabled{opacity:.55;cursor:wait}button:focus-visible,input:focus-visible{outline:3px solid #6aabff;outline-offset:2px}.blue{background:var(--blue)}.green{background:var(--green)}.red{background:var(--red)}input{width:100%;padding:11px;border-radius:9px;border:1px solid var(--line);background:#0d121a;color:white;font-size:16px;min-height:46px}input[type=range]{padding:0;min-height:52px;accent-color:var(--blue)}.small{color:var(--muted);font-size:13px}.message{position:sticky;bottom:8px;min-height:42px;margin-top:12px;padding:11px;border:1px solid var(--line);border-radius:11px;background:#111925;color:var(--muted);font-size:13px;z-index:2}.message.ok{background:#103a29;color:#75edb3}.message.error{background:#47191f;color:#ff969f}.stop{width:100%;margin-top:12px;min-height:64px;background:var(--red);font-size:20px}@media(max-width:600px){.grid{grid-template-columns:1fr}.wide{grid-column:auto}.wrap{padding:9px}.card{padding:13px}.head h1{font-size:25px;margin-bottom:5px}.row>*{flex-basis:calc(50% - 8px)}.row input{flex-basis:100%}}
</style></head><body><main class="wrap"><div class="head"><div><h1>AutoKart RC</h1><div class="small">No encoders · open-loop bench test</div></div><div id="state" class="state">DISARMED</div></div><section class="grid">
<div class="card wide"><div class="label">STEERING</div><div class="value"><span id="angle">0.0</span>&deg;</div><div class="small">Commanded estimate only · target <span id="target">0.0</span>&deg;</div><div class="row"><button onclick="limit(30)">&plusmn;30&deg;</button><button onclick="limit(60)">&plusmn;60&deg;</button></div><div class="row"><button class="blue" onclick="steer(-steeringLimit)">LEFT</button><button onclick="steer(0)">CENTER</button><button class="blue" onclick="steer(steeringLimit)">RIGHT</button></div><div class="row"><input id="custom" type="number" min="-60" max="60" value="0"><button onclick="steer(ui.custom.value)">GO TO ANGLE</button></div><div class="row"><button onclick="command('/hold')">HOLD</button><button onclick="calibrate()">CALIBRATE CENTER</button></div><div class="small" style="margin-top:12px">R86mini pulse configuration — apply only while disarmed and centered</div><div class="row"><input id="ppr" type="number" min="400" max="51200" value="3200" title="Pulses per revolution"><input id="startPps" type="number" min="20" max="1000" value="100" title="Start pulses per second"><input id="maxPps" type="number" min="20" max="3000" value="350" title="Maximum pulses per second"></div><div class="small">PPR · Start PPS · Maximum PPS</div><button style="width:100%;margin-top:9px" onclick="steerSettings()">APPLY STEERING PULSE SETTINGS</button></div>
<div class="card"><div class="label">BRAKE · 30&deg; CLOCKWISE</div><div id="brake" class="value" style="font-size:27px">APPLIED</div><div class="small">Open-loop position: <span id="brakeAngle">0.0</span>&deg;</div><div class="row"><button class="red" onclick="command('/brake?s=apply')">APPLY 30&deg; CW</button><button class="green" onclick="command('/brake?s=release')">RELEASE TO 0&deg;</button></div><button style="width:100%;margin-top:9px" onclick="calibrateBrake()">CALIBRATE CURRENT BRAKE POSITION AS RELEASED / 0&deg;</button></div>
<div class="card"><div class="label">BLDC DRIVE</div><div class="value"><span id="actual">0.00</span> V</div><div class="row"><button class="green" onclick="command('/arm')">ARM</button><button class="red" onclick="zeroThrottle();command('/disarm')">DISARM</button></div><div class="row"><button onclick="command('/direction?d=CW')">CW</button><button onclick="command('/direction?d=CCW')">CCW</button></div></div>
<div class="card wide"><div class="label">THROTTLE · MAXIMUM 3.0 V</div><input id="throttle" type="range" min="0" max="300" value="0"><div class="small">Requested: <span id="requested">0.00</span> V</div><div class="row"><button onclick="setThrottle(0)">0 V</button><button onclick="setThrottle(1)">1 V</button><button onclick="setThrottle(2)">2 V</button><button onclick="setThrottle(3)">3 V</button></div></div>
</section><button class="stop" onclick="zeroThrottle();command('/stop')">SAFE STOP + BRAKE</button><div id="message" class="message">Controller ready</div></main><script>
let steeringLimit=30;
let throttleTimer=null;
let pendingThrottle=0;
const byId=id=>document.getElementById(id);
const ui={state:byId('state'),angle:byId('angle'),target:byId('target'),brake:byId('brake'),brakeAngle:byId('brakeAngle'),actual:byId('actual'),requested:byId('requested'),throttle:byId('throttle'),custom:byId('custom'),ppr:byId('ppr'),startPps:byId('startPps'),maxPps:byId('maxPps'),message:byId('message')};
function showMessage(text,ok){ui.message.textContent=text;ui.message.className='message '+(ok?'ok':'error')}
async function request(path){try{return await fetch(path,{cache:'no-store'})}catch(error){return null}}
async function command(path){const response=await request(path);if(!response){showMessage('ESP32 did not respond',false);return false}const text=await response.text();showMessage(text||'OK',response.ok);return response.ok}
function limit(value){steeringLimit=value;ui.custom.min=-value;ui.custom.max=value;command('/limit?v='+value)}
function steer(value){command('/steer?v='+Math.max(-steeringLimit,Math.min(steeringLimit,Number(value))))}
function calibrate(){if(confirm('Steering is mechanically centered. Set current estimate to zero without movement?'))command('/calibrate')}
function calibrateBrake(){if(confirm('Brake is physically fully RELEASED. Set this position to 0 degrees without movement?'))command('/calibrate_brake')}
function steerSettings(){command('/steer_settings?ppr='+encodeURIComponent(ui.ppr.value)+'&start='+encodeURIComponent(ui.startPps.value)+'&max='+encodeURIComponent(ui.maxPps.value))}
function zeroThrottle(){pendingThrottle=0;ui.throttle.value=0;ui.requested.textContent='0.00'}
function sendPendingThrottle(){if(throttleTimer!==null){clearTimeout(throttleTimer);throttleTimer=null}command('/throttle?v='+pendingThrottle.toFixed(2))}
function setThrottle(value){pendingThrottle=Number(value);ui.throttle.value=Math.round(pendingThrottle*100);ui.requested.textContent=pendingThrottle.toFixed(2);sendPendingThrottle()}
ui.throttle.addEventListener('input',()=>{pendingThrottle=Number(ui.throttle.value)/100;ui.requested.textContent=pendingThrottle.toFixed(2);if(throttleTimer===null)throttleTimer=setTimeout(sendPendingThrottle,45)});
['change','pointerup','touchend'].forEach(eventName=>ui.throttle.addEventListener(eventName,()=>{pendingThrottle=Number(ui.throttle.value)/100;sendPendingThrottle()}));
async function update(){const response=await request('/status');if(!response||!response.ok)return;const text=await response.text();const p=text.split('|');ui.state.textContent=p[0];ui.state.classList.toggle('on',p[0]==='ARMED');ui.angle.textContent=Number(p[1]).toFixed(1);ui.target.textContent=Number(p[2]).toFixed(1);ui.brake.textContent=p[3];ui.actual.textContent=Number(p[4]).toFixed(2);ui.brakeAngle.textContent=Number(p[9]).toFixed(1);if(document.activeElement!==ui.ppr&&p[6])ui.ppr.value=p[6];if(document.activeElement!==ui.startPps&&p[7])ui.startPps.value=p[7];if(document.activeElement!==ui.maxPps&&p[8])ui.maxPps.value=p[8];if(p[0]!=='ARMED'||p[3]!=='RELEASED')zeroThrottle()}
setInterval(update,300);update();addEventListener('pagehide',()=>navigator.sendBeacon('/stop'));
</script></body></html>"""


def query(path, key, default):
    try:
        return path.split(key + "=", 1)[1].split("&", 1)[0]
    except IndexError:
        return default


def send_all(client, data):
    view = memoryview(data)
    while view:
        sent = client.send(view)
        if sent is None:
            return
        view = view[sent:]


def reply(client, body, status="200 OK", content_type="text/plain"):
    if isinstance(body, str):
        body = body.encode()
    header = (
        "HTTP/1.1 " + status + "\r\nContent-Type: " + content_type
        + "\r\nContent-Length: " + str(len(body))
        + "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"
    ).encode()
    send_all(client, header)
    send_all(client, body)


ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=SSID, password=PASSWORD)
while not ap.active():
    time.sleep_ms(100)

server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("0.0.0.0", 80))
server.listen(3)
server.settimeout(0)

print("AutoKart full RC test ready")
print("Wi-Fi:", SSID, "Password:", PASSWORD)
print("Open: http://" + ap.ifconfig()[0])

try:
    while True:
        steering.update()
        brake.update()
        update_brake()
        throttle_update()

        if armed and time.ticks_diff(time.ticks_ms(), last_contact_ms) > COMMAND_TIMEOUT_MS:
            safe_stop("WEB WATCHDOG TIMEOUT")

        try:
            client, _ = server.accept()
        except OSError:
            time.sleep_us(100)
            continue

        try:
            request = client.recv(1000).decode("utf-8", "ignore")
            first = request.split("\r\n", 1)[0].split(" ")
            path = first[1] if len(first) > 1 else "/bad"

            if path == "/":
                reply(client, HTML, content_type="text/html")
            elif path.startswith("/status"):
                if armed:
                    last_contact_ms = time.ticks_ms()
                reply(client, "{}|{:.2f}|{:.2f}|{}|{:.2f}|{}|{}|{}|{}|{:.2f}".format(
                    "ARMED" if armed else "DISARMED",
                    steering.degrees(), steering.target_degrees(),
                    brake_state, throttle_current, direction,
                    steering_ppr, steering_start_pps, steering_max_pps,
                    brake.degrees(),
                ))
            elif path.startswith("/arm"):
                armed = True
                last_contact_ms = time.ticks_ms()
                release_brake()
                reply(client, "ARMED; RELEASING BRAKE")
            elif path.startswith("/disarm") or path.startswith("/stop"):
                safe_stop("REMOTE STOP")
                reply(client, "STOPPED")
            elif path.startswith("/limit"):
                requested = float(query(path, "v", "30"))
                if requested not in (30.0, 60.0):
                    reply(client, "USE 30 OR 60", "400 Bad Request")
                else:
                    steering_limit = requested
                    steering.set_target(clamp(
                        steering.target_degrees(), -requested, requested
                    ))
                    reply(client, "LIMIT SET")
            elif path.startswith("/steer_settings"):
                if armed:
                    reply(client, "DISARM FIRST", "409 Conflict")
                elif not steering.at_target() or steering.current_steps != 0:
                    reply(client, "CALIBRATE CENTER FIRST", "409 Conflict")
                else:
                    requested_ppr = int(float(query(path, "ppr", "3200")))
                    requested_start = int(float(query(path, "start", "100")))
                    requested_max = int(float(query(path, "max", "350")))
                    if requested_ppr < 400 or requested_ppr > 51200:
                        reply(client, "PPR RANGE 400-51200", "400 Bad Request")
                    elif requested_start < 20 or requested_start > 1000:
                        reply(client, "START PPS RANGE 20-1000", "400 Bad Request")
                    elif requested_max < requested_start or requested_max > 3000:
                        reply(client, "MAX PPS MUST BE START..3000", "400 Bad Request")
                    else:
                        steering_ppr = requested_ppr
                        steering_start_pps = requested_start
                        steering_max_pps = requested_max
                        steering.set_pulses_per_revolution(requested_ppr)
                        steering.set_speed(requested_start, requested_max)
                        print(
                            "STEERING ->",
                            requested_ppr,
                            "PPR",
                            requested_start,
                            "to",
                            requested_max,
                            "PPS",
                        )
                        reply(client, "STEERING PULSE SETTINGS APPLIED")
            elif path.startswith("/steer"):
                if not armed:
                    reply(client, "ARM FIRST", "409 Conflict")
                else:
                    requested = float(query(path, "v", "0"))
                    steering.set_target(clamp(
                        requested, -steering_limit, steering_limit
                    ))
                    last_contact_ms = time.ticks_ms()
                    reply(client, "STEERING SET")
            elif path.startswith("/hold"):
                steering.hold()
                reply(client, "STEERING HELD")
            elif path == "/calibrate":
                if armed:
                    reply(client, "DISARM FIRST", "409 Conflict")
                else:
                    steering.calibrate_zero()
                    reply(client, "CENTER CALIBRATED; NO MOVEMENT")
            elif path == "/calibrate_brake":
                if armed:
                    reply(client, "DISARM FIRST", "409 Conflict")
                elif not brake.at_target():
                    reply(client, "WAIT FOR BRAKE TO STOP", "409 Conflict")
                else:
                    brake.calibrate_zero()
                    brake_state = "RELEASED"
                    print("BRAKE RELEASE POSITION CALIBRATED; NO MOVEMENT")
                    reply(client, "BRAKE RELEASED POSITION = 0; NO MOVEMENT")
            elif path.startswith("/brake"):
                if query(path, "s", "apply") == "release":
                    if armed:
                        release_brake()
                        reply(client, "BRAKE RELEASING")
                    else:
                        reply(client, "ARM FIRST", "409 Conflict")
                else:
                    apply_brake()
                    reply(client, "BRAKE APPLYING")
            elif path.startswith("/direction"):
                requested = query(path, "d", "CW")
                if armed or throttle_current > 0.01:
                    reply(client, "DISARM BEFORE DIRECTION CHANGE", "409 Conflict")
                elif requested not in ("CW", "CCW"):
                    reply(client, "BAD DIRECTION", "400 Bad Request")
                else:
                    direction = requested
                    bld_fr.value(1 if requested == "CCW" else 0)
                    reply(client, "DIRECTION " + requested)
            elif path.startswith("/throttle"):
                requested = clamp(float(query(path, "v", "0")), 0, THROTTLE_MAX_V)
                if armed and brake_state == "RELEASED":
                    throttle_target = requested
                    last_contact_ms = time.ticks_ms()
                    reply(client, "THROTTLE SET")
                else:
                    throttle_target = 0.0
                    reply(client, "BRAKE NOT RELEASED", "409 Conflict")
            else:
                reply(client, "NOT FOUND", "404 Not Found")
        except Exception as error:
            print("HTTP ERROR:", error)
        finally:
            try:
                client.close()
            except Exception:
                pass
finally:
    safe_stop("CONTROLLER EXIT")
    brake_enable.value(BRAKE_DISABLE_LEVEL)
    server.close()

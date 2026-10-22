"""AutoKart simple Wi-Fi RC bench controller for ESP32-S3 MicroPython.

Functions:
  - steering: left/center/right, 2000 pulses from center = 45 degrees
  - traction: forward=CCW, reverse=CW, five requested DAC voltage gears
  - momentary brake: press applies, release disengages while armed
  - arm releases stepper and BLD750 brakes; disarm applies both brakes
  - steering AS5600 through PCA9548A channel 0
  - AS5047P BLDC encoder through SPI
  - brake feedback is deliberately SIMULATED from commanded step count

The simulated brake angle cannot detect missed steps or mechanical slip. This
is a wheels-off-ground bench controller, not production vehicle firmware.
"""

import network
import socket
import time
from machine import I2C, Pin, SPI


SSID = "AutoKart-RC"
PASSWORD = "12345678"
WATCHDOG_MS = 1800

# Steering R86mini: exactly 2000 pulses from center to either 45-degree side.
STEER_STEP_GPIO = 4
STEER_DIR_GPIO = 5
STEER_RIGHT_DIR = 1
STEER_LIMIT_DEG = 45.0
STEER_PULSES_AT_45 = 2000
STEER_PULSES_PER_DEG = STEER_PULSES_AT_45 / STEER_LIMIT_DEG
STEER_START_PPS = 180
STEER_MAX_PPS = 700

# Brake TB6600 and simulated encoder.
BRAKE_STEP_GPIO = 38
BRAKE_DIR_GPIO = 39
BRAKE_EN_GPIO = 40
BRAKE_APPLY_DIR = 1
BRAKE_ENABLE_LEVEL = 0
BRAKE_DISABLE_LEVEL = 1
BRAKE_PULSES_PER_REV = 3200
BRAKE_PULSES_PER_DEG = BRAKE_PULSES_PER_REV / 360.0
BRAKE_APPLY_DEG = 45.0

# Shared I2C: PCA9548A + GP8630N.
SDA_GPIO = 1
SCL_GPIO = 2
PCA_ADDR = 0x70
AS5600_ADDR = 0x36
GP_ADDR = 0x58
GP_MODE_REG = 0x01
GP_DAC_REG = 0x02
GP_MODE_0_10V = 0x1C
THROTTLE_MAX_V = 5.0

# BLD750 transistor stages.
BLD_BRK_GPIO = 13
BLD_EN_GPIO = 14
BLD_FR_GPIO = 15

# AS5047P SPI.
ENC_CS_GPIO = 16
ENC_SCK_GPIO = 18
ENC_MOSI_GPIO = 41
ENC_MISO_GPIO = 42

STEP_HIGH_US = 20


def clamp(value, low, high):
    return max(low, min(high, value))


class StepperAxis:
    def __init__(self, step_gpio, dir_gpio, pulses_per_degree,
                 positive_direction, start_pps, max_pps):
        self.step = Pin(step_gpio, Pin.OUT, value=0)
        self.direction = Pin(dir_gpio, Pin.OUT, value=0)
        self.pulses_per_degree = float(pulses_per_degree)
        self.positive_direction = positive_direction
        self.start_period_us = 1_000_000 // start_pps
        self.fast_period_us = 1_000_000 // max_pps
        self.period_us = self.start_period_us
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

    def at_target(self):
        return self.current_steps == self.target_steps

    def update(self):
        if self.at_target():
            self.period_us = self.start_period_us
            self.last_direction = 0
            return
        move = 1 if self.target_steps > self.current_steps else -1
        if move != self.last_direction:
            self.last_direction = move
            self.period_us = self.start_period_us
            self.direction.value(
                self.positive_direction if move > 0
                else 1 - self.positive_direction
            )
        now = time.ticks_us()
        if time.ticks_diff(now, self.last_step_us) < self.period_us:
            return
        self.step.value(1)
        time.sleep_us(STEP_HIGH_US)
        self.step.value(0)
        self.current_steps += move
        self.last_step_us = now
        self.period_us = max(self.fast_period_us, self.period_us - 10)


def parity_is_odd(value):
    return bool(bin(value & 0xFFFF).count("1") & 1)


def as5047_command(register):
    frame = 0x4000 | (register & 0x3FFF)
    if parity_is_odd(frame):
        frame |= 0x8000
    return frame


def as5047_bytes(frame):
    return bytes(((frame >> 8) & 0xFF, frame & 0xFF))


# Outputs enter their safe state before Wi-Fi or buses initialize.
bld_en = Pin(BLD_EN_GPIO, Pin.OUT, value=0)
bld_brk = Pin(BLD_BRK_GPIO, Pin.OUT, value=0)
bld_fr = Pin(BLD_FR_GPIO, Pin.OUT, value=1)  # selected forward/CCW
brake_enable = Pin(BRAKE_EN_GPIO, Pin.OUT, value=BRAKE_ENABLE_LEVEL)

steering = StepperAxis(
    STEER_STEP_GPIO, STEER_DIR_GPIO, STEER_PULSES_PER_DEG,
    STEER_RIGHT_DIR, STEER_START_PPS, STEER_MAX_PPS,
)
brake = StepperAxis(
    BRAKE_STEP_GPIO, BRAKE_DIR_GPIO, BRAKE_PULSES_PER_DEG,
    BRAKE_APPLY_DIR, 250, 900,
)

i2c = I2C(0, sda=Pin(SDA_GPIO), scl=Pin(SCL_GPIO), freq=100_000)
i2c_devices = i2c.scan()
pca_present = PCA_ADDR in i2c_devices
gp_present = GP_ADDR in i2c_devices
if gp_present:
    i2c.writeto_mem(GP_ADDR, GP_MODE_REG, bytes((GP_MODE_0_10V,)))

spi = SPI(
    1, baudrate=1_000_000, polarity=0, phase=1, bits=8,
    firstbit=SPI.MSB, sck=Pin(ENC_SCK_GPIO),
    mosi=Pin(ENC_MOSI_GPIO), miso=Pin(ENC_MISO_GPIO),
)
encoder_cs = Pin(ENC_CS_GPIO, Pin.OUT, value=1)

armed = False
direction = "FORWARD"
gear = 0
throttle_command = 0.0
brake_state = "APPLYING"
last_contact_ms = time.ticks_ms()
last_encoder_ms = 0

steer_encoder = {
    "online": False, "magnet": "NO SENSOR", "raw": -1,
    "angle": 0.0, "errors": 0,
}
bldc_encoder = {
    "online": False, "magnet": "NO SENSOR", "raw": -1,
    "angle": 0.0, "errors": 0,
}


def pca_select(channel):
    if not pca_present:
        return False
    i2c.writeto(PCA_ADDR, bytes((1 << channel,)))
    return True


def as5600_update():
    try:
        if not pca_select(0):
            steer_encoder["magnet"] = "PCA9548A MISSING"
            return
        status = i2c.readfrom_mem(AS5600_ADDR, 0x0B, 1)[0]
        data = i2c.readfrom_mem(AS5600_ADDR, 0x0E, 2)
        raw = ((data[0] << 8) | data[1]) & 0x0FFF
        if status & 0x08:
            magnet = "TOO STRONG"
        elif status & 0x10:
            magnet = "TOO WEAK"
        elif status & 0x20:
            magnet = "OK"
        else:
            magnet = "NO MAGNET"
        steer_encoder.update(
            online=True, magnet=magnet, raw=raw,
            angle=raw * 360.0 / 4096.0,
        )
    except Exception:
        steer_encoder["online"] = False
        steer_encoder["magnet"] = "READ ERROR"
        steer_encoder["errors"] += 1


def as5047_transfer(frame):
    received = bytearray(2)
    encoder_cs.value(0)
    spi.write_readinto(as5047_bytes(frame), received)
    encoder_cs.value(1)
    time.sleep_us(2)
    return (received[0] << 8) | received[1]


def as5047_read(register):
    as5047_transfer(as5047_command(register))
    response = as5047_transfer(0)
    return response & 0x3FFF, bool(response & 0x4000), parity_is_odd(response)


def as5047_update():
    try:
        raw, sensor_error, parity_error = as5047_read(0x3FFF)
        diag, diag_error, diag_parity = as5047_read(0x3FFC)
        bad = sensor_error or parity_error or diag_error or diag_parity
        if diag & (1 << 10):
            magnet = "TOO STRONG"
        elif diag & (1 << 11):
            magnet = "TOO WEAK"
        elif diag & (1 << 9):
            magnet = "INVALID"
        elif not diag & (1 << 8):
            magnet = "NOT READY"
        else:
            magnet = "OK"
        bldc_encoder.update(
            online=not bad, magnet=magnet, raw=raw,
            angle=raw * 360.0 / 16384.0,
        )
        if bad:
            bldc_encoder["errors"] += 1
    except Exception:
        bldc_encoder["online"] = False
        bldc_encoder["magnet"] = "READ ERROR"
        bldc_encoder["errors"] += 1


def update_encoders():
    global last_encoder_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_encoder_ms) < 100:
        return
    last_encoder_ms = now
    as5600_update()
    as5047_update()


def write_voltage(voltage):
    global throttle_command, gp_present
    voltage = clamp(float(voltage), 0.0, THROTTLE_MAX_V)
    if gp_present:
        try:
            raw = int(voltage / 10.0 * 65535)
            i2c.writeto_mem(GP_ADDR, GP_DAC_REG, bytes((raw & 0xFF, raw >> 8)))
        except Exception:
            gp_present = False
            throttle_command = 0.0
            raise
    elif voltage > 0.0:
        raise OSError("GP8630N offline")
    throttle_command = voltage


def traction_stop():
    global gear, throttle_command
    gear = 0
    bld_en.value(0)
    try:
        write_voltage(0.0)
    except Exception as error:
        throttle_command = 0.0
        print("DAC STOP WRITE ERROR:", error)


def apply_brakes():
    global brake_state
    traction_stop()
    bld_brk.value(0)
    brake_enable.value(BRAKE_ENABLE_LEVEL)
    brake.set_target(BRAKE_APPLY_DEG)
    brake_state = "APPLYING"


def release_brakes():
    global brake_state
    traction_stop()
    bld_brk.value(0)
    brake_enable.value(BRAKE_ENABLE_LEVEL)
    brake.set_target(0.0)
    brake_state = "RELEASING"


def update_brake_state():
    global brake_state
    if not brake.at_target():
        return
    if brake_state == "APPLYING":
        brake_state = "APPLIED"
    elif brake_state == "RELEASING":
        brake_state = "RELEASED"
        if armed:
            bld_brk.value(1)
            bld_en.value(1 if gear else 0)


def arm_controller():
    global armed
    armed = True
    release_brakes()


def disarm_controller(reason):
    global armed
    armed = False
    steering.hold()
    apply_brakes()
    print("DISARM:", reason)


apply_brakes()


HTML = r'''<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>AutoKart RC</title><style>
:root{--bg:#070a0f;--card:#121923;--line:#263347;--text:#f8fbff;--muted:#8e9cb1;--blue:#397dff;--cyan:#52d7ff;--green:#19b878;--red:#ef4053}*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 50% -10%,#17253b 0,var(--bg) 42%);color:var(--text);font-family:system-ui;touch-action:manipulation;overscroll-behavior:none;-webkit-user-select:none;user-select:none}.app{max-width:540px;margin:auto;padding:12px 12px calc(12px + env(safe-area-inset-bottom))}.top{display:flex;align-items:center;justify-content:space-between}.top h1{font-size:25px;letter-spacing:-.04em}.state{padding:8px 13px;border:1px solid #73313a;border-radius:999px;background:#39151b;color:#ff9ca6;font-size:12px;font-weight:900;letter-spacing:.08em}.state.on{border-color:#226d50;background:#0f3828;color:#73e5af}.card{background:linear-gradient(145deg,#151e2a,#10161f);border:1px solid var(--line);border-radius:18px;padding:13px;margin-top:10px;box-shadow:0 10px 28px #0004}.label{color:var(--muted);font-size:10px;font-weight:900;letter-spacing:.16em;margin-bottom:10px}.row{display:flex;gap:8px}.row>*{flex:1}button{border:1px solid #ffffff12;border-radius:13px;min-height:54px;background:#2c384b;color:white;font-size:15px;font-weight:900;cursor:pointer;box-shadow:inset 0 1px #ffffff18;transition:.12s transform,.12s filter;-webkit-tap-highlight-color:transparent}button:active,.pressed{transform:scale(.95);filter:brightness(.78)}button:focus-visible{outline:3px solid #fff;outline-offset:3px}button:disabled{opacity:.32;filter:saturate(.3);cursor:not-allowed;transform:none}.green{background:linear-gradient(145deg,#20c986,#10935e)}.blue{background:linear-gradient(145deg,#438cff,#2867dc)}.red{background:linear-gradient(145deg,#ff4a5d,#cf2e42)}.pad{display:grid;grid-template-columns:repeat(3,78px);grid-template-rows:repeat(3,68px);justify-content:center;gap:8px;padding:2px}.pad button{min-height:68px;border-radius:22px;font-size:28px}.pad .up{grid-column:2;grid-row:1}.pad .left{grid-column:1;grid-row:2}.pad .center{grid-column:2;grid-row:2;border-radius:50%;color:var(--cyan)}.pad .right{grid-column:3;grid-row:2}.pad .down{grid-column:2;grid-row:3}.pad .active{outline:3px solid var(--cyan);outline-offset:2px;box-shadow:0 0 22px #52d7ff55}.gears button{min-width:0;font-size:18px;border-radius:16px}.gears button.active{outline:3px solid var(--cyan);outline-offset:2px}.off,.brake,.stop{width:100%;margin-top:8px}.off{background:#202b3a}.brake{min-height:76px;font-size:20px}.stop{position:sticky;bottom:calc(8px + env(safe-area-inset-bottom));z-index:5;min-height:58px;font-size:16px;box-shadow:0 8px 30px #ef405377}.status{display:grid;grid-template-columns:1fr 1fr;gap:7px}.item{background:#090e15;border:1px solid #202b3b;padding:9px;border-radius:11px}.item span{display:block;color:var(--muted);font-size:9px;letter-spacing:.08em}.item b{font-size:13px}.item i{font-style:normal}.sensors{color:var(--muted);font-size:10px;line-height:1.7;margin-top:9px}.message{text-align:center;color:#ffd475;font-size:11px;padding:9px;min-height:33px}.message.ok{color:#77e8b3}.message.bad{color:#ff8f9a}.hint{text-align:center;color:var(--muted);font-size:10px;margin-top:7px}@media(max-width:380px){.app{padding:7px 7px calc(7px + env(safe-area-inset-bottom))}.pad{grid-template-columns:repeat(3,70px);grid-template-rows:repeat(3,62px)}.pad button{min-height:62px}.row{gap:5px}.gears button{font-size:16px}}
</style></head><body><main class="app"><div class="top"><h1>AutoKart RC</h1><div id="armed" class="state">DISARMED</div></div>
<div class="card"><div class="row"><button id="armBtn" class="green" onclick="cmd('/arm')">ARM</button><button id="disarmBtn" class="red" onclick="cmd('/disarm')">DISARM</button></div></div>
<div class="card"><div class="label">DRIVE · STEERING 45° / 2000 PULSES</div><div class="pad"><button id="forward" class="up green" aria-label="Forward" title="Forward · CCW" onclick="cmd('/direction?d=FORWARD')">▲</button><button id="leftBtn" class="left blue" aria-label="Steer left" title="Steer left 45 degrees" onclick="cmd('/steer?v=-45')">◀</button><button id="centerBtn" class="center" aria-label="Center steering" title="Center steering" onclick="cmd('/steer?v=0')">●</button><button id="rightBtn" class="right blue" aria-label="Steer right" title="Steer right 45 degrees" onclick="cmd('/steer?v=45')">▶</button><button id="reverse" class="down" aria-label="Reverse" title="Reverse · CW" onclick="cmd('/direction?d=REVERSE')">▼</button></div><div class="hint">▲ / ▼ select direction · choose a gear to move</div></div>
<div class="card"><div class="label">GEAR</div><div class="row gears"><button data-gear="1" onclick="gear(1)">1</button><button data-gear="2" onclick="gear(2)">2</button><button data-gear="3" onclick="gear(3)">3</button><button data-gear="4" onclick="gear(4)">4</button><button data-gear="5" onclick="gear(5)">5</button></div><button id="offBtn" class="off" onclick="gear(0)">THROTTLE OFF</button></div>
<div class="card"><button id="brakeBtn" class="red brake">HOLD TO BRAKE</button><div class="hint">Release button to release brake while armed</div></div>
<div class="card"><div class="status"><div class="item"><span>DRIVE</span><b id="driveState">STOPPED</b></div><div class="item"><span>GEAR / COMMAND</span><b><i id="gear">0</i> · <i id="voltage">0.0 V</i></b></div><div class="item"><span>STEERING</span><b><i id="steerTarget">0</i>°</b></div><div class="item"><span>BRAKE / SIM ANGLE</span><b><i id="brakeState">APPLIED</i> · <i id="brakeAngle">0°</i></b></div></div><div class="sensors">AS5600: <b id="steerDetail">checking</b> · errors <b id="steerErrors">0</b><br>AS5047P: <b id="bldcEnc">checking</b> · raw/errors <b id="bldcDetail">--</b><br>GP8630N: <b id="gpState">checking</b></div></div>
<button id="emergencyBtn" class="red stop" onclick="cmd('/disarm')">EMERGENCY STOP</button><div id="message" class="message" role="status" aria-live="polite">Connecting…</div></main><script>
const $=id=>document.getElementById(id);let held=false;let latest=null;async function cmd(p){$('message').textContent='SENDING…';$('message').className='message';try{const r=await fetch(p,{cache:'no-store'}),t=await r.text();$('message').textContent=t;$('message').className='message '+(r.ok?'ok':'bad');await update();return r.ok}catch(e){$('message').textContent='CONTROLLER OFFLINE';$('message').className='message bad';return false}}function gear(n){cmd('/gear?n='+n)}const bb=$('brakeBtn');function down(e){e.preventDefault();if(held||bb.disabled)return;held=true;bb.classList.add('pressed');cmd('/brake?s=apply')}function up(e){if(e)e.preventDefault();if(!held)return;held=false;bb.classList.remove('pressed');cmd('/brake?s=release')}['pointerdown','touchstart'].forEach(e=>bb.addEventListener(e,down,{passive:false}));['pointerup','pointercancel','pointerleave','touchend','touchcancel'].forEach(e=>bb.addEventListener(e,up,{passive:false}));addEventListener('blur',up);addEventListener('pagehide',()=>navigator.sendBeacon('/disarm'));
async function update(){try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error();const s=await r.json();latest=s;$('armed').textContent=s.armed?'ARMED':'DISARMED';$('armed').classList.toggle('on',s.armed);$('armBtn').disabled=s.armed;$('disarmBtn').disabled=!s.armed;$('forward').disabled=!s.armed||s.gear!==0;$('reverse').disabled=!s.armed||s.gear!==0;$('leftBtn').disabled=!s.armed;$('centerBtn').disabled=!s.armed;$('rightBtn').disabled=!s.armed;bb.disabled=!s.armed;bb.textContent=s.armed?'HOLD TO BRAKE':'BRAKE APPLIED';$('forward').classList.toggle('active',s.direction==='FORWARD');$('reverse').classList.toggle('active',s.direction==='REVERSE');$('leftBtn').classList.toggle('active',s.steer_target<-1);$('centerBtn').classList.toggle('active',Math.abs(s.steer_target)<=1);$('rightBtn').classList.toggle('active',s.steer_target>1);document.querySelectorAll('[data-gear]').forEach(b=>{b.disabled=!s.armed||s.brake_state!=='RELEASED';b.classList.toggle('active',s.gear===Number(b.dataset.gear))});$('gear').textContent=s.gear;$('voltage').textContent=s.voltage.toFixed(1)+' V';$('steerTarget').textContent=s.steer_target.toFixed(0);$('brakeState').textContent=s.brake_state;$('brakeAngle').textContent=s.brake_angle.toFixed(0)+'°';$('driveState').textContent=s.armed&&s.gear?'RUNNING '+(s.direction==='FORWARD'?'▲':'▼'):'STOPPED';$('steerDetail').textContent=s.steer_online?s.steer_angle.toFixed(1)+'° / '+s.steer_magnet:s.steer_magnet;$('steerErrors').textContent=s.steer_errors;$('bldcEnc').textContent=s.bldc_online?s.bldc_angle.toFixed(1)+'° / '+s.bldc_magnet:s.bldc_magnet;$('bldcDetail').textContent=s.bldc_raw+'/'+s.bldc_errors;$('gpState').textContent=s.gp_online?'ONLINE':'OFFLINE'}catch(e){$('message').textContent='CONTROLLER OFFLINE';$('message').className='message bad'}}setInterval(update,300);update();
</script></body></html>'''


def query(path, key, default):
    try:
        return path.split(key + "=", 1)[1].split("&", 1)[0]
    except Exception:
        return default


def send_all(client, data):
    view = memoryview(data)
    while view:
        sent = client.send(view)
        if not sent:
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


def json_status():
    return (
        '{{"armed":{},"direction":"{}","gear":{},"voltage":{:.2f},'
        '"steer_target":{:.2f},"steer_angle":{:.2f},"steer_raw":{},'
        '"steer_errors":{},"steer_online":{},"steer_magnet":"{}",'
        '"brake_state":"{}","brake_angle":{:.2f},"brake_simulated":true,'
        '"bldc_angle":{:.2f},"bldc_raw":{},"bldc_errors":{},'
        '"bldc_online":{},"bldc_magnet":"{}","gp_online":{}}}'
    ).format(
        str(armed).lower(), direction, gear, throttle_command,
        steering.target_degrees(), steer_encoder["angle"],
        steer_encoder["raw"], steer_encoder["errors"],
        str(steer_encoder["online"]).lower(), steer_encoder["magnet"],
        brake_state, brake.degrees(), bldc_encoder["angle"],
        bldc_encoder["raw"], bldc_encoder["errors"],
        str(bldc_encoder["online"]).lower(), bldc_encoder["magnet"],
        str(gp_present).lower(),
    )


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

print("AUTOKART SIMPLE RC READY")
print("Wi-Fi:", SSID, "Password:", PASSWORD)
print("Open: http://" + ap.ifconfig()[0])
print("I2C:", [hex(item) for item in i2c_devices])
print("Brake feedback: SIMULATED")

try:
    while True:
        steering.update()
        brake.update()
        update_brake_state()
        update_encoders()

        if armed and time.ticks_diff(time.ticks_ms(), last_contact_ms) > WATCHDOG_MS:
            disarm_controller("WEB WATCHDOG")

        try:
            client, _ = server.accept()
        except OSError:
            time.sleep_us(100)
            continue

        try:
            request = client.recv(1200).decode("utf-8", "ignore")
            first = request.split("\r\n", 1)[0].split(" ")
            path = first[1] if len(first) > 1 else "/bad"

            if path == "/":
                reply(client, HTML, content_type="text/html")
            elif path.startswith("/status"):
                if armed:
                    last_contact_ms = time.ticks_ms()
                reply(client, json_status(), content_type="application/json")
            elif path.startswith("/arm"):
                arm_controller()
                last_contact_ms = time.ticks_ms()
                reply(client, "ARMED; BRAKES RELEASING")
            elif path.startswith("/disarm"):
                disarm_controller("REMOTE")
                reply(client, "DISARMED; BRAKES APPLYING")
            elif path.startswith("/steer"):
                if not armed:
                    reply(client, "ARM FIRST", "409 Conflict")
                else:
                    requested = clamp(float(query(path, "v", "0")), -45, 45)
                    steering.set_target(requested)
                    last_contact_ms = time.ticks_ms()
                    reply(client, "STEERING {:.0f} DEG".format(requested))
            elif path.startswith("/direction"):
                requested = query(path, "d", "FORWARD")
                if not armed:
                    reply(client, "ARM FIRST", "409 Conflict")
                elif gear != 0 or throttle_command > 0.01:
                    reply(client, "SELECT GEAR 0 BEFORE DIRECTION", "409 Conflict")
                elif requested not in ("FORWARD", "REVERSE"):
                    reply(client, "BAD DIRECTION", "400 Bad Request")
                else:
                    direction = requested
                    bld_fr.value(1 if requested == "FORWARD" else 0)
                    last_contact_ms = time.ticks_ms()
                    reply(client, requested + " SELECTED")
            elif path.startswith("/gear"):
                requested = int(query(path, "n", "0"))
                if requested < 0 or requested > 5:
                    reply(client, "GEAR MUST BE 0..5", "400 Bad Request")
                elif requested == 0:
                    traction_stop()
                    last_contact_ms = time.ticks_ms()
                    reply(client, "THROTTLE OFF")
                elif not gp_present:
                    reply(client, "GP8630N OFFLINE", "409 Conflict")
                elif not armed or brake_state != "RELEASED":
                    reply(client, "ARM AND WAIT FOR BRAKE RELEASE", "409 Conflict")
                else:
                    traction_stop()
                    write_voltage(float(requested))
                    gear = requested
                    bld_en.value(1)
                    last_contact_ms = time.ticks_ms()
                    reply(client, "GEAR {} / REQUESTED {:.1f} V".format(
                        requested, float(requested)
                    ))
            elif path.startswith("/brake"):
                requested = query(path, "s", "apply")
                if requested == "release":
                    if armed:
                        release_brakes()
                        last_contact_ms = time.ticks_ms()
                        reply(client, "BRAKES RELEASING TO SIMULATED 0 DEG")
                    else:
                        reply(client, "DISARMED: BRAKE REMAINS APPLIED", "409 Conflict")
                else:
                    apply_brakes()
                    if armed:
                        last_contact_ms = time.ticks_ms()
                    reply(client, "BRAKES APPLYING TO SIMULATED 45 DEG")
            else:
                reply(client, "NOT FOUND", "404 Not Found")
        except Exception as error:
            print("HTTP ERROR:", error)
            try:
                reply(client, "CONTROLLER ERROR", "500 Internal Server Error")
            except Exception:
                pass
        finally:
            try:
                client.close()
            except Exception:
                pass
finally:
    disarm_controller("CONTROLLER EXIT")
    brake_enable.value(BRAKE_DISABLE_LEVEL)
    server.close()

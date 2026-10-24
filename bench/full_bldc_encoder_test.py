"""ESP32-S3 full BLD750 + GP8630N + AS5047P Wi-Fi bench test.

The UI uses the AutoKart RC controls: ARM/DISARM, Forward/Reverse, gears 1-5,
Throttle Off, Wheel Lock and Emergency Stop. Normal starts, gear changes,
direction changes and link-loss stops use a voltage ramp. Direction reversal
reaches 0 V, turns EN off, waits, changes F/R, then enables and ramps in the
new direction.

Wheel Lock, Emergency Stop, explicit Disarm, an actuator interlock and program
failure are deliberately immediate: DAC=0 V, EN=STOP and BRK=APPLIED. The lock
remains asserted until ARM is explicitly requested. AS5047P faults remain
visible in telemetry but do not randomly stop this isolated BLDC bench test.

WHEELS-OFF-GROUND TESTING ONLY. The BLD750 control inputs are driven through
the existing PN2222 open-collector stages; never wire them directly to ESP32.

ESP32-S3 pins:
  GP8630N SDA GPIO7, SCL GPIO21, I2C(1), address 0x58
  BLD750 BRK GPIO13, EN GPIO14, F/R GPIO15 (through PN2222 stages)
  AS5047P CS GPIO16, SCK GPIO18, MOSI GPIO41, MISO GPIO42 (SPI mode 1)

Wi-Fi: BLD750-Encoder-Test / 12345678, then open http://192.168.4.1
"""

import network
import socket
import time
from machine import I2C, Pin, SPI


SSID = "BLD750-Encoder-Test"
PASSWORD = "12345678"

# GP8630N
SDA_GPIO = 7
SCL_GPIO = 21
GP_ADDR = 0x58
GP_MODE_REG = 0x01
GP_DAC_REG = 0x02
GP_MODE_0_10V = 0x1C

# BLD750 PN2222 control stages. GPIO HIGH pulls the BLD input LOW.
BLD_BRK_GPIO = 13
BLD_EN_GPIO = 14
BLD_FR_GPIO = 15

# AS5047P SPI
ENC_CS_GPIO = 16
ENC_SCK_GPIO = 18
ENC_MOSI_GPIO = 41
ENC_MISO_GPIO = 42
REG_ERRFL = 0x0001
REG_DIAAGC = 0x3FFC
REG_MAG = 0x3FFD
REG_ANGLE = 0x3FFF
COUNTS_PER_REV = 16384

# Bench limits and timing.
MAX_VOLTAGE = 5.0
RAMP_STEP_V = 0.08
RAMP_INTERVAL_MS = 35
ENCODER_INTERVAL_MS = 5
DIAGNOSTIC_INTERVAL_MS = 100
CONTROL_LINK_TIMEOUT_MS = 5000
ENCODER_STALE_MS = 500
MAX_VALID_RPM = 6000.0
RPM_TIME_CONSTANT_S = 0.030
NOISE_COUNTS = 2

# Encoder magnet is on 14T sprocket; 44T sprocket is fixed to rear axle.
MOTOR_REVS_PER_AXLE_REV = 44.0 / 14.0
TYRE_CIRCUMFERENCE_M = 3.141592653589793 * 11.0 * 0.0254
TRAVEL_PER_MOTOR_REV_M = TYRE_CIRCUMFERENCE_M / MOTOR_REVS_PER_AXLE_REV


def clamp(value, low, high):
    return max(low, min(high, value))


def wrapped_delta(current, previous, counts):
    half = counts // 2
    return ((current - previous + half) % counts) - half


def odd_parity(value):
    return bool(bin(value & 0xFFFF).count("1") & 1)


def command_frame(register):
    frame = 0x4000 | (register & 0x3FFF)
    if odd_parity(frame):
        frame |= 0x8000
    return frame


def frame_bytes(frame):
    return bytes(((frame >> 8) & 0xFF, frame & 0xFF))


# Safe output state is established before buses and Wi-Fi.
en_pin = Pin(BLD_EN_GPIO, Pin.OUT, value=0)       # transistor OFF -> STOP
brk_pin = Pin(BLD_BRK_GPIO, Pin.OUT, value=0)    # transistor OFF -> BRAKE
fr_pin = Pin(BLD_FR_GPIO, Pin.OUT, value=1)      # transistor ON -> CCW

armed = False
running = False
brake_released = False
wheel_locked = True
direction = "FORWARD"
requested_direction = "FORWARD"
selected_gear = 1
gear = 0
current_voltage = 0.0
target_voltage = 0.0
drive_state = "STOPPED"
last_command_ms = time.ticks_ms()
last_ramp_ms = time.ticks_ms()
reversal_pending = False
direction_settle_started_ms = None
link_timeout_active = False
DIRECTION_SETTLE_MS = 350


def enable_run():
    global running
    en_pin.value(1)  # transistor ON -> BLD EN LOW -> RUN
    running = True


def enable_stop():
    global running
    en_pin.value(0)
    running = False


def release_brake():
    global brake_released
    brk_pin.value(1)  # transistor ON -> BLD BRK LOW -> RELEASED
    brake_released = True


def apply_brake():
    global brake_released
    brk_pin.value(0)
    brake_released = False


def set_direction(value):
    global direction
    direction = value
    fr_pin.value(1 if value == "FORWARD" else 0)


i2c = I2C(1, sda=Pin(SDA_GPIO), scl=Pin(SCL_GPIO), freq=100_000)
gp_online = GP_ADDR in i2c.scan()
if not gp_online:
    raise RuntimeError("GP8630N not found at 0x58")
i2c.writeto_mem(GP_ADDR, GP_MODE_REG, bytes((GP_MODE_0_10V,)))


def write_voltage(voltage):
    global current_voltage, gp_online
    voltage = clamp(float(voltage), 0.0, MAX_VOLTAGE)
    raw = int(voltage / 10.0 * 65535)
    try:
        i2c.writeto_mem(GP_ADDR, GP_DAC_REG, bytes((raw & 0xFF, raw >> 8)))
        current_voltage = voltage
    except Exception:
        gp_online = False
        current_voltage = 0.0
        raise


spi = SPI(
    1, baudrate=1_000_000, polarity=0, phase=1, bits=8,
    firstbit=SPI.MSB, sck=Pin(ENC_SCK_GPIO),
    mosi=Pin(ENC_MOSI_GPIO), miso=Pin(ENC_MISO_GPIO),
)
cs = Pin(ENC_CS_GPIO, Pin.OUT, value=1)


def transfer(frame):
    received = bytearray(2)
    cs.value(0)
    spi.write_readinto(frame_bytes(frame), received)
    cs.value(1)
    time.sleep_us(2)
    return (received[0] << 8) | received[1]


def read_register(register):
    transfer(command_frame(register))
    response = transfer(0x0000)
    return response & 0x3FFF, bool(response & 0x4000), odd_parity(response)


encoder = {
    "online": False, "magnet": "NOT CHECKED", "raw": -1,
    "absolute_deg": 0.0, "relative_deg": 0.0, "zero_raw": -1,
    "initialized": False, "last_raw": -1, "last_us": 0,
    "rpm": 0.0, "axle_rpm": 0.0, "signed_counts": 0,
    "travel_counts": 0, "motor_revs": 0.0, "full_turns": 0,
    "distance_m": 0.0, "displacement_m": 0.0, "speed_kmh": 0.0,
    "magnitude": 0, "agc": 0, "error_flags": 0, "errors": 0,
    "last_valid_ms": 0,
}
last_encoder_ms = 0
last_diagnostic_ms = 0


def magnet_state(diaagc):
    if diaagc & (1 << 10):
        return "TOO STRONG"
    if diaagc & (1 << 11):
        return "TOO WEAK / NO MAGNET"
    if diaagc & (1 << 9):
        return "INVALID"
    if not diaagc & (1 << 8):
        return "NOT READY"
    return "OK"


def update_diagnostics():
    try:
        diag, diag_error, diag_parity = read_register(REG_DIAAGC)
        magnitude, mag_error, mag_parity = read_register(REG_MAG)
        flags, _, _ = read_register(REG_ERRFL)
        bad = diag_error or diag_parity or mag_error or mag_parity
        encoder.update(
            online=not bad, magnet=magnet_state(diag),
            magnitude=magnitude, agc=diag & 0xFF, error_flags=flags,
        )
        if bad:
            encoder["errors"] += 1
    except Exception:
        encoder["online"] = False
        encoder["magnet"] = "READ ERROR"
        encoder["errors"] += 1


def update_encoder():
    try:
        raw, sensor_error, parity_error = read_register(REG_ANGLE)
        if sensor_error or parity_error or encoder["magnet"] != "OK":
            encoder["online"] = False if sensor_error or parity_error else True
            encoder["errors"] += 1
            return

        now_us = time.ticks_us()
        now_ms = time.ticks_ms()
        encoder.update(
            online=True, raw=raw, absolute_deg=raw * 360.0 / COUNTS_PER_REV,
            last_valid_ms=now_ms,
        )
        if not encoder["initialized"]:
            encoder.update(
                initialized=True, zero_raw=raw, last_raw=raw,
                last_us=now_us, relative_deg=0.0,
            )
            return

        dt_us = time.ticks_diff(now_us, encoder["last_us"])
        delta = wrapped_delta(raw, encoder["last_raw"], COUNTS_PER_REV)
        encoder["last_raw"] = raw
        encoder["last_us"] = now_us
        encoder["relative_deg"] = (
            wrapped_delta(raw, encoder["zero_raw"], COUNTS_PER_REV)
            * 360.0 / COUNTS_PER_REV
        )
        if dt_us <= 0:
            return
        raw_rpm = delta * 60_000_000.0 / (COUNTS_PER_REV * dt_us)
        if abs(raw_rpm) > MAX_VALID_RPM:
            encoder["errors"] += 1
            return
        if abs(delta) <= NOISE_COUNTS:
            delta = 0
            raw_rpm = 0.0
        dt_s = dt_us / 1_000_000.0
        alpha = dt_s / (RPM_TIME_CONSTANT_S + dt_s)
        encoder["rpm"] += alpha * (raw_rpm - encoder["rpm"])
        if delta == 0 and abs(encoder["rpm"]) < 1.0:
            encoder["rpm"] = 0.0
        encoder["signed_counts"] += delta
        encoder["travel_counts"] += abs(delta)
        encoder["motor_revs"] = encoder["signed_counts"] / COUNTS_PER_REV
        travelled_revs = encoder["travel_counts"] / COUNTS_PER_REV
        encoder["full_turns"] = int(travelled_revs)
        encoder["axle_rpm"] = encoder["rpm"] / MOTOR_REVS_PER_AXLE_REV
        encoder["distance_m"] = travelled_revs * TRAVEL_PER_MOTOR_REV_M
        encoder["displacement_m"] = (
            encoder["motor_revs"] * TRAVEL_PER_MOTOR_REV_M
        )
        encoder["speed_kmh"] = abs(
            encoder["axle_rpm"] * TYRE_CIRCUMFERENCE_M * 60.0 / 1000.0
        )
    except Exception:
        encoder["online"] = False
        encoder["magnet"] = "READ ERROR"
        encoder["errors"] += 1


def service_encoder():
    global last_encoder_ms, last_diagnostic_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_diagnostic_ms) >= DIAGNOSTIC_INTERVAL_MS:
        last_diagnostic_ms = now
        update_diagnostics()
    if time.ticks_diff(now, last_encoder_ms) >= ENCODER_INTERVAL_MS:
        last_encoder_ms = now
        update_encoder()


def encoder_ready():
    return (
        encoder["online"] and encoder["initialized"]
        and encoder["magnet"] == "OK"
        and time.ticks_diff(time.ticks_ms(), encoder["last_valid_ms"])
        <= ENCODER_STALE_MS
    )


def zero_encoder():
    raw = encoder["raw"]
    now_us = time.ticks_us()
    encoder.update(
        initialized=True, zero_raw=raw, last_raw=raw, last_us=now_us,
        relative_deg=0.0, rpm=0.0, axle_rpm=0.0, signed_counts=0,
        travel_counts=0, motor_revs=0.0, full_turns=0,
        distance_m=0.0, displacement_m=0.0, speed_kmh=0.0,
    )


def lock_wheel(reason="WHEEL LOCK"):
    global armed, target_voltage, gear, reversal_pending, drive_state
    global direction_settle_started_ms, wheel_locked
    armed = False
    wheel_locked = True
    target_voltage = 0.0
    gear = 0
    reversal_pending = False
    direction_settle_started_ms = None
    drive_state = "WHEEL LOCKED"
    try:
        write_voltage(0.0)
    except Exception:
        pass
    apply_brake()
    enable_stop()
    print("WHEEL LOCKED:", reason, "| SV=0 V | EN=STOP | BRK=APPLIED")


def safe_stop(reason):
    lock_wheel(reason)
    print("SAFE STOP:", reason)


def request_soft_stop(reason="SOFT STOP"):
    global target_voltage, reversal_pending, direction_settle_started_ms
    global drive_state
    target_voltage = 0.0
    reversal_pending = False
    direction_settle_started_ms = None
    if wheel_locked:
        drive_state = "WHEEL LOCKED"
        return
    drive_state = "SLOWING" if current_voltage > 0.01 else "STOPPED"
    print(reason + ": ramping SV to zero")


def request_drive(new_direction):
    global requested_direction, target_voltage, reversal_pending
    global direction_settle_started_ms, drive_state
    requested_direction = new_direction
    if new_direction != direction:
        target_voltage = 0.0
        reversal_pending = True
        direction_settle_started_ms = None
        drive_state = "SLOWING FOR " + new_direction
    else:
        reversal_pending = False
        target_voltage = float(selected_gear)
        release_brake()
        enable_run()
        drive_state = "RAMPING " + direction


def arm_controller():
    global armed, drive_state, wheel_locked
    if armed:
        return
    write_voltage(0.0)
    enable_stop()
    release_brake()
    wheel_locked = False
    armed = True
    drive_state = "ARMED AT 0 V"
    print("ARMED AT 0 V")


def service_traction():
    global last_ramp_ms, direction, gear, target_voltage, drive_state
    global reversal_pending, direction_settle_started_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_ramp_ms) < RAMP_INTERVAL_MS:
        return
    last_ramp_ms = now

    if not armed or not brake_released:
        if current_voltage > 0.0:
            safe_stop("INTERLOCK")
        return

    if reversal_pending:
        if current_voltage > 0.0:
            write_voltage(max(0.0, current_voltage - RAMP_STEP_V))
            drive_state = "SLOWING FOR " + requested_direction
            return
        enable_stop()
        gear = 0
        if direction_settle_started_ms is None:
            direction_settle_started_ms = now
            drive_state = "DIRECTION PAUSE"
            return
        if time.ticks_diff(now, direction_settle_started_ms) < DIRECTION_SETTLE_MS:
            return
        set_direction(requested_direction)
        reversal_pending = False
        direction_settle_started_ms = None
        target_voltage = float(selected_gear)
        release_brake()
        enable_run()
        drive_state = "RAMPING " + direction
        return

    if current_voltage < target_voltage:
        write_voltage(min(target_voltage, current_voltage + RAMP_STEP_V))
        gear = selected_gear
        drive_state = "RAMPING " + direction
    elif current_voltage > target_voltage:
        write_voltage(max(target_voltage, current_voltage - RAMP_STEP_V))
        drive_state = "SLOWING"
    elif target_voltage > 0.0:
        gear = selected_gear
        drive_state = "RUNNING " + direction
    else:
        gear = 0
        enable_stop()
        drive_state = "STOPPED"


write_voltage(0.0)
update_diagnostics()
update_encoder()


HTML = r'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>BLD750 Encoder Test</title><style>
:root{--bg:#080c12;--card:#131c28;--line:#29384d;--text:#f5f8ff;--muted:#93a2b7;--green:#18b978;--red:#e94255;--blue:#397dff;--orange:#ef8d31}*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 50% -10%,#203451,var(--bg) 42%);color:var(--text);font-family:system-ui,sans-serif;touch-action:manipulation}.app{max-width:620px;margin:auto;padding:14px}.top{display:flex;justify-content:space-between;align-items:center;gap:10px}.top h1{font-size:24px;margin:8px 0}.badge{padding:8px 12px;border-radius:999px;background:#441821;color:#ff9aa5;font-size:11px;font-weight:900}.badge.on{background:#123d2c;color:#78e9b5}.card{background:linear-gradient(145deg,#172231,var(--card));border:1px solid var(--line);border-radius:18px;padding:14px;margin-top:11px}.label{color:var(--muted);font-size:10px;font-weight:900;letter-spacing:.14em;margin-bottom:9px}.voltage{text-align:center;font-size:44px;font-weight:900}.voltage small{font-size:16px;color:var(--muted)}.row{display:flex;gap:8px}.row>*{flex:1}.gears{display:grid;grid-template-columns:repeat(5,1fr);gap:6px}button{min-height:52px;border:0;border-radius:13px;color:white;background:#2b394d;font-size:15px;font-weight:850;box-shadow:inset 0 1px #ffffff22;transition:transform .09s,filter .09s}button:active,button.active{transform:scale(.96);filter:brightness(.85)}button.active{outline:3px solid #55d9ff;outline-offset:2px}button:disabled{opacity:.35}.green{background:var(--green)}.red{background:var(--red)}.blue{background:var(--blue)}.orange{background:var(--orange)}.wide{width:100%;margin-top:9px}.stop{width:100%;min-height:66px;font-size:19px;margin-top:10px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}.tile{background:#0c131d;border:1px solid #253449;border-radius:12px;padding:10px}.tile span{display:block;color:var(--muted);font-size:9px;font-weight:800;letter-spacing:.08em}.tile b{font-size:16px}.wideTile{grid-column:1/-1}.ok{color:#73e6ae}.bad{color:#ff929d}.msg{text-align:center;color:#ffd078;min-height:25px;margin-top:10px;font-size:12px}.warning{text-align:center;color:#ff929d;font-size:11px;font-weight:800;margin:14px}@media(max-width:390px){.app{padding:8px}.card{padding:11px}.top h1{font-size:21px}}
</style></head><body><main class="app"><div class="top"><h1>BLD750 Smooth Test</h1><div id="runState" class="badge">DISARMED</div></div>
<section class="card"><div class="row"><button id="armBtn" class="green">ARM + UNLOCK</button><button id="disarmBtn" class="red">DISARM</button></div><button id="lockBtn" class="red wide">🔒 LOCK WHEEL</button><div style="color:var(--muted);font-size:10px;text-align:center;margin-top:8px">Lock forces SV to 0 V, EN to STOP and BLD750 BRK to APPLIED. ARM releases the lock.</div></section>
<section class="card"><div class="label">DIRECTION</div><div class="row"><button id="forwardBtn" class="blue">▲ FORWARD</button><button id="reverseBtn" class="orange">▼ REVERSE</button></div><div style="color:var(--muted);font-size:10px;text-align:center;margin-top:8px">Direction changes ramp to 0 V, pause, switch F/R, then ramp up.</div></section>
<section class="card"><div class="label">SPEED GEAR</div><div class="gears"><button data-gear="1">1</button><button data-gear="2">2</button><button data-gear="3">3</button><button data-gear="4">4</button><button data-gear="5">5</button></div><button id="offBtn" class="wide">THROTTLE OFF · SMOOTH STOP</button><div class="voltage"><span id="targetV">0.00</span> <small>V target</small></div></section>
<section class="card"><button id="zeroBtn" class="wide">ZERO ENCODER + DISTANCE</button></section>
<section class="card"><div class="label">LIVE AS5047P FEEDBACK</div><div class="grid"><div class="tile"><span>DRIVE</span><b id="driveState">WHEEL LOCKED</b></div><div class="tile"><span>BLDC BRAKE LOCK</span><b id="lockState" class="bad">LOCKED</b></div><div class="tile"><span>OUTPUT / TARGET</span><b><i id="actualV">0.00 V</i> / <i id="commandV">0.00 V</i></b></div><div class="tile"><span>MOTOR RPM</span><b id="motorRpm">0</b></div><div class="tile"><span>AXLE RPM</span><b id="axleRpm">0</b></div><div class="tile"><span>FULL MOTOR TURNS</span><b id="turns">0</b></div><div class="tile"><span>MOTOR REVOLUTIONS</span><b id="revs">0.000</b></div><div class="tile"><span>NOMINAL SPEED</span><b id="speed">0.00 km/h</b></div><div class="tile"><span>DISTANCE TRAVELLED</span><b id="distance">0.000 m</b></div><div class="tile"><span>SIGNED DISPLACEMENT</span><b id="displacement">0.000 m</b></div><div class="tile"><span>ANGLE FROM ZERO</span><b id="angle">0.0°</b></div><div class="tile wideTile"><span>ENCODER HEALTH</span><b id="health">CHECKING</b><div id="detail" style="color:var(--muted);font-size:10px;margin-top:5px">--</div></div></div></section>
<button id="safeBtn" class="red stop">EMERGENCY STOP + BRAKE</button><div id="message" class="msg">Connecting…</div><div class="warning">BENCH TEST ONLY · DRIVE WHEELS MUST BE RAISED AND GUARDED</div></main><script>
const $=x=>document.getElementById(x);function msg(t,b=false){$('message').textContent=t;$('message').className='msg '+(b?'bad':'')}async function cmd(p,quiet=false){try{const r=await fetch(p,{cache:'no-store'}),t=await r.text();if(!quiet)msg(t,!r.ok);return r.ok}catch(e){if(!quiet)msg('CONTROLLER OFFLINE',true);return false}}$('armBtn').onclick=()=>cmd('/arm');$('disarmBtn').onclick=()=>cmd('/disarm');$('lockBtn').onclick=()=>cmd('/lock');$('forwardBtn').onclick=()=>cmd('/drive?d=FORWARD');$('reverseBtn').onclick=()=>cmd('/drive?d=REVERSE');$('offBtn').onclick=()=>cmd('/gear?n=0');$('safeBtn').onclick=()=>cmd('/lock');$('zeroBtn').onclick=()=>cmd('/zero');document.querySelectorAll('[data-gear]').forEach(b=>b.onclick=()=>cmd('/gear?n='+b.dataset.gear));
async function status(){try{const r=await fetch('/status',{cache:'no-store'}),s=await r.json();$('runState').textContent=s.wheel_locked?'WHEEL LOCKED':s.armed?'ARMED':'DISARMED';$('runState').classList.toggle('on',s.armed&&!s.wheel_locked);$('targetV').textContent=s.target_voltage.toFixed(2);$('actualV').textContent=s.voltage.toFixed(2)+' V';$('commandV').textContent=s.target_voltage.toFixed(2)+' V';$('driveState').textContent=s.drive_state;$('lockState').textContent=s.wheel_locked?'LOCKED':'RELEASED';$('lockState').className=s.wheel_locked?'bad':'ok';$('motorRpm').textContent=Math.abs(s.rpm).toFixed(0);$('axleRpm').textContent=Math.abs(s.axle_rpm).toFixed(0);$('turns').textContent=s.full_turns;$('revs').textContent=s.motor_revs.toFixed(3);$('speed').textContent=s.speed_kmh.toFixed(2)+' km/h';$('distance').textContent=s.distance_m.toFixed(3)+' m';$('displacement').textContent=s.displacement_m.toFixed(3)+' m';$('angle').textContent=s.relative_deg.toFixed(1)+'°';$('health').textContent=s.encoder_ready?'MAGNET OK':'CHECK: '+s.magnet;$('health').className=s.encoder_ready?'ok':'bad';$('detail').textContent='raw '+s.raw+' · errors '+s.errors+' · AGC '+s.agc+' · MAG '+s.magnitude;$('armBtn').disabled=s.armed&&!s.wheel_locked;$('disarmBtn').disabled=s.wheel_locked;$('lockBtn').disabled=s.wheel_locked;$('forwardBtn').disabled=!s.armed||s.wheel_locked;$('reverseBtn').disabled=!s.armed||s.wheel_locked;$('zeroBtn').disabled=s.armed||s.voltage>.01;$('forwardBtn').classList.toggle('active',s.direction==='FORWARD'&&s.target_voltage>0);$('reverseBtn').classList.toggle('active',s.direction==='REVERSE'&&s.target_voltage>0);document.querySelectorAll('[data-gear]').forEach(b=>b.classList.toggle('active',Number(b.dataset.gear)===s.selected_gear))}catch(e){msg('CONTROLLER OFFLINE',true)}}setInterval(status,250);addEventListener('pagehide',()=>navigator.sendBeacon('/soft_stop'));status();
</script></body></html>'''


def send_all(client, data):
    view = memoryview(data)
    while view:
        sent = client.send(view)
        if not sent:
            return
        view = view[sent:]


def reply(client, body, status="200 OK", content_type="text/plain; charset=utf-8"):
    if isinstance(body, str):
        body = body.encode()
    header = (
        "HTTP/1.1 " + status + "\r\nContent-Type: " + content_type
        + "\r\nContent-Length: " + str(len(body))
        + "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n"
    ).encode()
    send_all(client, header)
    send_all(client, body)


def query(path, key, default):
    try:
        return path.split(key + "=", 1)[1].split("&", 1)[0]
    except Exception:
        return default


def json_status():
    return (
        '{{"armed":{},"running":{},"brake_released":{},"wheel_locked":{},'
        '"direction":"{}","requested_direction":"{}",'
        '"drive_state":"{}","gear":{},"selected_gear":{},'
        '"voltage":{:.2f},"target_voltage":{:.2f},"gp_online":{},'
        '"link_timeout":{},'
        '"encoder_ready":{},"magnet":"{}","raw":{},'
        '"absolute_deg":{:.3f},"relative_deg":{:.3f},'
        '"rpm":{:.2f},"axle_rpm":{:.2f},"motor_revs":{:.5f},'
        '"full_turns":{},"speed_kmh":{:.3f},"distance_m":{:.4f},'
        '"displacement_m":{:.4f},"agc":{},"magnitude":{},"errors":{}}}'
    ).format(
        str(armed).lower(), str(running).lower(),
        str(brake_released).lower(), str(wheel_locked).lower(),
        direction, requested_direction,
        drive_state, gear, selected_gear,
        current_voltage, target_voltage, str(gp_online).lower(),
        str(link_timeout_active).lower(),
        str(encoder_ready()).lower(), encoder["magnet"], encoder["raw"],
        encoder["absolute_deg"], encoder["relative_deg"], encoder["rpm"],
        encoder["axle_rpm"], encoder["motor_revs"], encoder["full_turns"],
        encoder["speed_kmh"], encoder["distance_m"],
        encoder["displacement_m"], encoder["agc"], encoder["magnitude"],
        encoder["errors"],
    )


ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=SSID, password=PASSWORD)
while not ap.active():
    time.sleep_ms(50)

server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("0.0.0.0", 80))
server.listen(3)
server.settimeout(0)

print("BLD750 + AS5047P FULL BENCH TEST READY")
print("Wi-Fi:", SSID, "Password:", PASSWORD)
print("Open: http://" + ap.ifconfig()[0])
print("GP8630N I2C(1): SDA7 SCL21 | BLD: BRK13 EN14 F/R15")
print("AS5047P: CS16 SCK18 MOSI41 MISO42")
print("Encoder:", encoder["magnet"], "raw", encoder["raw"])

try:
    while True:
        service_encoder()
        service_traction()

        if (
            armed and not link_timeout_active
            and time.ticks_diff(time.ticks_ms(), last_command_ms)
            > CONTROL_LINK_TIMEOUT_MS
        ):
            request_soft_stop("CONTROL LINK TIMEOUT")
            link_timeout_active = True

        try:
            client, _ = server.accept()
        except OSError:
            time.sleep_us(100)
            continue

        try:
            request = client.recv(1200).decode("utf-8", "ignore")
            first = request.split("\r\n", 1)[0].split(" ")
            path = first[1] if len(first) > 1 else "/bad"

            last_command_ms = time.ticks_ms()
            link_timeout_active = False

            if path == "/":
                reply(client, HTML, content_type="text/html; charset=utf-8")
            elif path.startswith("/status"):
                reply(client, json_status(), content_type="application/json; charset=utf-8")
            elif path.startswith("/arm"):
                if not gp_online:
                    reply(client, "GP8630N OFFLINE", "409 Conflict")
                else:
                    arm_controller()
                    reply(client, "ARMED AT 0 V")
            elif path.startswith("/soft_stop"):
                request_soft_stop()
                reply(client, "THROTTLE RAMPING TO ZERO")
            elif path.startswith("/lock"):
                lock_wheel("REMOTE WHEEL LOCK")
                reply(client, "WHEEL LOCKED: SV=0 V, EN=STOP, BRK=APPLIED")
            elif path.startswith("/disarm"):
                safe_stop("REMOTE DISARM")
                reply(client, "DISARMED + WHEEL LOCKED")
            elif path.startswith("/drive"):
                requested = query(path, "d", "FORWARD")
                if not armed:
                    reply(client, "ARM FIRST", "409 Conflict")
                elif not gp_online:
                    reply(client, "GP8630N OFFLINE", "409 Conflict")
                elif requested not in ("FORWARD", "REVERSE"):
                    reply(client, "BAD DIRECTION", "400 Bad Request")
                else:
                    request_drive(requested)
                    reply(client, requested + " REQUESTED")
            elif path.startswith("/gear"):
                requested = int(query(path, "n", "0"))
                if requested < 0 or requested > 5:
                    reply(client, "GEAR MUST BE 0..5", "400 Bad Request")
                elif requested == 0:
                    request_soft_stop("THROTTLE OFF")
                    reply(client, "THROTTLE RAMPING TO ZERO")
                else:
                    selected_gear = requested
                    if target_voltage > 0.0 and not reversal_pending:
                        target_voltage = float(selected_gear)
                    reply(client, "GEAR {} SELECTED / {:.1f} V".format(
                        selected_gear, float(selected_gear)
                    ))
            elif path.startswith("/zero"):
                if armed or current_voltage > 0.01:
                    reply(client, "DISARM AND STOP BEFORE ZERO", "409 Conflict")
                elif not encoder_ready():
                    reply(client, "AS5047P NOT READY", "409 Conflict")
                else:
                    zero_encoder()
                    reply(client, "ENCODER, TURNS AND DISTANCE ZEROED")
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
    safe_stop("PROGRAM EXIT")
    server.close()

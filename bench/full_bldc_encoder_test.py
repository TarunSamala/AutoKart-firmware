"""ESP32-S3 full BLD750 + GP8630N + AS5047P Wi-Fi bench test.

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
COMMAND_TIMEOUT_MS = 1800
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

running = False
brake_released = False
direction = "CCW"
current_voltage = 0.0
target_voltage = 0.0
last_command_ms = time.ticks_ms()
last_ramp_ms = time.ticks_ms()


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
    fr_pin.value(1 if value == "CCW" else 0)


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


def safe_stop(reason):
    global target_voltage
    target_voltage = 0.0
    try:
        write_voltage(0.0)
    except Exception:
        pass
    apply_brake()
    enable_stop()
    print("SAFE STOP:", reason)


def service_ramp():
    global last_ramp_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_ramp_ms) < RAMP_INTERVAL_MS:
        return
    last_ramp_ms = now
    if not running or not brake_released:
        if current_voltage > 0.0:
            safe_stop("INTERLOCK")
        return
    if current_voltage < target_voltage:
        write_voltage(min(target_voltage, current_voltage + RAMP_STEP_V))
    elif current_voltage > target_voltage:
        write_voltage(max(target_voltage, current_voltage - RAMP_STEP_V))


write_voltage(0.0)
update_diagnostics()
update_encoder()


HTML = r'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>BLD750 Encoder Test</title><style>
:root{--bg:#080c12;--card:#131c28;--line:#29384d;--text:#f5f8ff;--muted:#93a2b7;--green:#18b978;--red:#e94255;--blue:#397dff;--orange:#ef8d31}*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 50% -10%,#203451,var(--bg) 42%);color:var(--text);font-family:system-ui,sans-serif}.app{max-width:620px;margin:auto;padding:14px}.top{display:flex;justify-content:space-between;align-items:center;gap:10px}.top h1{font-size:24px;margin:8px 0}.badge{padding:8px 12px;border-radius:999px;background:#441821;color:#ff9aa5;font-size:11px;font-weight:900}.badge.on{background:#123d2c;color:#78e9b5}.card{background:linear-gradient(145deg,#172231,var(--card));border:1px solid var(--line);border-radius:18px;padding:14px;margin-top:11px}.label{color:var(--muted);font-size:10px;font-weight:900;letter-spacing:.14em;margin-bottom:9px}.voltage{text-align:center;font-size:44px;font-weight:900}.voltage small{font-size:16px;color:var(--muted)}input{width:100%;accent-color:var(--blue)}.row{display:flex;gap:8px}.row>*{flex:1}button{min-height:52px;border:0;border-radius:13px;color:white;background:#2b394d;font-size:15px;font-weight:850;box-shadow:inset 0 1px #ffffff22}button:active{transform:scale(.97)}button:disabled{opacity:.35}.green{background:var(--green)}.red{background:var(--red)}.blue{background:var(--blue)}.orange{background:var(--orange)}.stop{width:100%;min-height:66px;font-size:19px;margin-top:10px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}.tile{background:#0c131d;border:1px solid #253449;border-radius:12px;padding:10px}.tile span{display:block;color:var(--muted);font-size:9px;font-weight:800;letter-spacing:.08em}.tile b{font-size:16px}.wide{grid-column:1/-1}.ok{color:#73e6ae}.bad{color:#ff929d}.msg{text-align:center;color:#ffd078;min-height:25px;margin-top:10px;font-size:12px}.warning{text-align:center;color:#ff929d;font-size:11px;font-weight:800;margin:14px}@media(max-width:390px){.app{padding:8px}.card{padding:11px}.top h1{font-size:21px}}
</style></head><body><main class="app"><div class="top"><h1>BLD750 Encoder Test</h1><div id="runState" class="badge">SAFE</div></div>
<section class="card"><div class="label">SV TARGET</div><div class="voltage"><span id="targetV">0.00</span> <small>V</small></div><input id="slider" type="range" min="0" max="500" value="0" step="1"><div class="row"><button id="runBtn" class="green">RUN</button><button id="stopBtn" class="red">STOP + BRAKE</button></div></section>
<section class="card"><div class="label">DIRECTION · CHANGE ONLY WHILE STOPPED</div><div class="row"><button id="ccwBtn" class="blue">CCW / FORWARD</button><button id="cwBtn" class="orange">CW / REVERSE</button></div><button id="zeroBtn" style="width:100%;margin-top:9px">ZERO ENCODER + DISTANCE</button></section>
<section class="card"><div class="label">LIVE AS5047P FEEDBACK</div><div class="grid"><div class="tile"><span>MOTOR RPM</span><b id="motorRpm">0</b></div><div class="tile"><span>AXLE RPM</span><b id="axleRpm">0</b></div><div class="tile"><span>FULL MOTOR TURNS</span><b id="turns">0</b></div><div class="tile"><span>MOTOR REVOLUTIONS</span><b id="revs">0.000</b></div><div class="tile"><span>NOMINAL SPEED</span><b id="speed">0.00 km/h</b></div><div class="tile"><span>DISTANCE TRAVELLED</span><b id="distance">0.000 m</b></div><div class="tile"><span>SIGNED DISPLACEMENT</span><b id="displacement">0.000 m</b></div><div class="tile"><span>ANGLE FROM ZERO</span><b id="angle">0.0°</b></div><div class="tile wide"><span>ENCODER HEALTH</span><b id="health">CHECKING</b><div id="detail" style="color:var(--muted);font-size:10px;margin-top:5px">--</div></div></div></section>
<button id="safeBtn" class="red stop">SAFE STOP</button><div id="message" class="msg">Connecting…</div><div class="warning">BENCH TEST ONLY · DRIVE WHEELS MUST BE RAISED AND GUARDED</div></main><script>
const $=x=>document.getElementById(x);let sending=false,pending=false;function msg(t,b=false){$('message').textContent=t;$('message').className='msg '+(b?'bad':'')}async function cmd(p,quiet=false){try{const r=await fetch(p,{cache:'no-store'}),t=await r.text();if(!quiet)msg(t,!r.ok);return r.ok}catch(e){if(!quiet)msg('CONTROLLER OFFLINE',true);return false}}function zeroSlider(){$('slider').value=0;$('targetV').textContent='0.00'}async function sendSpeed(){if(sending){pending=true;return}sending=true;const v=Number($('slider').value)/100;await cmd('/speed?v='+v.toFixed(2));sending=false;if(pending){pending=false;setTimeout(sendSpeed,40)}}$('slider').oninput=()=>{$('targetV').textContent=(Number($('slider').value)/100).toFixed(2);sendSpeed()};$('runBtn').onclick=()=>cmd('/run');$('stopBtn').onclick=()=>{zeroSlider();cmd('/stop')};$('safeBtn').onclick=()=>{zeroSlider();cmd('/stop')};$('ccwBtn').onclick=()=>cmd('/direction?d=CCW');$('cwBtn').onclick=()=>cmd('/direction?d=CW');$('zeroBtn').onclick=()=>cmd('/zero');
async function status(){try{const r=await fetch('/status',{cache:'no-store'}),s=await r.json();$('runState').textContent=s.running?'RUNNING':'SAFE';$('runState').classList.toggle('on',s.running);$('motorRpm').textContent=Math.abs(s.rpm).toFixed(0);$('axleRpm').textContent=Math.abs(s.axle_rpm).toFixed(0);$('turns').textContent=s.full_turns;$('revs').textContent=s.motor_revs.toFixed(3);$('speed').textContent=s.speed_kmh.toFixed(2)+' km/h';$('distance').textContent=s.distance_m.toFixed(3)+' m';$('displacement').textContent=s.displacement_m.toFixed(3)+' m';$('angle').textContent=s.relative_deg.toFixed(1)+'°';$('health').textContent=s.encoder_ready?'MAGNET OK':'NOT READY: '+s.magnet;$('health').className=s.encoder_ready?'ok':'bad';$('detail').textContent='raw '+s.raw+' · errors '+s.errors+' · AGC '+s.agc+' · MAG '+s.magnitude;$('ccwBtn').disabled=s.running;$('cwBtn').disabled=s.running;$('zeroBtn').disabled=s.running;$('slider').disabled=!s.running;if(!s.running&&s.target_voltage===0)$('slider').value=0}catch(e){msg('STATUS OFFLINE',true)}}setInterval(()=>cmd('/heartbeat',true),400);setInterval(status,250);addEventListener('pagehide',()=>navigator.sendBeacon('/stop'));status();
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
        '{{"running":{},"brake_released":{},"direction":"{}",'
        '"voltage":{:.2f},"target_voltage":{:.2f},"gp_online":{},'
        '"encoder_ready":{},"magnet":"{}","raw":{},'
        '"absolute_deg":{:.3f},"relative_deg":{:.3f},'
        '"rpm":{:.2f},"axle_rpm":{:.2f},"motor_revs":{:.5f},'
        '"full_turns":{},"speed_kmh":{:.3f},"distance_m":{:.4f},'
        '"displacement_m":{:.4f},"agc":{},"magnitude":{},"errors":{}}}'
    ).format(
        str(running).lower(), str(brake_released).lower(), direction,
        current_voltage, target_voltage, str(gp_online).lower(),
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
        service_ramp()

        dangerous = (
            running or brake_released or current_voltage > 0.01
            or target_voltage > 0.01
        )
        if dangerous and time.ticks_diff(time.ticks_ms(), last_command_ms) > COMMAND_TIMEOUT_MS:
            safe_stop("WEB WATCHDOG")
            last_command_ms = time.ticks_ms()
        if running and not encoder_ready():
            safe_stop("ENCODER INVALID OR STALE")

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
                reply(client, HTML, content_type="text/html; charset=utf-8")
            elif path.startswith("/status"):
                reply(client, json_status(), content_type="application/json; charset=utf-8")
            elif path.startswith("/heartbeat"):
                last_command_ms = time.ticks_ms()
                reply(client, "OK")
            elif path.startswith("/run"):
                if not gp_online:
                    reply(client, "GP8630N OFFLINE", "409 Conflict")
                elif not encoder_ready():
                    reply(client, "AS5047P NOT READY", "409 Conflict")
                elif current_voltage > 0.01 or target_voltage > 0.01:
                    reply(client, "SET THROTTLE TO ZERO", "409 Conflict")
                else:
                    release_brake()
                    enable_run()
                    last_command_ms = time.ticks_ms()
                    reply(client, "RUN ENABLED AT 0 V")
            elif path.startswith("/stop"):
                safe_stop("REMOTE STOP")
                last_command_ms = time.ticks_ms()
                reply(client, "STOPPED + BRAKE APPLIED")
            elif path.startswith("/speed"):
                requested = clamp(float(query(path, "v", "0")), 0.0, MAX_VOLTAGE)
                if not running or not brake_released:
                    reply(client, "PRESS RUN FIRST", "409 Conflict")
                elif not encoder_ready():
                    safe_stop("ENCODER INVALID")
                    reply(client, "ENCODER INVALID: STOPPED", "409 Conflict")
                else:
                    target_voltage = requested
                    last_command_ms = time.ticks_ms()
                    reply(client, "TARGET {:.2f} V".format(requested))
            elif path.startswith("/direction"):
                requested = query(path, "d", "CCW")
                if requested not in ("CW", "CCW"):
                    reply(client, "BAD DIRECTION", "400 Bad Request")
                elif running or brake_released or current_voltage > 0.01:
                    reply(client, "STOP + BRAKE BEFORE DIRECTION CHANGE", "409 Conflict")
                else:
                    set_direction(requested)
                    last_command_ms = time.ticks_ms()
                    reply(client, "DIRECTION " + requested)
            elif path.startswith("/zero"):
                if running or brake_released or current_voltage > 0.01:
                    reply(client, "STOP + BRAKE BEFORE ZERO", "409 Conflict")
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

"""ESP32-S3 BLD-750 controller with Pico/RA-02 fail-stop interlock.

BLDC outputs:
  GPIO13 -> BRK transistor
  GPIO14 -> EN transistor
  GPIO15 -> F/R transistor

RA-02 alternate pins, avoiding BLDC GPIO13/14/15:
  GPIO4=SCK, GPIO5=MOSI, GPIO6=MISO, GPIO7=CS
  GPIO10=RESET, GPIO18=DIO0

The controller starts safe. It will not permit traction unless a valid
ESTOP,0 heartbeat has arrived recently from the Pico 2.
"""

import network
import socket
import time
from machine import Pin, SPI, I2C
from ra02_lora_chat_common import SX1278


# -------------------- BLD-750 / GP8630N --------------------
SDA_PIN = 8
SCL_PIN = 9
GP_ADDR = 0x58
REG_MODE = 0x01
REG_DAC = 0x02
MODE_0_10V = 0x1C

BRK_PIN = 13
EN_PIN = 14
FR_PIN = 15

MAX_VOLTAGE = 5.0
RAMP_STEP = 0.05
RAMP_DELAY_MS = 40
WEB_TIMEOUT_MS = 1500
LORA_TIMEOUT_MS = 900


# -------------------- RA-02 on ESP32-S3 --------------------
radio_spi = SPI(
    2,
    baudrate=2000000,
    polarity=0,
    phase=0,
    sck=Pin(4),
    mosi=Pin(5),
    miso=Pin(6),
)
radio = SX1278(
    radio_spi,
    Pin(7, Pin.OUT, value=1),
    Pin(10, Pin.OUT, value=1),
    433000000,
)


en_pin = Pin(EN_PIN, Pin.OUT, value=0)
brk_pin = Pin(BRK_PIN, Pin.OUT, value=0)
fr_pin = Pin(FR_PIN, Pin.OUT, value=0)

en_running = False
brake_released = False
direction = "CW"
target_voltage = 0.0
current_voltage = 0.0
last_web_command = time.ticks_ms()
last_lora_heartbeat = None
remote_estop = True
last_remote_packet = "NONE"


def enable_run():
    global en_running
    en_pin.value(1)
    en_running = True
    print("EN -> RUN")


def enable_stop():
    global en_running
    en_pin.value(0)
    en_running = False
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


def direction_cw():
    global direction
    fr_pin.value(0)       # F/R high/open at BLD-750 input
    direction = "CW"
    print("DIRECTION -> CW")


def direction_ccw():
    global direction
    fr_pin.value(1)       # transistor pulls F/R low
    direction = "CCW"
    print("DIRECTION -> CCW")


def write_voltage(voltage):
    global current_voltage
    voltage = max(0.0, min(MAX_VOLTAGE, voltage))
    dac = int((voltage / 10.0) * 65535)
    i2c.writeto_mem(GP_ADDR, REG_DAC, bytes((dac & 0xFF, (dac >> 8) & 0xFF)))
    current_voltage = voltage


def safe_stop(reason):
    global target_voltage
    target_voltage = 0.0
    print("!!! {} !!!".format(reason))
    try:
        write_voltage(0.0)
    except Exception as error:
        print("DAC ZERO FAILED:", error)
    brake_apply()
    enable_stop()


def process_lora():
    global last_lora_heartbeat, remote_estop, last_remote_packet
    packet = radio.receive()
    if not packet:
        return
    try:
        text = packet.decode().strip()
        fields = text.split(",")
        if len(fields) != 3 or fields[0] != "ESTOP":
            return
        state = int(fields[1])
        int(fields[2])
        if state not in (0, 1):
            return
    except Exception:
        return
    last_lora_heartbeat = time.ticks_ms()
    last_remote_packet = text
    remote_estop = state == 1
    if remote_estop:
        safe_stop("REMOTE LORA E-STOP")
    print("LORA:", text)


def request_value(request, marker):
    try:
        start = request.find(marker)
        if start < 0:
            return None
        value = request[start + len(marker):].split(" ")[0].split("&")[0]
        return float(value)
    except Exception:
        return None


def response(client, body, status="200 OK", content_type="text/plain"):
    data = body.encode()
    header = (
        "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n"
    ).format(status, content_type, len(data))
    client.send(header.encode())
    client.send(data)


HTML = """<!doctype html><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>AutoKart LoRa BLDC E-Stop</title><style>
body{background:#111;color:#fff;font:18px Arial;text-align:center;padding:15px}
button{font-size:18px;padding:14px;margin:5px;border:0;border-radius:8px}
.run{background:#20a052;color:#fff}.stop{background:#d22;color:#fff}
input{width:95%}.safe{background:#f22;color:#fff;width:90%}
</style><h2>AutoKart BLDC</h2><div id='s'>SAFE</div>
<p><span id='v'>0.00</span> V</p>
<input id='slider' type='range' min='0' max='500' value='0' step='1'>
<p><button class='run' onclick='run()'>EN RUN</button>
<button class='stop' onclick='stop()'>EN STOP</button></p>
<p><button class='run' onclick='release()'>BRK RELEASE</button>
<button class='stop' onclick='apply()'>BRK APPLY</button></p>
<p><button onclick='cw()'>CW</button><button onclick='ccw()'>CCW</button></p>
<button class='safe' onclick='safe()'>SAFE STOP</button>
<script>
let en=false,br=false,busy=false,pending=false,s=document.getElementById('slider');
function q(u){fetch(u).catch(()=>{});}
function send(){if(!en||!br||busy||!pending)return;pending=false;busy=true;
fetch('/speed?v='+(s.value/100)).catch(()=>{}).finally(()=>{busy=false;if(pending)setTimeout(send,50)})}
s.oninput=()=>{document.getElementById('v').textContent=(s.value/100).toFixed(2);pending=true;send()};
function run(){q('/en?state=run');en=true} function stop(){en=false;s.value=0;q('/en?state=stop')}
function release(){q('/brake?state=release');br=true} function apply(){br=false;s.value=0;q('/brake?state=apply')}
function cw(){q('/direction?d=cw')} function ccw(){q('/direction?d=ccw')}
function safe(){en=false;br=false;s.value=0;q('/safe_stop')}
setInterval(()=>q('/heartbeat'),300);
</script>"""


# Safe output state before peripherals and network.
enable_stop()
brake_apply()
direction_cw()

i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=100000)
if GP_ADDR not in i2c.scan():
    raise RuntimeError("GP8630N NOT FOUND AT 0x58")
i2c.writeto_mem(GP_ADDR, REG_MODE, bytes((MODE_0_10V,)))
write_voltage(0.0)
radio.begin()

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid="BLD750-LoRa-EStop", password="12345678")
while not ap.active():
    time.sleep_ms(100)

server = socket.socket()
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(socket.getaddrinfo("0.0.0.0", 80)[0][-1])
server.listen(2)
server.settimeout(0.02)

print("BLDC + LORA E-STOP READY")
print("IP:", ap.ifconfig()[0])
print("REMOTE ESTOP REQUIRED BEFORE RUN")
print("RA-02 SPI: SCK4 MOSI5 MISO6 CS7 RST10 DIO18")
print("BLDC: BRK13 EN14 F/R15")

try:
    while True:
        process_lora()

        if (
            last_lora_heartbeat is None or
            time.ticks_diff(time.ticks_ms(), last_lora_heartbeat) > LORA_TIMEOUT_MS
        ):
            if en_running or brake_released or current_voltage > 0.01:
                safe_stop("LORA HEARTBEAT TIMEOUT")
            remote_estop = True

        try:
            client, address = server.accept()
            request = client.recv(1024).decode("utf-8", "ignore")
            last_web_command = time.ticks_ms()

            if "GET /safe_stop" in request:
                safe_stop("WEB SAFE STOP")
            elif "GET /en?state=run" in request:
                if not remote_estop and last_lora_heartbeat is not None:
                    enable_run()
                else:
                    response(client, "REMOTE E-STOP NOT CLEAR", "409 Conflict")
            elif "GET /en?state=stop" in request:
                safe_stop("WEB EN STOP")
            elif "GET /brake?state=release" in request:
                if not remote_estop and current_voltage <= 0.01:
                    brake_release()
                else:
                    response(client, "REMOTE STOP OR THROTTLE ACTIVE", "409 Conflict")
            elif "GET /brake?state=apply" in request:
                safe_stop("WEB BRAKE APPLY")
            elif "GET /direction?d=cw" in request:
                if current_voltage <= 0.01 and not en_running and not brake_released:
                    direction_cw()
                else:
                    response(client, "STOP + BRAKE REQUIRED", "409 Conflict")
            elif "GET /direction?d=ccw" in request:
                if current_voltage <= 0.01 and not en_running and not brake_released:
                    direction_ccw()
                else:
                    response(client, "STOP + BRAKE REQUIRED", "409 Conflict")
            elif "GET /speed" in request:
                value = request_value(request, "v=")
                if value is not None and en_running and brake_released and not remote_estop:
                    target_voltage = max(0.0, min(MAX_VOLTAGE, value))
                else:
                    target_voltage = 0.0
                    response(client, "INTERLOCK", "409 Conflict")
            elif "GET /heartbeat" in request:
                pass
            else:
                response(client, HTML, content_type="text/html")
            client.close()
        except OSError:
            pass

        if en_running or brake_released or target_voltage > 0.01 or current_voltage > 0.01:
            if time.ticks_diff(time.ticks_ms(), last_web_command) > WEB_TIMEOUT_MS:
                safe_stop("WEB HEARTBEAT TIMEOUT")
                last_web_command = time.ticks_ms()

        try:
            if current_voltage < target_voltage:
                write_voltage(min(current_voltage + RAMP_STEP, target_voltage))
            elif current_voltage > target_voltage:
                write_voltage(max(current_voltage - RAMP_STEP, target_voltage))
        except Exception as error:
            safe_stop("GP8630N I2C FAULT")
            print(error)
        time.sleep_ms(RAMP_DELAY_MS)
finally:
    safe_stop("PROGRAM EXIT")
    server.close()

import network
import socket
import time
from machine import Pin, I2C

# ===============  2877777777777/;'x=============================================
# ESP32-S3 + GP8630N + BLD-750
# Wi-Fi SPEED-ONLY BENCH CONTROLLER
#
# GP8630N:
# SDA = GPIO8
# SCL = GPIO9
# Address = 0x58
#
# BLD-750:
# GP8630N OUT -> SV
# GP8630N GND -> COM
#
# EN  -> COM (manual jumper)
# BRK -> COM (manual jumper)
#
# MAXIMUM ANALOG COMMAND = 3.00 V
# ============================================================


# ------------------------------------------------------------
# CONFIGURATION
# ------------------------------------------------------------

SSID = "BLD750-Control"
PASSWORD = "12345678"

SDA_PIN = 8
SCL_PIN = 9

GP_ADDR = 0x58

REG_MODE = 0x01
REG_DAC = 0x02

# GP8630N internal output mode
MODE_0_10V = 0x1C

# HARD BENCH LIMIT
MAX_VOLTAGE = 3.00

# Ramp
RAMP_STEP = 0.05
RAMP_DELAY = 0.04

# Browser watchdog
COMMAND_TIMEOUT_MS = 5000


# ------------------------------------------------------------
# I2C / GP8630N
# ------------------------------------------------------------

i2c = I2C(
    0,
    sda=Pin(SDA_PIN),
    scl=Pin(SCL_PIN),
    freq=100000
)

devices = i2c.scan()

if GP_ADDR not in devices:
    raise RuntimeError("GP8630N NOT FOUND")

print("GP8630N detected at", hex(GP_ADDR))

# Configure GP8630N for 0-10V mode
i2c.writeto_mem(
    GP_ADDR,
    REG_MODE,
    bytes([MODE_0_10V])
)

time.sleep_ms(200)


# ------------------------------------------------------------
# VARIABLES
# ------------------------------------------------------------

current_voltage = 0.0
target_voltage = 0.0
running = False

last_command_ms = time.ticks_ms()


# ------------------------------------------------------------
# DAC CONTROL
# ------------------------------------------------------------

def write_voltage(voltage):

    global current_voltage

    # ABSOLUTE SOFTWARE LIMIT
    if voltage < 0:
        voltage = 0.0

    if voltage > MAX_VOLTAGE:
        voltage = MAX_VOLTAGE

    # GP8630N configured as 0-10V
    #
    # 0V  -> 0
    # 10V -> 65535

    dac = int(
        (voltage / 10.0) * 65535
    )

    low = dac & 0xFF
    high = (dac >> 8) & 0xFF

    i2c.writeto_mem(
        GP_ADDR,
        REG_DAC,
        bytes([low, high])
    )

    current_voltage = voltage


def force_zero():

    global target_voltage
    global running

    target_voltage = 0.0
    running = False

    write_voltage(0.0)

    print("SV -> 0.00 V")


# ------------------------------------------------------------
# SAFE BOOT
# ------------------------------------------------------------

force_zero()


# ------------------------------------------------------------
# Wi-Fi ACCESS POINT
# ------------------------------------------------------------

ap = network.WLAN(network.AP_IF)

ap.active(True)

ap.config(
    essid=SSID,
    password=PASSWORD
)

while not ap.active():
    time.sleep_ms(100)

print()
print("================================")
print(" BLD-750 Wi-Fi Controller")
print("================================")
print("SSID:", SSID)
print("Password:", PASSWORD)
print("IP:", ap.ifconfig()[0])
print("MAX SV:", MAX_VOLTAGE, "V")
print("================================")
print()


# ------------------------------------------------------------
# HTML
# ------------------------------------------------------------

HTML = """<!DOCTYPE html>
<html>

<head>

<meta name="viewport"
content="width=device-width, initial-scale=1">

<title>BLD-750 Control</title>

<style>

body {
    font-family: Arial, sans-serif;
    background: #111;
    color: white;
    text-align: center;
    margin: 0;
    padding: 25px;
}

.container {
    max-width: 500px;
    margin: auto;
}

h1 {
    font-size: 28px;
}

.status {
    font-size: 28px;
    margin: 20px;
}

.voltage {
    font-size: 48px;
    font-weight: bold;
    margin: 25px;
}

.slider {
    width: 95%;
    margin: 30px 0;
}

button {
    width: 45%;
    padding: 18px;
    margin: 8px;
    font-size: 20px;
    border-radius: 10px;
    border: none;
}

.start {
    background: #21a55b;
    color: white;
}

.stop {
    background: #d93636;
    color: white;
}

.info {
    color: #aaa;
    margin-top: 30px;
}

</style>

</head>


<body>

<div class="container">

<h1>BLD-750 Bench Control</h1>

<div id="status" class="status">
STOPPED
</div>

<div class="voltage">
<span id="voltage">0.00</span> V
</div>


<input
    type="range"
    min="0"
    max="300"
    value="0"
    step="5"
    class="slider"
    id="slider"
    oninput="sliderChanged()"
>


<br>

<button
class="start"
onclick="startMotor()">
START
</button>

<button
class="stop"
onclick="stopMotor()">
STOP
</button>


<div class="info">

Maximum SV command: 3.00 V

<br><br>

EN → COM<br>
BRK → COM

</div>

</div>


<script>

let running = false;
let heartbeat = null;


function sliderChanged() {

    let slider = document.getElementById("slider");

    let voltage = slider.value / 100;

    document.getElementById("voltage").innerHTML =
        voltage.toFixed(2);

    if(running) {

        fetch("/speed?v=" + voltage);

    }

}


function startMotor() {

    running = true;

    document.getElementById("status").innerHTML =
        "RUNNING";

    let slider =
        document.getElementById("slider");

    let voltage =
        slider.value / 100;

    fetch("/start?v=" + voltage);

}


function stopMotor() {

    running = false;

    document.getElementById("status").innerHTML =
        "STOPPED";

    document.getElementById("slider").value = 0;

    document.getElementById("voltage").innerHTML =
        "0.00";

    fetch("/stop");

}


heartbeat = setInterval(function() {

    if(running) {

        fetch("/heartbeat");

    }

}, 1000);


window.addEventListener(
    "beforeunload",
    function() {
        fetch("/stop");
    }
);

</script>

</body>

</html>
"""


# ------------------------------------------------------------
# HTTP RESPONSE
# ------------------------------------------------------------

def send_response(client, body, content_type="text/html"):

    client.send(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Connection: close\r\n"
        "\r\n"
    )

    client.send(body)


# ------------------------------------------------------------
# PARSE VOLTAGE
# ------------------------------------------------------------

def get_voltage_from_request(request):

    try:

        marker = "v="

        position = request.find(marker)

        if position == -1:
            return 0.0

        value = request[
            position + len(marker):
        ]

        value = value.split(" ")[0]
        value = value.split("&")[0]

        voltage = float(value)

        # HARD LIMIT AGAIN
        voltage = max(
            0.0,
            min(MAX_VOLTAGE, voltage)
        )

        return voltage

    except:

        return 0.0


# ------------------------------------------------------------
# SERVER
# ------------------------------------------------------------

addr = socket.getaddrinfo(
    "0.0.0.0",
    80
)[0][-1]

server = socket.socket()

server.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1
)

server.bind(addr)

server.listen(2)

server.settimeout(0.05)

print("Web server ready")
print("Open: http://192.168.4.1")


# ------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------

try:

    while True:

        # ====================================================
        # WEB REQUEST
        # ====================================================

        try:

            client, address = server.accept()

            request = client.recv(1024)

            request = request.decode(
                "utf-8",
                "ignore"
            )

            # -----------------------------------------------
            # START
            # -----------------------------------------------

            if "GET /start" in request:

                voltage = get_voltage_from_request(
                    request
                )

                target_voltage = voltage
                running = True

                last_command_ms = time.ticks_ms()

                print(
                    "START ->",
                    target_voltage,
                    "V"
                )

                send_response(
                    client,
                    "OK",
                    "text/plain"
                )


            # -----------------------------------------------
            # STOP
            # -----------------------------------------------

            elif "GET /stop" in request:

                target_voltage = 0.0
                running = False

                last_command_ms = time.ticks_ms()

                print("STOP")

                send_response(
                    client,
                    "OK",
                    "text/plain"
                )


            # -----------------------------------------------
            # SPEED
            # -----------------------------------------------

            elif "GET /speed" in request:

                voltage = get_voltage_from_request(
                    request
                )

                if running:

                    target_voltage = voltage

                last_command_ms = time.ticks_ms()

                print(
                    "TARGET ->",
                    target_voltage,
                    "V"
                )

                send_response(
                    client,
                    "OK",
                    "text/plain"
                )


            # -----------------------------------------------
            # HEARTBEAT
            # -----------------------------------------------

            elif "GET /heartbeat" in request:

                last_command_ms = time.ticks_ms()

                send_response(
                    client,
                    "OK",
                    "text/plain"
                )


            # -----------------------------------------------
            # WEB PAGE
            # -----------------------------------------------

            else:

                send_response(
                    client,
                    HTML
                )


            client.close()


        except OSError:

            pass


        # ====================================================
        # WATCHDOG
        # ====================================================

        if running:

            elapsed = time.ticks_diff(
                time.ticks_ms(),
                last_command_ms
            )

            if elapsed > COMMAND_TIMEOUT_MS:

                print("COMMAND TIMEOUT -> 0V")

                target_voltage = 0.0
                running = False


        # ====================================================
        # RAMP CONTROL
        # ====================================================

        if current_voltage < target_voltage:

            new_voltage = min(
                current_voltage + RAMP_STEP,
                target_voltage
            )

            write_voltage(new_voltage)


        elif current_voltage > target_voltage:

            new_voltage = max(
                current_voltage - RAMP_STEP,
                target_voltage
            )

            write_voltage(new_voltage)


        time.sleep_ms(
            int(RAMP_DELAY * 1000)
        )


# ------------------------------------------------------------
# CTRL+C / EXCEPTION
# ------------------------------------------------------------

except KeyboardInterrupt:

    print("CTRL+C")


finally:

    try:
        force_zero()

    except:
        pass

    try:
        server.close()

    except:
        pass

    print("CONTROLLER STOPPED")
    print("SV REQUESTED = 0V")
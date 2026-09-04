import network
import socket
import time
from machine import Pin, I2C


# ============================================================
# AUTOKART - ESP32-S3 + GP8630N + BLD-750
# COMPLETE TRACTION BENCH CONTROLLER
#
# BENCH / WHEELS-OFF-GROUND TESTING ONLY
#
# GP8630N
#   SDA = GPIO8
#   SCL = GPIO9
#   Address = 0x58
#
# PN2222A CONTROL
#   GPIO13 -> EN transistor
#   GPIO14 -> BRK transistor
#   GPIO15 -> F/R transistor
#
# NPN LOGIC:
#   GPIO HIGH -> transistor ON -> BLD input pulled LOW
#   GPIO LOW  -> transistor OFF -> BLD input HIGH / OPEN
#
# BLD-750:
#   EN LOW  = RUN
#   EN HIGH = STOP
#
#   BRK LOW  = RUN / BRAKE RELEASED
#   BRK HIGH = QUICK BRAKE
#
#   F/R LOW  = CCW
#   F/R HIGH = CW
#
# SV:
#   GP8630N OUT -> BLD SV
#   GP8630N GND -> BLD COM
#
# ============================================================


# ============================================================
# CONFIGURATION
# ============================================================

SSID = "BLD750-Control"
PASSWORD = "12345678"


# ----------------------------
# GP8630N
# ----------------------------

SDA_PIN = 8
SCL_PIN = 9

GP_ADDR = 0x58

REG_MODE = 0x01
REG_DAC = 0x02

MODE_0_10V = 0x1C


# ----------------------------
# BLD DIGITAL CONTROL
# ----------------------------

EN_PIN = 13
BRK_PIN = 14
FR_PIN = 15


# ----------------------------
# THROTTLE LIMIT
# ----------------------------

MAX_VOLTAGE = 3.00

RAMP_STEP = 0.05
RAMP_DELAY_MS = 40


# ----------------------------
# WEB WATCHDOG
# ----------------------------
#
# Browser sends heartbeat every 400 ms.
#
# If communication disappears:
#   SV -> 0 immediately
#   BRK -> brake
#   EN -> stop
#
# BENCH ONLY.
#

COMMAND_TIMEOUT_MS = 1500


# ============================================================
# BLD CONTROL OUTPUTS
# ============================================================

en_pin = Pin(EN_PIN, Pin.OUT)
brk_pin = Pin(BRK_PIN, Pin.OUT)
fr_pin = Pin(FR_PIN, Pin.OUT)


# Internal states
en_running = False
brake_released = False
direction = "CW"


# ------------------------------------------------------------
# ENABLE
# ------------------------------------------------------------

def enable_run():

    global en_running

    # GPIO HIGH
    # transistor ON
    # BLD EN -> COM
    # EN LOW = RUN

    en_pin.value(1)

    en_running = True

    print("EN -> RUN")


def enable_stop():

    global en_running

    # transistor OFF
    # BLD EN open/high
    # EN HIGH = STOP

    en_pin.value(0)

    en_running = False

    print("EN -> STOP")


# ------------------------------------------------------------
# BRAKE
# ------------------------------------------------------------

def brake_release():

    global brake_released

    # GPIO HIGH
    # transistor ON
    # BRK LOW = RUN

    brk_pin.value(1)

    brake_released = True

    print("BRK -> RELEASED")


def brake_apply():

    global brake_released

    # GPIO LOW
    # transistor OFF
    # BRK HIGH/open = QUICK BRAKE

    brk_pin.value(0)

    brake_released = False

    print("BRK -> APPLIED")


# ------------------------------------------------------------
# DIRECTION
# ------------------------------------------------------------

def direction_cw():

    global direction

    # transistor OFF
    # F/R open/high
    # CW according to BLD manual

    fr_pin.value(0)

    direction = "CW"

    print("DIRECTION -> CW")


def direction_ccw():

    global direction

    # transistor ON
    # F/R pulled LOW
    # CCW according to BLD manual

    fr_pin.value(1)

    direction = "CCW"

    print("DIRECTION -> CCW")


# ============================================================
# CRITICAL SAFE BOOT
# ============================================================

# IMPORTANT:
# Do this BEFORE starting I2C, Wi-Fi, sockets, etc.

enable_stop()
brake_apply()
direction_cw()


# ============================================================
# GP8630N I2C
# ============================================================

i2c = I2C(
    0,
    sda=Pin(SDA_PIN),
    scl=Pin(SCL_PIN),
    freq=100000
)


devices = i2c.scan()

print("I2C:", [hex(x) for x in devices])


if GP_ADDR not in devices:

    enable_stop()
    brake_apply()

    raise RuntimeError(
        "GP8630N NOT FOUND AT 0x58"
    )


print(
    "GP8630N detected:",
    hex(GP_ADDR)
)


# ------------------------------------------------------------
# SET 0-10V MODE
# ------------------------------------------------------------

i2c.writeto_mem(
    GP_ADDR,
    REG_MODE,
    bytes([MODE_0_10V])
)

time.sleep_ms(100)


# ============================================================
# THROTTLE VARIABLES
# ============================================================

current_voltage = 0.0
target_voltage = 0.0

last_command_ms = time.ticks_ms()


# ============================================================
# DAC FUNCTIONS
# ============================================================

def write_voltage(voltage):

    global current_voltage

    # Clamp
    if voltage < 0.0:
        voltage = 0.0

    if voltage > MAX_VOLTAGE:
        voltage = MAX_VOLTAGE


    # GP8630N:
    #
    # 0 V  = 0
    # 10 V = 65535

    dac = int(
        (voltage / 10.0) * 65535
    )


    low = dac & 0xFF
    high = (dac >> 8) & 0xFF


    i2c.writeto_mem(
        GP_ADDR,
        REG_DAC,
        bytes([
            low,
            high
        ])
    )


    current_voltage = voltage


# ------------------------------------------------------------
# IMMEDIATE SAFE STATE
# ------------------------------------------------------------

def safe_stop(reason="SAFE STOP"):

    global target_voltage
    global current_voltage

    print()
    print("!!!", reason, "!!!")


    # First remove throttle request
    target_voltage = 0.0


    # Attempt immediate analog zero
    try:

        write_voltage(0.0)

    except Exception as e:

        print(
            "DAC ZERO FAILED:",
            e
        )


    # Hardware control inputs provide another stopping layer
    brake_apply()

    enable_stop()


    print("SV -> 0V")
    print("BRK -> APPLIED")
    print("EN -> STOP")
    print()


# ============================================================
# DAC SAFE START
# ============================================================

write_voltage(0.0)

print("SV -> 0.00 V")


# ============================================================
# WI-FI ACCESS POINT
# ============================================================

ap = network.WLAN(network.AP_IF)

ap.active(True)

ap.config(
    essid=SSID,
    password=PASSWORD
)


while not ap.active():

    time.sleep_ms(100)


print()
print("==============================")
print(" BLD-750 COMPLETE CONTROLLER")
print("==============================")
print("SSID:", SSID)
print("Password:", PASSWORD)
print("IP:", ap.ifconfig()[0])
print("MAX SV:", MAX_VOLTAGE, "V")
print("EN GPIO:", EN_PIN)
print("BRK GPIO:", BRK_PIN)
print("F/R GPIO:", FR_PIN)
print("==============================")
print()


# ============================================================
# HTML
# ============================================================

HTML = """<!DOCTYPE html>
<html>

<head>

<meta
name="viewport"
content="width=device-width, initial-scale=1">

<title>AutoKart BLD750</title>

<style>

body {
    background: #111;
    color: white;
    font-family: Arial;
    text-align: center;
    margin: 0;
    padding: 20px;
}

.container {
    max-width: 600px;
    margin: auto;
}

h1 {
    margin-bottom: 5px;
}

.subtitle {
    color: #888;
    margin-bottom: 25px;
}

.voltage {
    font-size: 52px;
    font-weight: bold;
    margin: 20px;
}

.slider {
    width: 95%;
    margin: 25px 0 35px 0;
}

.row {
    margin: 15px 0;
}

button {
    width: 42%;
    padding: 17px;
    margin: 6px;
    border: none;
    border-radius: 10px;
    font-size: 18px;
}

.green {
    background: #24a35a;
    color: white;
}

.red {
    background: #d43c3c;
    color: white;
}

.blue {
    background: #2878cf;
    color: white;
}

.orange {
    background: #d68125;
    color: white;
}

.safe {
    width: 90%;
    background: #ff2020;
    color: white;
    font-weight: bold;
    font-size: 22px;
}

.label {
    font-size: 18px;
    color: #aaa;
    margin-top: 20px;
}

#status {
    font-size: 24px;
    margin: 15px;
}

.warning {
    margin-top: 30px;
    color: #ff7777;
}

</style>

</head>


<body>

<div class="container">

<h1>AutoKart Traction Bench</h1>

<div class="subtitle">
ESP32-S3 · GP8630N · BLD-750
</div>


<div id="status">
SAFE
</div>


<div class="voltage">

<span id="voltage">
0.00
</span>

V

</div>


<input
type="range"
min="0"
max="300"
value="0"
step="5"
class="slider"
id="slider"
oninput="speedChanged()"
>


<div class="label">
ENABLE
</div>

<div class="row">

<button
class="green"
onclick="enableRun()">
EN RUN
</button>

<button
class="red"
onclick="enableStop()">
EN STOP
</button>

</div>


<div class="label">
BRAKE
</div>

<div class="row">

<button
class="green"
onclick="brakeRelease()">
BRK RELEASE
</button>

<button
class="red"
onclick="brakeApply()">
BRK APPLY
</button>

</div>


<div class="label">
DIRECTION
</div>

<div class="row">

<button
class="blue"
onclick="setCW()">
CW
</button>

<button
class="orange"
onclick="setCCW()">
CCW
</button>

</div>


<br>


<button
class="safe"
onclick="safeStop()">
SAFE STOP
</button>


<div class="warning">

BENCH TEST ONLY<br>
WHEELS OFF GROUND

</div>

</div>


<script>

let enRunning = false;
let brakeReleased = false;


function setStatus(text) {

    document.getElementById(
        "status"
    ).innerHTML = text;

}


function zeroSlider() {

    document.getElementById(
        "slider"
    ).value = 0;

    document.getElementById(
        "voltage"
    ).innerHTML = "0.00";

}


function speedChanged() {

    let slider =
        document.getElementById(
            "slider"
        );

    let voltage =
        slider.value / 100;


    document.getElementById(
        "voltage"
    ).innerHTML =
        voltage.toFixed(2);


    if(
        enRunning &&
        brakeReleased
    ) {

        fetch(
            "/speed?v=" +
            voltage
        );

    }

}


function enableRun() {

    fetch("/en?state=run")
    .then(function(response) {

        if(response.ok) {

            enRunning = true;

            setStatus(
                "EN ENABLED"
            );

        }

    });

}


function enableStop() {

    enRunning = false;

    zeroSlider();

    fetch("/en?state=stop");

    setStatus(
        "EN STOPPED"
    );

}


function brakeRelease() {

    fetch("/brake?state=release")
    .then(function(response) {

        if(response.ok) {

            brakeReleased = true;

            setStatus(
                "BRAKE RELEASED"
            );

        }

    });

}


function brakeApply() {

    brakeReleased = false;

    zeroSlider();

    fetch(
        "/brake?state=apply"
    );

    setStatus(
        "BRAKE APPLIED"
    );

}


function setCW() {

    fetch("/direction?d=cw")
    .then(function(response) {

        if(response.ok) {

            setStatus(
                "DIRECTION CW"
            );

        }
        else {

            setStatus(
                "STOP FIRST"
            );

        }

    });

}


function setCCW() {

    fetch("/direction?d=ccw")
    .then(function(response) {

        if(response.ok) {

            setStatus(
                "DIRECTION CCW"
            );

        }
        else {

            setStatus(
                "STOP FIRST"
            );

        }

    });

}


function safeStop() {

    enRunning = false;
    brakeReleased = false;

    zeroSlider();

    fetch("/safe_stop");

    setStatus(
        "SAFE STOP"
    );

}


/*
Heartbeat runs continuously.

If the ESP32 stops receiving it while
the drive is armed, ESP32 forces safe state.
*/

setInterval(
    function() {

        fetch("/heartbeat")
        .catch(function() {});

    },
    400
);


window.addEventListener(
    "beforeunload",
    function() {

        fetch("/safe_stop");

    }
);

</script>

</body>

</html>
"""


# ============================================================
# HTTP RESPONSE
# ============================================================

def send_response(
    client,
    body,
    content_type="text/html",
    status="200 OK"
):

    body_bytes = body.encode()

    header = (
        "HTTP/1.1 " + status + "\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Content-Length: " + str(len(body_bytes)) + "\r\n"
        "Connection: close\r\n"
        "\r\n"
    )

    client.send(header.encode())
    client.send(body_bytes)


# ============================================================
# REQUEST VOLTAGE
# ============================================================

def get_voltage_from_request(request):

    try:

        marker = "v="

        position = request.find(marker)


        if position == -1:

            return 0.0


        value = request[
            position + 2:
        ]

        value = value.split(
            " "
        )[0]

        value = value.split(
            "&"
        )[0]


        voltage = float(value)


        voltage = max(
            0.0,
            min(
                MAX_VOLTAGE,
                voltage
            )
        )


        return voltage


    except:

        return 0.0


# ============================================================
# WEB SERVER
# ============================================================

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
print(
    "Open: http://" +
    ap.ifconfig()[0]
)


# ============================================================
# MAIN LOOP
# ============================================================

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
            # SAFE STOP
            # -----------------------------------------------

            if "GET /safe_stop" in request:

                safe_stop(
                    "WEB SAFE STOP"
                )

                last_command_ms = time.ticks_ms()

                send_response(
                    client,
                    "SAFE",
                    "text/plain"
                )


            # -----------------------------------------------
            # ENABLE RUN
            # -----------------------------------------------

            elif "GET /en?state=run" in request:

                # Enable can be activated,
                # but motor cannot turn unless
                # BRK is also released.

                enable_run()

                last_command_ms = time.ticks_ms()

                send_response(
                    client,
                    "EN RUN",
                    "text/plain"
                )


            # -----------------------------------------------
            # ENABLE STOP
            # -----------------------------------------------

            elif "GET /en?state=stop" in request:

                target_voltage = 0.0

                try:
                    write_voltage(0.0)
                except:
                    pass

                enable_stop()

                last_command_ms = time.ticks_ms()

                send_response(
                    client,
                    "EN STOP",
                    "text/plain"
                )


            # -----------------------------------------------
            # BRAKE RELEASE
            # -----------------------------------------------

            elif "GET /brake?state=release" in request:

                # Do not release brake if throttle
                # is already non-zero.

                if current_voltage <= 0.01:

                    brake_release()

                    send_response(
                        client,
                        "BRAKE RELEASED",
                        "text/plain"
                    )

                else:

                    send_response(
                        client,
                        "THROTTLE MUST BE ZERO",
                        "text/plain",
                        "409 Conflict"
                    )


                last_command_ms = time.ticks_ms()


            # -----------------------------------------------
            # BRAKE APPLY
            # -----------------------------------------------

            elif "GET /brake?state=apply" in request:

                target_voltage = 0.0

                try:
                    write_voltage(0.0)
                except:
                    pass

                brake_apply()

                last_command_ms = time.ticks_ms()

                send_response(
                    client,
                    "BRAKE APPLIED",
                    "text/plain"
                )


            # -----------------------------------------------
            # CW
            # -----------------------------------------------

            elif "GET /direction?d=cw" in request:

                # Direction reversal allowed only
                # in full safe state.

                if (
                    current_voltage <= 0.01 and
                    target_voltage <= 0.01 and
                    not en_running and
                    not brake_released
                ):

                    direction_cw()

                    send_response(
                        client,
                        "CW",
                        "text/plain"
                    )

                else:

                    send_response(
                        client,
                        "STOP + BRAKE REQUIRED",
                        "text/plain",
                        "409 Conflict"
                    )


                last_command_ms = time.ticks_ms()


            # -----------------------------------------------
            # CCW
            # -----------------------------------------------

            elif "GET /direction?d=ccw" in request:

                if (
                    current_voltage <= 0.01 and
                    target_voltage <= 0.01 and
                    not en_running and
                    not brake_released
                ):

                    direction_ccw()

                    send_response(
                        client,
                        "CCW",
                        "text/plain"
                    )

                else:

                    send_response(
                        client,
                        "STOP + BRAKE REQUIRED",
                        "text/plain",
                        "409 Conflict"
                    )


                last_command_ms = time.ticks_ms()


            # -----------------------------------------------
            # SPEED
            # -----------------------------------------------

            elif "GET /speed" in request:

                voltage = get_voltage_from_request(
                    request
                )


                # Critical interlock:
                #
                # Throttle only accepted when
                # EN = RUN and BRK = RELEASED

                if (
                    en_running and
                    brake_released
                ):

                    target_voltage = voltage

                    print(
                        "TARGET SV ->",
                        target_voltage,
                        "V"
                    )


                    send_response(
                        client,
                        "OK",
                        "text/plain"
                    )


                else:

                    target_voltage = 0.0

                    send_response(
                        client,
                        "EN + BRK INTERLOCK",
                        "text/plain",
                        "409 Conflict"
                    )


                last_command_ms = time.ticks_ms()


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
            # PAGE
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
        # COMMUNICATION WATCHDOG
        # ====================================================

        dangerous_state = (
            en_running or
            brake_released or
            target_voltage > 0.01 or
            current_voltage > 0.01
        )


        if dangerous_state:

            elapsed = time.ticks_diff(
                time.ticks_ms(),
                last_command_ms
            )


            if elapsed > COMMAND_TIMEOUT_MS:

                safe_stop(
                    "COMMUNICATION TIMEOUT"
                )

                last_command_ms = time.ticks_ms()


        # ====================================================
        # THROTTLE RAMP
        # ====================================================

        try:

            # Rising throttle
            if current_voltage < target_voltage:

                new_voltage = min(
                    current_voltage + RAMP_STEP,
                    target_voltage
                )

                write_voltage(
                    new_voltage
                )


            # Falling throttle
            elif current_voltage > target_voltage:

                new_voltage = max(
                    current_voltage - RAMP_STEP,
                    target_voltage
                )

                write_voltage(
                    new_voltage
                )


        except Exception as e:

            print(
                "DAC ERROR:",
                e
            )

            safe_stop(
                "GP8630N I2C FAULT"
            )


        time.sleep_ms(
            RAMP_DELAY_MS
        )


# ============================================================
# CTRL+C / SOFTWARE FAILURE
# ============================================================

except KeyboardInterrupt:

    print("CTRL+C")


except Exception as e:

    print(
        "FATAL ERROR:",
        e
    )


finally:

    safe_stop(
        "PROGRAM EXIT"
    )


    try:

        server.close()

    except:

        pass


    print(
        "CONTROLLER STOPPED"
    )
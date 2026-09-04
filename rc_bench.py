import network
import socket
import time
from machine import Pin, I2C


# ============================================================
# AUTOKART RC - ESP32-S3
# ============================================================
#
# STEERING:
#   GPIO4  -> TB6600 PUL+
#   GPIO5  -> TB6600 DIR+
#   GPIO6  -> TB6600 ENA+
#
# BRAKE:
#   GPIO10 -> TB6600 PUL+
#   GPIO11 -> TB6600 DIR+
#   GPIO12 -> TB6600 ENA+
#
# GP8630N:
#   GPIO8 -> SDA
#   GPIO9 -> SCL
#   3.3V  -> VCC
#   GND   -> GND
#
# GP8630N OUT -> BLD750 SV
# GP8630N GND -> BLD750 COM
#
# BLD750:
#   EN  -> COM
#   BRK -> COM
#
# IMPORTANT:
# Steering and brake are currently OPEN LOOP.
# Start with:
#   steering physically centered
#   brake physically released
# ============================================================


# ============================================================
# WIFI
# ============================================================

SSID = "AutoKart-RC"
PASSWORD = "12345678"

# If RC heartbeat disappears:
COMMAND_TIMEOUT_MS = 1200


# ============================================================
# CURRENT TEST LIMITS
# ============================================================

# KEEP THESE CONSERVATIVE UNTIL MECHANICAL CALIBRATION

STEER_LIMIT = 10.0
BRAKE_LIMIT = 10.0

# BLD750 recently faulted around 0.75V.
# Keep below that until fault is diagnosed.
THROTTLE_MAX_V = 0.70


# ============================================================
# FINAL VALUES - ONLY AFTER VALIDATION
# ============================================================
#
# STEER_LIMIT = 45.0
# BRAKE_LIMIT = 30.0
# THROTTLE_MAX_V = 3.0
#
# DO NOT unlock these yet.
# ============================================================


# ============================================================
# TB6600 GPIO
# ============================================================

STEER_STEP = Pin(4, Pin.OUT, value=0)
STEER_DIR  = Pin(5, Pin.OUT, value=0)

BRAKE_STEP = Pin(10, Pin.OUT, value=0)
BRAKE_DIR  = Pin(11, Pin.OUT, value=0)


# ============================================================
# TB6600 ENABLE
# ============================================================
#
# Current assumption:
# EN active LOW.
#
# If your tested TB6600 behaves opposite,
# swap these two values.
# ============================================================

TB_ENABLE_LEVEL = 0
TB_DISABLE_LEVEL = 1

STEER_EN = Pin(
    6,
    Pin.OUT,
    value=TB_DISABLE_LEVEL
)

BRAKE_EN = Pin(
    12,
    Pin.OUT,
    value=TB_DISABLE_LEVEL
)


# ============================================================
# TB6600 MICROSTEP CONFIGURATION
# ============================================================
#
# Your selected setting:
#
# S1 = OFF
# S2 = OFF
# S3 = ON
#
# 3200 pulses / revolution
# ============================================================

PULSES_PER_REV = 3200


# ============================================================
# MECHANICAL CALIBRATION
# ============================================================
#
# THESE RATIOS MUST EVENTUALLY BE MEASURED.
#
# If motor gear = 15 teeth
# steering gear = 45 teeth:
#
# STEER_GEAR_RATIO = 45 / 15 = 3
#
# For now = 1.0.
# ============================================================

STEER_GEAR_RATIO = 1.0
BRAKE_GEAR_RATIO = 1.0


STEER_PULSES_PER_DEG = (
    PULSES_PER_REV / 360.0
) * STEER_GEAR_RATIO


BRAKE_PULSES_PER_DEG = (
    PULSES_PER_REV / 360.0
) * BRAKE_GEAR_RATIO


# ============================================================
# DIRECTION
# ============================================================
#
# User-established:
#
# Steering RIGHT = clockwise
# Steering LEFT  = anticlockwise
#
# Brake APPLY = clockwise
#
# If physical direction is backwards,
# change 1 -> 0.
# ============================================================

STEER_POSITIVE_DIR = 1
BRAKE_POSITIVE_DIR = 1


# ============================================================
# STEPPER SPEED
# ============================================================
#
# Lower period = faster movement.
#
# Previous code could also feel slow because the HTTP socket
# blocked the control loop for ~10ms.
#
# Server below is now NON-BLOCKING.
# ============================================================

STEER_STEP_PERIOD_US = 2500
BRAKE_STEP_PERIOD_US = 3000

STEP_PULSE_US = 10


# ============================================================
# STEPPER AXIS
# ============================================================

class StepperAxis:

    def __init__(
        self,
        step_pin,
        dir_pin,
        pulses_per_deg,
        min_deg,
        max_deg,
        positive_dir,
        step_period_us
    ):

        self.step_pin = step_pin
        self.dir_pin = dir_pin

        self.pulses_per_deg = pulses_per_deg

        self.min_deg = min_deg
        self.max_deg = max_deg

        self.positive_dir = positive_dir

        self.step_period_us = step_period_us

        # OPEN-LOOP POSITION
        #
        # Assumes physical mechanism starts at zero.

        self.current_steps = 0
        self.target_steps = 0

        self.last_step_us = time.ticks_us()


    # --------------------------------------------------------
    # SET TARGET
    # --------------------------------------------------------

    def set_target(self, degrees):

        degrees = float(degrees)

        degrees = max(
            self.min_deg,
            min(
                self.max_deg,
                degrees
            )
        )

        self.target_steps = round(
            degrees *
            self.pulses_per_deg
        )


    # --------------------------------------------------------
    # CURRENT POSITION
    # --------------------------------------------------------

    def current_deg(self):

        return (
            self.current_steps /
            self.pulses_per_deg
        )


    # --------------------------------------------------------
    # TARGET POSITION
    # --------------------------------------------------------

    def target_deg(self):

        return (
            self.target_steps /
            self.pulses_per_deg
        )


    # --------------------------------------------------------
    # NON-BLOCKING UPDATE
    # --------------------------------------------------------

    def update(self):

        if self.current_steps == self.target_steps:
            return

        now = time.ticks_us()

        elapsed = time.ticks_diff(
            now,
            self.last_step_us
        )

        if elapsed < self.step_period_us:
            return


        # POSITIVE MOTION

        if self.target_steps > self.current_steps:

            direction = 1

            self.dir_pin.value(
                self.positive_dir
            )


        # NEGATIVE MOTION

        else:

            direction = -1

            self.dir_pin.value(
                1 - self.positive_dir
            )


        # STEP PULSE

        self.step_pin.value(1)

        time.sleep_us(
            STEP_PULSE_US
        )

        self.step_pin.value(0)


        self.current_steps += direction

        self.last_step_us = now


# ============================================================
# CREATE STEERING / BRAKE
# ============================================================

steering = StepperAxis(

    STEER_STEP,
    STEER_DIR,

    STEER_PULSES_PER_DEG,

    -STEER_LIMIT,
    STEER_LIMIT,

    STEER_POSITIVE_DIR,

    STEER_STEP_PERIOD_US
)


brake = StepperAxis(

    BRAKE_STEP,
    BRAKE_DIR,

    BRAKE_PULSES_PER_DEG,

    0,
    BRAKE_LIMIT,

    BRAKE_POSITIVE_DIR,

    BRAKE_STEP_PERIOD_US
)


# ============================================================
# GP8630N I2C
# ============================================================

SDA_PIN = 8
SCL_PIN = 9

GP_ADDR = 0x58

REG_MODE = 0x01
REG_DAC  = 0x02

MODE_0_10V = 0x1C


i2c = I2C(

    0,

    sda=Pin(SDA_PIN),
    scl=Pin(SCL_PIN),

    freq=100000
)


print()
print("Scanning I2C...")


devices = i2c.scan()


if GP_ADDR not in devices:

    print(
        "I2C devices:",
        devices
    )

    raise RuntimeError(
        "GP8630N NOT FOUND"
    )


print(
    "GP8630N FOUND:",
    hex(GP_ADDR)
)


# ============================================================
# GP8630N MODE
# ============================================================

i2c.writeto_mem(

    GP_ADDR,

    REG_MODE,

    bytes([
        MODE_0_10V
    ])
)


time.sleep_ms(100)


# ============================================================
# THROTTLE
# ============================================================

throttle_current = 0.0
throttle_target = 0.0


# Rise slowly
THROTTLE_RISE_STEP = 0.01

# Fall significantly faster
THROTTLE_FALL_STEP = 0.05

THROTTLE_UPDATE_MS = 20


last_throttle_update = (
    time.ticks_ms()
)


# ============================================================
# DAC OUTPUT
# ============================================================

def set_dac_voltage(voltage):

    global throttle_current


    voltage = max(
        0.0,
        min(
            THROTTLE_MAX_V,
            float(voltage)
        )
    )


    # GP8630N currently configured 0-10V.

    dac = int(
        (voltage / 10.0)
        * 65535
    )


    low = dac & 0xFF

    high = (
        dac >> 8
    ) & 0xFF


    i2c.writeto_mem(

        GP_ADDR,

        REG_DAC,

        bytes([
            low,
            high
        ])
    )


    throttle_current = voltage


# ============================================================
# THROTTLE UPDATE
# ============================================================

def throttle_update():

    global throttle_current
    global last_throttle_update


    now = time.ticks_ms()


    if time.ticks_diff(
        now,
        last_throttle_update
    ) < THROTTLE_UPDATE_MS:

        return


    last_throttle_update = now


    # ACCELERATION

    if throttle_current < throttle_target:

        value = min(

            throttle_current
            + THROTTLE_RISE_STEP,

            throttle_target
        )

        set_dac_voltage(
            value
        )


    # DECELERATION

    elif throttle_current > throttle_target:

        value = max(

            throttle_current
            - THROTTLE_FALL_STEP,

            throttle_target
        )

        set_dac_voltage(
            value
        )


# ============================================================
# IMMEDIATE THROTTLE ZERO
# ============================================================

def throttle_zero():

    global throttle_target

    throttle_target = 0.0

    set_dac_voltage(
        0.0
    )


# ============================================================
# SAFE INITIALIZATION
# ============================================================

throttle_zero()

steering.set_target(0)

brake.set_target(0)


# Enable stepper drivers only after software initialization

STEER_EN.value(
    TB_ENABLE_LEVEL
)

BRAKE_EN.value(
    TB_ENABLE_LEVEL
)


armed = False

last_contact = time.ticks_ms()


# ============================================================
# WIFI ACCESS POINT
# ============================================================

ap = network.WLAN(
    network.AP_IF
)

ap.active(True)


ap.config(

    essid=SSID,
    password=PASSWORD
)


while not ap.active():

    time.sleep_ms(100)


IP = ap.ifconfig()[0]


print()
print("================================")
print("        AUTOKART RC")
print("================================")
print("WiFi :", SSID)
print("Pass :", PASSWORD)
print("URL  : http://" + IP)
print()
print(
    "Steering limit : +/-",
    STEER_LIMIT,
    "deg"
)
print(
    "Brake limit    :",
    BRAKE_LIMIT,
    "deg"
)
print(
    "Throttle limit :",
    THROTTLE_MAX_V,
    "V"
)
print("================================")


# ============================================================
# HTML TEMPLATE
# ============================================================

HTML = """
<!DOCTYPE html>

<html>

<head>

<meta
name="viewport"
content="
width=device-width,
initial-scale=1,
maximum-scale=1,
user-scalable=no
">

<title>
AutoKart RC
</title>


<style>

* {

    box-sizing: border-box;

    -webkit-tap-highlight-color:
    transparent;

    user-select: none;
}


body {

    margin: 0;

    background:
    #0b0d10;

    color:
    white;

    font-family:
    Arial,
    sans-serif;

    overscroll-behavior:
    none;
}


.app {

    max-width:
    650px;

    margin:
    auto;

    padding:
    16px;
}


/* HEADER */

.top {

    display:
    flex;

    justify-content:
    space-between;

    align-items:
    center;

    margin-bottom:
    18px;
}


.title {

    font-size:
    25px;

    font-weight:
    bold;
}


#state {

    padding:
    9px 15px;

    border-radius:
    25px;

    background:
    #333;

    font-weight:
    bold;
}


.state-armed {

    background:
    #178c50 !important;
}


.state-stopped {

    background:
    #a12e2e !important;
}


/* MAIN RC */

.rc {

    display:
    grid;

    grid-template-columns:
    1fr 120px;

    gap:
    15px;
}


.controls {

    background:
    #171a20;

    border-radius:
    22px;

    padding:
    18px;

    min-width:
    0;
}


.section-title {

    text-align:
    center;

    color:
    #9ca3ad;

    font-size:
    12px;

    letter-spacing:
    2px;

    margin-bottom:
    15px;
}


/* STEERING */

.steering {

    display:
    grid;

    grid-template-columns:
    1fr 1fr;

    gap:
    16px;
}


.steer-button {

    height:
    120px;

    border:
    none;

    border-radius:
    24px;

    background:
    #2b3039;

    color:
    white;

    font-size:
    56px;

    font-weight:
    bold;

    touch-action:
    none;
}


.steer-button.active {

    background:
    #596375;

    transform:
    scale(0.96);
}


.steer-status {

    text-align:
    center;

    margin:
    18px 0 12px;

    font-size:
    34px;

    font-weight:
    bold;
}


.center-button {

    width:
    100%;

    height:
    45px;

    border:
    0;

    border-radius:
    12px;

    background:
    #343a44;

    color:
    white;

    font-size:
    14px;

    font-weight:
    bold;

    margin-bottom:
    25px;
}


/* BRAKE */

.brake-button {

    width:
    100%;

    height:
    95px;

    border:
    none;

    border-radius:
    20px;

    background:
    #bd3030;

    color:
    white;

    font-size:
    26px;

    font-weight:
    bold;

    touch-action:
    none;
}


.brake-button.braking {

    background:
    #ff3939;

    transform:
    scale(0.97);
}


/* SYSTEM */

.system {

    display:
    grid;

    grid-template-columns:
    1fr 1fr;

    gap:
    12px;

    margin-top:
    20px;
}


.system button {

    height:
    58px;

    border:
    none;

    border-radius:
    14px;

    color:
    white;

    font-size:
    17px;

    font-weight:
    bold;
}


.arm {

    background:
    #178c50;
}


.stop {

    background:
    #a62b2b;
}


/* THROTTLE */

.throttle-panel {

    background:
    #171a20;

    border-radius:
    22px;

    padding:
    12px 5px;

    text-align:
    center;

    min-width:
    0;
}


.throttle-title {

    font-size:
    11px;

    letter-spacing:
    1px;

    color:
    #9ca3ad;
}


.throttle-value {

    margin-top:
    8px;

    font-size:
    21px;

    font-weight:
    bold;
}


.throttle-box {

    height:
    320px;

    position:
    relative;

    display:
    flex;

    justify-content:
    center;

    align-items:
    center;
}


#throttle {

    width:
    260px;

    height:
    55px;

    transform:
    rotate(-90deg);

    touch-action:
    none;

    accent-color:
    #36a76c;
}


.max {

    position:
    absolute;

    top:
    12px;

    color:
    #9ca3ad;

    font-size:
    11px;
}


.zero {

    position:
    absolute;

    bottom:
    12px;

    color:
    #9ca3ad;

    font-size:
    11px;
}


/* FOOTER */

.footer {

    text-align:
    center;

    margin-top:
    14px;

    font-size:
    12px;

    color:
    #8e959f;
}


#connection {

    font-weight:
    bold;
}


/* MOBILE */

@media(max-width:420px) {

    .app {
        padding:
        10px;
    }

    .rc {

        grid-template-columns:
        minmax(0,1fr)
        100px;

        gap:
        8px;
    }

    .controls {
        padding:
        12px;
    }

    .steer-button {

        height:
        100px;

        font-size:
        46px;
    }

    .throttle-box {

        height:
        290px;
    }

    #throttle {

        width:
        235px;
    }
}

</style>

</head>


<body>


<div class="app">


<div class="top">

<div class="title">
AutoKart RC
</div>

<div
id="state"
class="state-stopped">

STOPPED

</div>

</div>



<div class="rc">


<!-- ======================================================
     LEFT CONTROL AREA
     ====================================================== -->

<div class="controls">


<div class="section-title">
STEERING
</div>


<div class="steering">


<button
id="left"
class="steer-button">

&#9664;

</button>


<button
id="right"
class="steer-button">

&#9654;

</button>


</div>


<div
id="steerValue"
class="steer-status">

0&deg;

</div>


<button
id="center"
class="center-button">

CENTER

</button>



<div class="section-title">
BRAKE
</div>


<button
id="brake"
class="brake-button">

BRAKE

</button>



<div class="system">


<button
id="arm"
class="arm">

ARM

</button>


<button
id="stop"
class="stop">

STOP

</button>


</div>


</div>



<!-- ======================================================
     THROTTLE
     ====================================================== -->

<div class="throttle-panel">


<div class="throttle-title">

THROTTLE

</div>


<div class="throttle-value">

<span id="throttleValue">
0.00
</span>

V

</div>


<div class="throttle-box">


<div class="max">
MAX
</div>


<input
id="throttle"
type="range"
min="0"
max="__THROTTLE_MAX_CV__"
step="1"
value="0">


<div class="zero">
0V
</div>


</div>


</div>


</div>



<div class="footer">

Connection:

<span id="connection">
READY
</span>

<br><br>

Hold arrows to progressively steer.

<br>

Release arrow to HOLD current steering angle.

<br>

Hold BRAKE to apply.

</div>


</div>



<script>


/* ==========================================================
   CONFIG
   ========================================================== */

const STEER_LIMIT =
__STEER_LIMIT__;

const BRAKE_LIMIT =
__BRAKE_LIMIT__;

const STEER_STEP =
1;

const STEER_INTERVAL_MS =
120;


/* ==========================================================
   STATE
   ========================================================== */

let armed =
false;

let steeringAngle =
0;

let steeringTimer =
null;


/* ==========================================================
   ELEMENTS
   ========================================================== */

const leftButton =
document.getElementById("left");

const rightButton =
document.getElementById("right");

const brakeButton =
document.getElementById("brake");

const centerButton =
document.getElementById("center");

const throttle =
document.getElementById("throttle");

const throttleValue =
document.getElementById(
    "throttleValue"
);

const steerValue =
document.getElementById(
    "steerValue"
);

const state =
document.getElementById(
    "state"
);

const connection =
document.getElementById(
    "connection"
);


/* ==========================================================
   NETWORK
   ========================================================== */

function send(url) {

    fetch(
        url,
        {
            cache:
            "no-store"
        }
    )

    .then(() => {

        connection.textContent =
        "CONNECTED";

    })

    .catch(() => {

        connection.textContent =
        "LOST";

    });

}


/* ==========================================================
   UPDATE STEERING
   ========================================================== */

function updateSteering() {

    steeringAngle =
    Math.max(
        -STEER_LIMIT,
        Math.min(
            STEER_LIMIT,
            steeringAngle
        )
    );


    steerValue.textContent =
    steeringAngle + "°";


    send(
        "/steer?v="
        + steeringAngle
    );

}


/* ==========================================================
   LEFT STEP
   ========================================================== */

function steerLeftStep() {

    if (!armed) {
        return;
    }


    steeringAngle -=
    STEER_STEP;


    updateSteering();

}


/* ==========================================================
   RIGHT STEP
   ========================================================== */

function steerRightStep() {

    if (!armed) {
        return;
    }


    steeringAngle +=
    STEER_STEP;


    updateSteering();

}


/* ==========================================================
   STOP ARROW INPUT
   ========================================================== */

function stopSteeringInput() {

    if (
        steeringTimer !== null
    ) {

        clearInterval(
            steeringTimer
        );

        steeringTimer =
        null;
    }


    leftButton.classList.remove(
        "active"
    );

    rightButton.classList.remove(
        "active"
    );

}


/* ==========================================================
   LEFT PRESS
   ========================================================== */

function startLeft(e) {

    if (!armed) {
        return;
    }


    e.preventDefault();


    stopSteeringInput();


    leftButton.classList.add(
        "active"
    );


    if (
        leftButton.setPointerCapture
    ) {

        try {

            leftButton.setPointerCapture(
                e.pointerId
            );

        } catch(err) {}

    }


    steerLeftStep();


    steeringTimer =
    setInterval(

        steerLeftStep,

        STEER_INTERVAL_MS
    );

}


/* ==========================================================
   RIGHT PRESS
   ========================================================== */

function startRight(e) {

    if (!armed) {
        return;
    }


    e.preventDefault();


    stopSteeringInput();


    rightButton.classList.add(
        "active"
    );


    if (
        rightButton.setPointerCapture
    ) {

        try {

            rightButton.setPointerCapture(
                e.pointerId
            );

        } catch(err) {}

    }


    steerRightStep();


    steeringTimer =
    setInterval(

        steerRightStep,

        STEER_INTERVAL_MS
    );

}


/* ==========================================================
   POINTER EVENTS
   ========================================================== */

leftButton.addEventListener(
    "pointerdown",
    startLeft
);


rightButton.addEventListener(
    "pointerdown",
    startRight
);


[
    "pointerup",
    "pointercancel",
    "lostpointercapture"
]
.forEach(eventName => {

    leftButton.addEventListener(
        eventName,
        stopSteeringInput
    );

    rightButton.addEventListener(
        eventName,
        stopSteeringInput
    );

});


/* ==========================================================
   CENTER STEERING
   ========================================================== */

centerButton.addEventListener(
    "click",
    function() {

        if (!armed) {
            return;
        }


        stopSteeringInput();


        steeringAngle =
        0;


        updateSteering();

    }
);


/* ==========================================================
   BRAKE
   ========================================================== */

function brakePress(e) {

    if (!armed) {
        return;
    }


    e.preventDefault();


    if (
        brakeButton.setPointerCapture
    ) {

        try {

            brakeButton.setPointerCapture(
                e.pointerId
            );

        } catch(err) {}

    }


    brakeButton.classList.add(
        "braking"
    );


    /*
    THROTTLE ZERO FIRST
    */

    throttle.value =
    0;


    throttleValue.textContent =
    "0.00";


    send(
        "/throttle?v=0"
    );


    /*
    THEN BRAKE APPLY
    */

    send(
        "/brake?v="
        + BRAKE_LIMIT
    );

}


/* ==========================================================
   BRAKE RELEASE
   ========================================================== */

function brakeRelease() {

    brakeButton.classList.remove(
        "braking"
    );


    if (!armed) {
        return;
    }


    send(
        "/brake?v=0"
    );

}


brakeButton.addEventListener(
    "pointerdown",
    brakePress
);


[
    "pointerup",
    "pointercancel",
    "lostpointercapture"
]
.forEach(eventName => {

    brakeButton.addEventListener(
        eventName,
        brakeRelease
    );

});


/* ==========================================================
   THROTTLE
   ========================================================== */

throttle.addEventListener(
    "input",
    function() {

        const voltage =

        Number(
            throttle.value
        ) / 100;


        throttleValue.textContent =

        voltage.toFixed(2);


        if (armed) {

            send(
                "/throttle?v="
                + voltage
            );

        }

    }
);


/* ==========================================================
   ARM
   ========================================================== */

document
.getElementById("arm")
.addEventListener(
    "click",
    function() {

        stopSteeringInput();


        throttle.value =
        0;


        throttleValue.textContent =
        "0.00";


        send(
            "/arm"
        );


        send(
            "/throttle?v=0"
        );


        send(
            "/brake?v=0"
        );


        armed =
        true;


        state.textContent =
        "ARMED";


        state.className =
        "state-armed";

    }
);


/* ==========================================================
   STOP
   ========================================================== */

document
.getElementById("stop")
.addEventListener(
    "click",
    function() {

        armed =
        false;


        stopSteeringInput();


        throttle.value =
        0;


        throttleValue.textContent =
        "0.00";


        send(
            "/throttle?v=0"
        );


        /*
        Apply brake when STOP is pressed.
        */

        send(
            "/brake?v="
            + BRAKE_LIMIT
        );


        send(
            "/stop"
        );


        state.textContent =
        "STOPPED";


        state.className =
        "state-stopped";

    }
);


/* ==========================================================
   HEARTBEAT
   ========================================================== */

setInterval(

    function() {

        if (armed) {

            send(
                "/heartbeat"
            );

        }

    },

    300
);


/* ==========================================================
   PAGE EXIT
   ========================================================== */

window.addEventListener(
    "pagehide",
    function() {

        send(
            "/throttle?v=0"
        );

        send(
            "/stop"
        );

    }
);


</script>


</body>

</html>
"""


# ============================================================
# INSERT PYTHON LIMITS INTO HTML
# ============================================================

HTML = HTML.replace(

    "__STEER_LIMIT__",

    str(
        int(STEER_LIMIT)
    )
)


HTML = HTML.replace(

    "__BRAKE_LIMIT__",

    str(
        int(BRAKE_LIMIT)
    )
)


HTML = HTML.replace(

    "__THROTTLE_MAX_CV__",

    str(
        int(
            THROTTLE_MAX_V
            * 100
        )
    )
)


# ============================================================
# HTTP HELPERS
# ============================================================

def send_all(
    sock,
    data
):

    if isinstance(
        data,
        str
    ):

        data = data.encode()


    while data:

        sent = sock.send(
            data
        )


        if sent <= 0:
            break


        data = data[sent:]


# ============================================================
# HTTP RESPONSE
# ============================================================

def response(
    client,
    body,
    content_type="text/plain"
):

    header = (

        "HTTP/1.1 200 OK\r\n"

        "Content-Type: "
        + content_type
        + "\r\n"

        "Cache-Control: no-store\r\n"

        "Connection: close\r\n"

        "\r\n"
    )


    send_all(
        client,
        header
    )


    send_all(
        client,
        body
    )


# ============================================================
# QUERY VALUE
# ============================================================

def query_value(path):

    try:

        value = path.split(
            "v=",
            1
        )[1]


        value = value.split(
            "&",
            1
        )[0]


        return float(
            value
        )


    except:

        return 0.0


# ============================================================
# HTTP SERVER
# ============================================================

addr = socket.getaddrinfo(

    "0.0.0.0",

    80

)[0][-1]


server = socket.socket()


try:

    server.setsockopt(

        socket.SOL_SOCKET,

        socket.SO_REUSEADDR,

        1
    )

except:
    pass


server.bind(
    addr
)


server.listen(
    2
)


# ============================================================
# IMPORTANT
#
# NON-BLOCKING SOCKET
#
# Previous 10ms socket timeout could dramatically limit the
# stepper update frequency.
# ============================================================

server.settimeout(
    0
)


print()
print(
    "RC WEB SERVER READY"
)

print(
    "Open http://"
    + IP
)


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while True:


        # ----------------------------------------------------
        # MOTOR UPDATES
        # ----------------------------------------------------

        steering.update()

        brake.update()

        throttle_update()


        now = time.ticks_ms()


        # ----------------------------------------------------
        # RC WATCHDOG
        # ----------------------------------------------------

        if armed:

            elapsed = time.ticks_diff(

                now,

                last_contact
            )


            if elapsed > COMMAND_TIMEOUT_MS:

                print(
                    "RC TIMEOUT"
                )


                armed = False


                # Immediate traction zero

                throttle_zero()


                # Request braking

                brake.set_target(
                    BRAKE_LIMIT
                )


        # ----------------------------------------------------
        # HTTP
        # ----------------------------------------------------

        try:

            client, remote = (
                server.accept()
            )


            try:

                request = client.recv(
                    1024
                )


                if not request:

                    client.close()

                    continue


                request = request.decode(

                    "utf-8",

                    "ignore"
                )


                first_line = request.split(

                    "\r\n",

                    1

                )[0]


                parts = first_line.split(
                    " "
                )


                if len(parts) < 2:

                    client.close()

                    continue


                path = parts[1]


                # --------------------------------------------
                # HOME PAGE
                # --------------------------------------------

                if path == "/":

                    response(

                        client,

                        HTML,

                        "text/html"
                    )


                # --------------------------------------------
                # ARM
                # --------------------------------------------

                elif path.startswith(
                    "/arm"
                ):

                    throttle_zero()


                    brake.set_target(
                        0
                    )


                    armed = True


                    last_contact = (
                        time.ticks_ms()
                    )


                    print(
                        "ARMED"
                    )


                    response(
                        client,
                        "OK"
                    )


                # --------------------------------------------
                # STOP
                # --------------------------------------------

                elif path.startswith(
                    "/stop"
                ):

                    armed = False


                    throttle_zero()


                    brake.set_target(
                        BRAKE_LIMIT
                    )


                    print(
                        "STOP"
                    )


                    response(
                        client,
                        "OK"
                    )


                # --------------------------------------------
                # HEARTBEAT
                # --------------------------------------------

                elif path.startswith(
                    "/heartbeat"
                ):

                    last_contact = (
                        time.ticks_ms()
                    )


                    response(
                        client,
                        "OK"
                    )


                # --------------------------------------------
                # STEERING
                # --------------------------------------------

                elif path.startswith(
                    "/steer"
                ):

                    value = query_value(
                        path
                    )


                    value = max(

                        -STEER_LIMIT,

                        min(
                            STEER_LIMIT,
                            value
                        )
                    )


                    if armed:

                        steering.set_target(
                            value
                        )


                        last_contact = (
                            time.ticks_ms()
                        )


                    print(
                        "STEER:",
                        value
                    )


                    response(
                        client,
                        "OK"
                    )


                # --------------------------------------------
                # BRAKE
                # --------------------------------------------

                elif path.startswith(
                    "/brake"
                ):

                    value = query_value(
                        path
                    )


                    value = max(

                        0,

                        min(
                            BRAKE_LIMIT,
                            value
                        )
                    )


                    if armed:

                        brake.set_target(
                            value
                        )


                        last_contact = (
                            time.ticks_ms()
                        )


                    response(
                        client,
                        "OK"
                    )


                # --------------------------------------------
                # THROTTLE
                # --------------------------------------------

                elif path.startswith(
                    "/throttle"
                ):

                    value = query_value(
                        path
                    )


                    value = max(

                        0.0,

                        min(
                            THROTTLE_MAX_V,
                            value
                        )
                    )


                    if armed:

                        throttle_target = value


                        last_contact = (
                            time.ticks_ms()
                        )


                    else:

                        throttle_target = 0.0


                    print(
                        "THROTTLE:",
                        value,
                        "V"
                    )


                    response(
                        client,
                        "OK"
                    )


                # --------------------------------------------
                # NOT FOUND
                # --------------------------------------------

                else:

                    response(

                        client,

                        "NOT FOUND"
                    )


            finally:

                try:

                    client.close()

                except:
                    pass


        except OSError:

            # Expected because socket is non-blocking
            pass


        # Small yield without slowing stepper control heavily

        time.sleep_us(
            200
        )


# ============================================================
# CTRL+C
# ============================================================

except KeyboardInterrupt:

    print()
    print(
        "CTRL+C"
    )


# ============================================================
# SHUTDOWN
# ============================================================

finally:

    # Traction zero

    try:

        throttle_zero()

    except:
        pass


    # Request brake

    try:

        brake.set_target(
            BRAKE_LIMIT
        )

    except:
        pass


    # Disable TB6600s

    try:

        STEER_EN.value(
            TB_DISABLE_LEVEL
        )

        BRAKE_EN.value(
            TB_DISABLE_LEVEL
        )

    except:
        pass


    try:

        server.close()

    except:
        pass


    print(
        "CONTROLLER STOPPED"
    )

    print(
        "THROTTLE = 0V"
    )
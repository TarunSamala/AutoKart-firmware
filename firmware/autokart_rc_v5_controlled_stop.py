import network
import socket
import time
from machine import Pin, I2C


# ============================================================
# AUTOKART RC - ESP32-S3
# RESPONSIVE OPEN-LOOP BENCH CONTROLLER
# ============================================================
#
# STEERING TB6600
#   GPIO4  -> PUL+
#   GPIO5  -> DIR+
#   GPIO6  -> ENA+
#
# BRAKE TB6600
#   GPIO10 -> PUL+
#   GPIO11 -> DIR+
#   GPIO12 -> ENA+
#
# GP8630N
#   GPIO8  -> SDA
#   GPIO9  -> SCL
#   3.3V   -> VCC
#   GND    -> GND
#
# GP8630N OUT -> BLD750 SV
# GP8630N GND -> BLD750 COM
#
# BLD750 bench jumpers:
#   EN  -> COM
#   BRK -> COM
#
# IMPORTANT:
#   Steering and brake position are still OPEN LOOP.
#   "90 degrees" and "15 degrees" are only correct if the
#   pulses/degree and mechanical ratios below are correct and
#   the mechanism starts at the assumed zero position.
# ============================================================


# ============================================================
# WIFI
# ============================================================

SSID = "AutoKart-RC"
PASSWORD = "12345678"

# Fast RC fail-safe. If the browser stops heartbeating:
# traction -> 0, steering -> hold, brake -> apply.
COMMAND_TIMEOUT_MS = 1800


# ============================================================
# MOTION LIMITS
# ============================================================

# Steering: 90 degrees each side from center.
STEER_LIMIT = 90.0

# Brake: clockwise application only, max 15 degrees.
BRAKE_LIMIT = 15.0

# Requested software ceiling. WARNING: your BLD750 previously
# faulted around 0.75 V, so 3.0 V is NOT yet electrically validated.
# Bench / wheels-off-ground testing only until that fault is resolved.
THROTTLE_MAX_V = 3.00


# ============================================================
# TB6600 GPIO
# ============================================================

STEER_STEP = Pin(4, Pin.OUT, value=0)
STEER_DIR  = Pin(5, Pin.OUT, value=0)
STEER_EN   = Pin(6, Pin.OUT, value=1)

BRAKE_STEP = Pin(10, Pin.OUT, value=0)
BRAKE_DIR  = Pin(11, Pin.OUT, value=0)
BRAKE_EN   = Pin(12, Pin.OUT, value=1)


# ============================================================
# TB6600 ENABLE POLARITY
# ============================================================

# Existing setup assumption: EN active LOW.
TB_ENABLE_LEVEL = 0
TB_DISABLE_LEVEL = 1


# ============================================================
# TB6600 MICROSTEP SETTING
# ============================================================
#
# KEEP the existing physical switch setting for this version:
#
#   S1 = OFF
#   S2 = OFF
#   S3 = ON
#
# Existing calibration assumption:
#   3200 pulses / motor revolution
#
# Do not change S1/S2/S3 unless the exact table printed on
# YOUR TB6600 is checked, then change PULSES_PER_REV to match.
# ============================================================

PULSES_PER_REV = 3200


# ============================================================
# MECHANICAL CALIBRATION
# ============================================================
#
# Ratio means:
#   motor revolutions / output revolutions
#
# 1.0 is valid only for direct 1:1 motion.
# Example:
#   15T motor gear -> 45T steering gear = 3.0
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
# Established convention:
#   Steering RIGHT = clockwise = positive
#   Steering LEFT  = anticlockwise = negative
#   Brake APPLY    = clockwise = positive
#
# If a physical axis is reversed, change its value 1 -> 0.
# ============================================================

STEER_POSITIVE_DIR = 1
# DIR level used for clockwise brake APPLY.
# If the physical brake turns anticlockwise during APPLY,
# invert ONLY this value to 0 after a bench check.
BRAKE_POSITIVE_DIR = 1


# ============================================================
# RESPONSIVE STEPPER TIMING
# ============================================================
#
# Old steering period was 2500 us:
#   400 pulses/s
#   at 3200 pulses/rev -> 45 deg/s motor speed
#
# New steering fast period = 500 us:
#   2000 pulses/s
#   at 3200 pulses/rev -> 225 deg/s motor speed @ 1:1
#
# Acceleration starts slower to reduce missed-step risk.
# If steering skips/stalls mechanically, increase FAST period.
# ============================================================

STEP_PULSE_US = 10

STEER_START_PERIOD_US = 1200
STEER_FAST_PERIOD_US = 500
STEER_ACCEL_US_PER_STEP = 25

BRAKE_START_PERIOD_US = 1500
BRAKE_FAST_PERIOD_US = 700
BRAKE_ACCEL_US_PER_STEP = 20


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
        start_period_us,
        fast_period_us,
        accel_us_per_step
    ):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.pulses_per_deg = pulses_per_deg
        self.min_deg = min_deg
        self.max_deg = max_deg
        self.positive_dir = positive_dir

        self.start_period_us = start_period_us
        self.fast_period_us = fast_period_us
        self.accel_us_per_step = accel_us_per_step

        # OPEN-LOOP position estimate.
        self.current_steps = 0
        self.target_steps = 0

        self.current_period_us = start_period_us
        self.last_direction = 0
        self.last_step_us = time.ticks_us()


    def set_target(self, degrees):
        degrees = float(degrees)

        degrees = max(
            self.min_deg,
            min(self.max_deg, degrees)
        )

        self.target_steps = round(
            degrees * self.pulses_per_deg
        )


    def hold(self):
        # Freeze target at our current OPEN-LOOP estimate.
        self.target_steps = self.current_steps
        self.current_period_us = self.start_period_us
        self.last_direction = 0


    def current_deg(self):
        return self.current_steps / self.pulses_per_deg


    def target_deg(self):
        return self.target_steps / self.pulses_per_deg


    def nudge(self, delta_deg):
        # Small deterministic tap movement from the estimated CURRENT
        # steering position. This makes an opposite-direction tap reverse
        # immediately from where the steering actually is in software.
        new_target = self.current_deg() + float(delta_deg)
        self.set_target(new_target)


    def update(self):
        if self.current_steps == self.target_steps:
            self.current_period_us = self.start_period_us
            self.last_direction = 0
            return

        direction = (
            1
            if self.target_steps > self.current_steps
            else -1
        )

        # Restart acceleration ramp on direction reversal.
        if direction != self.last_direction:
            self.current_period_us = self.start_period_us
            self.last_direction = direction

        now = time.ticks_us()

        if time.ticks_diff(
            now,
            self.last_step_us
        ) < self.current_period_us:
            return

        if direction > 0:
            self.dir_pin.value(self.positive_dir)
        else:
            self.dir_pin.value(1 - self.positive_dir)

        self.step_pin.value(1)
        time.sleep_us(STEP_PULSE_US)
        self.step_pin.value(0)

        self.current_steps += direction
        self.last_step_us = now

        # Smoothly accelerate toward the configured fast period.
        if self.current_period_us > self.fast_period_us:
            self.current_period_us = max(
                self.fast_period_us,
                self.current_period_us - self.accel_us_per_step
            )


# ============================================================
# CREATE AXES
# ============================================================

steering = StepperAxis(
    STEER_STEP,
    STEER_DIR,
    STEER_PULSES_PER_DEG,
    -STEER_LIMIT,
    STEER_LIMIT,
    STEER_POSITIVE_DIR,
    STEER_START_PERIOD_US,
    STEER_FAST_PERIOD_US,
    STEER_ACCEL_US_PER_STEP
)

brake = StepperAxis(
    BRAKE_STEP,
    BRAKE_DIR,
    BRAKE_PULSES_PER_DEG,
    0,
    BRAKE_LIMIT,
    BRAKE_POSITIVE_DIR,
    BRAKE_START_PERIOD_US,
    BRAKE_FAST_PERIOD_US,
    BRAKE_ACCEL_US_PER_STEP
)


# ============================================================
# GP8630N I2C
# ============================================================

SDA_PIN = 8
SCL_PIN = 9

GP_ADDR = 0x58
REG_MODE = 0x01
REG_DAC = 0x02
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
    print("I2C devices:", devices)
    raise RuntimeError("GP8630N NOT FOUND")

print("GP8630N FOUND:", hex(GP_ADDR))

i2c.writeto_mem(
    GP_ADDR,
    REG_MODE,
    bytes([MODE_0_10V])
)

time.sleep_ms(100)


# ============================================================
# THROTTLE
# ============================================================

throttle_current = 0.0
throttle_target = 0.0

# Responsive but still ramped.
THROTTLE_RISE_STEP = 0.02
THROTTLE_FALL_STEP = 0.10

# Controlled DISARM ramp:
# 0.04 V every 15 ms ~= 2.67 V/s.
# 3.0 V -> 0 V takes about 1.1 s, then brake applies.
CONTROLLED_STOP_FALL_STEP = 0.04

THROTTLE_UPDATE_MS = 15

last_throttle_update = time.ticks_ms()


def set_dac_voltage(voltage):
    global throttle_current

    voltage = max(
        0.0,
        min(THROTTLE_MAX_V, float(voltage))
    )

    # GP8630N is configured for 0-10 V range.
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

    throttle_current = voltage


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

    if throttle_current < throttle_target:
        set_dac_voltage(
            min(
                throttle_current + THROTTLE_RISE_STEP,
                throttle_target
            )
        )

    elif throttle_current > throttle_target:
        fall_step = (
            CONTROLLED_STOP_FALL_STEP
            if controlled_stop_active
            else THROTTLE_FALL_STEP
        )

        set_dac_voltage(
            max(
                throttle_current - fall_step,
                throttle_target
            )
        )


def throttle_zero():
    global throttle_target
    throttle_target = 0.0
    set_dac_voltage(0.0)


# ============================================================
# SAFE INITIALIZATION
# ============================================================

throttle_zero()
steering.set_target(0)
brake.set_target(BRAKE_LIMIT)

STEER_EN.value(TB_ENABLE_LEVEL)
BRAKE_EN.value(TB_ENABLE_LEVEL)

armed = False
brake_active = True
estop_latched = False
controlled_stop_active = False
last_contact = time.ticks_ms()

# Reject stale/out-of-order steering HTTP commands.
steer_cmd_seq = -1


# ============================================================
# WIFI ACCESS POINT
# ============================================================

ap = network.WLAN(network.AP_IF)
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
print("Steering : -90 to +90 deg")
print("Brake    : 0 to +15 deg CW")
print("Throttle : software max 3.0 V (bench only)")
print("================================")


# ============================================================
# UI
# No angle, voltage, percentages, or numeric readouts.
# ============================================================

HTML = r"""
<!DOCTYPE html>
<html>
<head>
<meta name="viewport"
      content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">

<title>AutoKart RC</title>

<style>
* {
    box-sizing: border-box;
    -webkit-tap-highlight-color: transparent;
    user-select: none;
}

html, body {
    margin: 0;
    width: 100%;
    min-height: 100%;
    background: #090b0e;
    color: white;
    font-family: Arial, sans-serif;
    overscroll-behavior: none;
}

body {
    min-height: 100vh;
    display: flex;
    align-items: center;
    justify-content: center;
    padding: 18px;
}

.rc {
    width: min(980px, 100%);
    display: grid;
    grid-template-columns: minmax(0, 1fr) 175px;
    gap: 16px;
    align-items: stretch;
}

.left {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 14px;
}

.left > .card:first-child {
    grid-column: 1 / -1;
}

.card,
.throttle-card {
    background: #171a20;
    border: 1px solid #252a32;
    border-radius: 22px;
    padding: 15px;
}

.label {
    text-align: center;
    font-size: 12px;
    letter-spacing: 2px;
    color: #a7adb6;
    margin-bottom: 12px;
    font-weight: 800;
}

.steering-buttons {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 14px;
}

.steer {
    height: 132px;
    border: 0;
    border-radius: 21px;
    background: #2a2f38;
    color: white;
    font-size: 64px;
    font-weight: 900;
    cursor: pointer;
    touch-action: none;
}

.steer:hover {
    background: #343b46;
}

.steer.active {
    background: #596474;
    transform: scale(.985);
}

.steer.limit {
    outline: 3px solid #ff5050;
    background: #5a2528;
}

.center {
    width: 100%;
    height: 50px;
    margin-top: 11px;
    border: 0;
    border-radius: 15px;
    background: #3b424d;
    color: white;
    font-size: 15px;
    font-weight: 800;
    cursor: pointer;
}

.center:active {
    transform: scale(.99);
    background: #596375;
}

.brake {
    width: 100%;
    height: 112px;
    border: 0;
    border-radius: 20px;
    background: #bd3030;
    color: white;
    font-size: 24px;
    font-weight: 900;
    cursor: pointer;
    touch-action: none;
}

.brake.active {
    background: #ff3a3a;
    transform: scale(.985);
}

.system {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
}

.system button {
    height: 52px;
    border: 0;
    border-radius: 15px;
    color: white;
    font-size: 15px;
    font-weight: 900;
    cursor: pointer;
}

.arm {
    background: #198b50;
}

.disarm {
    background: #8b3030;
}

.estop {
    width: 100%;
    height: 58px;
    margin-top: 10px;
    border: 3px solid #ff7777;
    border-radius: 16px;
    background: #df1111;
    color: white;
    font-size: 20px;
    font-weight: 900;
    letter-spacing: 1px;
    cursor: pointer;
}

.estop:active {
    background: #ff1616;
    transform: scale(.99);
}

.throttle-card {
    min-height: 0;
    display: flex;
    flex-direction: column;
    align-items: center;
}

.throttle-wrap {
    width: 100%;
    height: 315px;
    min-height: 0;
    display: flex;
    align-items: center;
    justify-content: center;
    overflow: hidden;
}

#throttle {
    width: 265px;
    height: 54px;
    transform: rotate(-90deg);
    touch-action: none;
    accent-color: #36a76c;
    cursor: pointer;
}

.disabled {
    opacity: .42;
}

/* E-STOP modal */
.estop-overlay {
    position: fixed;
    inset: 0;
    display: none;
    align-items: center;
    justify-content: center;
    padding: 20px;
    background: rgba(0,0,0,.90);
    z-index: 100;
}

.estop-overlay.show {
    display: flex;
}

.estop-popup {
    width: min(420px, 100%);
    padding: 28px 22px;
    border: 3px solid #ff3b3b;
    border-radius: 24px;
    background: #1b0c0c;
    text-align: center;
}

.estop-title {
    color: #ff4b4b;
    font-size: 30px;
    font-weight: 900;
    margin-bottom: 16px;
}

.estop-text {
    color: white;
    font-size: 18px;
    line-height: 1.55;
    margin-bottom: 22px;
    font-weight: 800;
}

.estop-reset {
    width: 100%;
    height: 58px;
    border: 0;
    border-radius: 15px;
    background: #444b55;
    color: white;
    font-size: 17px;
    font-weight: 900;
    cursor: pointer;
}

/* Steering-limit popup */
.limit-toast {
    position: fixed;
    left: 50%;
    top: 22px;
    transform: translateX(-50%) translateY(-18px);
    min-width: 250px;
    max-width: calc(100vw - 32px);
    padding: 15px 22px;
    border: 2px solid #ff6767;
    border-radius: 16px;
    background: #341315;
    color: white;
    text-align: center;
    font-size: 16px;
    font-weight: 900;
    letter-spacing: .5px;
    opacity: 0;
    pointer-events: none;
    transition: opacity .16s ease, transform .16s ease;
    z-index: 80;
}

.limit-toast.show {
    opacity: 1;
    transform: translateX(-50%) translateY(0);
}

/* Tablet / phone */
@media(max-width: 720px) {
    body {
        align-items: flex-start;
        padding: 8px;
    }

    .rc {
        grid-template-columns: minmax(0, 1fr) 104px;
        gap: 8px;
    }

    .left {
        grid-template-columns: 1fr;
        gap: 8px;
    }

    .left > .card:first-child {
        grid-column: auto;
    }

    .card,
    .throttle-card {
        padding: 9px;
        border-radius: 17px;
    }

    .steering-buttons {
        gap: 8px;
    }

    .steer {
        height: 96px;
        border-radius: 16px;
        font-size: 48px;
    }

    .center {
        height: 43px;
        margin-top: 8px;
    }

    .brake {
        height: 72px;
        font-size: 20px;
        border-radius: 15px;
    }

    .system {
        gap: 7px;
    }

    .system button {
        height: 48px;
        font-size: 13px;
    }

    .estop {
        height: 52px;
        font-size: 18px;
        margin-top: 7px;
    }

    .throttle-wrap {
        height: 280px;
    }

    #throttle {
        width: 235px;
        height: 48px;
    }
}
</style>
</head>

<body>

<div class="rc">

    <div class="left">

        <div class="card">
            <div class="label">STEERING</div>

            <div class="steering-buttons">
                <button id="left" class="steer">&#9664;</button>
                <button id="right" class="steer">&#9654;</button>
            </div>

            <button id="center" class="center">CENTER</button>
        </div>

        <div class="card">
            <div class="label">BRAKE</div>
            <button id="brake" class="brake">BRAKE</button>
        </div>

        <div class="card">
            <div class="system">
                <button id="arm" class="arm">ARM</button>
                <button id="disarm" class="disarm">DISARM</button>
            </div>

            <button id="estop" class="estop">E-STOP</button>
        </div>

    </div>

    <div class="throttle-card">
        <div class="label">THROTTLE</div>

        <div class="throttle-wrap">
            <input
                id="throttle"
                type="range"
                min="0"
                max="__THROTTLE_MAX_CV__"
                step="1"
                value="0">
        </div>
    </div>

</div>


<div id="limitToast" class="limit-toast">STEERING LIMIT</div>

<div id="estopOverlay" class="estop-overlay">
    <div class="estop-popup">
        <div class="estop-title">EMERGENCY STOP</div>
        <div class="estop-text">
            TRACTION COMMAND ZERO<br>
            BRAKE APPLIED CLOCKWISE<br>
            SYSTEM DISARMED
        </div>
        <button id="resetEstop" class="estop-reset">RESET E-STOP</button>
    </div>
</div>


<script>
let armed = false;
let estopLatched = false;
let throttleSendTimer = null;
let pendingThrottle = 0;

// Steering interaction:
//   quick tap  -> small deterministic nudge
//   hold       -> continuous motion toward +/-90 degrees
//   release    -> hold exactly where the ESP32 open-loop estimate is
const STEER_TAP_DEG = 4;
const STEER_HOLD_DELAY_MS = 110;

let steerHoldTimer = null;
let steeringContinuous = false;
let steeringPressedDirection = 0;
let steerSeq = 0;

const leftButton = document.getElementById("left");
const rightButton = document.getElementById("right");
const centerButton = document.getElementById("center");
const brakeButton = document.getElementById("brake");
const throttle = document.getElementById("throttle");
const estopOverlay = document.getElementById("estopOverlay");
const limitToast = document.getElementById("limitToast");

let previousSteerLimit = "MID";
let limitToastTimer = null;


function showSteeringLimit(side) {
    if (side === "LEFT") {
        limitToast.textContent = "FULL LEFT — STEERING LIMIT";
        leftButton.classList.add("limit");
        rightButton.classList.remove("limit");
    } else if (side === "RIGHT") {
        limitToast.textContent = "FULL RIGHT — STEERING LIMIT";
        rightButton.classList.add("limit");
        leftButton.classList.remove("limit");
    } else {
        leftButton.classList.remove("limit");
        rightButton.classList.remove("limit");
        return;
    }

    limitToast.classList.add("show");

    if (limitToastTimer !== null) {
        clearTimeout(limitToastTimer);
    }

    limitToastTimer = setTimeout(
        function() {
            limitToast.classList.remove("show");
        },
        1200
    );
}


function processServerState(text) {
    const parts = text.split("|");
    const state = parts[0];
    const steerLimit = parts.length > 1 ? parts[1] : "MID";

    if (state !== "ARMED") {
        armed = false;
        throttle.value = 0;
        pendingThrottle = 0;

        document.getElementById("arm").classList.remove("disabled");
        document.getElementById("disarm").classList.add("disabled");
    }

    if (steerLimit !== previousSteerLimit) {
        if (steerLimit === "LEFT" || steerLimit === "RIGHT") {
            showSteeringLimit(steerLimit);
        } else {
            showSteeringLimit("MID");
        }

        previousSteerLimit = steerLimit;
    }
}


function send(url) {
    return fetch(url, {
        cache: "no-store"
    }).catch(() => {});
}


function sendSteer(path) {
    steerSeq += 1;

    const separator = path.indexOf("?") >= 0 ? "&" : "?";
    send(path + separator + "seq=" + steerSeq);
}


function clearSteerTimer() {
    if (steerHoldTimer !== null) {
        clearTimeout(steerHoldTimer);
        steerHoldTimer = null;
    }
}


function steeringPress(e, direction) {
    if (!armed || estopLatched) return;

    e.preventDefault();

    clearSteerTimer();

    steeringPressedDirection = direction;
    steeringContinuous = false;

    const button = direction < 0 ? leftButton : rightButton;
    const other = direction < 0 ? rightButton : leftButton;

    button.classList.add("active");
    other.classList.remove("active");

    if (button.setPointerCapture) {
        try {
            button.setPointerCapture(e.pointerId);
        } catch(err) {}
    }

    // IMMEDIATE game-like response:
    // pointer-down instantly nudges from the current command position.
    sendSteer("/steer_nudge?v=" + (direction * STEER_TAP_DEG));

    // If the same press is held, transition to continuous travel.
    steerHoldTimer = setTimeout(
        function() {
            if (!armed || steeringPressedDirection !== direction) return;

            steeringContinuous = true;

            if (direction < 0) {
                sendSteer("/steer_left");
            } else {
                sendSteer("/steer_right");
            }
        },
        STEER_HOLD_DELAY_MS
    );
}


function steeringRelease(direction) {
    if (!armed) {
        clearSteerTimer();
        steeringPressedDirection = 0;
        steeringContinuous = false;
        leftButton.classList.remove("active");
        rightButton.classList.remove("active");
        return;
    }

    if (steeringPressedDirection !== direction) return;

    clearSteerTimer();

    leftButton.classList.remove("active");
    rightButton.classList.remove("active");

    if (steeringContinuous) {
        // Continuous hold was active. Stop at the current position.
        sendSteer("/steer_hold");
    }
    // Quick tap already nudged immediately on pointer-down.

    steeringPressedDirection = 0;
    steeringContinuous = false;
}


leftButton.addEventListener(
    "pointerdown",
    function(e) { steeringPress(e, -1); }
);

rightButton.addEventListener(
    "pointerdown",
    function(e) { steeringPress(e, 1); }
);

[
    "pointerup",
    "pointercancel",
    "lostpointercapture"
].forEach(name => {
    leftButton.addEventListener(
        name,
        function() { steeringRelease(-1); }
    );

    rightButton.addEventListener(
        name,
        function() { steeringRelease(1); }
    );
});


centerButton.addEventListener(
    "click",
    function() {
        if (!armed || estopLatched) return;

        clearSteerTimer();
        steeringPressedDirection = 0;
        steeringContinuous = false;
        leftButton.classList.remove("active");
        rightButton.classList.remove("active");

        // Drive back to the software-defined mechanical center.
        previousSteerLimit = "MID";
        showSteeringLimit("MID");
        sendSteer("/steer_center");
    }
);


function brakePress(e) {
    if (!armed || estopLatched) return;

    e.preventDefault();

    if (brakeButton.setPointerCapture) {
        try {
            brakeButton.setPointerCapture(e.pointerId);
        } catch(err) {}
    }

    brakeButton.classList.add("active");

    // UI immediately returns throttle to zero.
    throttle.value = 0;

    // Server performs traction-zero + brake-apply atomically.
    send("/brake_on");
}


function brakeRelease() {
    brakeButton.classList.remove("active");

    if (armed) {
        send("/brake_off");
    }
}


brakeButton.addEventListener("pointerdown", brakePress);

[
    "pointerup",
    "pointercancel",
    "lostpointercapture"
].forEach(name => {
    brakeButton.addEventListener(name, brakeRelease);
});


function sendPendingThrottle() {
    throttleSendTimer = null;

    if (!armed || estopLatched) return;

    send("/throttle?v=" + pendingThrottle);
}


throttle.addEventListener("input", function() {
    pendingThrottle = Number(throttle.value) / 100;

    if (!armed || estopLatched) {
        throttle.value = 0;
        pendingThrottle = 0;
        return;
    }

    // Avoid flooding MicroPython with a request for every
    // tiny browser slider event.
    if (throttleSendTimer === null) {
        throttleSendTimer = setTimeout(
            sendPendingThrottle,
            40
        );
    }
});


[
    "change",
    "pointerup",
    "pointercancel"
].forEach(name => {
    throttle.addEventListener(name, function() {
        pendingThrottle = Number(throttle.value) / 100;

        if (armed && !estopLatched) {
            send("/throttle?v=" + pendingThrottle);
        }
    });
});


document.getElementById("arm").addEventListener(
    "click",
    function() {
        if (estopLatched) {
            estopOverlay.classList.add("show");
            return;
        }

        throttle.value = 0;
        pendingThrottle = 0;

        send("/arm")
        .then(function(r) { return r.text(); })
        .then(function(t) {
            if (t === "OK") {
                armed = true;
                document.getElementById("arm").classList.add("disabled");
                document.getElementById("disarm").classList.remove("disabled");
            } else {
                armed = false;
            }
        });
    }
);


document.getElementById("disarm").addEventListener(
    "click",
    function() {
        armed = false;
        throttle.value = 0;
        pendingThrottle = 0;

        leftButton.classList.remove("active");
        rightButton.classList.remove("active");
        brakeButton.classList.remove("active");

        // DISARM requests controlled deceleration.
        send("/stop");

        document.getElementById("arm").classList.remove("disabled");
        document.getElementById("disarm").classList.add("disabled");
    }
);


document.getElementById("estop").addEventListener(
    "click",
    function() {
        armed = false;
        estopLatched = true;

        clearSteerTimer();
        steeringPressedDirection = 0;
        steeringContinuous = false;
        leftButton.classList.remove("active");
        rightButton.classList.remove("active");
        brakeButton.classList.remove("active");

        throttle.value = 0;
        pendingThrottle = 0;

        estopOverlay.classList.add("show");
        send("/estop");
    }
);


document.getElementById("resetEstop").addEventListener(
    "click",
    function() {
        send("/estop_reset").then(function() {
            estopLatched = false;
            armed = false;
            throttle.value = 0;
            pendingThrottle = 0;
            estopOverlay.classList.remove("show");
        });
    }
);


setInterval(
    function() {
        if (armed && !estopLatched) {
            send("/heartbeat")
            .then(function(r) { return r.text(); })
            .then(function(t) {
                processServerState(t);
            });
        }
    },
    250
);


window.addEventListener(
    "pagehide",
    function() {
        // Losing the RC page is a communication-loss event.
        send("/estop");
    }
);


// Initial UI state.
document.getElementById("disarm").classList.add("disabled");
</script>

</body>
</html>
"""

HTML = HTML.replace(
    "__THROTTLE_MAX_CV__",
    str(int(THROTTLE_MAX_V * 100))
)


# ============================================================
# HTTP HELPERS
# ============================================================

def send_all(sock, data):
    if isinstance(data, str):
        data = data.encode()

    while data:
        sent = sock.send(data)

        if sent <= 0:
            break

        data = data[sent:]


def response(client, body, content_type="text/plain"):
    header = (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n"
        "\r\n"
    )

    send_all(client, header)
    send_all(client, body)


def query_value(path):
    try:
        value = path.split("v=", 1)[1]
        value = value.split("&", 1)[0]
        return float(value)
    except:
        return 0.0


def query_int(path, key, default=-1):
    try:
        value = path.split(key + "=", 1)[1]
        value = value.split("&", 1)[0]
        return int(value)
    except:
        return default


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

server.bind(addr)
server.listen(2)
server.settimeout(0)

print()
print("RC WEB SERVER READY - E-STOP ENABLED")
print("Open http://" + IP)


# ============================================================
# MAIN LOOP
# ============================================================

try:
    while True:

        steering.update()
        brake.update()
        throttle_update()

        now = time.ticks_ms()

        # ----------------------------------------------------
        # CONTROLLED STOP SEQUENCE
        # ----------------------------------------------------

        if controlled_stop_active:
            if throttle_current <= 0.01:
                set_dac_voltage(0.0)

                brake_active = True
                brake.set_target(BRAKE_LIMIT)

                controlled_stop_active = False

                print(
                    "CONTROLLED STOP COMPLETE - "
                    "THROTTLE ZERO / BRAKE CW"
                )

        # ----------------------------------------------------
        # RC WATCHDOG
        # ----------------------------------------------------

        if armed:
            elapsed = time.ticks_diff(
                now,
                last_contact
            )

            if elapsed > COMMAND_TIMEOUT_MS:
                print("RC TIMEOUT - FAIL SAFE")

                controlled_stop_active = False
                armed = False
                brake_active = True

                throttle_zero()
                steering.hold()
                brake.set_target(BRAKE_LIMIT)

        # ----------------------------------------------------
        # HTTP
        # ----------------------------------------------------

        try:
            client, remote = server.accept()

            try:
                request = client.recv(1024)

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

                parts = first_line.split(" ")

                if len(parts) < 2:
                    client.close()
                    continue

                path = parts[1]

                # --------------------------------------------
                # LATCHED E-STOP
                # --------------------------------------------

                if path.startswith("/estop_reset"):
                    controlled_stop_active = False
                    estop_latched = False
                    armed = False
                    brake_active = True

                    throttle_zero()
                    steering.hold()
                    brake.set_target(BRAKE_LIMIT)

                    print("E-STOP RESET - STILL DISARMED")
                    response(client, "OK")
                    continue

                elif path.startswith("/estop"):
                    controlled_stop_active = False
                    estop_latched = True
                    armed = False
                    brake_active = True

                    # TRUE E-STOP: immediate traction zero + brake.
                    throttle_zero()
                    steering.hold()
                    brake.set_target(BRAKE_LIMIT)

                    print("!!! E-STOP - THROTTLE ZERO / BRAKE CW !!!")
                    response(client, "OK")
                    continue

                # While latched, all commands except page load/reset are blocked.
                if estop_latched and path != "/":
                    response(client, "ESTOP_LATCHED")
                    continue

                # --------------------------------------------
                # HOME
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

                elif path.startswith("/arm"):
                    controlled_stop_active = False

                    throttle_zero()
                    steering.hold()

                    brake_active = False
                    brake.set_target(0)

                    # New UI arm session starts steering sequence from zero.
                    steer_cmd_seq = -1

                    armed = True
                    last_contact = time.ticks_ms()

                    print("ARMED")
                    response(client, "OK")

                # --------------------------------------------
                # DISARM / STOP
                # --------------------------------------------

                elif path.startswith("/stop"):
                    # CONTROLLED STOP / DISARM:
                    # ramp traction command down first, then brake.
                    armed = False
                    controlled_stop_active = True

                    throttle_target = 0.0
                    steering.hold()

                    print("CONTROLLED STOP - RAMPING THROTTLE")
                    response(client, "STOPPING")

                # --------------------------------------------
                # HEARTBEAT
                # --------------------------------------------

                elif path.startswith("/heartbeat"):
                    steer_deg = steering.current_deg()

                    if steer_deg <= (-STEER_LIMIT + 0.25):
                        steer_limit_state = "LEFT"
                    elif steer_deg >= (STEER_LIMIT - 0.25):
                        steer_limit_state = "RIGHT"
                    else:
                        steer_limit_state = "MID"

                    if armed:
                        last_contact = time.ticks_ms()
                        response(
                            client,
                            "ARMED|" + steer_limit_state
                        )
                    else:
                        response(
                            client,
                            "DISARMED|" + steer_limit_state
                        )

                # --------------------------------------------
                # STEERING LEFT / RIGHT / TAP / HOLD / CENTER
                # --------------------------------------------

                elif path.startswith("/steer_left"):
                    seq = query_int(path, "seq", -1)

                    if armed and (seq < 0 or seq > steer_cmd_seq):
                        if seq >= 0:
                            steer_cmd_seq = seq

                        steering.set_target(-STEER_LIMIT)
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                elif path.startswith("/steer_right"):
                    seq = query_int(path, "seq", -1)

                    if armed and (seq < 0 or seq > steer_cmd_seq):
                        if seq >= 0:
                            steer_cmd_seq = seq

                        steering.set_target(STEER_LIMIT)
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                elif path.startswith("/steer_nudge"):
                    seq = query_int(path, "seq", -1)
                    value = query_value(path)

                    if armed and (seq < 0 or seq > steer_cmd_seq):
                        if seq >= 0:
                            steer_cmd_seq = seq

                        # UI currently uses +/-4 deg per quick tap.
                        value = max(-10.0, min(10.0, value))
                        steering.nudge(value)
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                elif path.startswith("/steer_hold"):
                    seq = query_int(path, "seq", -1)

                    if armed and (seq < 0 or seq > steer_cmd_seq):
                        if seq >= 0:
                            steer_cmd_seq = seq

                        steering.hold()
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                elif path.startswith("/steer_center"):
                    seq = query_int(path, "seq", -1)

                    if armed and (seq < 0 or seq > steer_cmd_seq):
                        if seq >= 0:
                            steer_cmd_seq = seq

                        steering.set_target(0)
                        last_contact = time.ticks_ms()

                    response(client, "OK")

                # --------------------------------------------
                # BRAKE APPLY
                # --------------------------------------------

                elif path.startswith("/brake_on"):
                    if armed:
                        # Brake/throttle interlock:
                        # brake command always kills traction first.
                        throttle_zero()

                        brake_active = True
                        brake.set_target(BRAKE_LIMIT)

                        last_contact = time.ticks_ms()

                    response(client, "OK")

                # --------------------------------------------
                # BRAKE RELEASE
                # --------------------------------------------

                elif path.startswith("/brake_off"):
                    if armed:
                        brake_active = False
                        brake.set_target(0)

                        # Require a new throttle command after braking.
                        throttle_zero()

                        last_contact = time.ticks_ms()

                    response(client, "OK")

                # --------------------------------------------
                # THROTTLE
                # --------------------------------------------

                elif path.startswith("/throttle"):
                    value = query_value(path)

                    value = max(
                        0.0,
                        min(THROTTLE_MAX_V, value)
                    )

                    if armed and not brake_active:
                        throttle_target = value
                        last_contact = time.ticks_ms()
                    else:
                        throttle_target = 0.0

                    response(client, "OK")

                else:
                    response(client, "NOT FOUND")

            finally:
                try:
                    client.close()
                except:
                    pass

        except OSError:
            # Expected from non-blocking accept().
            pass

        # Short cooperative yield.
        time.sleep_us(150)


except KeyboardInterrupt:
    print()
    print("CTRL+C")


finally:
    try:
        throttle_zero()
    except:
        pass

    try:
        steering.hold()
    except:
        pass

    try:
        brake.set_target(BRAKE_LIMIT)
    except:
        pass

    # Give brake a short opportunity to move before disabling
    # only when this controlled shutdown path is reached.
    shutdown_start = time.ticks_ms()

    try:
        while time.ticks_diff(
            time.ticks_ms(),
            shutdown_start
        ) < 300:
            brake.update()
            time.sleep_us(150)
    except:
        pass

    try:
        STEER_EN.value(TB_DISABLE_LEVEL)
        BRAKE_EN.value(TB_DISABLE_LEVEL)
    except:
        pass

    try:
        server.close()
    except:
        pass

    print("CONTROLLER STOPPED")
    print("THROTTLE = 0")


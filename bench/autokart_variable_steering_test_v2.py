"""
AutoKart Variable Steering Test UI v2.0
ESP32-S3 + R86mini + NEMA34 + MicroPython

Purpose
-------
Bench-test steering with:
- center calibration
- hard software steering limit: +/-70 degrees
- variable target angle
- variable start PPS / max PPS / ramp pulses
- smooth finite S-curve motion
- jog buttons
- driver enable/release
- automatic pulse/microstep/speed calculations
- experiment logging to CSV

IMPORTANT
---------
This is OPEN LOOP.
The ESP32 only knows how many pulses it SENT.
It does not know whether the steering actually reached the commanded angle.
Do not use this as final vehicle steering control without encoder feedback.

Direct common-cathode bench wiring:
GPIO4 -> PUL+
GPIO5 -> DIR+
GPIO6 -> ENA+
GND   -> PUL-, DIR-, ENA-

Assumes factory-default R86mini ENA behavior:
GPIO6 LOW  = enabled / holding
GPIO6 HIGH = disabled / free
"""

from machine import Pin
import network
import socket
import time
import os

# ============================================================
# CONFIG
# ============================================================

FW_NAME = "autokart-variable-steering-test"
FW_VERSION = "2.0.0"

AP_SSID = "AutoKart-Steering"
AP_PASSWORD = "autokart86"

STEP_GPIO = 4
DIR_GPIO = 5
ENA_GPIO = 6

STEP_ACTIVE = 1
STEP_INACTIVE = 0

ENA_ENABLED = 0
ENA_DISABLED = 1

# Swap if physical direction is reversed.
RIGHT_DIRECTION = 1
LEFT_DIRECTION = 0

STEP_HIGH_US = 40
DIR_SETTLE_MS = 80

# User requested maximum steering angle.
MAX_STEERING_DEG = 70.0

# Conservative defaults.
DEFAULT_START_PPS = 100
DEFAULT_MAX_PPS = 350
DEFAULT_RAMP_PULSES = 120

MIN_START_PPS = 20
MAX_ALLOWED_PPS = 1500

DEFAULT_GEAR_RATIO = 1.0

RESULTS_FILE = "variable_steering_test_log.csv"
COUNTER_FILE = "variable_steering_counter.txt"

STEP = Pin(STEP_GPIO, Pin.OUT, value=STEP_INACTIVE)
DIR = Pin(DIR_GPIO, Pin.OUT, value=LEFT_DIRECTION)
ENA = Pin(ENA_GPIO, Pin.OUT, value=ENA_DISABLED)

# ============================================================
# R86mini tables
# ============================================================

CURRENT_TABLE = {
    ("ON","ON","ON"):    (2.40,2.00),
    ("OFF","ON","ON"):   (3.08,2.57),
    ("ON","OFF","ON"):   (3.77,3.14),
    ("OFF","OFF","ON"):  (4.45,3.71),
    ("ON","ON","OFF"):   (5.14,4.28),
    ("OFF","ON","OFF"):  (5.83,4.86),
    ("ON","OFF","OFF"):  (6.52,5.43),
    ("OFF","OFF","OFF"): (7.20,6.00),
}

PPR_TABLE = {
    ("ON","ON","ON","ON"):      400,
    ("OFF","ON","ON","ON"):     800,
    ("ON","OFF","ON","ON"):     1600,
    ("OFF","OFF","ON","ON"):    3200,
    ("ON","ON","OFF","ON"):     6400,
    ("OFF","ON","OFF","ON"):    12800,
    ("ON","OFF","OFF","ON"):    25600,
    ("OFF","OFF","OFF","ON"):   51200,
    ("ON","ON","ON","OFF"):     1000,
    ("OFF","ON","ON","OFF"):    2000,
    ("ON","OFF","ON","OFF"):    4000,
    ("OFF","OFF","ON","OFF"):   5000,
    ("ON","ON","OFF","OFF"):    8000,
    ("OFF","ON","OFF","OFF"):   10000,
    ("ON","OFF","OFF","OFF"):   20000,
    ("OFF","OFF","OFF","OFF"):  40000,
}

# ============================================================
# State
# ============================================================

state = {
    "sw1":"OFF","sw2":"ON","sw3":"OFF","sw4":"ON",
    "sw5":"OFF","sw6":"OFF","sw7":"ON","sw8":"ON",

    "gear_ratio": DEFAULT_GEAR_RATIO,

    "start_pps": DEFAULT_START_PPS,
    "max_pps": DEFAULT_MAX_PPS,
    "ramp_pulses": DEFAULT_RAMP_PULSES,

    "center_valid": False,
    "position_pulses": 0,

    "enabled": False,
    "last_move": None,
    "flash": "",
}

# ============================================================
# Math / utility
# ============================================================

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def get_ppr():
    return PPR_TABLE.get(
        (state["sw5"],state["sw6"],state["sw7"],state["sw8"]),
        3200
    )


def get_current():
    return CURRENT_TABLE.get(
        (state["sw1"],state["sw2"],state["sw3"]),
        ("","")
    )


def microstep_divisor():
    return float(get_ppr()) / 200.0


def pulses_per_deg():
    return get_ppr() * float(state["gear_ratio"]) / 360.0


def deg_per_pulse():
    ppd = pulses_per_deg()
    return 1.0 / ppd if ppd > 0 else 0.0


def deg_to_pulses(deg):
    return int(round(float(deg) * pulses_per_deg()))


def pulses_to_deg(pulses):
    ppd = pulses_per_deg()
    return float(pulses) / ppd if ppd > 0 else 0.0


def motor_rpm(pps):
    return float(pps) * 60.0 / float(get_ppr())


def steering_deg_s(pps):
    ppd = pulses_per_deg()
    return float(pps) / ppd if ppd > 0 else 0.0


def safe_float(v, default):
    try:
        return float(v)
    except Exception:
        return default


def safe_int(v, default):
    try:
        return int(v)
    except Exception:
        return default


def next_run_id():
    try:
        with open(COUNTER_FILE,"r") as f:
            n = int((f.read() or "0").strip())
    except Exception:
        n = 0

    n += 1

    with open(COUNTER_FILE,"w") as f:
        f.write(str(n))

    return "VS%05d" % n


# ============================================================
# Driver control
# ============================================================

def enable_driver():
    ENA.value(ENA_ENABLED)
    state["enabled"] = True
    time.sleep_ms(50)


def disable_driver():
    STEP.value(STEP_INACTIVE)
    ENA.value(ENA_DISABLED)
    state["enabled"] = False


# ============================================================
# Motion profile
# ============================================================

def smoothstep(x):
    x = clamp(float(x),0.0,1.0)
    return x*x*(3.0 - 2.0*x)


def low_us(rate):
    rate = max(1.0,float(rate))
    period = int(1000000.0 / rate)
    return max(1,period - STEP_HIGH_US)


def execute_scurve(pulse_count):
    pulse_count = abs(int(pulse_count))

    if pulse_count <= 0:
        return 0.0

    start_pps = int(clamp(
        state["start_pps"],
        MIN_START_PPS,
        MAX_ALLOWED_PPS
    ))

    max_pps = int(clamp(
        state["max_pps"],
        start_pps,
        MAX_ALLOWED_PPS
    ))

    ramp = int(clamp(state["ramp_pulses"],10,1000))

    # Accel and decel each cannot exceed half the move.
    ramp = min(ramp,pulse_count // 2)

    # Very small jogs remain intentionally slow.
    local_max = min(max_pps,180) if pulse_count < 20 else max_pps

    t0 = time.ticks_ms()

    for i in range(pulse_count):
        edge = min(i,pulse_count - 1 - i)

        if ramp <= 1 or edge >= ramp:
            ratio = 1.0
        else:
            ratio = smoothstep(edge / float(ramp))

        rate = start_pps + (local_max - start_pps)*ratio

        STEP.value(STEP_ACTIVE)
        time.sleep_us(STEP_HIGH_US)
        STEP.value(STEP_INACTIVE)
        time.sleep_us(low_us(rate))

    return time.ticks_diff(time.ticks_ms(),t0)/1000.0


def calibrate_center():
    state["position_pulses"] = 0
    state["center_valid"] = True
    state["last_move"] = None


def move_absolute(target_deg,action="ABSOLUTE"):
    if not state["center_valid"]:
        return False,"Center not calibrated."

    requested = float(target_deg)

    # HARD +/-70 degree clamp.
    target_deg = clamp(
        requested,
        -MAX_STEERING_DEG,
        MAX_STEERING_DEG
    )

    start_p = int(state["position_pulses"])
    target_p = deg_to_pulses(target_deg)
    delta = target_p - start_p

    if delta == 0:
        return False,"Already at target."

    start_deg = pulses_to_deg(start_p)

    enable_driver()

    DIR.value(RIGHT_DIRECTION if delta > 0 else LEFT_DIRECTION)
    time.sleep_ms(DIR_SETTLE_MS)

    duration = execute_scurve(abs(delta))

    # OPEN LOOP ESTIMATE ONLY.
    state["position_pulses"] = target_p

    run_id = next_run_id()

    state["last_move"] = {
        "run_id":run_id,
        "action":action,
        "start_deg":round(start_deg,4),
        "target_deg":round(target_deg,4),
        "delta_deg":round(target_deg-start_deg,4),
        "pulses":abs(delta),
        "duration":round(duration,4),
        "requested_deg":requested,
    }

    return True,"%s complete." % run_id


def jog(delta_deg):
    if not state["center_valid"]:
        return False,"Center not calibrated."

    current = pulses_to_deg(state["position_pulses"])
    target = current + float(delta_deg)

    if target < -MAX_STEERING_DEG or target > MAX_STEERING_DEG:
        return False,"Jog blocked: would exceed +/-70 deg."

    return move_absolute(target,"JOG")


# ============================================================
# Logging
# ============================================================

HEADER = (
    "run_id,uptime_ms,action,"
    "sw1,sw2,sw3,sw4,sw5,sw6,sw7,sw8,"
    "driver_peak_A,driver_avg_A,ppr,microstep_divisor,"
    "gear_ratio,start_pps,max_pps,ramp_pulses,"
    "start_angle_deg,target_angle_deg,delta_angle_deg,"
    "commanded_pulses,motor_rpm,steering_deg_s,duration_s,"
    "vibration_0_5,wobble,noise_0_5,missed_steps,mechanical_bind,"
    "supply_V,supply_A,power_W,motor_temp_C,driver_temp_C,"
    "result,notes\n"
)


def ensure_log():
    try:
        os.stat(RESULTS_FILE)
    except OSError:
        with open(RESULTS_FILE,"w") as f:
            f.write(HEADER)

    try:
        os.stat(COUNTER_FILE)
    except OSError:
        with open(COUNTER_FILE,"w") as f:
            f.write("0")


def csvq(v):
    s = "" if v is None else str(v)
    s = s.replace('"',"'").replace("\n"," ").replace("\r"," ")
    return '"' + s + '"'


def append_log(row):
    ensure_log()

    keys = [
        "run_id","uptime_ms","action",
        "sw1","sw2","sw3","sw4","sw5","sw6","sw7","sw8",
        "driver_peak_A","driver_avg_A","ppr","microstep_divisor",
        "gear_ratio","start_pps","max_pps","ramp_pulses",
        "start_angle_deg","target_angle_deg","delta_angle_deg",
        "commanded_pulses","motor_rpm","steering_deg_s","duration_s",
        "vibration_0_5","wobble","noise_0_5","missed_steps","mechanical_bind",
        "supply_V","supply_A","power_W","motor_temp_C","driver_temp_C",
        "result","notes"
    ]

    with open(RESULTS_FILE,"a") as f:
        f.write(",".join(csvq(row.get(k,"")) for k in keys) + "\n")


def save_observation(p):
    m = state["last_move"]

    if not m:
        return False,"No move to save."

    peak,avg = get_current()

    volts = safe_float(p.get("volts"),"")
    amps = safe_float(p.get("amps"),"")

    if volts != "" and amps != "":
        power = round(float(volts)*float(amps),4)
    else:
        power = ""

    row = {
        "run_id":m["run_id"],
        "uptime_ms":time.ticks_ms(),
        "action":m["action"],

        "sw1":state["sw1"],"sw2":state["sw2"],
        "sw3":state["sw3"],"sw4":state["sw4"],
        "sw5":state["sw5"],"sw6":state["sw6"],
        "sw7":state["sw7"],"sw8":state["sw8"],

        "driver_peak_A":peak,
        "driver_avg_A":avg,
        "ppr":get_ppr(),
        "microstep_divisor":round(microstep_divisor(),3),

        "gear_ratio":state["gear_ratio"],

        "start_pps":state["start_pps"],
        "max_pps":state["max_pps"],
        "ramp_pulses":state["ramp_pulses"],

        "start_angle_deg":m["start_deg"],
        "target_angle_deg":m["target_deg"],
        "delta_angle_deg":m["delta_deg"],

        "commanded_pulses":m["pulses"],
        "motor_rpm":round(motor_rpm(state["max_pps"]),4),
        "steering_deg_s":round(steering_deg_s(state["max_pps"]),4),
        "duration_s":m["duration"],

        "vibration_0_5":p.get("vibration",""),
        "wobble":p.get("wobble",""),
        "noise_0_5":p.get("noise",""),
        "missed_steps":p.get("missed",""),
        "mechanical_bind":p.get("bind",""),

        "supply_V":volts,
        "supply_A":amps,
        "power_W":power,
        "motor_temp_C":p.get("motor_temp",""),
        "driver_temp_C":p.get("driver_temp",""),

        "result":p.get("result",""),
        "notes":p.get("notes",""),
    }

    append_log(row)

    return True,"%s saved." % m["run_id"]


# ============================================================
# HTTP utilities
# ============================================================

def esc(s):
    if s is None:
        return ""
    return (str(s).replace("&","&amp;")
                  .replace("<","&lt;")
                  .replace(">","&gt;")
                  .replace('"',"&quot;"))


def url_decode(s):
    s = s.replace("+"," ")
    out = ""
    i = 0

    while i < len(s):
        if s[i] == "%" and i+2 < len(s):
            try:
                out += chr(int(s[i+1:i+3],16))
                i += 3
                continue
            except Exception:
                pass
        out += s[i]
        i += 1

    return out


def parse_query(q):
    d = {}

    if not q:
        return d

    for item in q.split("&"):
        if "=" in item:
            k,v = item.split("=",1)
        else:
            k,v = item,""

        d[url_decode(k)] = url_decode(v)

    return d


def send(c,body,status="200 OK",ctype="text/html; charset=utf-8"):
    if isinstance(body,str):
        body = body.encode("utf-8")

    h = (
        "HTTP/1.1 %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-store\r\n\r\n"
    ) % (status,ctype,len(body))

    c.send(h.encode("utf-8"))
    c.send(body)


def redirect(c,path="/"):
    c.send((
        "HTTP/1.1 303 See Other\r\n"
        "Location: %s\r\n"
        "Connection: close\r\n\r\n"
    ) % path)


def set_flash(msg):
    state["flash"] = str(msg)


def consume_flash():
    s = state["flash"]
    state["flash"] = ""
    return s


# ============================================================
# UI
# ============================================================

STYLE = """
<style>
body{font-family:Arial,sans-serif;background:#101318;color:#eef2f6;margin:0}
header{background:#181d24;padding:16px;border-bottom:1px solid #303844}
main{max-width:1050px;margin:auto;padding:16px}
.card{background:#1a2028;border:1px solid #394351;border-radius:12px;padding:16px;margin-bottom:14px}
.good{background:#17351f;border-color:#438e55}
.warn{background:#432a16;border-color:#a9692e}
.danger{background:#411a1d;border-color:#a74b52}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
.kpi{font-size:1.25rem;font-weight:bold}
.small{font-size:.88rem;color:#b9c2cc}
label{display:block;margin:8px 0 4px}
input,select,textarea{width:100%;box-sizing:border-box;background:#0e1217;color:white;border:1px solid #4b5665;border-radius:7px;padding:10px;font-size:16px}
button,.btn{display:inline-block;padding:11px 14px;margin:3px;border:0;border-radius:8px;background:#344354;color:#fff;font-weight:bold;text-decoration:none}
.green{background:#276d39}.red{background:#922d35}.blue{background:#315e8c}.amber{background:#835620}
.actions{display:flex;flex-wrap:wrap;gap:4px}
.rangebox{background:#0e1217;border:1px solid #394351;border-radius:10px;padding:12px}
</style>
"""


def sw_select(name,value):
    return (
        '<select name="%s"><option %s>ON</option><option %s>OFF</option></select>'
    ) % (
        name,
        "selected" if value=="ON" else "",
        "selected" if value=="OFF" else ""
    )


def page():
    flash = consume_flash()

    peak,avg = get_current()
    current_deg = pulses_to_deg(state["position_pulses"])

    html = """<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AutoKart Variable Steering</title>%s</head>
<body><header>
<div style="font-size:1.3rem;font-weight:bold">AutoKart Variable Steering Test</div>
<div class="small">v%s | hard software range -70° to +70°</div>
</header><main>
""" % (STYLE,FW_VERSION)

    if flash:
        html += '<div class="card good"><b>%s</b></div>' % esc(flash)

    html += """
<div class="card danger">
<b>BENCH TEST / OPEN LOOP.</b> Maximum command is hard-limited to ±70°.
Physically verify that ±70° cannot damage your real steering mechanism before testing it.
Keep a hardware E-stop / driver power cut accessible.
</div>
"""

    html += """
<div class="card">
<h2>System status</h2>
<div class="grid">
<div><div class="small">Center</div><div class="kpi">%s</div></div>
<div><div class="small">Estimated angle</div><div class="kpi">%.2f°</div></div>
<div><div class="small">Driver</div><div class="kpi">%s</div></div>
<div><div class="small">Hard range</div><div class="kpi">-70° ... +70°</div></div>
</div>
</div>
""" % (
        "CALIBRATED" if state["center_valid"] else "NOT CALIBRATED",
        current_deg,
        "ENABLED" if state["enabled"] else "DISABLED"
    )

    html += """
<div class="card">
<h2>1. Driver + pulse configuration</h2>
<form action="/setup" method="get">
<div class="grid">
"""

    for i in range(1,9):
        key = "sw%d" % i
        html += '<div><label>SW%d</label>%s</div>' % (
            i,sw_select(key,state[key])
        )

    html += """
<div><label>Gear ratio (motor rev / steering rev)</label>
<input name="ratio" type="number" min="0.1" max="20" step="0.01" value="%.2f"></div>
</div>

<h3>Motion profile</h3>
<div class="grid">
<div><label>Start PPS</label><input name="start_pps" type="number" min="20" max="%d" value="%d"></div>
<div><label>Maximum PPS</label><input name="max_pps" type="number" min="20" max="%d" value="%d"></div>
<div><label>Ramp pulses</label><input name="ramp" type="number" min="10" max="1000" value="%d"></div>
</div>

<button type="submit">SAVE CONFIGURATION</button>
</form>
</div>
""" % (
        state["gear_ratio"],
        MAX_ALLOWED_PPS,state["start_pps"],
        MAX_ALLOWED_PPS,state["max_pps"],
        state["ramp_pulses"]
    )

    html += """
<div class="card">
<h2>Automatic calculations</h2>
<div class="grid">
<div><div class="small">Current table setting</div><div class="kpi">%s A peak</div><div>%s A avg</div></div>
<div><div class="small">PPR</div><div class="kpi">%d</div></div>
<div><div class="small">Microstep</div><div class="kpi">1/%.1f</div></div>
<div><div class="small">Pulses/steering degree</div><div class="kpi">%.4f</div></div>
<div><div class="small">Steering degree/pulse</div><div class="kpi">%.5f°</div></div>
<div><div class="small">Max motor RPM</div><div class="kpi">%.3f</div></div>
<div><div class="small">Max steering rate</div><div class="kpi">%.3f°/s</div></div>
<div><div class="small">70° pulse count</div><div class="kpi">%d</div></div>
</div>
</div>
""" % (
        peak,avg,
        get_ppr(),
        microstep_divisor(),
        pulses_per_deg(),
        deg_per_pulse(),
        motor_rpm(state["max_pps"]),
        steering_deg_s(state["max_pps"]),
        deg_to_pulses(70)
    )

    html += """
<div class="card">
<h2>2. Center calibration</h2>
<p>Physically place steering at its true center. Then click calibration.</p>
<div class="actions">
<a class="btn amber" href="/calibrate">CALIBRATE CURRENT POSITION = 0°</a>
<a class="btn green" href="/enable">ENABLE / HOLD</a>
<a class="btn red" href="/disable">RELEASE / DISABLE</a>
</div>
</div>
"""

    html += """
<div class="card">
<h2>3. Jog steering</h2>
<div class="actions">
<a class="btn blue" href="/jog?deg=-10">LEFT 10°</a>
<a class="btn blue" href="/jog?deg=-5">LEFT 5°</a>
<a class="btn blue" href="/jog?deg=-2">LEFT 2°</a>
<a class="btn blue" href="/jog?deg=-1">LEFT 1°</a>
<a class="btn blue" href="/jog?deg=-0.5">LEFT 0.5°</a>

<a class="btn blue" href="/jog?deg=0.5">RIGHT 0.5°</a>
<a class="btn blue" href="/jog?deg=1">RIGHT 1°</a>
<a class="btn blue" href="/jog?deg=2">RIGHT 2°</a>
<a class="btn blue" href="/jog?deg=5">RIGHT 5°</a>
<a class="btn blue" href="/jog?deg=10">RIGHT 10°</a>
</div>
</div>
"""

    html += """
<div class="card">
<h2>4. Variable steering target</h2>

<div class="rangebox">
<form action="/move" method="get">
<label>Target steering angle: -70° to +70°</label>
<input id="angleRange" name="target" type="range" min="-70" max="70" step="1" value="0">
<div style="font-size:1.8rem;font-weight:bold;text-align:center;margin:10px">
<span id="angleValue">0</span>°
</div>
<button class="green" type="submit">MOVE TO SELECTED ANGLE</button>
</form>
</div>

<div class="actions" style="margin-top:10px">
<a class="btn" href="/move?target=-70">-70°</a>
<a class="btn" href="/move?target=-45">-45°</a>
<a class="btn" href="/move?target=-30">-30°</a>
<a class="btn" href="/move?target=-15">-15°</a>
<a class="btn amber" href="/move?target=0">CENTER 0°</a>
<a class="btn" href="/move?target=15">+15°</a>
<a class="btn" href="/move?target=30">+30°</a>
<a class="btn" href="/move?target=45">+45°</a>
<a class="btn" href="/move?target=70">+70°</a>
</div>
</div>
"""

    if state["last_move"]:
        m = state["last_move"]
        html += """
<div class="card good">
<h2>Last move %s</h2>
<div class="grid">
<div><div class="small">Start</div><div class="kpi">%.2f°</div></div>
<div><div class="small">Target</div><div class="kpi">%.2f°</div></div>
<div><div class="small">Delta</div><div class="kpi">%.2f°</div></div>
<div><div class="small">Pulses</div><div class="kpi">%d</div></div>
<div><div class="small">Duration</div><div class="kpi">%.3fs</div></div>
</div>
</div>
""" % (
            m["run_id"],m["start_deg"],m["target_deg"],
            m["delta_deg"],m["pulses"],m["duration"]
        )

        html += """
<div class="card">
<h2>5. Record experiment result</h2>
<form action="/save" method="get">
<div class="grid">
<div><label>Vibration 0-5</label><select name="vibration"><option>0</option><option>1</option><option>2</option><option>3</option><option>4</option><option>5</option></select></div>
<div><label>Visible wobble</label><select name="wobble"><option>NO</option><option>YES</option></select></div>
<div><label>Noise 0-5</label><select name="noise"><option>0</option><option>1</option><option>2</option><option>3</option><option>4</option><option>5</option></select></div>
<div><label>Missed steps suspected</label><select name="missed"><option>NO</option><option>YES</option></select></div>
<div><label>Mechanical binding</label><select name="bind"><option>NO</option><option>YES</option></select></div>
<div><label>Result</label><select name="result"><option>PASS</option><option>VIBRATION</option><option>WOBBLE</option><option>STALL</option><option>BINDING</option><option>ABORT</option></select></div>
</div>

<h3>Electrical</h3>
<div class="grid">
<div><label>Supply V</label><input name="volts" type="number" step="0.01"></div>
<div><label>Supply A</label><input name="amps" type="number" step="0.001"></div>
<div><label>Motor temp °C</label><input name="motor_temp" type="number" step="0.1"></div>
<div><label>Driver temp °C</label><input name="driver_temp" type="number" step="0.1"></div>
</div>

<label>Notes</label>
<textarea name="notes" rows="3"></textarea>
<button class="green" type="submit">SAVE RESULT</button>
</form>
</div>
"""

    html += """
<div class="card">
<h2>6. Suggested variable-speed test</h2>
<p>Keep PPR/current fixed. Test one maximum speed at a time:</p>
<p><b>180 → 250 → 350 → 450 → 600 PPS</b></p>
<p>At each setting test: 0° → +15° → 0° → -15° → 0°.</p>
<p>Only progress if wobble = NO and vibration stays low.</p>
</div>

<div class="card">
<a class="btn" href="/download">DOWNLOAD CSV</a>
</div>

<script>
const r = document.getElementById('angleRange');
const o = document.getElementById('angleValue');
if (r && o) {
  const update = () => { o.textContent = r.value; };
  r.addEventListener('input', update);
  update();
}
</script>

</main></body></html>
"""

    return html


# ============================================================
# Web actions
# ============================================================

def apply_setup(p):
    for i in range(1,9):
        key = "sw%d" % i
        v = p.get(key,state[key]).upper()
        if v in ("ON","OFF"):
            state[key] = v

    state["gear_ratio"] = clamp(
        safe_float(p.get("ratio"),state["gear_ratio"]),
        0.1,20.0
    )

    state["start_pps"] = int(clamp(
        safe_int(p.get("start_pps"),state["start_pps"]),
        MIN_START_PPS,MAX_ALLOWED_PPS
    ))

    state["max_pps"] = int(clamp(
        safe_int(p.get("max_pps"),state["max_pps"]),
        state["start_pps"],MAX_ALLOWED_PPS
    ))

    state["ramp_pulses"] = int(clamp(
        safe_int(p.get("ramp"),state["ramp_pulses"]),
        10,1000
    ))

    # PPR/ratio changes invalidate open-loop angle reference.
    state["center_valid"] = False
    state["position_pulses"] = 0
    state["last_move"] = None


# ============================================================
# Network
# ============================================================

def start_ap():
    ap = network.WLAN(network.AP_IF)
    ap.active(True)

    try:
        ap.config(essid=AP_SSID,password=AP_PASSWORD)
    except Exception:
        ap.config(ssid=AP_SSID,password=AP_PASSWORD)

    while not ap.active():
        time.sleep_ms(100)

    print("SSID:",AP_SSID)
    print("Password:",AP_PASSWORD)
    print("Open: http://%s" % ap.ifconfig()[0])


def serve():
    ensure_log()
    disable_driver()
    start_ap()

    addr = socket.getaddrinfo("0.0.0.0",80)[0][-1]

    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    srv.bind(addr)
    srv.listen(3)

    print("Variable steering test ready.")

    while True:
        c = None

        try:
            c,_ = srv.accept()
            c.settimeout(3)

            req = c.recv(3072)
            if not req:
                continue

            line = req.decode("utf-8","ignore").split("\r\n",1)[0]
            parts = line.split(" ")

            if len(parts) < 2:
                send(c,"Bad request","400 Bad Request","text/plain")
                continue

            target = parts[1]

            if "?" in target:
                path,qs = target.split("?",1)
            else:
                path,qs = target,""

            p = parse_query(qs)

            if path == "/":
                send(c,page())

            elif path == "/setup":
                apply_setup(p)
                set_flash("Configuration saved. Re-center physically and calibrate.")
                redirect(c,"/")

            elif path == "/calibrate":
                calibrate_center()
                set_flash("Center calibrated to 0 degrees.")
                redirect(c,"/")

            elif path == "/enable":
                enable_driver()
                set_flash("Driver enabled / holding.")
                redirect(c,"/")

            elif path == "/disable":
                disable_driver()
                set_flash("Driver disabled / released.")
                redirect(c,"/")

            elif path == "/jog":
                delta = safe_float(p.get("deg"),0.0)
                ok,msg = jog(delta)
                set_flash(msg)
                redirect(c,"/")

            elif path == "/move":
                target_deg = safe_float(p.get("target"),0.0)
                ok,msg = move_absolute(target_deg,"ABSOLUTE")
                set_flash(msg)
                redirect(c,"/")

            elif path == "/save":
                ok,msg = save_observation(p)
                set_flash(msg)
                redirect(c,"/")

            elif path == "/download":
                with open(RESULTS_FILE,"rb") as f:
                    data = f.read()

                head = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/csv\r\n"
                    "Content-Disposition: attachment; filename=variable_steering_test_log.csv\r\n"
                    "Content-Length: %d\r\n"
                    "Connection: close\r\n\r\n"
                ) % len(data)

                c.send(head.encode())
                c.send(data)

            elif path == "/favicon.ico":
                send(c,b"","204 No Content","image/x-icon")

            else:
                send(c,"Not found","404 Not Found","text/plain")

        except KeyboardInterrupt:
            STEP.value(STEP_INACTIVE)
            disable_driver()
            raise

        except Exception as e:
            print("WEB ERROR:",e)
            try:
                if c:
                    send(c,"Internal error: %s" % e,
                         "500 Internal Server Error","text/plain")
            except Exception:
                pass

        finally:
            if c:
                try:
                    c.close()
                except Exception:
                    pass


print()
print("AutoKart Variable Steering Test v%s" % FW_VERSION)
print("Hard software steering range: -70 to +70 deg")
print("GPIO4 -> PUL+")
print("GPIO5 -> DIR+")
print("GPIO6 -> ENA+")
print("GND -> PUL-/DIR-/ENA-")
print()

serve()


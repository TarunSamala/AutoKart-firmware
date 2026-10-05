"""
AutoKart Steering Characterization Lab v1.0
ESP32-S3 + R86mini + NEMA34 + MicroPython Web UI

DIRECT COMMON-CATHODE BENCH WIRING
GPIO4 -> PUL+
GPIO5 -> DIR+
GPIO6 -> ENA+
GND   -> PUL-, DIR-, ENA-

Factory-default ENA assumption:
LOW = driver enabled, HIGH = driver disabled.

This is OPEN LOOP bench firmware.
A sent pulse does NOT prove the steering moved.
Keep a physical E-stop / driver power cut accessible.
"""

from machine import Pin
import network
import socket
import time
import os

FW_VERSION = "1.0.0"

AP_SSID = "AutoKart-Steering-Lab"
AP_PASSWORD = "autokart86"

STEP_GPIO = 4
DIR_GPIO = 5
ENA_GPIO = 6

STEP_ACTIVE = 1
STEP_INACTIVE = 0
ENA_ENABLED = 0
ENA_DISABLED = 1

RIGHT_DIRECTION = 1
LEFT_DIRECTION = 0

STEP_HIGH_US = 40
DIR_SETTLE_MS = 80

DEFAULT_START_PPS = 100
DEFAULT_MAX_PPS = 180
HARD_MAX_PPS = 1500

DEFAULT_LIMIT_DEG = 30.0
HARD_LIMIT_DEG = 90.0

DEFAULT_GEAR_RATIO = 1.0
DEFAULT_RAMP_PULSES = 120
DEFAULT_LEVER_M = 0.200

G = 9.80665

RESULTS_FILE = "steering_experiment_log.csv"
EVENTS_FILE = "steering_events.csv"
COUNTER_FILE = "steering_counter.txt"

STEP = Pin(STEP_GPIO, Pin.OUT, value=STEP_INACTIVE)
DIR = Pin(DIR_GPIO, Pin.OUT, value=LEFT_DIRECTION)
ENA = Pin(ENA_GPIO, Pin.OUT, value=ENA_DISABLED)

CURRENT_TABLE = {
    ("ON","ON","ON"):    (2.40, 2.00),
    ("OFF","ON","ON"):   (3.08, 2.57),
    ("ON","OFF","ON"):   (3.77, 3.14),
    ("OFF","OFF","ON"):  (4.45, 3.71),
    ("ON","ON","OFF"):   (5.14, 4.28),
    ("OFF","ON","OFF"):  (5.83, 4.86),
    ("ON","OFF","OFF"):  (6.52, 5.43),
    ("OFF","OFF","OFF"): (7.20, 6.00),
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

state = {
    "wiring": "PARALLEL",
    "sw1": "OFF", "sw2": "ON", "sw3": "OFF", "sw4": "ON",
    "sw5": "OFF", "sw6": "OFF", "sw7": "ON", "sw8": "ON",
    "gear_ratio": DEFAULT_GEAR_RATIO,
    "limit_deg": DEFAULT_LIMIT_DEG,
    "start_pps": DEFAULT_START_PPS,
    "max_pps": DEFAULT_MAX_PPS,
    "ramp_pulses": DEFAULT_RAMP_PULSES,
    "center_valid": False,
    "position_pulses": 0,
    "enabled": False,
    "last_move": None,
    "last_saved": None,
    "flash": "",
}

RESULT_HEADER = (
    "run_id,uptime_ms,action,wiring,"
    "sw1,sw2,sw3,sw4,sw5,sw6,sw7,sw8,"
    "driver_peak_A,driver_avg_A,ppr,microstep_divisor,"
    "gear_ratio,limit_deg,start_pps,max_pps,ramp_pulses,"
    "start_angle_deg,target_angle_deg,delta_angle_deg,commanded_pulses,"
    "motor_rpm,steering_deg_s,duration_s,"
    "vibration_0_5,wobble,noise_0_5,missed_steps,mechanical_bind,"
    "supply_V,supply_A,power_W,motor_temp_C,driver_temp_C,"
    "lever_m,scale_kg,torque_Nm,result,notes\n"
)

EVENT_HEADER = "uptime_ms,event,details\n"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


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


def csvq(v):
    s = "" if v is None else str(v)
    s = s.replace('"', "'").replace("\n", " ").replace("\r", " ")
    return '"' + s + '"'


def ensure_files():
    try:
        os.stat(RESULTS_FILE)
    except OSError:
        with open(RESULTS_FILE, "w") as f:
            f.write(RESULT_HEADER)

    try:
        os.stat(EVENTS_FILE)
    except OSError:
        with open(EVENTS_FILE, "w") as f:
            f.write(EVENT_HEADER)

    try:
        os.stat(COUNTER_FILE)
    except OSError:
        with open(COUNTER_FILE, "w") as f:
            f.write("0")


def log_event(event, details=""):
    ensure_files()
    with open(EVENTS_FILE, "a") as f:
        f.write(",".join([
            csvq(time.ticks_ms()),
            csvq(event),
            csvq(details)
        ]) + "\n")


def next_run_id():
    ensure_files()
    try:
        with open(COUNTER_FILE, "r") as f:
            n = int((f.read() or "0").strip())
    except Exception:
        n = 0
    n += 1
    with open(COUNTER_FILE, "w") as f:
        f.write(str(n))
    return "ST%05d" % n


def get_ppr():
    return PPR_TABLE.get(
        (state["sw5"], state["sw6"], state["sw7"], state["sw8"]),
        3200
    )


def get_current():
    return CURRENT_TABLE.get(
        (state["sw1"], state["sw2"], state["sw3"]),
        ("", "")
    )


def microstep_divisor():
    return float(get_ppr()) / 200.0


def pulses_per_steering_deg():
    return get_ppr() * float(state["gear_ratio"]) / 360.0


def pulse_angle_deg():
    ppd = pulses_per_steering_deg()
    return 1.0 / ppd if ppd > 0 else 0.0


def steering_deg_to_pulses(deg):
    return int(round(float(deg) * pulses_per_steering_deg()))


def pulses_to_steering_deg(pulses):
    ppd = pulses_per_steering_deg()
    return float(pulses) / ppd if ppd > 0 else 0.0


def motor_rpm(pps):
    return float(pps) * 60.0 / float(get_ppr())


def steering_deg_s(pps):
    ppd = pulses_per_steering_deg()
    return float(pps) / ppd if ppd > 0 else 0.0


def enable_driver():
    ENA.value(ENA_ENABLED)
    state["enabled"] = True
    time.sleep_ms(50)


def disable_driver():
    STEP.value(STEP_INACTIVE)
    ENA.value(ENA_DISABLED)
    state["enabled"] = False


def smoothstep(x):
    x = clamp(float(x), 0.0, 1.0)
    return x*x*(3.0 - 2.0*x)


def low_time_us(rate_pps):
    rate_pps = max(1.0, float(rate_pps))
    period = int(1000000.0 / rate_pps)
    return max(1, period - STEP_HIGH_US)


def execute_scurve(count):
    count = abs(int(count))
    if count <= 0:
        return 0.0

    start_pps = int(clamp(state["start_pps"], 20, HARD_MAX_PPS))
    max_pps = int(clamp(state["max_pps"], start_pps, HARD_MAX_PPS))
    ramp = int(clamp(state["ramp_pulses"], 10, 600))
    local_ramp = min(ramp, count // 2)

    # Tiny moves must stay slow.
    local_max = min(max_pps, 180) if count < 20 else max_pps

    t0 = time.ticks_ms()

    for i in range(count):
        edge = min(i, count - 1 - i)

        if local_ramp <= 1 or edge >= local_ramp:
            ratio = 1.0
        else:
            ratio = smoothstep(edge / float(local_ramp))

        rate = start_pps + (local_max - start_pps) * ratio

        STEP.value(STEP_ACTIVE)
        time.sleep_us(STEP_HIGH_US)
        STEP.value(STEP_INACTIVE)
        time.sleep_us(low_time_us(rate))

    return time.ticks_diff(time.ticks_ms(), t0) / 1000.0


def calibrate_center():
    state["position_pulses"] = 0
    state["center_valid"] = True
    state["last_move"] = None
    log_event("CENTER_CALIBRATED", "Physical steering position assigned 0 deg.")


def command_absolute(target_deg, action="ABSOLUTE"):
    if not state["center_valid"]:
        return False, "Center is not calibrated."

    limit = float(state["limit_deg"])
    target_deg = clamp(float(target_deg), -limit, limit)

    start_p = int(state["position_pulses"])
    target_p = steering_deg_to_pulses(target_deg)
    delta = target_p - start_p

    if delta == 0:
        return False, "Already at target."

    start_deg = pulses_to_steering_deg(start_p)

    enable_driver()

    DIR.value(RIGHT_DIRECTION if delta > 0 else LEFT_DIRECTION)
    time.sleep_ms(DIR_SETTLE_MS)

    duration = execute_scurve(abs(delta))

    # OPEN LOOP ESTIMATE ONLY
    state["position_pulses"] = target_p

    run_id = next_run_id()
    peak, avg = get_current()

    state["last_move"] = {
        "run_id": run_id,
        "action": action,
        "start_deg": round(start_deg, 5),
        "target_deg": round(target_deg, 5),
        "delta_deg": round(target_deg - start_deg, 5),
        "pulses": abs(delta),
        "duration": round(duration, 5),
        "motor_rpm": round(motor_rpm(state["max_pps"]), 5),
        "steering_deg_s": round(steering_deg_s(state["max_pps"]), 5),
        "peak_a": peak,
        "avg_a": avg,
    }

    log_event(
        "MOVE_COMPLETED",
        "{} {} {}->{} pulses={}".format(
            run_id, action, start_deg, target_deg, abs(delta)
        )
    )

    return True, "{} completed. Record observation.".format(run_id)


def command_jog(delta_deg):
    if not state["center_valid"]:
        return False, "Center is not calibrated."

    current = pulses_to_steering_deg(state["position_pulses"])
    target = current + float(delta_deg)

    if abs(target) > float(state["limit_deg"]):
        return False, "Jog would exceed software steering limit."

    return command_absolute(target, "JOG")


def append_result(row):
    ensure_files()
    keys = [
        "run_id","uptime_ms","action","wiring",
        "sw1","sw2","sw3","sw4","sw5","sw6","sw7","sw8",
        "driver_peak_A","driver_avg_A","ppr","microstep_divisor",
        "gear_ratio","limit_deg","start_pps","max_pps","ramp_pulses",
        "start_angle_deg","target_angle_deg","delta_angle_deg","commanded_pulses",
        "motor_rpm","steering_deg_s","duration_s",
        "vibration_0_5","wobble","noise_0_5","missed_steps","mechanical_bind",
        "supply_V","supply_A","power_W","motor_temp_C","driver_temp_C",
        "lever_m","scale_kg","torque_Nm","result","notes"
    ]
    with open(RESULTS_FILE, "a") as f:
        f.write(",".join(csvq(row.get(k, "")) for k in keys) + "\n")


def save_observation(p):
    m = state["last_move"]
    if not m:
        return False, "No move available to save."

    peak, avg = get_current()

    volts = safe_float(p.get("volts"), "")
    amps = safe_float(p.get("amps"), "")
    power = ""
    if volts != "" and amps != "":
        power = round(float(volts) * float(amps), 5)

    lever = safe_float(p.get("lever"), "")
    kg = safe_float(p.get("kg"), "")
    torque = ""
    if lever != "" and kg != "":
        torque = round(float(kg) * G * float(lever), 5)

    row = {
        "run_id": m["run_id"],
        "uptime_ms": time.ticks_ms(),
        "action": m["action"],
        "wiring": state["wiring"],

        "sw1": state["sw1"], "sw2": state["sw2"],
        "sw3": state["sw3"], "sw4": state["sw4"],
        "sw5": state["sw5"], "sw6": state["sw6"],
        "sw7": state["sw7"], "sw8": state["sw8"],

        "driver_peak_A": peak,
        "driver_avg_A": avg,
        "ppr": get_ppr(),
        "microstep_divisor": round(microstep_divisor(), 4),

        "gear_ratio": state["gear_ratio"],
        "limit_deg": state["limit_deg"],
        "start_pps": state["start_pps"],
        "max_pps": state["max_pps"],
        "ramp_pulses": state["ramp_pulses"],

        "start_angle_deg": m["start_deg"],
        "target_angle_deg": m["target_deg"],
        "delta_angle_deg": m["delta_deg"],
        "commanded_pulses": m["pulses"],
        "motor_rpm": m["motor_rpm"],
        "steering_deg_s": m["steering_deg_s"],
        "duration_s": m["duration"],

        "vibration_0_5": p.get("vibration", ""),
        "wobble": p.get("wobble", ""),
        "noise_0_5": p.get("noise", ""),
        "missed_steps": p.get("missed", ""),
        "mechanical_bind": p.get("bind", ""),

        "supply_V": volts,
        "supply_A": amps,
        "power_W": power,
        "motor_temp_C": p.get("motor_temp", ""),
        "driver_temp_C": p.get("driver_temp", ""),

        "lever_m": lever,
        "scale_kg": kg,
        "torque_Nm": torque,

        "result": p.get("result", ""),
        "notes": p.get("notes", ""),
    }

    append_result(row)

    state["last_saved"] = {
        "target_deg": m["target_deg"],
        "action": m["action"],
    }

    log_event("RESULT_SAVED", m["run_id"])

    return True, "{} saved.".format(m["run_id"])


def repeat_last():
    if not state["last_saved"]:
        return False, "No saved move to repeat."
    return command_absolute(
        state["last_saved"]["target_deg"],
        "REPEAT_" + state["last_saved"]["action"]
    )


def esc(s):
    if s is None:
        return ""
    return (str(s)
            .replace("&","&amp;")
            .replace("<","&lt;")
            .replace(">","&gt;")
            .replace('"',"&quot;")
            .replace("'","&#39;"))


def url_decode(s):
    s = s.replace("+", " ")
    out = ""
    i = 0
    while i < len(s):
        if s[i] == "%" and i + 2 < len(s):
            try:
                out += chr(int(s[i+1:i+3], 16))
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


def send(client, body, status="200 OK", content_type="text/html; charset=utf-8"):
    if isinstance(body, str):
        body = body.encode("utf-8")
    header = (
        "HTTP/1.1 {}\r\n"
        "Content-Type: {}\r\n"
        "Content-Length: {}\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-store\r\n\r\n"
    ).format(status, content_type, len(body))
    client.send(header.encode("utf-8"))
    client.send(body)


def redirect(client, path="/"):
    client.send((
        "HTTP/1.1 303 See Other\r\n"
        "Location: {}\r\n"
        "Connection: close\r\n\r\n"
    ).format(path).encode("utf-8"))


def set_flash(msg):
    state["flash"] = str(msg)


def flash_html():
    if not state["flash"]:
        return ""
    msg = state["flash"]
    state["flash"] = ""
    return '<div class="card good"><b>{}</b></div>'.format(esc(msg))


STYLE = """
<style>
body{font-family:Arial,sans-serif;background:#101318;color:#eef2f6;margin:0}
header{background:#181d24;border-bottom:1px solid #303844;padding:16px}
main{max-width:1100px;margin:auto;padding:16px}
.card{background:#1a2028;border:1px solid #394351;border-radius:12px;padding:16px;margin-bottom:14px}
.warn{background:#432a16;border-color:#a9692e}
.danger{background:#411a1d;border-color:#a74b52}
.good{background:#17351f;border-color:#438e55}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
.kpi{font-size:1.25rem;font-weight:bold}
.small{font-size:.88rem;color:#b9c2cc}
label{display:block;margin:8px 0 4px}
input,select,textarea{width:100%;box-sizing:border-box;background:#0e1217;color:#fff;border:1px solid #4b5665;border-radius:7px;padding:10px;font-size:16px}
button,.btn{display:inline-block;border:0;border-radius:8px;padding:11px 14px;margin:3px;background:#344354;color:white;font-weight:bold;text-decoration:none}
.green{background:#276d39}.red{background:#922d35}.blue{background:#315e8c}.amber{background:#835620}
.actions{display:flex;flex-wrap:wrap;gap:5px}
table{width:100%;border-collapse:collapse}
th,td{text-align:left;padding:7px;border-bottom:1px solid #35404b}
code{background:#0c1014;padding:2px 5px;border-radius:4px}
</style>
"""


def sw_select(name, value):
    return (
        '<select name="{0}">'
        '<option {1}>ON</option>'
        '<option {2}>OFF</option>'
        '</select>'
    ).format(
        name,
        "selected" if value == "ON" else "",
        "selected" if value == "OFF" else ""
    )


def derived_html():
    ppr = get_ppr()
    peak, avg = get_current()
    ppd = pulses_per_steering_deg()
    angle_per_pulse = pulse_angle_deg()
    current_deg = pulses_to_steering_deg(state["position_pulses"])

    return """
<div class="card">
<h2>Derived calculations</h2>
<div class="grid">
<div><div class="small">Driver current table</div><div class="kpi">{peak} A peak</div><div>{avg} A avg</div></div>
<div><div class="small">Pulses/rev</div><div class="kpi">{ppr}</div></div>
<div><div class="small">Microstep subdivision</div><div class="kpi">1/{micro:.1f}</div></div>
<div><div class="small">Pulses/steering degree</div><div class="kpi">{ppd:.4f}</div></div>
<div><div class="small">Steering degree/pulse</div><div class="kpi">{angle:.5f}°</div></div>
<div><div class="small">Max motor speed</div><div class="kpi">{rpm:.3f} RPM</div></div>
<div><div class="small">Max steering speed</div><div class="kpi">{degs:.3f}°/s</div></div>
<div><div class="small">Estimated angle</div><div class="kpi">{current:.3f}°</div><div>{valid}</div></div>
</div>
<p class="small">
pulse angle = 360/(PPR×ratio);
pulses = angle×PPR×ratio/360;
motor RPM = PPS×60/PPR;
steering °/s = PPS×360/(PPR×ratio).
</p>
</div>
""".format(
        peak=peak, avg=avg, ppr=ppr, micro=microstep_divisor(),
        ppd=ppd, angle=angle_per_pulse,
        rpm=motor_rpm(state["max_pps"]),
        degs=steering_deg_s(state["max_pps"]),
        current=current_deg,
        valid="CENTER VALID" if state["center_valid"] else "CENTER NOT CALIBRATED"
    )


def last_move_html():
    m = state["last_move"]
    if not m:
        return '<div class="card"><h2>Last motion</h2><p>No motion executed yet.</p></div>'

    return """
<div class="card good">
<h2>Last motion: {run}</h2>
<div class="grid">
<div><div class="small">Action</div><div class="kpi">{action}</div></div>
<div><div class="small">Start</div><div class="kpi">{start}°</div></div>
<div><div class="small">Target</div><div class="kpi">{target}°</div></div>
<div><div class="small">Pulses</div><div class="kpi">{pulses}</div></div>
<div><div class="small">Duration</div><div class="kpi">{duration}s</div></div>
<div><div class="small">Max theoretical steering speed</div><div class="kpi">{speed}°/s</div></div>
</div>
</div>
""".format(
        run=m["run_id"], action=m["action"], start=m["start_deg"],
        target=m["target_deg"], pulses=m["pulses"],
        duration=m["duration"], speed=m["steering_deg_s"]
    )


def observation_html():
    m = state["last_move"]
    if not m:
        return '<div class="card"><h2>Record observation</h2><p>Run a jog or angle move first.</p></div>'

    return """
<div class="card">
<h2>Record observation for {run}</h2>
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

<h3>Optional torque measurement</h3>
<div class="grid">
<div><label>Lever m</label><input name="lever" type="number" step="0.001" value="{lever}"></div>
<div><label>Scale kg</label><input name="kg" type="number" step="0.001"></div>
</div>

<label>Notes</label>
<textarea name="notes" rows="3" placeholder="snap at start, resonance mid-move, smooth wheels-up, vibration on ground..."></textarea>
<button class="green" type="submit">SAVE EXPERIMENT RESULT</button>
</form>
</div>
""".format(run=m["run_id"], lever=DEFAULT_LEVER_M)


def page():
    html = """<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AutoKart Steering Lab</title>{style}</head>
<body><header><b>AutoKart Steering Characterization Lab</b>
<div class="small">v{ver}</div></header><main>
""".format(style=STYLE, ver=FW_VERSION)

    html += flash_html()

    html += """
<div class="card danger">
<b>OPEN LOOP BENCH TEST.</b> Keep a physical E-stop / driver power cut accessible.
Never drive into a mechanical stop. Physically center before calibrating.
</div>
"""

    html += """
<div class="card">
<h2>1. Configuration</h2>
<form action="/setup" method="get">
<div class="grid">
<div><label>Motor wiring</label><select name="wiring">
<option {par}>PARALLEL</option>
<option {ser}>SERIES</option>
<option {uni}>UNIPOLAR</option>
</select></div>
<div><label>Motor rev / steering rev</label><input name="ratio" type="number" min="0.1" max="20" step="0.01" value="{ratio}"></div>
<div><label>Software ±limit deg</label><input name="limit" type="number" min="1" max="{hardlim}" step="0.5" value="{limit}"></div>
</div>
<h3>DIP switches</h3>
<div class="grid">
""".format(
        par="selected" if state["wiring"]=="PARALLEL" else "",
        ser="selected" if state["wiring"]=="SERIES" else "",
        uni="selected" if state["wiring"]=="UNIPOLAR" else "",
        ratio=state["gear_ratio"], hardlim=HARD_LIMIT_DEG, limit=state["limit_deg"]
    )

    for i in range(1,9):
        key = "sw{}".format(i)
        html += '<div><label>SW{}</label>{}</div>'.format(i, sw_select(key,state[key]))

    html += """
</div>
<h3>Motion profile</h3>
<div class="grid">
<div><label>Start PPS</label><input name="start_pps" type="number" min="20" max="{hard}" value="{start}"></div>
<div><label>Max PPS</label><input name="max_pps" type="number" min="20" max="{hard}" value="{maxpps}"></div>
<div><label>Ramp pulses</label><input name="ramp" type="number" min="10" max="600" value="{ramp}"></div>
</div>
<button type="submit">SAVE CONFIGURATION</button>
</form>
</div>
""".format(
        hard=HARD_MAX_PPS, start=state["start_pps"],
        maxpps=state["max_pps"], ramp=state["ramp_pulses"]
    )

    html += derived_html()

    html += """
<div class="card">
<h2>2. Center + driver</h2>
<p>Physically center steering first. Calibration itself does not move the motor.</p>
<div class="actions">
<a class="btn amber" href="/calibrate">CALIBRATE CENTER = 0°</a>
<a class="btn green" href="/enable">ENABLE / HOLD</a>
<a class="btn red" href="/disable">RELEASE / DISABLE</a>
</div>
<p>Driver: <b>{driver}</b> | Center: <b>{center}</b></p>
</div>
""".format(
        driver="ENABLED" if state["enabled"] else "DISABLED",
        center="VALID" if state["center_valid"] else "NOT CALIBRATED"
    )

    html += """
<div class="card">
<h2>3. Service jog</h2>
<div class="actions">
<a class="btn blue" href="/jog?deg=-5">LEFT 5°</a>
<a class="btn blue" href="/jog?deg=-2">LEFT 2°</a>
<a class="btn blue" href="/jog?deg=-1">LEFT 1°</a>
<a class="btn blue" href="/jog?deg=-0.5">LEFT 0.5°</a>
<a class="btn blue" href="/jog?deg=-0.1">LEFT 0.1°</a>
<a class="btn blue" href="/jog?deg=0.1">RIGHT 0.1°</a>
<a class="btn blue" href="/jog?deg=0.5">RIGHT 0.5°</a>
<a class="btn blue" href="/jog?deg=1">RIGHT 1°</a>
<a class="btn blue" href="/jog?deg=2">RIGHT 2°</a>
<a class="btn blue" href="/jog?deg=5">RIGHT 5°</a>
</div>
</div>

<div class="card">
<h2>4. Absolute steering</h2>
<form action="/move" method="get">
<label>Target angle from center</label>
<input name="target" type="number" step="0.1" min="-{lim}" max="{lim}" value="0">
<button type="submit">MOVE TO TARGET</button>
</form>
<p class="small">Baseline: 0 → +5 → 0 → -5 → 0 at 100 start PPS / 180 max PPS.</p>
</div>
""".format(lim=state["limit_deg"])

    html += last_move_html()
    html += observation_html()

    html += """
<div class="card">
<h2>Recommended experiment</h2>
<table>
<tr><th>Stage</th><th>Start</th><th>Max PPS</th><th>Move</th></tr>
<tr><td>1</td><td>100</td><td>180</td><td>±5°</td></tr>
<tr><td>2</td><td>100</td><td>250</td><td>±5°</td></tr>
<tr><td>3</td><td>100</td><td>350</td><td>±10°</td></tr>
<tr><td>4</td><td>100</td><td>450+</td><td>Only if previous stage passes</td></tr>
</table>
<p class="small">Repeat motor-only → linkage with wheels lifted → tyres on ground.</p>
</div>

<div class="card">
<h2>Logs</h2>
<div class="actions">
<a class="btn" href="/download">DOWNLOAD RESULTS CSV</a>
<a class="btn" href="/events">DOWNLOAD EVENT LOG</a>
<a class="btn" href="/repeat">REPEAT LAST SAVED MOVE</a>
</div>
</div>
</main></body></html>
"""
    return html


def apply_setup(p):
    wiring = p.get("wiring",state["wiring"]).upper()
    if wiring in ("PARALLEL","SERIES","UNIPOLAR"):
        state["wiring"] = wiring

    for i in range(1,9):
        key = "sw{}".format(i)
        v = p.get(key,state[key]).upper()
        if v in ("ON","OFF"):
            state[key] = v

    state["gear_ratio"] = clamp(
        safe_float(p.get("ratio"),state["gear_ratio"]),0.1,20.0
    )
    state["limit_deg"] = clamp(
        safe_float(p.get("limit"),state["limit_deg"]),1.0,HARD_LIMIT_DEG
    )
    state["start_pps"] = int(clamp(
        safe_int(p.get("start_pps"),state["start_pps"]),20,HARD_MAX_PPS
    ))
    state["max_pps"] = int(clamp(
        safe_int(p.get("max_pps"),state["max_pps"]),
        state["start_pps"],HARD_MAX_PPS
    ))
    state["ramp_pulses"] = int(clamp(
        safe_int(p.get("ramp"),state["ramp_pulses"]),10,600
    ))

    # Any PPR/ratio change makes previous open-loop angle estimate invalid.
    state["center_valid"] = False
    state["position_pulses"] = 0
    state["last_move"] = None

    log_event(
        "CONFIG_CHANGED",
        "ppr={} ratio={} start={} max={} ramp={}".format(
            get_ppr(),state["gear_ratio"],state["start_pps"],
            state["max_pps"],state["ramp_pulses"]
        )
    )


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
    print("Open: http://{}".format(ap.ifconfig()[0]))


def send_file(client,filename,download_name):
    try:
        with open(filename,"rb") as f:
            data = f.read()
    except OSError:
        data = b""

    header = (
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/csv\r\n"
        "Content-Disposition: attachment; filename={}\r\n"
        "Content-Length: {}\r\n"
        "Connection: close\r\n\r\n"
    ).format(download_name,len(data))

    client.send(header.encode("utf-8"))
    client.send(data)


def serve():
    ensure_files()
    disable_driver()
    start_ap()

    addr = socket.getaddrinfo("0.0.0.0",80)[0][-1]
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    srv.bind(addr)
    srv.listen(3)

    print("Steering lab ready.")

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
                set_flash("Configuration saved. Physically re-center and calibrate.")
                redirect(c,"/")

            elif path == "/calibrate":
                calibrate_center()
                set_flash("Center calibrated to 0°. No movement commanded.")
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
                ok,msg = command_jog(delta)
                set_flash(msg)
                redirect(c,"/")

            elif path == "/move":
                target_deg = safe_float(p.get("target"),0.0)
                ok,msg = command_absolute(target_deg,"ABSOLUTE")
                set_flash(msg)
                redirect(c,"/")

            elif path == "/save":
                ok,msg = save_observation(p)
                set_flash(msg)
                redirect(c,"/")

            elif path == "/repeat":
                ok,msg = repeat_last()
                set_flash(msg)
                redirect(c,"/")

            elif path == "/download":
                send_file(c,RESULTS_FILE,"steering_experiment_log.csv")

            elif path == "/events":
                send_file(c,EVENTS_FILE,"steering_events.csv")

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
                    send(c,"Internal error: {}".format(e),
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
print("AutoKart Steering Characterization Lab",FW_VERSION)
print("GPIO4 -> PUL+")
print("GPIO5 -> DIR+")
print("GPIO6 -> ENA+")
print("GND -> PUL-/DIR-/ENA-")
print("SAFE BOOT: driver disabled.")
print()

serve()

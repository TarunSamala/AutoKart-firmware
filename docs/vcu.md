# AutoKart — VCU Engineering Specification

**Document:** `vcu.md`  
**Revision:** 0.3 — 2026-10-18
**Project:** AutoKart / AKGOS / RC-06  
**Target:** ESP32-S3 VCU ↔ Raspberry Pi (ROS 2 Humble)  
**Design verdict:** **APPROVED WITH CONDITIONS** for implementation and traction-isolated bench development.  
**Vehicle release:** **NO-GO** — independent E-stop, stopping/brake functionality, calibration, actual I/O, timing, and road tests are not yet verified.

> **Document rule:** Reported dimensions and proposed data rates are **not** verified acceptance results. Keep `UNKNOWN` values explicit. Never represent a commanded actuator value as measured feedback.

## 1. Responsibilities and system boundary

| Layer | Ownership | Must not depend on |
|---|---|---|
| **Raspberry Pi** | HP60C RGB/depth SDK, YOLO26s, cone localization, Delaunay/Gabriel mapping, skidpad/braking mission planning, Pure Pursuit, MPU-6500, wheel–IMU odometry, AKGOS dashboard | Browser/Wi-Fi availability for local control safety |
| **ESP32-S3 VCU** | Local steering, brake and traction control, encoder acquisition, actuator status, bounds/plausibility, command expiry, watchdogs, state machine, fault response | Raspberry Pi commands arriving forever |
| **Independent safety hardware** | Physical/remote emergency stop once validated, traction isolation, independent watchdog/interlocks and verified stopping fallback | ROS, Linux, the ESP32 task scheduler, wireless link |

**Required safe power-up behavior:** Hardware must inhibit propulsion before ESP32 firmware executes. Booting the Pi or plugging in the camera must never arm or move the kart.

## 2. Known configuration and unresolved measurements

### 2.1 Drivetrain and geometry

| Parameter | Current reported value | Qualification |
|---|---:|---|
| Motor sprocket | **14 teeth** | Verify physically |
| Rear-axle sprocket | **44 teeth** | Verify physically |
| Chain reduction | **44/14 = 3.142857:1** | Valid if encoder is on the shaft driving the 14T sprocket, no additional reduction |
| Rear tyre marking | **11×7.10−5, SM-68** | Nominal, not loaded rolling measurement |
| Rear tyre nominal diameter | **279.4 mm** | Replaces old 295 mm assumption for preliminary calculations |
| Front tyre marking | **10×4.5−5, SM-68** | Nominal |
| Front tyre nominal diameter | **254 mm** | Verify |
| Wheelbase | **1117.6 mm** | Reported, verify axle-centre measurement |
| Rear track | **1181.1 mm** | Reported, verify centre-to-centre |
| Front track | **1066.8 mm spreadsheet** versus **38 in = 965.2 mm mechanical note** | **CONFLICT — do not freeze** |
| Front outside-to-outside width | **43 in = 1092.2 mm** | Reported, verify |
| Chassis frame length / width | **1422.4 / 660.4 mm** | Reported |
| Ground clearance | **38.1 mm** | Reported |
| Front tyre width | Spreadsheet **279.4 mm** vs tyre marking nominal **114.3 mm** | **CONFLICT — measure inflated section width** |
| Total operating mass | Prior estimate **130 kg** | Needs weighed value/load distribution |
| Brake physical feedback | **AS5600 selected; mounted/integrated validation pending** | Autonomous arming remains blocked until calibration and stopping tests pass |
| AS5047P mounting | **BLDC/motor side** | Confirm exact shaft, magnet installation and intervening gearbox |

### 2.2 Steering — latest correction overrides earlier notes

| Parameter | Current corrected value | Qualification |
|---|---:|---|
| Steering **motor/servo gear** | **34 teeth** | Most recent user correction |
| Steering **shaft gear** | **34 teeth** | Most recent user correction |
| Steering ratio | **1:1** (`34/34`) | No ideal gear reduction; externally meshing gears reverse direction |
| Tie-rod length | **290 mm** | Define if joint-centre-to-joint-centre; verify |
| NEMA34 driver | **R86mini** | Reported current hardware |
| Driver setting | **3200 pulses/motor rev**, if unchanged | Check R86mini DIP settings physically |
| Actual left/right road-wheel angle and linkage conversion | **UNKNOWN** | Must measure; shaft angle ≠ road-wheel angle |
| AS5600 function | **Steering feedback plus separate brake-position feedback** | Use TCA9548A channels; mounted calibration remains pending |

### 2.3 Architecture decisions and current proposals

| Item | Decision |
|---|---|
| Drivetrain encoder | AS5047P on the BLDC/motor side |
| Steering encoder | AS5600 |
| Brake encoder | AS5600 on NEMA23/brake shaft through TCA9548A channel 1 |
| Pi ↔ VCU transport | Native USB with micro-ROS Agent on Raspberry Pi |
| Production VCU firmware | Proposed baseline: ESP-IDF + FreeRTOS; MicroPython remains for bench tests only |
| LoRa E-stop | Pico 2 + RA-02 + LAY37 transmitter selected; ESP32-S3 receiver uses GPIO47 as a stop-request output to an approved external interface |

At 3200 pulses/rev and 1:1 shaft gearing, **motor pulses per shaft degree** = `3200/360 = 8.8889 pulses/°`:

| Shaft movement | Approximate pulses |
|---:|---:|
| 10° | 89 |
| 20° | 178 |
| 45° | 400 |
| 70° | 622 |
| 90° | 800 |

These are *shaft motion* estimates only; do **not** use them as calibrated front-wheel steering commands.

## 3. Sampling, processing and publication rates

**Proposed baseline — PROTOTYPE TEST REQUIRED.** Rates distinguish hardware capture, controller execution and Pi telemetry.

| Signal or task | Owner | Acquisition/control rate | Publication |
|---|---|---:|---:|
| AS5047P motor/drivetrain encoder | VCU | Continuous ABI hardware count + **200 Hz** snapshots **or** validated SPI sampling | 100 Hz in vehicle state |
| AS5600 steering measurement | VCU | **200 Hz** | 100 Hz |
| Steering closed-loop calculation | VCU | **200 Hz** | Target/actual 100 Hz |
| Rear speed estimation | VCU | **200 Hz** | 100 Hz |
| Brake command and supervision | VCU | **100 Hz control proposed**, AS5600 acquired at 200 Hz | Request, measured position and validity; calibration pending |
| BLDC bounded traction loop | VCU | **100 Hz** | 100 Hz |
| Driver critical fault handling | Independent hardware / VCU | Interrupt + supervision up to **1 kHz** | Immediate event + 10 Hz summary |
| Supply measurement | VCU | **100 Hz** | 10 Hz |
| Physical E-stop action | Independent safety hardware | Asynchronous; must not wait for software | Monitor/status event + 10 Hz |
| MPU-6500 gyro + accelerometer | **Pi** | **200 Hz** | Pi-local ROS IMU, not VCU telemetry |
| Wheel–IMU odometry | **Pi** | **100 Hz** | 100 Hz ROS odometry |
| High-level motion command | **Pi → VCU** | **50 Hz** | 20 ms nominal command interval |
| Fast VCU vehicle telemetry | **VCU → Pi** | — | **100 Hz** |
| Health report | **VCU → Pi** | — | **10 Hz** + immediate fault events |
| Link/clock-offset estimation | Pi ↔ VCU | Initial target | 1 Hz; not used to enforce command expiry |

**Timing implementation:** Use ESP-IDF + FreeRTOS, hardware PCNT/timers where applicable, bounded tasks and nonblocking queues. Communications cannot block steering/braking calculations or safety action. Measure worst-case task latency/jitter and CPU/stack headroom before accepting these rates.

## 4. Pi ↔ VCU communication contract

**Selected production transport:** native USB ↔ micro-ROS Agent, subject to real Pi/ESP32 testing. The ESP32-S3 native USB interface is the target; UART GPIO21/GPIO47 is no longer the production Pi link. Confirm the exact USB device/transport configuration during ESP-IDF bring-up.

### 4.1 VCU → Pi `vehicle_state` (100 Hz)

| Field | Type / unit | Source / meaning |
|---|---|---|
| `protocol_version` | uint16 | Schema compatibility |
| `boot_id` | uint32 | Changes every VCU boot; detect resets |
| `sequence` | uint32 | Monotonic packet sequence; detect drops/reordering |
| `sample_time_us` | uint64 | VCU monotonic **acquisition** timestamp |
| `encoder_count_total` | int64, counts | Cumulative **motor or axle**, explicitly identified by mounting |
| `encoder_mount_location` | enum | `MOTOR_SHAFT` / `AXLE` / `UNKNOWN` |
| `motor_speed_rad_s` | float32, rad/s | Direct encoder-derived speed when mounted on motor shaft |
| `axle_speed_rad_s` | float32, rad/s | Chain-reduction estimate (when valid) or axle measurement |
| `vehicle_speed_mps` | float32, m/s | **Estimated**, not verified ground-truth speed |
| `steering_angle_rad` | float32, rad | Measured/calibrated road-wheel angle **only if linkage calibration supports it** |
| `steering_target_rad` | float32, rad | Accepted target, not feedback |
| `brake_request` | float32, 0…1 | Requested brake; not proof of applied force |
| `brake_feedback` | float32 + validity | AS5600-derived brake travel after mounted released/applied calibration; invalid until proven |
| `traction_request` | float32, 0…1 | Accepted traction request, not verified motor output |
| `vcu_state` | enum | State machine below |
| `validity_flags` | bitmask | Status per sensor/signal; missing feedback **INVALID** |
| `fault_flags` | bitmask | Active/latched fault information |
| `last_valid_command_age_ms` | uint16 | VCU monotonic time since most recent **accepted** command |

For the ROS `.msg` implementation, settle exact field types and any unknown-value representation in `NODE_CONTRACTS.md` without silently changing existing shared message definitions. Use named unit conventions and source identifiers; include valid ranges and sample-age checks.

### 4.2 VCU → Pi `diagnostics` (10 Hz + events)

Firmware hash, calibration/configuration CRC, AS5600 magnetic health, AS5047P diagnostics **when available through the wired interface**, supply-voltage/driver fault status, traction-enable feedback **if actually sensed**, task overrun counts, watchdog health, USB errors, sample freshness, rejected commands and fault history. Critical faults trigger local safety action immediately, independent of publication.

### 4.3 Pi → VCU `motion_command` (50 Hz)

| Field | Unit / rule |
|---|---|
| `protocol_version` | Must match negotiated version |
| `session_id` | Bound to current VCU boot/handshake; stale sessions rejected |
| `sequence` | Monotonic, duplicates/out-of-order rejected |
| `steering_target_rad` | **Road-wheel** target; finite and within calibrated limits |
| `target_speed_mps` | Finite, permitted speed bound |
| `deceleration_request_mps2` | Nonnegative, inside verified braking limits |
| `stop_request` | Boolean; stop priority over drive |
| `command_valid_for_ms` | Bounded by VCU; **must not exceed configured timeout** |
| `mission_mode` | `SKIDPAD` or `BRAKING`; VCU validates active state |

**Required validation:** packet sequence, matching session, finite values, range/steering-rate/speed-rate limits, legal state transition, actual physical authorization, monotonic receive time, and expiry. A heartbeat does **not** refresh a motion command. State/ARM requests require explicit acknowledgment and are event-driven. Application-level integrity checks must be designed and tested for the selected transport; DDS/USB alone is not proof of end-to-end command freshness.

**Failure semantics:** communication lost → reject stale commands, inhibit propulsion, request the **verified** controlled-stopping strategy; if no demonstrated brake fallback exists, physical autonomous operation is **NO-GO**.

## 5. VCU boot-to-autonomous startup sequence

```text
POWER_APPLIED
  └─ Hardware holds traction inhibit independent of ESP32
BOOT
  ├─ Drive outputs to safe logic states (verify polarity + boot strapping)
  ├─ Motor speed/DAC request = zero; pulse generation inhibited
  ├─ Establish boot_id, monotonic timer, watchdogs, configuration CRC
  └─ Never auto-arm after power restoration
SELF_TEST
  ├─ Supply/brownout thresholds and output-state readback
  ├─ Independent E-stop / traction isolation feedback (if instrumented)
  ├─ Driver fault inputs + steering actuator wiring/limits
  ├─ Both AS5600 channels: communications, magnet health, absolute readings
  ├─ AS5047P interface, count acquisition, direction configuration
  ├─ Brake command path, feedback direction/span, interlock and stopping readiness
  └─ Calibration version, units, integrity and sensor freshness
SENSOR_INITIALIZATION
  ├─ Read steering angle; load zero and steering-linkage map
  ├─ Initialize filters from valid real samples; do NOT assume angle = 0
  ├─ Establish cumulative encoder count within this boot session
  ├─ Record sensor timestamps and validity
  └─ Never infer stopped solely from no encoder pulses
SAFE
  ├─ Publish telemetry + diagnostic faults
  ├─ Wait for Raspberry Pi micro-ROS handshake
  ├─ Verify protocol/configuration compatibility
  └─ Stay disarmed indefinitely if Pi/sensors/safety not healthy
READY (requires physical operator authorization)
  ├─ All required feedback and stopping systems proven
  ├─ Valid session, commands, permissions, hardware safety conditions
  └─ No movement merely because Pi/dashboard is ready
AUTONOMOUS (requires distinct authorized mission START)
  ├─ Accept only fresh, bounded motion commands
  ├─ Close local feedback loops and supervise faults
  └─ Stop/lock out on invalid condition according to verified response
```

**Proposed timing:** sensor initialization completes or faults within 2 s; initial filter samples ≥100 ms when stationary conditions have been independently verified. These are validation targets, not reasons to skip a test. Never move the steering to a hard stop during normal startup.

### 5.1 Allowed states / restart behavior

`POWER_OFF → BOOT → SELF_TEST → SAFE → READY → AUTONOMOUS`. Also explicitly support `MANUAL`, `DEGRADED` (only if safe behavior defined), `CONTROLLED_STOP`, `EMERGENCY_STOP`, `FAULT_LOCKOUT`, and `SERVICE_MODE` (restricted bench access). Re-arming after faults/reboots requires deliberate authorization; no autonomous restart by default.

## 6. Encoder calculations and filter strategy

### 6.1 AS5047P — BLDC/motor-side encoder

**Do not label this as independently measured axle speed** until mounting is confirmed. With assumed direct motor-shaft mounting and intact 14T→44T chain:

- Reduction `G = 44 / 14 = 3.142857 motor rev / axle rev`.
- Nominal rear tyre diameter `D = 0.2794 m`.
- Nominal circumference `C = πD ≈ 0.87776 m / axle rev`.
- Nominal translation `d_motor_rev = C × 14 / 44 ≈ 0.27929 m / motor rev`.

Given encoder count difference `ΔN`, measured counts/motor revolution `N_rev`, and elapsed time `Δt`:

```text
motor_revs      = ΔN / N_rev
axle_revs       = motor_revs × (14 / 44)
est_distance_m  = axle_revs × effective_loaded_circumference_m
est_speed_mps   = est_distance_m / Δt
```

**Use the measured loaded rolling circumference**, not just the tyre marking, for calibration. If the encoder is actually axle-mounted, **remove the 14/44 factor**. If there is an intervening motor gearbox, account for that ratio too.

| Nominal road speed | Axle RPM | Motor RPM, 14T:44T |
|---:|---:|---:|
| 5 km/h | ≈94.9 | ≈298.4 |
| 10 km/h | ≈189.9 | ≈596.8 |
| 15 km/h | ≈284.8 | ≈895.1 |

**Acquisition:** AS5047P offers SPI absolute angle or configurable ABI incremental outputs. Final driver and count scaling depend on verified actual wiring. ABI PCNT: configure counts/rev and direction, inspect glitches; SPI: validate reads, unwrap across revolutions and handle reset. A motor encoder alone cannot detect chain breakage, chain slip or actual tyre slip. Direct axle feedback is a recommended additional measurement for drivetrain-fault detection.

**Initial filtering:** 200 Hz count snapshots → short count-window speed estimate → approximately 30 ms first-order LPF (tune against step/stop tests). Use pulse periods at very low speeds if necessary. Validate **raw sample validity before filtering**; mark missing/invalid counts stale, never synthesize zero speed from silence.

### 6.2 AS5600 — steering feedback

12-bit absolute-angle sensor; read 200 Hz where achievable, check magnet status and communications, unwrap circular angle, apply zero and calibrated linkage conversion, then start with approximately 10 ms low-pass filtering. Measure actual lag. Track absolute angle error, steering rate, movement time, hardware end stops, saturation and mismatches. Never interpret motor microstep count as proof the road wheels moved.

### 6.3 MPU-6500 — on Raspberry Pi

Read gyro/accelerometer at proposed 200 Hz with monotonic timestamps, valid sensor configuration and bias calibration when confirmed stationary. Pi fuses measured/estimated wheel motion with yaw rate at 100 Hz; handle slip, gyro bias and uncertainty. An EKF is optional after data show a need; do not hide sensor faults inside filter estimates.

### 6.4 Filtering contract

- Keep both raw input and filtered estimate for logs and plausibility checks.
- Initialize filters only from valid samples; reject impossible jumps and missed timestamps.
- Never use a filter's last value as fresh feedback after input loss.
- Fault detection observes raw freshness and independent feedback paths.
- Validate tyre circumference and steering wheel-angle mapping physically; use consistent SI units throughout ROS and VCU.

## 7. Fault timing and safe response

| Fault | Initial *test* threshold | Required behavior |
|---|---:|---|
| No newly **accepted** Pi motion command | **100 ms** | Expire command; remove propulsion request; use verified stopping strategy |
| Steering feedback stale | **25 ms** | Fault and prohibit unsafe steering/traction operation |
| Encoder speed feedback stale | **50 ms** | Invalidate speed; inhibit traction; validated stopping response |
| USB connection lost | Within motion-command timeout | Never hold an active command indefinitely |
| Driver fault / task deadline miss | Hardware interrupt / bounded supervision | Inhibit unsafe output; fault/event log |
| VCU reset/brownout | Hardware safe default | Traction disabled; no auto-rearm |
| Physical E-stop asserted | Independent hardware | Independent isolation + proven stop behavior |
| Brake feedback unavailable or stale | **Blocking fault** | The selected brake AS5600 must be mounted, calibrated and valid; otherwise refuse AUTONOMOUS authorization |

At 10 km/h, `v = 2.78 m/s`; for a 100 ms command timeout, `d_timeout = v × t ≈ 0.278 m` **before** physical actuation and deceleration. This is not total stopping distance.

**Critical blocking issue:** A brake AS5600 is selected but has not yet been mounted, calibrated or validated as proof of physical braking. Position alone also does not demonstrate brake force, actual wheel deceleration, hold on power loss or independent stopping capability. Until an approved and tested stopping safety case plus physical E-stop isolation are proven, vehicle autonomous release is **REJECTED**.

## 8. FMEA — minimum failure register

| Function | Failure | Effect | Detection | Mitigation | Severity | Next action |
|---|---|---|---|---|---|---|
| Drive | Pi crash / stale throttle | Uncontrolled propulsion | VCU command age + sequence | Independent limit and verified stop; inhibit traction | Critical | Test disconnect/reboot |
| Steering | Stepper stall or linkage break | No steering / wrong angle | Output-side AS5600 plus plausibility; hardware inspection | Fault + safe stop strategy | Critical | Validate encoder mounting/angle mapping |
| Braking | Brake motor moves but wheel brake does not | Inability to stop | Independent brake position/force/pressure and stopping test | Verified fail-safe stop fallback | Critical | Instrument and test brake |
| Encoder | Motor encoder valid, chain breaks | Incorrect assumed road speed | Separate axle feedback/other speed consistency | Fault detection and safe response | High | Inspect drivetrain; add axle observation |
| Telemetry | Old message displayed as live | Misleading dashboard | Boot ID, sequence, timestamp and age | STALE/DISCONNECTED indication | High | Gateway fault-injection test |
| Power | ESP32 resets or brownout | Outputs unpredictable | Hardware supervisor + voltage | Traction inhibit defaults, lockout | Critical | Power-cycle/inrush testing |
| Safety | E-stop circuit fails | Stop not achieved | Independent periodic end-to-end validation | Redundant/fault-aware stop design | Critical | Hardware E-stop verification |

## 9. AKGOS dashboard data contract

Read-only LIVE dashboard must expose real values and source provenance for: VCU state, Pi↔VCU connectivity, firmware/config ID, encoder count/speed, estimated odometry, measured versus requested steering, brake request versus **measured** feedback (or **UNAVAILABLE**), traction request/status, E-stop status (hardware monitored), data latency, fault flags, encoder calibration and sensor validity. When VCU goes offline, show **STALE/DISCONNECTED** rather than the last sample as current. Distinguish **LIVE / REPLAY / SIMULATED** data everywhere.

## 10. Verification gates (hardware isolated first)

| Test | Evidence / pass criterion |
|---|---|
| BOOT/SELF_TEST | Outputs remain electrically inhibited from power application through boot; sensor/config faults block READY |
| One shaft revolution | Raw encoder counts/rev, correct direction, no drops; establish interface/scale |
| Loaded roll test | Measured travel over several rear-wheel rotations agrees with calibrated count conversion; quantify uncertainty |
| Steering range sweep | Measure motor angle → shaft angle → **actual left/right road-wheel angle** at multiple points |
| Brake availability | Instrumented brake engagement, release, power-loss response, linkage-failure detection and stopping proof |
| Control scheduling | 200 Hz steering/encoder; 100 Hz loops/telemetry; bounded measured jitter and no overrun |
| USB communications | 50 Hz commands / 100 Hz feedback; sequence loss, reboot and reconnection measured |
| Fault injection | Encoder disconnect, magnet issue, Pi/USB loss, invalid values, faulted drivers, E-stop and brownout |
| Dashboard LIVE | Display live timestamped ROS data, stale flags and no hidden simulation fallback |
| Safety | Independent emergency stop and braking fallback proven in a separate validated test plan |

Test progression: electronics desk → bench actuators → wheels-off-ground → low-speed manual → assisted → low-speed autonomous → fault injection → progressive speed. **Do not start vehicle tests because software tests pass.**

## 11. Required decisions before final VCU firmware

| Item | Status / next action |
|---|---|
| AS5047P BLDC mounting and electrical interface (`SPI` / `A/B/I`) | **MORE DATA REQUIRED** — verify exact motor-side shaft, wiring and magnet |
| BLDC-to-axle mechanical chain ratio | **APPROVED WITH CONDITIONS** — verify 14T/44T and any internal reduction |
| AS5047P configured count scale | **MORE DATA REQUIRED** — one-rev bench measurement |
| Loaded rear tyre rolling circumference | **MORE DATA REQUIRED** — measured rolling test |
| Steering gears | **APPROVED WITH CONDITIONS** — latest correction 34T/34T, ratio 1:1 |
| Steering AS5600 location and road-wheel calibration | **MORE DATA REQUIRED** — downstream measurement and sweep |
| Front track conflicting values | **MORE DATA REQUIRED** — centre-to-centre measurement |
| Brake feedback | AS5600 selected; mounting, span calibration and stopping validation pending |
| Independent stopping safety case | **REJECTED for autonomous release** until demonstrated |
| LoRa E-stop transmitter | **SELECTED WITH VALIDATION PENDING** — Pico 2 + RA-02 + LAY37 |
| LoRa E-stop isolation | **DESIGN AND TEST REQUIRED** — must not depend on normal VCU/ROS operation |
| USB micro-ROS timings, reliability, recovery | **PROTOTYPE TEST REQUIRED** |
| Raspberry Pi/VCU startup and command expiry | **PROTOTYPE TEST REQUIRED** |

**Next engineering task:** Confirm the AS5047P motor-side mounting and choose `SPI` versus `A/B/I`; bring up native USB micro-ROS under ESP-IDF; then select Pico 2 or XIAO ESP32-C3 and validate the separate LoRa E-stop isolation path. Freeze the ROS message contracts only after those interfaces are demonstrated without changing the working RC-03–05 bench algorithms.

## 12. Reference documents

Read these before finalizing the design:

- **AS5047P Datasheet** — ams OSRAM; absolute SPI, ABI interface, magnetic and diagnostic specifications. <https://look.ams-osram.com/m/d05ee39221f9857/original/AS5047P-DS000324.pdf>
- **AS5600 Datasheet** — ams OSRAM; magnetic status, angle registers and filtering. <https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf>
- **ESP32-S3 PCNT peripheral** — Espressif; incremental counting, quadrature decoding and glitch filtering. <https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/pcnt.html>
- **MPU-6500 product specification** — TDK InvenSense; sample rates, DLPF and interfaces. <https://invensense.tdk.com/wp-content/uploads/2020/06/PS-MPU-6500A-01-v1.3.pdf>

---

**Configuration precedence:** This revision intentionally supersedes earlier draft statements of **17T motor / 34T steering (2:1)** with the user's latest **34T motor / 34T shaft (1:1)** correction. It replaces the earlier **295 mm rear tyre nominal diameter** with the reported **279.4 mm** nominal diameter but does not claim a measured loaded circumference. Older references calling the AS5047P an *axle encoder* must not override the still-unverified BLDC-side mounting description.

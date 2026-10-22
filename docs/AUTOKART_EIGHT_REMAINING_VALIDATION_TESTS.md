# AutoKart eight remaining validation tests

Date prepared: 22 October 2026  
Repository branch: `bench-test`  
Purpose: convert the eight unresolved VCU areas into executable tests with
objective evidence and pass/fail criteria.

## Safety and evidence rules

- A source-code check, device detection, or successful isolated bench test is
  not proof that the installed vehicle system works.
- Record the firmware commit, operator, date, wiring revision, equipment and
  raw results for every run.
- Begin with the traction battery and all actuator power disconnected.
- Use jack stands, guarding and an accessible physical power disconnect before
  any powered rotating test.
- Do not conduct a ground-driving test until Tests 1–7 have passed at their
  applicable desk, bench and wheels-off-ground stages.
- Any unexplained motion, reset, encoder jump, communication loss or driver
  alarm is a failed test until its cause is identified and the test is repeated.
- Autonomous vehicle release remains **NO-GO** until every blocking test passes.

Status terms:

- **PASS:** physically run with saved evidence and all criteria met.
- **PARTIAL:** some component evidence exists, but the complete test has not
  passed.
- **FAIL:** test was run and one or more criteria were not met.
- **NOT TESTED:** executable but not yet run.
- **BLOCKED:** required hardware, measurement or prerequisite is unavailable.

## Current status summary

| ID | Validation area | Current status | Main blocker |
|---:|---|---|---|
| 1 | Integrated GPIO wiring | PARTIAL | Final assembled harness continuity and simultaneous subsystem test |
| 2 | BLDC sprocket/axle calibration | PARTIAL | Loaded tyre circumference and measured count/direction calibration |
| 3 | Steering calibration | PARTIAL / loaded test failed | Road-wheel mapping, backlash and loaded stutter/slip |
| 4 | Brake calibration | NOT TESTED / BLOCKING | Mounted released/applied span, force and stopping measurements |
| 5 | BLDC electrical verification | FAIL / PARTIAL | GP8630N voltage error and unmeasured terminal truth table |
| 6 | LoRa E-stop communication | PARTIAL PASS | Final disconnect policy, latency, reset behavior and output integration |
| 7 | Independent stopping hardware | BLOCKED | Independent traction isolation and fail-safe braking not implemented |
| 8 | Pi–VCU integration measurements | PARTIAL | micro-ROS, timing/jitter, geometry and deceleration measurements |

---

## Test 1 — Integrated GPIO wiring and bus coexistence

### Objective

Prove that the final ESP32-S3 GPIO-to-terminal wiring matches the master
pinout, has no shorts/conflicts, and supports all installed buses together.

### Final allocation under test

| Function | ESP32-S3 pins |
|---|---|
| PCA9548A + GP8630N upstream I2C | SDA GPIO1, SCL GPIO2 |
| R86mini steering | STEP GPIO4, DIR GPIO5, optional ENA GPIO6 |
| RA-02 receiver | RST8, CS9, MISO10, MOSI11, SCK12, DIO0 17 |
| BLD-750 transistor controls | BRK13, EN14, F/R15 |
| AS5047P SPI | CS16, SCK18, MOSI41, MISO42 |
| TB6600 brake | STEP38, DIR39, ENA40 |
| Raspberry Pi native USB | GPIO19/20 reserved by the ESP32-S3 USB interface |

The two AS5600 devices are not separate upstream I2C buses. They are fixed
address `0x36` devices isolated on PCA9548A channel 0 and channel 1.

### Equipment

- Multimeter with continuity and voltage modes
- Logic analyser or oscilloscope
- USB/power-bank power only for the first powered stage
- Current-limited 3.3 V supply if available
- Final wiring diagram and labelled harness

### Procedure

1. Disconnect traction battery, BLD-750 power, R86mini power, TB6600 power and
   all motors.
2. With power off, continuity-check every GPIO to its intended terminal.
3. Check every signal against adjacent signals, 3V3, 5V and GND for unintended
   shorts.
4. Confirm common control ground only where required. Confirm no motor power
   conductor reaches an ESP32 GPIO.
5. Power the ESP32-S3 from USB only and verify 3.3 V rails before installing
   sensors.
6. Scan the upstream I2C bus. Require PCA9548A `0x70` and GP8630N `0x58`.
7. Select PCA channel 0 only and require steering AS5600 `0x36`.
8. Disable channel 0, select channel 1 only and require brake AS5600 `0x36` if
   that encoder remains in the final architecture.
9. Read AS5047P over SPI mode 1 and require valid parity, diagnostics and angle.
10. Read RA-02 version register and require `0x12`.
11. Run all sensors together for 10 minutes without actuator power. Record I2C
    errors, SPI errors, resets and stale samples.
12. Use the logic analyser to confirm idle and active levels on all STEP, DIR,
    ENA, BRK, EN and F/R outputs while drivers remain disconnected.

### Pass criteria

- 100% continuity to intended terminals and no unintended shorts.
- Stable `0x70`, `0x58`, and channel-isolated `0x36` detections.
- Valid AS5047P and RA-02 SPI reads with no bus contention.
- No GPIO overlap with the master pinout.
- No unexpected reset or increasing communication-error counter during the
  10-minute combined run.
- Saved evidence includes continuity table, bus scans and logic traces.

### Current evidence

Individual AS5047P SPI, RA-02, GP8630N and Pico/PCA/dual-AS5600 tests worked.
The complete integrated ESP32-S3 harness has not passed this procedure.

Result: [ ] PASS  [ ] FAIL  [x] PARTIAL  [ ] BLOCKED

---

## Test 2 — BLDC encoder, sprocket ratio and axle calibration

### Objective

Calibrate the AS5047P mounted over the magnet at the centre of the 14-tooth
BLDC sprocket and convert its readings into axle RPM, speed and distance.

### Recorded geometry

- Encoder magnet: centred on 14T BLDC sprocket.
- Encoder PCB: stationary, perpendicular to sprocket face and facing magnet.
- Drive sprocket: 14 teeth.
- Rear-axle sprocket: 44 teeth.
- Reported chain length: 72.5 cm.
- Rear tyre: BKT SM 68, `11 × 7.10 - 5`.
- Nominal reduction: `44/14 = 3.142857` encoder-sprocket revolutions per axle
  revolution.

Detailed calculations and the mechanical inspection sheet are in
[`BLDC_AS5047P_DRIVETRAIN_CALIBRATION.md`](BLDC_AS5047P_DRIVETRAIN_CALIBRATION.md).

### Procedure

1. Confirm the magnet is diametrically magnetized, mechanically retained,
   centred, and within the AS5047P air-gap requirement.
2. Mark the 14T sprocket and rotate it exactly one revolution by hand.
3. Require one unwrapped encoder revolution, approximately 16384 ideal SPI
   counts, with `MAGNET OK` and no parity/diagnostic errors.
4. Repeat in both directions and freeze the sign convention: forward must be
   positive.
5. Mark both sprockets. Rotate the 44T axle sprocket one full revolution and
   verify approximately `44/14` encoder-sprocket revolutions.
6. Record chain pitch/type, link count, centre distance, tension and how the
   72.5 cm chain length was measured.
7. Inflate the tyre to the chosen operating pressure and apply representative
   vehicle load.
8. Roll straight for at least five complete tyre revolutions. Repeat three
   times and calculate loaded circumference from distance/revolutions.
9. Enter the measured count scale, ratio and circumference into the firmware.
10. At restrained low speed, compare encoder RPM against an independent
    tachometer and compare estimated distance against at least 10 measured
    metres.

### Pass criteria

- `MAGNET OK` and valid SPI frames over a complete revolution and intended RPM
  range.
- Repeatable count scale and correct forward/reverse sign.
- Physically verified 14T and 44T sprockets with rigid axle coupling.
- Three circumference measurements agree within 1%.
- Initial engineering target: estimated distance within 2% over 10 m and RPM
  within 2% of the reference instrument. These targets require project
  approval before becoming safety requirements.
- Raw and filtered readings are both logged; stale data is never treated as
  zero speed.

### Current evidence

AS5047P SPI angle and magnet diagnostics passed in isolation. PWM did not pass
and is not accepted. The nominal ratio is known; mounted count scale, loaded
circumference and end-to-end distance remain unmeasured.

Result: [ ] PASS  [ ] FAIL  [x] PARTIAL  [ ] BLOCKED

---

## Test 3 — Steering centre, range, mapping and backlash

### Objective

Calibrate the NEMA34/R86mini steering system and steering AS5600 so commanded
angle, measured shaft angle and actual road-wheel angle are related and
repeatable under load.

### Procedure

1. Raise the front wheels and mechanically inspect steering stops, linkage,
   fasteners, chain/gears and sensor mount.
2. Place the road wheels at measured mechanical straight-ahead—not merely the
   motor's assumed zero—and save AS5600 zero.
3. Confirm `MAGNET OK`, stable raw angle and correct left/right sign.
4. Command the following points from centre: `-45, -30, -15, 0, +15, +30,
   +45` degrees, or lower limits if the mechanism reaches a stop first.
5. At every point record requested motor angle, AS5600 angle, actual left road
   wheel angle, actual right road-wheel angle, pulse count and movement time.
6. Repeat the sweep from left-to-right and right-to-left at least three times.
   The difference at the same target is backlash/hysteresis.
7. Repeat centre commands from both directions and record centre repeatability.
8. Repeat at progressively higher pulse rates while unloaded. Stop increasing
   speed at the first stall, slip or stutter.
9. Repeat the approved low-speed sweep with tyres supporting the kart while
   stationary. Use an observer and accessible power disconnect.
10. Disconnect or misalign the AS5600 and verify the controller inhibits
    further unsafe steering commands rather than continuing open-loop.

### Pass criteria

- No mechanical-stop impact and no lost motion or unexplained encoder jumps.
- `MAGNET OK`; no stale, I2C or rejected-sample accumulation in normal motion.
- Left/right sign and physical wheel-angle mapping are documented.
- Centre returns consistently from both directions.
- Proposed initial target: road-wheel repeatability within 1 degree and
  steady-state target error within 2 degrees. Final limits require approval.
- Loaded steering completes the full approved range without the previously
  observed stutter/slip.
- Encoder failure inhibits unsafe operation.

### Current evidence

Unloaded movement, UI commands and software centre calibration worked. Loaded
steering stuttered/slipped and therefore remains a recorded failure.

Result: [ ] PASS  [x] FAIL  [ ] PARTIAL  [ ] BLOCKED

---

## Test 4 — Brake travel, release, force and stopping calibration

### Objective

Prove that the TB6600/NEMA23 brake moves between a repeatable released position
and a physically effective applied position, returns to release, and can stop
the kart within measured limits.

### Frozen feedback allocation

AS5600 #2 is allocated to brake-position feedback through PCA9548A channel 1.
It must still be mounted, calibrated and validated. Brake position alone does
not prove brake force or stopping performance, so autonomous release remains
blocked until the complete stopping test passes.

### Procedure

1. Disconnect mechanical brake linkage and verify TB6600/NEMA23 direction,
   pulse scaling and enable polarity at low speed.
2. Reconnect the linkage with traction isolated and identify the mechanical
   released position without forcing an end stop. Save it as zero.
3. If fitted, align the brake AS5600 and require `MAGNET OK` over full travel.
4. Increase application in small steps while measuring angle, motor pulses,
   cable/linkage travel and brake force or wheel resistance.
5. Define the fully applied setpoint from measured braking engagement—not from
   an assumed 30-degree or 45-degree value.
6. Apply and release for at least 20 cycles. Record released-position error,
   applied-position error, movement time, current and temperature.
7. Intentionally interrupt power during released, moving and applied states.
   Record the physical brake state and confirm it matches the approved
   fail-safe requirement.
8. With wheels raised, command low traction then brake. Verify throttle reaches
   zero before or at brake application.
9. Only after all prior tests pass, conduct controlled low-speed stopping runs
   at several repeatable entry speeds. Record speed, stop-command time,
   brake-response latency, stopping time, stopping distance and deceleration.
10. Define the conservative braking limit from repeated measured results.

### Pass criteria

- Repeatable release and application without mechanical binding or lost steps.
- Brake application physically resists/stops the wheel; a software state alone
  is not evidence.
- Throttle is zero before or simultaneous with brake engagement.
- Power-loss behavior satisfies the approved fail-safe design.
- Minimum measured deceleration and worst-case stopping distance are documented
  across repeated runs.
- No autonomous authorization if feedback is unavailable, stale or inconsistent.

### Current evidence

NEMA23 and driver rotation worked in isolation. Mounted brake travel, physical
force, power-loss behavior, deceleration and stopping distance have not passed.

Result: [ ] PASS  [ ] FAIL  [ ] PARTIAL  [x] BLOCKED

---

## Test 5 — BLD-750 electrical truth table and speed response

### Objective

Verify the complete ESP32-S3 → PN2222 → BLD-750 BRK/EN/F-R path, correct the
GP8630N output error, and characterize BLDC direction and speed response.

### Procedure

1. Keep traction power disconnected. Monitor ESP32 GPIO13/14/15 and the
   corresponding BLD-750 terminals with a meter or logic analyser.
2. Exercise every safe combination and record GPIO level, transistor state and
   measured BLD terminal voltage:

   | Command | BRK13 | EN14 | F/R15 | BLD BRK terminal | BLD EN terminal | BLD F/R terminal |
   |---|---:|---:|---:|---:|---:|---:|
   | Boot/safe | | | | | | |
   | Armed, throttle 0 | | | | | | |
   | Forward selected | | | | | | |
   | Reverse selected | | | | | | |
   | Brake command | | | | | | |
   | Watchdog/disconnect | | | | | | |

3. Confirm outputs remain safe during ESP32 reset, bootloader mode, unplug and
   brownout simulation.
4. Resolve the GP8630N voltage error before motor testing. Measure supply,
   configuration register, output mode, OUT-to-COM reference and loading.
5. Repeat commanded `0, 0.5, 1, 2, 3, 4, 5 V` with BLD-750 disconnected.
6. Require measured DAC output to meet the approved tolerance at every point.
7. Raise and restrain drive wheels; attach the AS5047P and an independent RPM
   reference where available.
8. Increase command in small steps, recording requested voltage, measured SV,
   encoder RPM, reference RPM, current, direction and any BLD alarm.
9. Repeat the same points in both directions.
10. Apply brake, disarm, disconnect control communication and reset the ESP32;
    measure time until EN is inactive, SV is zero and brake is asserted.

### Pass criteria

- Complete measured transistor/terminal truth table matches intended behavior.
- Safe output state throughout reset, startup and communication loss.
- DAC output passes the defined voltage tolerance; no software compensation is
  accepted until the electrical/configuration cause is understood.
- Repeatable monotonic RPM response with no unexplained driver alarm.
- CW/CCW mismatch is explained or corrected; both directions are bounded.
- AS5047P and reference RPM agree within the approved tolerance.

### Current evidence

GPIO13/14/15 produced working bench motion. However, the GP8630N test failed:
5.0 V requested produced approximately 3.3 V, and CW/CCW RPM differed greatly.

Result: [ ] PASS  [x] FAIL  [ ] PARTIAL  [ ] BLOCKED

---

## Test 6 — Pico 2 ↔ RA-02 ↔ ESP32-S3 E-stop communication

### Objective

Freeze one LoRa protocol and measure packet validity, transition reliability,
latency, range, startup and reset behavior before connecting it to a safety
output.

### Current working profile

- Transmitter: Pico 2 + RA-02 + LAY37 NC contact.
- Receiver: ESP32-S3 + RA-02.
- Frequency: 433 MHz.
- Modem: SF7, BW125, hardware CRC enabled.
- Packet: `ESTOP,state,sequence`.
- Released/NC closed: state `0`.
- Pressed or open wire: state `1`.
- Current Pico firmware sends state at startup and transitions, repeats each
  transition five times, and repeats STOP while active.

### Configuration still to resolve

The standalone receiver intentionally has no timeout, while another receiver
uses a 300 ms timeout. Revision D now reserves GPIO15 exclusively for BLD-750
F/R and allocates GPIO47 for the LoRa stop request. The disconnect policy and
external independent safety interface still require production validation.

### Procedure

1. Confirm antennas, 3.3 V supplies, local decoupling and SX1278 version `0x12`
   on both radios.
2. Save complete radio register/configuration output from both ends.
3. Power the transmitter with the switch released. Require a valid startup
   `ESTOP,0,sequence` reception.
4. Run at least 100 press/release cycles. Log transmit timestamp, receive
   timestamp, state, sequence, CRC errors, duplicates and missing sequences.
5. Hold STOP active for five minutes and verify repeated STOP reception.
6. Open the LAY37 wire without pressing the switch. It must be interpreted as
   STOP because the NC circuit is fail-aware locally.
7. Reset and power-cycle transmitter and receiver in every state. Confirm no
   unintended CLEAR operation.
8. Perform 1 m, 5 m, 10 m and intended maximum-range tests with the real
   enclosure, antenna orientation and power source.
9. Inject malformed packets, wrong state values, stale/duplicate sequences and
   CRC faults; require rejection.
10. Measure worst-case switch-to-valid-receiver-state latency.
11. If a radio-loss timeout is selected, transmit healthy state often enough
    to support it and prove that radio loss produces the approved fail-safe
    action. If no heartbeat is selected, document that radio loss cannot be
    detected while CLEAR and must not be claimed as fail-safe communication.

### Pass criteria

- 100/100 switch transitions produce the correct receiver state.
- Open NC wire produces STOP.
- No corrupted packet changes safety state.
- Sequence loss, duplicates, resets and CRC failures are logged.
- Worst-case stop latency is measured and accepted by the stopping safety case.
- Startup, reboot and power loss never generate an unintended CLEAR.
- The selected heartbeat/disconnect policy is identical in transmitter,
  receiver and documentation.

### Current evidence

Radio detection, hardware CRC and state packets worked in isolated testing.
Production timeout behavior, latency, long-cycle evidence, maximum range and
safety-output integration remain incomplete.

Result: [ ] PASS  [ ] FAIL  [x] PARTIAL  [ ] BLOCKED

---

## Test 7 — Independent stopping hardware and power-loss behavior

### Objective

Prove that emergency stopping does not depend on Raspberry Pi software, ROS,
normal ESP32 tasks, Wi-Fi, a browser, or continued LoRa communication.

### Required architecture

An approved independent safety chain must directly remove or inhibit traction
energy and place the braking system into its verified safe state. Examples may
include a safety-rated contactor/interlock and independently supervised brake
path. Component selection and circuit design require qualified electrical and
vehicle-safety review.

### Procedure

1. Produce the safety schematic, component ratings, wire sizes, fuse/protection
   design, normally closed loop and reset/re-arm logic.
2. Verify contactor/interlock current, voltage, DC interruption and fault
   ratings against the actual battery/controller system.
3. Confirm physical E-stop action cuts traction even with Pi and ESP32 software
   frozen or outputs stuck in the run state.
4. Confirm loss of control power, broken E-stop wire and disconnected receiver
   each produce the approved safe state.
5. Confirm clearing the E-stop does not automatically restart traction.
6. Verify deliberate manual reset/re-arm is required.
7. Perform welded-contact/stuck-output detection tests where the design supports
   them.
8. Measure traction-removal latency and brake-response latency independently.
9. Repeat at desk, wheels-off-ground and controlled low speed.
10. Inspect for overheating, arcing, voltage transients and unintended reset.

### Pass criteria

- Traction is independently inhibited under every injected software and wiring
  fault in the approved test matrix.
- Brake reaches its verified safe state.
- No automatic restart after E-stop clear, reboot or power restoration.
- Faults are detectable and logged without making the independent action depend
  on logging software.
- Measured total stop response satisfies the approved stopping-distance case.

### Current evidence

Only LoRa message communication is proven. No independent traction-power cut,
approved contactor/interlock or complete stopping safety case has been tested.

Result: [ ] PASS  [ ] FAIL  [ ] PARTIAL  [x] BLOCKED

---

## Test 8 — Raspberry Pi/VCU integration, timing and vehicle measurements

### Objective

Measure real Pi↔ESP32 communication, control-loop scheduling, startup/recovery,
vehicle geometry and braking performance using production-target firmware.

### Measurements required

- Exact ESP32 board, firmware version, build hash and configuration CRC.
- Raspberry Pi OS/kernel, ROS 2 and micro-ROS Agent versions.
- Actual wheelbase measured axle-centre to axle-centre.
- Rear track measured tyre-centre to tyre-centre.
- Loaded rear tyre circumference from Test 2.
- Minimum verified braking deceleration from Test 4.
- Command, feedback, encoder and control-loop rates.
- Worst-case scheduling jitter, task execution time, queue depth/overrun and
  communication latency.

### Procedure

1. Capture ESP32 `os.uname()` for bench MicroPython and the production ESP-IDF
   build/version identifiers separately.
2. Implement and bring up the selected native USB micro-ROS transport. A serial
   REPL connection alone is not micro-ROS validation.
3. Require clean connect, disconnect and reconnect for at least 20 cycles.
4. Verify command sequence numbers, timestamps, stale detection and reboot/boot
   identifiers.
5. Run the proposed rates: 50 Hz Pi commands, 100 Hz VCU telemetry/control and
   200 Hz encoder/steering acquisition where applicable.
6. Instrument every task with monotonic timestamps. Run for at least 30 minutes
   while logging period, execution time, deadline misses, queue overflows,
   memory/stack headroom and USB errors.
7. Stop Pi commands and unplug/restart the Pi. Measure local VCU response and
   require traction inhibition inside the approved command timeout.
8. Measure the physical wheelbase and track rather than retaining simulation
   assumptions.
9. Feed calibrated Test 2 speed/distance and Test 4 braking measurements into
   the vehicle-state interface.
10. Verify dashboard fields show source, timestamp, age and validity, and show
    `STALE/DISCONNECTED` rather than presenting old values as live.
11. Inject encoder disconnect, magnet fault, Pi loss, USB loss, invalid command,
    task overrun, ESP32 reset and brownout.

### Pass criteria

- Production transport reconnects reliably without unintended arm/restart.
- No missed safety deadline or unbounded blocking operation.
- Measured rates meet the approved schedule with documented worst-case jitter
  and CPU/stack headroom.
- Stale commands expire and traction is inhibited inside the approved timeout.
- Vehicle geometry and braking fields contain physical measurements, not
  nominal or simulation-only values.
- All injected faults enter the documented safe state and require deliberate
  recovery.

### Current evidence

ESP32 MicroPython, Raspberry Pi serial access and the Pi MPU-9250/6500 test
worked. Native USB micro-ROS, production scheduling, repeated recovery, actual
geometry capture and measured braking deceleration remain unvalidated.

Result: [ ] PASS  [ ] FAIL  [x] PARTIAL  [ ] BLOCKED

---

## Required execution order

```text
1. Integrated GPIO and bus validation
          ↓
2. BLDC encoder/drivetrain calibration
3. Steering calibration
4. Brake calibration
5. BLDC electrical verification
          ↓
6. LoRa communication validation
          ↓
7. Independent stopping hardware
          ↓
8. Pi/VCU timing and vehicle integration
          ↓
Controlled low-speed vehicle validation only after formal review
```

Tests 2–6 may have isolated desk portions performed in parallel, but powered
vehicle portions must respect their prerequisites.

## Test-run record

| Field | Value |
|---|---|
| Test ID | |
| Date/time | |
| Operator and observer | |
| Firmware commit/hash | |
| Wiring revision | |
| Vehicle configuration | |
| Battery/supply voltage | |
| Tyre pressure/load | |
| Equipment and calibration date | |
| Raw-log path | |
| Photos/video path | |
| Result | PASS / FAIL / BLOCKED |
| Failure/corrective action | |
| Reviewer/sign-off | |

## Release decision

Current decision: **NO-GO**.

The eight test procedures are now fully specified, but they are not physically
completed by this document. A status changes to PASS only after the hardware
run is performed, raw evidence is saved, and the stated criteria are reviewed.

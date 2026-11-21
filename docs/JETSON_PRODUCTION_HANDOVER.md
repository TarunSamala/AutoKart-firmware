# AutoKart Jetson production handover

Date: 27 October 2026  
Target: NVIDIA Jetson Orin Nano Super 8 GB, headless  
Decision: **NO-GO for autonomous vehicle motion; GO for fail-closed bench and commissioning work**

This document is the current engineering truth for moving AutoKart from the
working Brake_Test and VCU benches to a portable, standalone Jetson system. A
value marked **measured** came from physical output or an operator tape
measurement. **Nominal** is usable only for calculations. **TBD** is a release
blocker and must not be replaced by a guess.

## 1. Computer and ROS

| Item | Frozen answer | Evidence/state |
|---|---|---|
| Computer | NVIDIA Jetson Orin Nano Super 8 GB | Live target |
| CPU architecture | `aarch64` | Live `uname -m` |
| Operating system | Ubuntu 22.04.5 LTS, kernel `5.15.148-tegra` | Live target |
| NVIDIA stack | L4T R36.4.4; CUDA/TensorRT platform present | Live target |
| TensorRT | 10.3.0 | Live `trtexec` |
| Power mode | 25 W | Live `nvpmodel -q` |
| RAM/swap | 7.4 GiB RAM, 3.7 GiB swap | Live target |
| ROS 2 distribution | Humble is the required production target | Installation on the Jetson must still be explicitly verified |
| HP60C | USB, device `3482:6723` | Live Jetson enumeration and image stream |
| VCU connection | USB serial, not Wi-Fi | Required architecture |
| VCU path | Prefer `/dev/serial/by-id/usb-Espressif_*`; rediscover every boot | Device names can change |
| Dashboard link | Jetson binds `0.0.0.0:8781`; laptop uses Jetson Ethernet/WR850N IP or USB gadget `192.168.55.1` | Dashboard must never own the VCU connection |

TensorRT engine generation passed on the Jetson. The engine was 21.77 MiB,
build time was 698.54 s, and a 100-frame synthetic benchmark measured 32.18 ms
mean, 35.94 ms p95, or 31.07 detector FPS. That benchmark excludes camera/UI
cost and does not test detection accuracy. Live Brake_Test display processing
was approximately 11.8 to 14.3 FPS (70 to 85 ms).

## 2. Exact VCU

### Physically validated telemetry VCU

| Item | Answer |
|---|---|
| Board | ESP32-S3; existing validated board, plus a new N16R8 board under bring-up |
| Validated firmware | `bench/esp32s3_vcu_odometry_telemetry.py` |
| Firmware ID | `esp32s3-odom-telemetry-0.1.0` |
| Protocol | framed serial protocol version 1 |
| Baud | 115200 |
| Frames | `HELLO`, `MOTION`, `CONTROL_FEEDBACK` |
| Calibration ID | `UNVERIFIED-LAPTOP-V1` |
| Capabilities | bit mask 3 in the validated run |
| Transport | USB serial |

The fresh ESP32-S3 N16R8 has been identified as revision v0.2 with 16 MB quad
flash and 8 MB embedded PSRAM. This is useful headroom for firmware, buffers,
OTA assets, and micro-ROS, but it does not improve GPIO timing, sensor accuracy,
or safety by itself. It has not yet replaced the physically validated VCU.

### Production VCU decision

The production direction is ESP-IDF + FreeRTOS with a single ROS 2 bridge
owning the VCU command transport. MicroPython remains appropriate for
commissioning and diagnostics, but the present MicroPython telemetry program
does not implement the complete production actuator controller. The reference
firmware tree is architecture/reference code, not a released, physically
qualified VCU.

There are currently two real protocols:

1. The validated `HELLO/MOTION/CONTROL_FEEDBACK` telemetry protocol.
2. The Brake_Test `@AKCMD` command/status protocol.

Production must merge these into one versioned, CRC-protected, session-aware
protocol with command sequence, TTL, heartbeat, feedback, fault bits, boot ID,
and an explicit safe-state acknowledgement. Exactly one `vcu_bridge` process
may open the serial port. The browser/dashboard publishes requests to ROS and
never writes to serial directly.

## 3. Steering hardware and calibration

| Item | Current answer |
|---|---|
| Actuator | NEMA34 stepper steering mechanism |
| Driver | Bench code identifies DM860; older material mentions R86mini—physically read the installed label and freeze one answer |
| Control | STEP/DIR |
| STEP | ESP32-S3 GPIO4 |
| DIR | ESP32-S3 GPIO5 |
| ENA | GPIO6 allocated, but deliberately unused by the validated closed-loop bench test |
| Feedback | Steering AS5600 |
| AS5600 bus | SDA GPIO1, SCL GPIO2 |
| Sign convention | left negative, right positive |
| Mechanical centre | software zero set at the true physical centre |

Repeated hand tests measured centre near 0 degrees (typical return values 0 to
-2.29 degrees), left approximately -47.4 to -48.5 degrees, and right
approximately +69.4 to +78.5 degrees. These are mechanism/sensor angles, not
yet proven road-wheel angles.

The powered, wheels-raised closed-loop test passed:

| Command | Settled feedback | Absolute error |
|---:|---:|---:|
| -25.00 deg | -25.05 deg | 0.05 deg |
| 0.00 deg | 0.00 deg | 0.00 deg |
| +25.00 deg | +24.87 deg | 0.13 deg |
| final 0.00 deg | 0.00 deg | 0.00 deg |

The test finished disarmed. It proves closed-loop direction and tracking while
raised. It does not prove steering under tyre load. Production still needs:

- road-wheel angle versus AS5600 angle at left, centre, and right;
- mechanical hard-stop margin and an approved command limit;
- steering ratio/non-linearity, backlash, repeatability, and loaded stutter;
- current/thermal/stall behaviour and a safe failure response;
- verification of the actual driver model and ENA polarity.

Only +/-25 degrees is approved for the next raised commissioning test. The
larger measured sensor travel is not an approved command envelope.

## 4. BLDC drive and wheel feedback

| Item | Frozen answer |
|---|---|
| Motor controller | BLD-750 |
| Throttle | GP8630N DAC, I2C address `0x58` |
| GP8630N bus | SDA GPIO7, SCL GPIO21 |
| BLD BRK | GPIO13 through PN2222 |
| BLD EN | GPIO14 through PN2222 |
| BLD F/R | GPIO15 through PN2222 |
| Bench throttle | 0.40 V |
| Encoder | AS5047P, 14-bit SPI, mounted on the 14T motor sprocket |
| AS5047P pins | CS16, SCK18, MOSI41, MISO42; SPI mode 1 |
| Gear ratio | 14T motor to 44T axle; 3.142857 motor revolutions per axle revolution |
| Tyre | BKT SM 68 `11 x 7.10 - 5` |
| Forward encoder sign | positive after telemetry direction correction |

One-turn hand test passed diagnostically:

- forward: +16,541 counts, +1.00958 motor revolution, nominal +0.2820 m;
- reverse: -16,860 counts, -1.02905 motor revolution, nominal -0.2874 m;
- stationary count spans: 0 before and after;
- forward + reverse net: -319 counts;
- result: `PASS_DIAGNOSTIC`.

The metre conversion is still nominal because the 11-inch tyre designation is
not its loaded rolling circumference. Before production, measure three loaded
five-revolution rollouts, freeze metres-per-count, and pass a measured 10 m
forward/reverse distance test (target error no greater than 2%). Also qualify
maximum encoder RPM, magnet retention, air gap, parity/errors, chain coupling,
and slip. The motor-sprocket encoder cannot detect a broken/slipping chain or
tyre slip by itself.

Brake_Test allows the AS5047P to be optional only for isolated, wheels-raised
demonstration. It is mandatory for production speed control and odometry.
Unavailable speed must never be interpreted as zero.

## 5. Brake system

The kart has two different stop mechanisms:

1. BLD-750 electronic brake through GPIO13/PN2222.
2. Mechanical brake actuator: NEMA23 + TB6600, STEP38, DIR39, ENA40.

**There is no brake encoder.** Any older PCA9548A/second-AS5600 brake-feedback
material is superseded and must not be used as the production design.

Brake_Test proved that a command, process exit, controller fault, or LoRa STOP
can place the BLD controller into wheel-lock state: 0 V, EN stop, BRK applied.
It did not qualify the mechanical brake. The following remain TBD:

- released and fully-applied step positions;
- homing/reference method after power-up;
- end-stop or force/pressure feedback;
- power-loss behaviour;
- command-to-deceleration latency;
- stopping distance versus speed, mass, surface, battery, and tyre condition;
- repeated worst-case fade and thermal performance.

Because there is no brake-position sensor, production must add at least a
validated released/applied limit scheme, position sensor, or force/pressure
feedback. A step count alone cannot prove that the brake moved.

## 6. E-stop and safety

The LoRa bench consists of Pico 2 + RA-02 + LAY37 transmitter and ESP32-S3 +
RA-02 receiver. Radio settings were 433 MHz, SF7, BW125, CRC on. ESP32 RA-02
pins are RESET8, NSS9, MISO10, MOSI11, SCK12, DIO0 17.

Live logs proved that `ESTOP,1,<sequence>` reached the ESP32 and immediately
caused `BRK APPLIED`. `ESTOP,0` cleared the radio state but did not restart the
motor; manual ARM was required. This is the correct restart policy.

This radio/software path is not an independent, safety-rated E-stop. The
production kart requires a normally-closed, hardwired device that removes
traction energy through suitably rated contactors and makes a broken wire a
STOP. It should also provide an isolated monitored contact to the VCU and
require manual reset. LoRa, dashboard STOP, camera stop, and VCU watchdog are
supplementary layers.

Required E-stop qualification: range and shadowing, worst-case stop latency,
packet loss/interference, transmitter/receiver resets, stuck input, broken
wire, brownout, and confirmation that no CLEAR packet can re-arm propulsion.

## 7. Vehicle geometry

The following values were supplied but are not all present in the current
single-source runtime geometry and therefore remain **unconfirmed**:

| Item | Supplied value | Release state |
|---|---:|---|
| Wheelbase | 1.1176 m | Must remeasure |
| Vehicle width | 1.330 m | Must define body versus wheel envelope |
| Front extent from `base_link` | 1.650 m | Must remeasure and define `base_link` |
| Rear overhang | 0.400 m | Must remeasure |

The current measured camera/bumper geometry is:

| Item | Frozen value |
|---|---:|
| Camera optical-centre height | 0.83 m |
| Camera pitch down | **18.0 degrees** |
| Camera-ground projection to front bumper | 1.30 m |
| Bumper width | 1.016 m (40 in) |
| Corridor lateral margin | 0.15 m each side |
| Conservative commissioning stop gap in geometry config | 0.40 m |

The Brake_Test UI additionally offers 0.40, 0.70, 1.00, and 1.20 m blue-cone
stop gaps; the launch default is 1.20 m.

Still measure and freeze `base_link`, front/rear/left/right extents, ground
clearance, vehicle mass with battery/driver, loaded tyre radius/circumference,
and usable road-wheel steering angle. Every value needs provenance: measured,
manufacturer, or estimated.

## 8. HP60C camera calibration and diagnostics

### Frozen runtime calibration

**Camera pitch is 18.0 degrees downward. 16.37 and 17.8 degrees are
superseded.** Active vehicle geometry, Brake_Test UI, cone guard, portable
launcher, Jetson launcher, and pitch-fit configured default use 18.0 degrees.
Historical JSON, logs, and reports retain their original values as evidence;
they are not runtime configuration.

| Item | Current value/state |
|---|---|
| Model | HP60C RGB-D |
| USB ID | `3482:6723` |
| Runtime image | 640 x 480 observed |
| Height | 0.83 m measured |
| Pitch | 18.0 deg down, operator physical measurement and frozen configuration |
| Roll/yaw | TBD |
| Forward offset from `base_link` | TBD; camera-to-bumper horizontal distance is 1.30 m |
| Left/right offset | TBD |
| Depth scale | nominal SDK conversion only; independent scale qualification TBD |
| Example depth intrinsics seen in logs | `fy=585.01947`, `cy=239.96358`; complete signed artifact TBD |
| RGB-depth registration | not independently qualified |
| Reliable min/max depth | TBD by target/lighting survey |

### What passed

- Jetson USB enumeration and live RGB-D startup.
- Live RGB image delivery and cone boxes in the dashboard.
- TensorRT FP16 engine build and synthetic throughput benchmark.
- Focused geometry/depth/calibration unit tests: 20 passed after the 18-degree
  runtime freeze.
- A latest single-station 1.70 m cone metric run reported
  `PASS_DIAGNOSTIC`, but also explicitly remained `calibration_verified=false`
  and `diagnostic_only=true`.

### What did not pass or is incomplete

- A four-station diagnostic produced about 20.3, 23.3, 21.8, and 24.9 cm
  errors and `FAIL_DIAGNOSTIC`; its fitted pitch hit an unreliable boundary.
- An 18-degree 1.55 m cone check had -13.2 cm error and failed its 7.75 cm
  tolerance.
- Recorded 30-second rate was RGB 7.20 Hz, depth/paired 4.59 Hz; another run
  was RGB 5.51 Hz and depth/paired 4.81 Hz. These prove streaming, not the
  required production rate or bounded latency.
- Earlier calibration points used an operator tape grid, not independent
  validation targets.
- Three independent distance-validation stations, complete intrinsics and
  distortion, RGB-depth registration, depth scale, optical extrinsics, and
  lighting/range limits remain incomplete.
- Visual cone detections exist, but false positives were observed; no final
  target-domain precision/recall, false-stop rate, or missed-stop rate is
  approved.

Therefore 18 degrees is now the authoritative physical configuration, but the
camera is not yet a production-qualified metric safety sensor.

## 9. IMU and odometry

| Item | Answer |
|---|---|
| Wheel odometry | AS5047P motor-sprocket counts through 14T:44T conversion |
| Steering feedback | AS5600 available |
| IMU | Jetson IMU package/config exists; exact installed IMU, bus/address, orientation, bias, rate, and noise are not physically qualified |
| ROS packages | `vcu_bridge` and `autonomous_odometry` |
| Boot/session identity | validated boot ID changes across ESP32 reset |
| Calibration | fail-closed; metres-per-count and wheelbase must be nonzero and verified before `/odom` is trusted |

The 30-second VCU serial validation passed with MOTION 49.54 Hz,
CONTROL_FEEDBACK 20.004 Hz, HELLO 1 Hz, zero CRC/malformed/non-frame errors,
zero sequence gaps/duplicates/reversals, zero timestamp reversals, validity 3,
fault 0, steering -47.99 to +76.99 degrees, and stationary count/steering spans
of zero. The reboot test passed: boot ID changed and the new epoch restarted
cleanly. The ROS bridge has published typed identity, link, wheel encoder,
steering, and VCU-state topics from live serial.

This is good interface evidence. It is not yet ground-truth odometry. Required
next: loaded metres-per-count, IMU commissioning, straight/reverse push tests,
10 m ground truth, turns and figure-eight, time synchronization, EKF covariance
tuning, slip detection, reboot during motion, stale-input injection, and
`odom -> base_link` continuity checks.

## 10. Brake_Test: exact behaviour and software

Brake_Test is a wheels-raised/controlled demonstrator, not full autonomy.

- starts the BLD test at 0.40 V after explicit ARM/START;
- detects cones using YOLO26s;
- uses blue cones only for the automatic full-brake rule;
- selectable front-bumper gaps: 0.40, 0.70, 1.00, 1.20 m;
- Jetson launch default: 1.20 m;
- display confidence: 0.35;
- blue brake confidence: 0.80;
- one confirmation frame in the current demonstrator;
- locks on threshold, controller/camera failure, command failure, or exit;
- AS5047P and LoRa may be optional only in the isolated Brake_Test profile;
- Delaunay and centreline overlays are diagnostic display features and do not
  turn Brake_Test into a steering autonomy stack.

### Jetson system packages used by Brake_Test

`python3-venv`, `python3-numpy`, `python3-opencv`, `python3-scipy`,
`python3-yaml`, `python3-serial`, `python3-libnvinfer`, `tensorrt`, `cmake`,
`g++`, `make`, `libusb-1.0-0`, `curl`, `xdg-utils`, `iproute2`, and
`network-manager`, plus the Jetson CUDA runtime and the HP60C vendor SDK/rules.
The TensorRT engine is built on the target from `models/yolo26s/best.onnx`;
engine files must not be copied between different Jetson/runtime combinations.

The portable CPU builds use `python3`, `python3-venv`, `python3-pip`,
`python3-opencv`, `python3-numpy`, `python3-yaml`, `cmake`, `g++`, `make`,
`curl`, `xdg-utils`, `iproute2`, `libusb-1.0-0`, `ca-certificates`, and pinned
`onnxruntime==1.23.2` in a venv.

### Software verification already recorded

- Current focused 18-degree geometry/depth/calibration tests: 20 passed.
- Brake/perception campaigns recorded 180, 196, and 72 passing tests in the
  latest full session.
- Other focused campaigns recorded 32 ROS Python, 35 frontend, 140 Phase-1,
  and 150 kart tests passing.
- VCU default/ARM-lock logic recorded 83 passing checks.
- No-encoder/bridge/link/localization: 16 passing tests.
- Latency/lost-target/mapping software checks: 21 passing tests.
- VCU autonomy bundle: autonomous odometry 28 tests and VCU bridge 5 tests,
  zero failures in the packaged build.

These counts overlap across campaigns and are not additive certification.
Software tests do not replace live stopping, steering, odometry, or fault tests.

## 11. Driving policy and track rules

Until measured commissioning closes the TBDs, the system remains unable to
ARM for ground autonomy. Proposed first-ground-test policy (requires safety
sign-off before use):

| Policy | Proposed value/action |
|---|---|
| First powered ground speed | <=0.3 m/s after valid speed and brake qualification |
| Normal maximum speed | TBD from stopping envelope |
| Blue obstacle stop gap | 1.20 m initial conservative setting |
| Detection confidence | >=0.80 for brake trigger |
| Consecutive frames | Increase from demonstrator value 1 only after measured latency/false-trigger study |
| Camera loss/stale | STOP and latch fault |
| Odometry loss/stale | STOP and latch fault |
| VCU telemetry loss/stale | VCU local safe state + STOP and latch fault |
| Unknown/ambiguous object | HOLD/STOP |
| Re-arm | manual, only after fault cause is cleared |

Track semantics cannot yet be frozen from physical evidence. Recommended
policy is blue=left boundary and yellow=right boundary only when the event
rules guarantee that convention; paired orange cones may define special
boundaries; orange obstacles may be manoeuvred around only with a validated
free corridor; a blue object explicitly classified as an obstacle, an orange
wall across the corridor, or an ambiguous scene means STOP. Encode these as a
versioned venue rules file rather than hidden constants.

## 12. Autonomous phases and current completion

### Existing software foundation

The repositories contain message definitions, camera/perception tooling,
vehicle geometry, localization/odometry, Delaunay planning, controllers,
safety bridge, VCU safety, VCU bridge, diagnostics, dashboard gateway,
bring-up, vehicle description, scenario tooling, and simulation adapters.
Existing algorithmic work also includes SLAM Toolbox integration,
route recording, Pure Pursuit/Stanley concepts, and safety-gated control.

### Critical boundary

The current Delaunay implementation is local/reactive (about a 12 m local
horizon in the simulation implementation). The requested workflow—drive one
or two teaching laps, persist and deduplicate cones, build a global Delaunay
map/centreline, close the loop, then transition to repeat autonomy—is not yet
implemented as a production live-sensor system.

No complete live chain has passed on the kart:

`HP60C -> metric registered detections -> persistent map/localization -> path -> steering/speed command -> one ROS VCU bridge -> actuator feedback -> safe stop`.

### Honest readiness estimate

| Area | Estimated completion | Meaning |
|---|---:|---|
| Software architecture and test scaffolding | 70% | Most components/interfaces exist; integration and production hardening remain |
| Bench sensing and VCU interface | 55% | Serial, encoders, raised steering, BLDC brake, and LoRa have useful evidence |
| Live perception/metric calibration | 45% | Detection runs and GPU backend work; metric/accuracy/latency qualification incomplete |
| Vehicle controls and odometry | 35% | Raised/hand tests pass; loaded steering, tyre scale, IMU/EKF and braking incomplete |
| Teach-and-repeat autonomy | 25% | Local planning components exist; persistent global workflow and live integration do not |
| Safety case and production release | 15% | Fail-closed intent exists; independent E-stop, stopping envelope, fault/endurance qualification absent |
| Overall production autonomy | **about 30% complete / 70% remaining** | Engineering estimate, not a certified metric |

The project is much further along as a software/bench demonstrator than as a
production vehicle. Calling it 70% complete because many ROS packages exist
would ignore the hardest part: measured vehicle behaviour and safety evidence.

## 13. Required production sequence

Do not jump directly to autonomous driving. Complete these gates in order:

1. **Freeze hardware:** schematic, exact driver labels/polarities, connectors,
   fusing, grounding, power domains, GPIO truth table, hardwired E-stop.
2. **Freeze VCU:** one production firmware/protocol, boot identity, CRC,
   heartbeat/TTL, local watchdog, command acknowledgement, latched faults,
   deterministic safe outputs, brownout/reset tests.
3. **Commission each actuator:** mechanical brake feedback/homing, loaded
   steering mapping/backlash/stall, throttle voltage and BLD truth table.
4. **Calibrate motion:** loaded tyre circumference, 10 m counts, wheelbase,
   road-wheel steering, IMU orientation/bias/noise, EKF covariance.
5. **Qualify perception:** signed 18-degree calibration artifact, RGB-depth
   registration, scale/range/lighting, target-domain precision/recall,
   end-to-end age and dropout.
6. **Build live ROS ownership:** only `vcu_bridge` opens serial; launch under
   systemd; dashboard is read/request only; record rosbag/event/fault logs.
7. **Stationary fault injection:** unplug camera/VCU/encoder/IMU, corrupt CRC,
   stale frames, restart nodes/VCU/Jetson, E-stop, brownout.
8. **Wheels-raised HIL:** steering/throttle/brake sequences, watchdog and
   automatic blue-cone stop, with independent physical power cut.
9. **Ground commissioning:** push tests, <=0.3 m/s straight runs, brake latency
   and stopping envelope, steering/yaw, reverse, figure-eight, repeatability.
10. **Mapping/autonomy:** implement persistent cone association/global map,
    loop closure and route validation; then supervised low-speed closed-course
    runs with staged speed increases.
11. **Production qualification:** long-duration soak, thermal, vibration,
    EMI/EMC, water/dust as applicable, connector pull, low battery, storage/log
    exhaustion, regression matrix, operating procedures, and signed release.

## 14. Portable standalone deployment definition

A deployable Jetson image/package must contain source and target-specific build
scripts, not laptop binaries. It must provide:

- pinned Ubuntu/JetPack/ROS/dependency manifest;
- on-target TensorRT engine build with model hash and metadata;
- udev rules for HP60C and ESP32 stable identities;
- one systemd target that orders camera, perception, odometry, safety, VCU
  bridge, dashboard, logging, and health supervision;
- separate `commissioning` and `production` profiles;
- no replay/simulation inputs in the production profile;
- fail-closed calibration schema and unique calibration ID;
- automatic log rotation and export;
- offline installation artifacts or an explicitly documented network install;
- install, verify, start, stop, rollback, and recovery commands;
- a manifest/checksum and a hardware acceptance checklist.

The existing Brake_Test and VCU-autonomy archives are useful deployment
building blocks, but they are not yet one production image and must not be
described as autonomous-vehicle-ready.


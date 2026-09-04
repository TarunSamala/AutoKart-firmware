.nr PS 10
.nr VS 13
.po 0.7i
.ll 6.6i
.tl ''AutoKart RC - Actuator Control Report''02 September 2026
.sp 0.6
.ps 22
.ft B
AutoKart RC - Actuator Control
.ps 11
.ft R
Compact implementation report
.sp 0.5
.nf
PRIMARY IMPLEMENTATION: autokart_rc_v5_controlled_stop.py
.fi
.sp 0.4
This report summarizes how the workspace controller commands the steering,
brake, and traction actuators. The v5 file is treated as the primary integrated
design; full_bldc_test.py is documented as a separate direct-drive bench test.

.ps 14
.ft B
1. System at a glance
.ps 10
.ft R
.nf
Browser UI  ->  ESP32-S3 HTTP server/state machine  ->  actuator interfaces
                         |                 |
                         |                 +-- TB6600 step/dir: steering + brake
                         +-------------------- GP8630N I2C -> BLD750 SV (throttle)
.fi

.ps 12
.ft B
Steering actuator
.ps 10
.ft R
Stepper motor through a TB6600. GPIO4 produces STEP pulses, GPIO5 selects DIR,
and GPIO6 controls ENA. Positive motion is configured as clockwise/right.

.ps 12
.ft B
Brake actuator
.ps 10
.ft R
Second TB6600 axis: GPIO10 STEP, GPIO11 DIR, GPIO12 ENA. Positive motion is
clockwise brake application. v5 range: 0 to +15 degrees.

.ps 12
.ft B
Throttle / traction actuator
.ps 10
.ft R
The ESP32 writes a 16-bit value to GP8630N over I2C. The module is in 0-10 V
mode and its OUT feeds BLD750 SV; v5 software ceiling is 3.0 V.

.ps 14
.ft B
2. Wiring and signal map
.ps 10
.ft R
.nf
Function             ESP32-S3             Destination / meaning
Steering STEP        GPIO4                TB6600 PUL+
Steering DIR         GPIO5                TB6600 DIR+; positive = right
Steering ENA         GPIO6                TB6600 ENA+; active-low in code
Brake STEP/DIR/ENA   GPIO10/11/12         Second TB6600 axis
I2C                  GPIO8 SDA, GPIO9 SCL GP8630N, address 0x58
Throttle analog      GP8630N OUT          BLD750 SV; GP GND -> BLD COM
.fi
Supporting wiring references are rc_connection.txt, Steering Brake
connections.txt, and throttle _details.

.ps 14
.ft B
3. Steering and brake control
.ps 10
.ft R
Each axis is a StepperAxis object. A browser command sets a target or nudge;
the ESP32 clamps the angle, converts degrees to pulses, and the main loop emits
one pulse at a time until the software target is reached.

.nf
v5 limits:       steering -90 to +90 deg       brake 0 to +15 deg
Calibration:     pulses/degree = (3200 / 360) * gear ratio; ratios are 1.0 now
Timing:          steering 1200 -> 500 us        brake 1500 -> 700 us
Commands:        left, right, nudge, hold, center
.fi
The position is OPEN LOOP: it is only a pulse count from an assumed physical
zero. There is no encoder or limit-switch feedback. Physical zero and gear
ratios therefore must be calibrated before relying on the angle estimate.

.bp
.tl ''AutoKart RC - Operation and Safety''02 September 2026
.ps 22
.ft B
Actuator Control - Operation and Safety
.ps 10
.ft R
.ps 14
.ft B
4. Throttle control sequence
.ps 10
.ft R
.IP "1." 4
The UI sends a throttle request. The server clamps it to 0-3.0 V and accepts it
only while armed and while the brake is released.
.IP "2." 4
The request becomes throttle_target. Every 15 ms, throttle_update ramps the
actual output toward the target.
.IP "3." 4
v5 rises by 0.02 V/update and normally falls by 0.10 V/update. The DAC code is
int((V / 10) * 65535), written to GP8630N register 0x02.
.IP "4." 4
GP8630N OUT controls BLD750 SV.

.ps 12
.ft B
BENCH-ONLY WARNING
.ps 10
.ft R
The source notes that the BLD750 previously faulted near 0.75 V. The 3.0 V
ceiling is not electrically validated. Keep wheels off the ground until the
fault and command scaling are resolved.

.ps 14
.ft B
5. Brake, arm, stop, and watchdog logic
.ps 10
.ft R
.nf
Boot:       throttle 0 V; steering center; brake applied; disarmed.
Arm:        throttle 0; steering holds; brake releases; commands accepted.
Brake on:   throttle zero immediately; brake target = +15 deg clockwise.
Brake off:  brake target = 0; throttle forced zero; new throttle is required.
Stop:       disarm; ramp throttle down at 0.04 V every 15 ms; then brake.
Timeout:    no heartbeat for 1.8 s while armed -> disarm, throttle zero,
            steering hold, brake apply.
E-stop:     latched; throttle zero, steering hold, brake apply; commands block
            until reset. Reset leaves the system disarmed.
.fi

.ps 14
.ft B
6. Standalone BLD750 actuator test
.ps 10
.ft R
full_bldc_test.py separately drives BLD750 digital inputs through PN2222A
transistors. A GPIO high turns the transistor on and pulls the input low.
.nf
GPIO13 -> EN:  high/pulled low = RUN; low/open = STOP
GPIO14 -> BRK: high/pulled low = brake released; low/open = quick brake
GPIO15 -> F/R: direction selection (software labels CW and CCW)
.fi
Its safe stop sets GP8630N to 0 V, applies the BLD750 brake, and stops enable.
Direction reversal is allowed only in the full safe state.

.ps 14
.ft B
7. Practical validation checklist
.ps 10
.ft R
.IP "-" 4
Confirm TB6600 microstep switches match 3200 pulses/revolution.
.IP "-" 4
Measure travel; update gear ratios and software limits.
.IP "-" 4
Verify DIR polarity and brake apply direction at low speed.
.IP "-" 4
Measure GP8630N output and verify 0-10 V scaling before traction testing.
.IP "-" 4
Test arm, brake/throttle interlock, controlled stop, E-stop, and 1.8 s timeout
with wheels off the ground.
.IP "-" 4
Add physical end stops or feedback before treating open-loop angle as safety-
critical position.

.ft I
Source basis: autokart_rc_v5_controlled_stop.py (primary), rc_bench.py (earlier
bench controller), full_bldc_test.py (standalone BLD750 test), and wiring notes.
.ft R


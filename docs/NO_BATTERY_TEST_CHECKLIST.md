# AutoKart no-battery test checklist

Date: __________  Operator: __________  Firmware commit: __________

ESP32-S3 port: __________  Pico 2 port: __________

This checklist uses USB or power-bank power only. Keep the traction battery,
BLD750 motor supply, R86mini supply, TB6600 supply, and all motors disconnected.

Result marking: `[x] PASS` / `[x] FAIL` / `[x] BLOCKED`. Record evidence for
every result. Detection or GPIO activity does not prove that a disconnected
driver or actuator works.

## A. Safety and preparation

- [ ] Traction battery disconnected
- [ ] BLD750 motor supply disconnected
- [ ] R86mini motor supply disconnected
- [ ] TB6600 motor supply disconnected
- [ ] Motors cannot move unexpectedly
- [ ] Multimeter set correctly before probing
- [ ] Common ground verified where required
- [ ] RA-02 antenna attached before radio transmission
- [ ] Serial ports identified with:

  ```bash
  find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print
  ```

## B. ESP32-S3 MicroPython

- [ ] ESP32-S3 appears as a serial device
- [ ] MicroPython REPL opens
- [ ] `os.uname()` reports ESP32-S3
- [ ] File copy with `mpremote` succeeds
- [ ] Soft reset returns to MicroPython
- [ ] No boot loop or unexpected exception

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Evidence/notes: _____________________________________________________________

## C. GP8630N I2C connectivity — Test 1.1

- [x] GP8630N wired to finalized GPIO1 SDA / GPIO2 SCL
- [x] GP8630N detected at `0x58`
- [x] Detection remains stable after reset

Result: [x] PASS  [ ] FAIL  [ ] BLOCKED — completed 2026-10-22

Evidence/notes: I2C ACK confirmed on ESP32-S3. This does not prove voltage
accuracy.

## D. GP8630N unloaded voltage — Test 1.2

- [ ] GP8630N OUT disconnected from BLD750 SV
- [ ] Multimeter connected OUT-to-GND
- [ ] Commanded 0 V measured and recorded
- [x] 0.5–5.0 V command points measured
- [ ] GP8630N supply voltage measured: __________ V
- [ ] Output mode/register verified
- [ ] Output loading verified absent
- [ ] Root cause of scale/ceiling error identified
- [ ] Corrected voltage sweep passes tolerance

Result: [ ] PASS  [x] FAIL  [ ] BLOCKED — measured 2026-10-22

Evidence/notes: 5.0 V requested produced 3.3 V. Approximate measured response:
`measured_V = 0.5933 * requested_V + 0.1333`.

## E. PCA9548A multiplexer

- [x] VIN connected to Pico `3V3(OUT)`, not `3V3_EN`
- [x] RST held at `3V3(OUT)`
- [x] A0/A1/A2 grounded
- [x] GP0 SDA / GP1 SCL connected
- [x] PCA9548A detected at `0x70`
- [x] Detection remains stable after reset

Result: [x] PASS  [ ] FAIL  [ ] BLOCKED

Evidence/notes: _____________________________________________________________

## F. Steering AS5600 — PCA channel 0

- [x] AS5600 detected at `0x36` only when channel 0 is selected
- [ ] Magnet state reports `MAGNET OK`
- [ ] Raw angle changes smoothly through manual rotation
- [ ] Filtered angle changes smoothly
- [ ] Center calibrated with `zero steering`
- [ ] Power-cycle restores saved zero correctly
- [ ] No unexplained jumps
- [ ] Rejected-sample count remains stable during normal motion

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Magnet/motion notes: ________________________________________________________

## G. Brake AS5600 — PCA channel 1

- [x] AS5600 detected at `0x36` only when channel 1 is selected
- [ ] Magnet state reports `MAGNET OK`
- [ ] Raw angle changes smoothly through manual rotation
- [ ] Filtered angle changes smoothly
- [ ] Rest position calibrated with `zero brake`
- [ ] Power-cycle restores saved zero correctly
- [ ] No unexplained jumps
- [ ] Rejected-sample count remains stable during normal motion

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Magnet/motion notes: ________________________________________________________

## H. Steering encoder noise and fault diagnostics

- [ ] Direct-AS5600 diagnostic wiring used, or correct PCA-aware test selected
- [ ] Center calibrated
- [ ] Five-second stationary noise test completed
- [ ] Peak-to-peak noise recorded: __________ degrees
- [ ] Standard deviation recorded: __________ degrees
- [ ] No I2C errors
- [ ] No magnet errors
- [ ] No stale-feedback errors
- [ ] No unexplained angle-jump errors
- [ ] Manual lock-to-lock movement stays within expected calibrated range

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Evidence/notes: _____________________________________________________________

## I. AS5047P SPI encoder

- [ ] AS5047P powered from one valid supply input only
- [ ] Common ground connected
- [ ] SPI wiring checked
- [ ] `SPI OK` reported
- [ ] `MAGNET OK` reported
- [ ] Angle changes smoothly through manual rotation
- [ ] Angle wraps correctly at 0/360 degrees
- [ ] Clockwise/counter-clockwise RPM signs are opposite
- [ ] Stationary filtered RPM returns near zero
- [ ] No parity or diagnostic errors
- [ ] No unexplained count jumps

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Evidence/notes: _____________________________________________________________

## J. RA-02 module detection

- [ ] Correct 433 MHz antenna installed
- [ ] 3.3 V supply verified
- [ ] Local 100 nF and 10–100 uF decoupling installed
- [ ] SPI pinout verified
- [ ] SX1278 version register returns `0x12`
- [ ] Radio enters 433 MHz/SF7/BW125/CRC receive mode
- [ ] Detection remains stable after reset

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Evidence/notes: _____________________________________________________________

## K. LAY37 and LoRa E-stop messaging

- [ ] Released NC contact reads LOW/clear
- [ ] Pressed or open wire reads HIGH/STOP
- [ ] Startup released state sends `ESTOP,0,sequence`
- [ ] Button press sends `ESTOP,1,sequence`
- [ ] Button release sends `ESTOP,0,sequence`
- [ ] ESP32-S3 receives and parses all three states
- [ ] Twenty switch cycles completed
- [ ] No unexplained STOP screen/state
- [ ] No CRC errors during stationary test
- [ ] 1 m range test passed
- [ ] 5 m range test passed
- [ ] 10 m range test passed
- [ ] Transmitter works from power bank without laptop

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Dropped packets/notes: ______________________________________________________

## L. Wi-Fi control UI without drivers

- [ ] BLD750, R86mini and TB6600 disconnected
- [ ] ESP32 access point appears
- [ ] UI loads on phone
- [ ] UI loads on laptop
- [ ] Slider responds smoothly
- [ ] EN buttons respond
- [ ] BRK buttons respond
- [ ] Direction buttons respond
- [ ] SAFE STOP returns requested throttle to zero
- [ ] Page refresh does not activate an output unexpectedly
- [ ] Phone layout remains usable
- [ ] Reconnection after Wi-Fi interruption succeeds

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Evidence/notes: _____________________________________________________________

## M. No-load watchdog output test

- [ ] BLD750 remains disconnected
- [ ] GP8630N OUT monitored with multimeter
- [ ] GPIO13/14/15 monitored with meter or logic analyser
- [ ] Small DAC value requested
- [ ] Browser/Wi-Fi connection intentionally removed
- [ ] Time to GP8630N OUT = 0 V recorded: __________ ms
- [ ] BRK logic enters safe state
- [ ] EN logic enters safe state
- [ ] No automatic restart after reconnection

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Current expected browser timeout: approximately 1500 ms.

## N. Steering STEP/DIR electrical output

- [ ] R86mini disconnected
- [ ] GPIO4 STEP monitored
- [ ] GPIO5 DIR monitored
- [ ] GPIO6 ENA monitored if used
- [ ] Left and right commands produce opposite DIR states
- [ ] Larger command produces the correct additional pulse count
- [ ] Speed setting changes pulse frequency
- [ ] Center command generates return pulses
- [ ] Center calibration generates no motion pulse
- [ ] SAFE STOP stops new steering pulses

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Evidence/notes: _____________________________________________________________

## O. Brake STEP/DIR electrical output

- [ ] TB6600 disconnected
- [ ] GPIO38 STEP monitored
- [ ] GPIO39 DIR monitored
- [ ] GPIO40 ENA monitored
- [ ] Apply command produces pulses in apply direction
- [ ] Release command produces pulses in reverse direction
- [ ] Calibration generates no pulses
- [ ] Repeated same-position command generates no extra travel
- [ ] SAFE STOP requests brake application

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Evidence/notes: _____________________________________________________________

## P. Raspberry Pi communication

- [ ] ESP32 enumerates on Raspberry Pi as `/dev/ttyACM*`
- [ ] MicroPython REPL opens from Raspberry Pi
- [ ] File upload succeeds
- [ ] Soft reset succeeds
- [ ] Serial output is readable and uncorrupted
- [ ] Raspberry Pi connects to ESP32 access point
- [ ] Health report reaches ESP32 when that firmware is installed
- [ ] Three reconnect/reset cycles succeed

Result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Evidence/notes: _____________________________________________________________

## Final no-battery gate

- [ ] All failures above have an owner and corrective action
- [ ] No unexplained encoder jumps remain
- [ ] GP8630N voltage-accuracy failure is resolved
- [ ] Safe boot confirmed after three resets
- [ ] SAFE STOP and watchdog outputs verified electrically
- [ ] Pinout matches `AUTOKART_ESP32S3_MASTER_PINOUT.md`
- [ ] Test evidence saved with date and firmware commit

Overall result: [ ] PASS  [ ] FAIL  [ ] BLOCKED

Approved by: ____________________  Date: ____________________

## Not authorized by this checklist

Completion of this USB-only checklist does not authorize:

- Connecting the traction or motor-driver batteries
- Rotating the BLDC, NEMA34 or brake motor
- Maximum-speed testing
- Brake-under-load testing
- Ground driving
- Stopping-distance testing
- Declaring the LoRa message link an independent physical power cut

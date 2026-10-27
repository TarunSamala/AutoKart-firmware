# AutoKart Jetson LoRa E-stop test bundle

This directory packages the LoRa emergency-stop test separately from the
Jetson camera/brake dashboard. It does not contain a second implementation of
the safety logic: `build_bundle.sh` copies the currently tested ESP32 and Pico
firmware from `bench/` and adds a Jetson-side validation tool.

The intended topology is:

```text
LAY37 -- Pico 2 -- RA-02 ))) 433 MHz ((( RA-02 -- ESP32-S3 -- USB -- Jetson
                                                               |
                                                        dashboard :8781
```

The dashboard continues to start the BLDC at 0.40 V. The E-stop is an
independent override: every received `ESTOP,1,sequence` packet immediately
applies BRK, disables the drive, sets the DAC to 0 V and requires a fresh
manual START after `ESTOP,0,sequence` CLEAR. It also commands the open-loop
NEMA23/TB6600 mechanical brake on GPIO38/39/40 from assumed released 0 degrees
to 30 degrees.

Before every ESP32 boot, reset or flash, physically place the mechanical brake
at its released/home position. There is no brake encoder. CLEAR does not
retract the actuator; only a later manual ARM returns it to assumed 0 degrees.

## Build the transferable directory

From the AutoKart-firmware repository:

```bash
bash jetson_estop/build_bundle.sh
```

Outputs:

```text
dist/AutoKart-Jetson-EStop/
dist/AutoKart-Jetson-EStop.tar.gz
dist/AutoKart-Jetson-EStop.tar.gz.sha256
```

Copy the archive and checksum to the Jetson, verify them, and extract:

```bash
sha256sum -c AutoKart-Jetson-EStop.tar.gz.sha256
tar -xzf AutoKart-Jetson-EStop.tar.gz
cd AutoKart-Jetson-EStop
```

## Firmware files

- `esp32/main.py`: ESP32-S3 BLD750/RA-02 receiver and serial controller.
- `esp32/bldc_bench_config.py`: dedicated fail-closed E-stop profile. The
  encoder is optional, but three consecutive CLEAR packets are mandatory for
  ARM and 1.5 seconds without a LoRa heartbeat produces a latched stop.
- `esp32/ra02_lora_chat_common.py`: SX1278 driver.
- `pico/main.py`: Pico 2 LAY37 transmitter with continuous CLEAR/STOP heartbeat.
- `pico/ra02_lora_chat_common.py`: SX1278 driver.

Use `flash_esp32.sh PORT` or `flash_pico.sh PORT`. Stop the dashboard before
flashing the ESP32 because only one process can own its serial port.

Examples:

```bash
./flash_esp32.sh /dev/serial/by-id/usb-Espressif_Systems_Espressif_Device_...
./flash_pico.sh /dev/ttyACM1
```

The scripts deliberately require an explicit port so the Pico and ESP32 cannot
be accidentally exchanged.

The Jetson/dashboard command heartbeat is disabled in this profile, avoiding
the earlier host-control timeout. The LoRa heartbeat watchdog remains enabled
because suppressing radio-loss detection would make the E-stop unsafe. Radio
loss is reported as a locked/interlocked state; it does not crash the ESP32.

## Run the E-stop validation on Jetson

Start the normal Jetson Brake_Test dashboard first. Keep the rear wheels raised
and restrained and keep a physical power cut within reach. Then, in a second
Jetson SSH terminal:

```bash
cd AutoKart-Jetson-EStop
./validate_estop.sh
```

The validator is read-only: it never starts or arms the motor. It waits for a
received CLEAR, asks you to click START in the existing dashboard,
then validates that pressing E-stop produces a fully locked state. After the
switch is released, it also checks that CLEAR does not restart the motor.

To monitor a dashboard from another machine:

```bash
./validate_estop.sh --url http://192.168.55.1:8781
```

Passing this diagnostic proves the observed software/telemetry sequence on the
bench. It is not certification of the radio link, relay/transistor wiring,
mechanical stopping distance, or vehicle safety.

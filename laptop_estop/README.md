# AutoKart laptop LoRa E-stop test

This bundle tests the Pico 2/RA-02 to ESP32-S3 to BLD750 E-stop path directly
over laptop USB serial. ESP32 Wi-Fi and the camera dashboard are not required.

The guided test starts the raised/restrained BLDC at 0.40 V, observes a valid
`ESTOP,1,sequence` packet, verifies `BRK=APPLIED`, `EN=STOP` and DAC `0.00 V`,
applies the GPIO38/39/40 NEMA23 mechanical brake to 30 degrees, then confirms
that releasing the switch does not restart traction.

This is a wheels-off diagnostic, not vehicle safety certification. Keep an
independent physical power cut within reach.

Before every ESP32 boot, reset or firmware flash, physically place the brake
mechanism at its released/home position. The brake is open-loop and has no
encoder. A STOP moves once from assumed 0 degrees to 30 degrees. CLEAR leaves
it applied; only a later manual ARM returns it to assumed 0 degrees.

## Requirements

```bash
python3 -m pip install pyserial mpremote
```

On the development laptop, `/home/satvara/python/bin/python3` can be selected:

```bash
export AUTOKART_PYTHON=/home/satvara/python/bin/python3
```

## Identify ports

Connect the ESP32-S3 and Pico 2, then run:

```bash
ls -l /dev/serial/by-id/ 2>/dev/null
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print
```

Use the Espressif by-id path for the ESP32. Confirm the Pico port separately;
never guess from `ttyACM` numbering.

## Flash

Close all REPLs, dashboards and serial monitors first:

```bash
./flash_esp32.sh /dev/serial/by-id/usb-Espressif_...
./flash_pico.sh /dev/serial/by-id/usb-MicroPython_...
```

## Validate

Keep the rear wheels raised, restrained and guarded:

```bash
./validate_estop.sh --port /dev/serial/by-id/usb-Espressif_...
```

The confirmation phrase is printed for the selected test voltage. The tool
requires `MOTOR-SPINNING` after the traction command; commanded voltage alone
is not accepted as proof that the BLDC physically rotated. The tool applies a
final wheel-lock command whether the diagnostic passes or fails. A successful
run saves `estop-laptop-validation-latest.json` and reports `PASS_DIAGNOSTIC`.

If 0.40 V is below the installed BLD750/motor starting threshold, first check
the GP8630N OUT-to-GND, BLD750 SV-to-COM, EN-to-COM and BRK-to-COM voltages.
Then the same guarded test can use 1.00 V:

```bash
./validate_estop.sh --port /dev/serial/by-id/usb-Espressif_... \
  --test-voltage 1.0
```

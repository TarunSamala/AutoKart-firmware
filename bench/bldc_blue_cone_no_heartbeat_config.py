"""Dedicated jacked bench profile for the blue-cone camera test.

Copy this file to the ESP32 as ``bldc_bench_config.py`` alongside ``main.py``.
The ESP32 requires a continuing Jetson control heartbeat while traction is
armed. Automatic camera-side braking remains restricted to the blue-cone rule
in bldc_encoder_camera_ui.py.

WHEELS-OFF, RESTRAINED TESTING ONLY. Do not install this profile for a ground
or production run.
"""

# Fail closed if the Jetson dashboard stops sending serial STATUS/control
# frames while traction is armed.  The dashboard polls at 4 Hz, so 1500 ms
# allows normal scheduling jitter while still stopping after a dead process,
# unplugged USB cable, or crashed perception host.
CONTROL_LINK_TIMEOUT_MS = 1500

# Fixed wheels-off Brake_Test throttle command.
BLUE_CONE_TEST_VOLTAGE = 1.0

# Brake_Test can run without the AS5047P. Encoder telemetry remains visible
# when available, but absence/fault does not block the wheels-off START action.
REQUIRE_ENCODER_FOR_START = False

# TEMPORARY WHEELS-OFF BRAKE_TEST PROFILE: allow START when the Pico/RA-02
# link is unavailable.  A received ESTOP,1 packet still immediately applies
# and latches both brakes.  Because LoRa is optional, loss of its heartbeat is
# diagnostic only and does not stop traction.
REQUIRE_LORA_FOR_START = False
LORA_PACKET_TIMEOUT_MS = 1500
LORA_CLEAR_CONFIRMATIONS = 3

# Open-loop NEMA23/TB6600 mechanical brake.  The current vehicle has no brake
# encoder, so the actuator must be physically at RELEASED/HOME before every
# ESP32 boot/reset.  The physically checked apply direction is GPIO39 LOW.
MECHANICAL_BRAKE_ENABLED = True
MECHANICAL_BRAKE_APPLY_DEG = 30.0
MECHANICAL_BRAKE_PULSES_PER_REV = 3200
MECHANICAL_BRAKE_APPLY_DIR_LEVEL = 0
MECHANICAL_BRAKE_ENABLE_LEVEL = 0

# Closed-loop straight hold.  This count was captured at the mechanically
# aligned centre during the 27 October calibration.  Recalibrate and replace
# it after any magnet, encoder, coupler, steering-linkage or wheel-alignment
# change.  The hold is armed with the Brake_Test traction controller; it never
# moves steering merely because the ESP32 booted, and an E-stop/LOCK disables
# STEP output immediately.
STEERING_CENTER_HOLD_ENABLED = True
STEERING_CENTER_RAW = 1525
STEERING_CENTER_DEADBAND_COUNTS = 2
STEERING_CENTER_REENGAGE_COUNTS = 4
STEERING_CENTER_MAX_ERROR_COUNTS = 80
STEERING_CENTER_REENGAGE_MS = 120
STEERING_CENTER_MOVE_TIMEOUT_MS = 2500
STEERING_CENTER_NO_PROGRESS_MS = 600

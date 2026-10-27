"""Wheels-off BLDC profile with mandatory fail-closed LoRa E-stop.

The Jetson may control the ESP32 through USB serial without a host heartbeat,
but traction cannot arm until the Pico/RA-02 CLEAR heartbeat is confirmed.
Loss of LoRa packets while operating produces a latched hardware stop.
"""

# Prevent the old dashboard/serial heartbeat timeout from stopping an otherwise
# healthy USB-controlled bench run.
CONTROL_LINK_TIMEOUT_MS = 0

# Encoder telemetry remains useful but the AS5047P is not required to prove the
# independent LoRa-to-BLD750 emergency-stop path.
REQUIRE_ENCODER_FOR_START = False

# This dedicated E-stop profile is fail-closed.
REQUIRE_LORA_FOR_START = True
LORA_PACKET_TIMEOUT_MS = 1500
LORA_CLEAR_CONFIRMATIONS = 3

# Open-loop NEMA23/TB6600 mechanical brake. Before every ESP32 boot/reset, the
# mechanism must physically be at the released/home position because there is
# deliberately no brake encoder in the current vehicle architecture.
MECHANICAL_BRAKE_ENABLED = True
MECHANICAL_BRAKE_APPLY_DEG = 30.0
MECHANICAL_BRAKE_PULSES_PER_REV = 3200
# Physically verified on the installed brake mechanism: LOW pushes the pin in
# (clockwise/apply), HIGH retracts it (release).
MECHANICAL_BRAKE_APPLY_DIR_LEVEL = 0
MECHANICAL_BRAKE_ENABLE_LEVEL = 0

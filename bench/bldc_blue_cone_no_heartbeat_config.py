"""Dedicated jacked bench profile for the blue-cone camera test.

Copy this file to the ESP32 as ``bldc_bench_config.py`` alongside ``main.py``.
It disables only the HTTP link timeout. Automatic camera-side braking remains
restricted to the qualified blue-cone rule in bldc_encoder_camera_ui.py.

WHEELS-OFF, RESTRAINED TESTING ONLY. Do not install this profile for a ground
or production run.
"""

CONTROL_LINK_TIMEOUT_MS = 0

#!/usr/bin/env python3
"""Automatic Raspberry Pi and camera health reporter for AutoKart RC.

Run this on the Raspberry Pi while it is connected to the ESP32-S3
``AutoKart-RC`` Wi-Fi access point. The ESP32 dashboard marks the Pi stale if
reports stop for more than three seconds. Camera presence is discovered from
Linux video-device nodes; it is not manually selected.
"""

import glob
import time
import urllib.error
import urllib.request


ESP32_BASE_URL = "http://192.168.4.1"
REPORT_INTERVAL_SECONDS = 1.0
REQUEST_TIMEOUT_SECONDS = 0.8


def camera_connected():
    return bool(glob.glob("/dev/video*"))


def report_health():
    camera = 1 if camera_connected() else 0
    url = "{}/pi_health?camera={}".format(ESP32_BASE_URL, camera)
    with urllib.request.urlopen(url, timeout=REQUEST_TIMEOUT_SECONDS) as reply:
        reply.read()
    return camera


def main():
    print("AutoKart Pi health reporter")
    print("ESP32:", ESP32_BASE_URL)
    while True:
        try:
            camera = report_health()
            print("Pi: CONNECTED | Camera:", "CONNECTED" if camera else "MISSING")
        except (OSError, urllib.error.URLError) as error:
            print("ESP32 unavailable:", error)
        time.sleep(REPORT_INTERVAL_SECONDS)


if __name__ == "__main__":
    main()

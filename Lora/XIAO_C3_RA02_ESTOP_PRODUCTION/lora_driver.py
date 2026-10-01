# SX127x RA-02 MicroPython driver wrapper
# Pin mapping:
# SCK GPIO8
# MISO GPIO9
# MOSI GPIO10
# NSS GPIO5
# RESET GPIO4
# DIO0 GPIO3

from machine import Pin, SPI
import time

class LoRa:
    def __init__(self):
        self.spi = SPI(1,
            baudrate=8000000,
            polarity=0,
            phase=0,
            sck=Pin(8),
            mosi=Pin(10),
            miso=Pin(9))

        self.cs = Pin(5, Pin.OUT, value=1)
        self.reset_pin = Pin(4, Pin.OUT)

    def reset(self):
        self.reset_pin.value(0)
        time.sleep_ms(10)
        self.reset_pin.value(1)
        time.sleep_ms(10)

    def begin(self):
        self.reset()
        # Radio register initialization goes here
        # Keep this hardware layer isolated.

    def send(self, payload):
        # Replace with SX127x transmit implementation.
        # Payload must be bytes.
        if isinstance(payload, str):
            payload = payload.encode()
        return True

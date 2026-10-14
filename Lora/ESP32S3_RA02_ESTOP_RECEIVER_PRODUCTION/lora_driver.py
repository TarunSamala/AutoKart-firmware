# SX1278 RA-02 driver hardware layer

from machine import Pin, SPI
import time

class LoRa:

    def __init__(self):

        self.spi = SPI(
            1,
            baudrate=8000000,
            polarity=0,
            phase=0,
            sck=Pin(12),
            mosi=Pin(11),
            miso=Pin(10)
        )

        self.cs = Pin(9, Pin.OUT, value=1)
        self.reset_pin = Pin(8, Pin.OUT)


    def begin(self):

        self.reset_pin.value(0)
        time.sleep_ms(10)

        self.reset_pin.value(1)
        time.sleep_ms(10)


    def receive(self):

        # SX1278 receive implementation
        # returns decoded string packet
        return None

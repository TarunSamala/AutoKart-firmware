from machine import Pin, SPI
from ra02_lora_chat_common import SX1278, chat_loop


# ESP32-S3 RA-02 wiring from the connection sheet.
spi = SPI(
    2,
    baudrate=2000000,
    polarity=0,
    phase=0,
    sck=Pin(14),
    mosi=Pin(13),
    miso=Pin(15),
)
cs = Pin(16, Pin.OUT, value=1)
reset = Pin(10, Pin.OUT, value=1)
dio0 = Pin(18, Pin.IN)

radio = SX1278(spi, cs, reset, 433000000)
chat_loop(radio, "ESP32-S3")

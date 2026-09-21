from machine import Pin, SPI
from ra02_lora_chat_common import SX1278, chat_loop


# ESP8266 board labels: D5=SCK, D6=MISO, D7=MOSI.
spi = SPI(1, baudrate=2000000, polarity=0, phase=0)
cs = Pin(2, Pin.OUT, value=1)       # D4 / GPIO2
reset = Pin(5, Pin.OUT, value=1)    # D1 / GPIO5
dio0 = Pin(4, Pin.IN)               # D2 / GPIO4 (not required for polling)

radio = SX1278(spi, cs, reset, 433000000)
chat_loop(radio, "ESP8266MOD")

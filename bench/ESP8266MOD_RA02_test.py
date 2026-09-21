from machine import Pin, SPI
import time


# ============================================================
# ESP8266 + RA-02 SX1278
# SIMPLE LORA STRING TRANSMITTER
# ============================================================


# ------------------------------------------------------------
# CONNECTIONS
# ------------------------------------------------------------

# ESP8266 HSPI fixed pins:
#
# D5 / GPIO14 -> SCK
# D7 / GPIO13 -> MOSI
# D6 / GPIO12 -> MISO
#
# D4 / GPIO2  -> NSS / CS
# D1 / GPIO5  -> RESET
# D2 / GPIO4  -> DIO0


CS_PIN = 2
RESET_PIN = 5
DIO0_PIN = 4


# ------------------------------------------------------------
# LORA SETTINGS
# Must exactly match ESP32-S3 receiver
# ------------------------------------------------------------

FREQUENCY = 433_000_000


# ============================================================
# SX1278 DRIVER
# ============================================================

class SX1278:

    REG_FIFO = 0x00
    REG_OP_MODE = 0x01

    REG_FRF_MSB = 0x06
    REG_FRF_MID = 0x07
    REG_FRF_LSB = 0x08

    REG_PA_CONFIG = 0x09
    REG_OCP = 0x0B
    REG_LNA = 0x0C

    REG_FIFO_ADDR_PTR = 0x0D
    REG_FIFO_TX_BASE_ADDR = 0x0E
    REG_FIFO_RX_BASE_ADDR = 0x0F

    REG_IRQ_FLAGS = 0x12

    REG_MODEM_CONFIG_1 = 0x1D
    REG_MODEM_CONFIG_2 = 0x1E

    REG_PREAMBLE_MSB = 0x20
    REG_PREAMBLE_LSB = 0x21

    REG_PAYLOAD_LENGTH = 0x22

    REG_MODEM_CONFIG_3 = 0x26

    REG_SYNC_WORD = 0x39

    REG_DIO_MAPPING_1 = 0x40

    REG_VERSION = 0x42


    MODE_LONG_RANGE = 0x80

    MODE_SLEEP = 0x00
    MODE_STANDBY = 0x01
    MODE_TX = 0x03


    IRQ_TX_DONE = 0x08


    def __init__(
        self,
        spi,
        cs,
        reset,
        dio0,
        frequency
    ):

        self.spi = spi
        self.cs = cs
        self.reset = reset
        self.dio0 = dio0
        self.frequency = frequency


    # --------------------------------------------------------

    def write_reg(self, address, value):

        self.cs.value(0)

        self.spi.write(
            bytes((
                address | 0x80,
                value & 0xFF
            ))
        )

        self.cs.value(1)


    # --------------------------------------------------------

    def read_reg(self, address):

        self.cs.value(0)

        self.spi.write(
            bytes((
                address & 0x7F,
            ))
        )

        data = self.spi.read(
            1,
            0x00
        )

        self.cs.value(1)

        return data[0]


    # --------------------------------------------------------

    def write_fifo(self, data):

        self.cs.value(0)

        self.spi.write(
            bytes((
                self.REG_FIFO | 0x80,
            ))
        )

        self.spi.write(data)

        self.cs.value(1)


    # --------------------------------------------------------

    def reset_radio(self):

        self.reset.value(0)
        time.sleep_ms(10)

        self.reset.value(1)
        time.sleep_ms(20)


    # --------------------------------------------------------

    def set_frequency(self, frequency):

        frf = int(
            (frequency * 524288)
            / 32_000_000
        )

        self.write_reg(
            self.REG_FRF_MSB,
            (frf >> 16) & 0xFF
        )

        self.write_reg(
            self.REG_FRF_MID,
            (frf >> 8) & 0xFF
        )

        self.write_reg(
            self.REG_FRF_LSB,
            frf & 0xFF
        )


    # --------------------------------------------------------

    def standby(self):

        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE |
            self.MODE_STANDBY
        )


    # --------------------------------------------------------

    def begin(self):

        self.cs.value(1)

        self.reset_radio()

        version = self.read_reg(
            self.REG_VERSION
        )

        print(
            "SX1278 VERSION:",
            hex(version)
        )


        if version != 0x12:

            raise OSError(
                "RA-02 NOT FOUND. "
                "Expected 0x12, got 0x%02X"
                % version
            )


        # LoRa mode
        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE |
            self.MODE_SLEEP
        )

        time.sleep_ms(10)


        # Frequency
        self.set_frequency(
            self.frequency
        )


        # FIFO
        self.write_reg(
            self.REG_FIFO_TX_BASE_ADDR,
            0x00
        )

        self.write_reg(
            self.REG_FIFO_RX_BASE_ADDR,
            0x00
        )


        # LNA boost
        self.write_reg(
            self.REG_LNA,
            self.read_reg(
                self.REG_LNA
            ) | 0x03
        )


        # ----------------------------------------------------
        # LoRa configuration
        #
        # BW = 125 kHz
        # CR = 4/5
        # Explicit header
        # ----------------------------------------------------

        self.write_reg(
            self.REG_MODEM_CONFIG_1,
            0x72
        )


        # SF7 + CRC ON
        self.write_reg(
            self.REG_MODEM_CONFIG_2,
            0x74
        )


        # AGC ON
        self.write_reg(
            self.REG_MODEM_CONFIG_3,
            0x04
        )


        # Preamble 8
        self.write_reg(
            self.REG_PREAMBLE_MSB,
            0x00
        )

        self.write_reg(
            self.REG_PREAMBLE_LSB,
            0x08
        )


        # Sync word
        self.write_reg(
            self.REG_SYNC_WORD,
            0x12
        )


        # PA_BOOST
        # approximately +17 dBm
        self.write_reg(
            self.REG_PA_CONFIG,
            0x8F
        )


        self.write_reg(
            self.REG_OCP,
            0x2B
        )


        self.write_reg(
            self.REG_IRQ_FLAGS,
            0xFF
        )


        self.standby()

        print(
            "RA-02 INITIALIZED"
        )


    # --------------------------------------------------------

    def send(self, message):

        # Convert string -> bytes
        if isinstance(
            message,
            str
        ):

            payload = message.encode(
                "utf-8"
            )

        else:

            payload = message


        if len(payload) > 255:

            raise ValueError(
                "Message too long"
            )


        self.standby()


        # DIO0 = TxDone
        self.write_reg(
            self.REG_DIO_MAPPING_1,
            0x40
        )


        # Clear IRQ
        self.write_reg(
            self.REG_IRQ_FLAGS,
            0xFF
        )


        # Start FIFO at 0
        self.write_reg(
            self.REG_FIFO_ADDR_PTR,
            0x00
        )


        # Write data
        self.write_fifo(
            payload
        )


        # Tell SX1278 payload length
        self.write_reg(
            self.REG_PAYLOAD_LENGTH,
            len(payload)
        )


        # Start transmit
        self.write_reg(
            self.REG_OP_MODE,
            self.MODE_LONG_RANGE |
            self.MODE_TX
        )


        start = time.ticks_ms()


        # Wait for DIO0 TxDone
        while self.dio0.value() == 0:

            if time.ticks_diff(
                time.ticks_ms(),
                start
            ) > 2000:

                self.standby()

                raise OSError(
                    "LORA TX TIMEOUT"
                )

            time.sleep_ms(1)


        # Clear TxDone
        self.write_reg(
            self.REG_IRQ_FLAGS,
            self.IRQ_TX_DONE
        )


        self.standby()


# ============================================================
# ESP8266 HARDWARE SPI
# ============================================================

spi = SPI(
    1,
    baudrate=2_000_000,
    polarity=0,
    phase=0
)


# ============================================================
# PINS
# ============================================================

cs = Pin(
    CS_PIN,
    Pin.OUT,
    value=1
)


reset = Pin(
    RESET_PIN,
    Pin.OUT,
    value=1
)


dio0 = Pin(
    DIO0_PIN,
    Pin.IN
)


# ============================================================
# CREATE RADIO
# ============================================================

radio = SX1278(
    spi,
    cs,
    reset,
    dio0,
    FREQUENCY
)


# ============================================================
# START
# ============================================================

print()
print("==============================")
print(" ESP8266 LORA TRANSMITTER")
print(" RA-02 SX1278 433 MHz")
print("==============================")
print()


radio.begin()


# ============================================================
# SEND STRING CONTINUOUSLY
# ============================================================

counter = 0


while True:

    counter += 1


    message = (
        "HELLO FROM ESP8266 "
        + str(counter)
    )


    try:

        radio.send(
            message
        )

        print(
            "SENT:",
            message
        )


    except Exception as e:

        print(
            "TX ERROR:",
            e
        )


    time.sleep(1)
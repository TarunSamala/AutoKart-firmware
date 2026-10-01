from machine import WDT

class Watchdog:

    def __init__(self, timeout):
        self.wdt = WDT(timeout=timeout)

    def feed(self):
        self.wdt.feed()

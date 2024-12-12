
import pyb

class Bumpsensor:

    def __init__(self, pin):
        self.sensor = pyb.ADC(pin)
        pass

    def detect(self):
        self.value = self.sensor.read()
        return self.value
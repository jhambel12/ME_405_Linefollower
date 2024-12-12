

import pyb

class IRSensor:

    def __init__(self, IR_pins):
        self.sensors = [pyb.ADC(pin) for pin in IR_pins]
        pass
    
    def update(self):
        self.values = [0 if sensor.read() > 3700 else 1 for sensor in self.sensors]
        self.weight = [-4,-2,0,2,4]
        self.numerator = sum(a * b for a, b in zip(self.weight, self.values))
        self.denominator = sum(self.values)
        if self.denominator != 0:
            self.P = self.numerator / self.denominator
        else:
            self.P = 0
        pass

    def get_modifier(self):
        return self.P
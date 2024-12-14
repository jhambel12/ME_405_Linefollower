'''
@file          bumpsensor.py
@brief         Module containing the bump sensor class
@details       This module contains the class for the setup and reading from the bump sensor.
@author        Jack Ellis, Jacob Hambel
@date          December 12, 2024
'''

import pyb

class Bumpsensor:
    '''
    @brief       Interface with quadrature encoders
    @details     The functions in this class take an input to intialize the pin to be used by the bump sensor, 
                 and performs an adc read of the bump sensor output value.
                 to find motor speed.
    '''

    def __init__(self, pin):
        '''
        @brief    Constructs a bump sensor object
        @details  This function initilizes the bump sensor to a pin.
        @param    pin: The pin input pin for the bump sensor
        '''
        self.sensor = pyb.ADC(pin)
        pass

    def detect(self):
        '''
        @brief    Reads the current adc value of the bump sensor
        @details  This function reads the current output of the bump sensor to be used by the main script.
        '''
        self.value = self.sensor.read()
        return self.value
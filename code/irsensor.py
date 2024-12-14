'''
@file          irsensor.py
@brief         Module containing the IR sensor class
@details       This module contains the class for the setup, reading, 
               and processing of data from the infrared line sensors.
@author        Jack Ellis, Jacob Hambel
@date          December 12, 2024
'''

import pyb

class IRSensor:
    '''
    @brief       Interface with quadrature encoders
    @details     The functions in this class take inputs to intialize the pins used for
                 reading IR sensor data, update the line position, and prepares values
    '''

    def __init__(self, IR_pins):
        '''
        @brief    Initializes the pins used to read the IR sensors
        @details  This function initilizes all pins that are being used by an IR sensor array.
        @param    IR_pins: list of pins to initialize as the IR sensors
        '''
        self.sensors = [pyb.ADC(pin) for pin in IR_pins]
        pass
    
    def update(self):
        '''
        @brief    Updates the speed modifier based on line location
        @details  This function initilizes all necessary parameters for the encoder to function.
                  The necessary parameters are explained below.
        '''
        self.values = [0 if sensor.read() > 3600 else 1 for sensor in self.sensors]
        self.weight = [-6,-3,0,3,6]
        self.numerator = sum(a * b for a, b in zip(self.weight, self.values))
        self.denominator = sum(self.values)
        if self.denominator != 0:
            self.P = self.numerator / self.denominator
        else:
            self.P = 0
        pass

    def get_modifier(self):
        '''
        @brief    Constructs an encoder object
        @details  This function returns the current modifier value to adjust speed to follow a line.
        '''
        return self.P
    
    def get_denominator(self):
        '''
        @brief    Return sum of values
        @details  This function returns the denominator of the update calculation to read
                  the current number of sensors reading a line.
        '''
        return self.denominator
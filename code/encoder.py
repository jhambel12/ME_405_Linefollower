'''
@file          encoder.py
@brief         A driver for reading from Quadrature Encoders
@details       This module contains the class that is used for setup and operations of the encoders.
@author        Jack Ellis, Jacob Hambel
@date          November 21, 2024
'''

import pyb
import utime

class Encoder:
    '''
    @brief       Interface with quadrature encoders
    @details     The functions in this class take inputs to intialize the pins to be used by the encoders, 
                 perform measurements of the rotation of the encoders, and perform calculations needed
                 to find motor speed.
    '''

    def __init__(self, N, AR, PS, ENC_Pin_A, ENC_Pin_B):
        '''
        @brief    Constructs an encoder object
        @details  This function initilizes all necessary parameters for the encoder to function.
                  The necessary parameters are explained below.
        @param    N: The timer number associated with the encoder
        @param    AR: The auto-reload value for the encoder ticks
        @param    PS: Prescaler value for the encoder
        @param    ENC_Pin_A: Channel A pin connection
        @param    ENC_Pin_B: Channel B pin connection
        '''
        self.tim = pyb.Timer(N, period = AR, prescaler = PS)
        self.tim.channel(1, pin = ENC_Pin_A, mode=pyb.Timer.ENC_AB)
        self.tim.channel(2, pin = ENC_Pin_B, mode=pyb.Timer.ENC_AB)
        pass

    def update(self):
        '''
        @brief    Updates encoder position and delta
        @details  This function measures the current position of the motor, determines the distance 
                  it has traveled since the last measaurement, accounts for overflow of the encoder ticks,
                  and loads the old count storage with the most recent measurement.
        '''
        self.new_count = self.tim.counter()
        self.delta = utime.ticks_diff(self.new_count, self.old_count)
        if self.delta > 65536/2:
            self.delta -= 65536
        elif self.delta < -65536/2:
            self.delta += 65536
        self.old_count = self.new_count
        pass

    def get_position(self):
        '''
        @brief    Gets the most recent encoder position
        @details  The function returns the value of self.new_count to the object calling the function.
                  It does not perform any calculations, it is only a return.
        @return   The most recently measured position of the encoder.
        '''
        return self.new_count

    def get_delta(self):
        '''
        @brief    Gets the most recent encoder delta
        @details  The function returns the value of self.delta to the object calling the function.
                  It does not perform any calculations, it is only a return.
        @return   The most recently measured delta of the encoder.
        '''
        return self.delta

    def zero(self):
        '''
        @brief    Resets the encoder position to zero
        @details  This function initializes the encoder position to reference future movement off of.
        '''
        self.old_count = self.tim.counter()
        pass

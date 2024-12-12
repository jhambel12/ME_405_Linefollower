'''
@file          motor.py
@brief         Module containing the romi motor class
@details       This module contains the class for the romi motors that will be accessed
               by the main file to create motor objects for use in the data collection.
@author        Jack Ellis, Jacob Hambel
@date          November 21, 2024
'''

from pyb import Pin, Timer

class Motor:
    '''
    @brief       Interface with quadrature encoders
    @details     The functions in this class take inputs to intialize the pins to be used by the motors, 
                 control their speed using PWM, and enable the motors to function in PWM. 
    '''    

    def __init__(self,PWM_tim,EN_pin,DIR_pin,EFF_pin):
        '''
        @brief    Constructs an encoder object
        @details  This function initilizes all necessary parameters for the motor to function.
                  The necessary parameters are explained below.
        @param    PWM_tim: The timer number associated with the motor for PWM
        @param    EN_pin: Enables motor to function
        @param    DIR_pin: Sets motor direction
        @param    EFF_pin: Sets PWM for the motor
        '''
        self.ON = Pin(EN_pin, mode=Pin.OUT_PP)
        self.DIR = Pin(DIR_pin, mode=Pin.OUT_PP)
        self.EFF = PWM_tim.channel(1, mode=Timer.PWM, pin = EFF_pin)
        pass

    def set_duty(self,duty):
        '''
        @brief    Sets PWM duty cycle
        @details  This function takes in a value to be used as the percentage duty cycle,
                  and the duty cycle is set using the pyb function pulse_width_percent()
        @param    duty: The duty cycle for the PWM
        '''
        if duty > 100:
            self.EFF.pulse_width_percent(100)
        elif duty < 0:
            self.EFF.pulse_width_percent(0)
        else:
            self.EFF.pulse_width_percent(duty)
        pass
   
    def set_dir(self,forward):
        '''
        @brief    Enables the motor
        @details  This function is called once to enable the motor's function.
        @param    forward: Boolean to set motor direction
        '''
        if forward == True:
            self.DIR.low()
        elif forward == False:
            self.DIR.high()
        pass

    def enable(self):
        '''
        @brief    Enables the motor
        @details  This function is called once to enable the motor's function.
        '''
        self.ON.high()
        pass

    def disable(self):
        '''
        @brief    DIsables the motor
        @details  This function is called once to disable the motor's function.
        '''
        self.ON.low()
        pass
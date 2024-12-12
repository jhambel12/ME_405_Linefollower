'''
@file          main.py
@brief         
@details       
@author        Jack Ellis, Jacob Hambel
@date          December 13, 2024
'''

#==================================================================================#
# Import necessary modules and classes                                             #
#==================================================================================#

import cotask
import task_share
import gc
import utime

from pyb import Pin, Timer, I2C
from array import array
from math import pi

from encoder import Encoder
from motor import Motor
from BNO055 import BNO055
from irsensor import IRSensor
from bumpsensor import Bumpsensor

#==================================================================================#
# Tasks                                                                            #
#==================================================================================#

# Task 1
def mot_enc_L(shares):
    '''
    @brief    Task 1:
    @details  
    '''

    share_bump , share_omega_L = shares

    state = 0
    while True:
        
        if state == 0:
            mot_L.enable()          # Enable the left motor
            enc_L.zero()            # Zero the encoder position
            old = utime.ticks_us()  # Store the current time
            mot_L.set_dir(True)     # Set motor direction
            K_p = 60
            K_i = 30
            K_d = 1
            int_err = 0             # Integral error for integral control
            state = 1      

        elif state == 1:
            if share_bump.get() == 1:
                int_err = 0
                omega_ref = share_omega_L.get()
                print(omega_ref)
            else:
                adjust = line_sensor.get_modifier()
                omega_ref = (V_romi + ((adjust*V_romi)/4)) / r_wheel
            state = 2

        elif state == 2:
            now = utime.ticks_us()
            t_step = utime.ticks_diff(now, old)
            old = now
            enc_L.update()
            omega_meas = enc_L.get_delta() * ((12500 * pi) / 9) // t_step
            int_err += (omega_ref - omega_meas) // t_step
            int_err = max(-50, min(int_err, 50))
            state = 3

        elif state == 3:
            duty = (K_p * (omega_ref - abs(omega_meas)) + K_i * int_err)/28
            mot_L.set_duty(duty)
            state = 1

        yield None

#----------------------------------------------------------------------------------#

# Task 2
def mot_enc_R(shares):
    '''
    @brief    Task 2:
    @details  
    '''

    share_bump, share_omega_R = shares

    state = 0
    while True:
        
        if state == 0:
            mot_R.enable()          # Enable the right motor
            enc_R.zero()            # Zero the encoder position
            old = utime.ticks_us()  # Store the current time
            mot_L.set_dir(True)     # Set motor direction
            K_p = 60
            K_i = 30
            K_d = 1
            int_err = 0             # Integral error for integral control
            state = 1      

        elif state == 1:
            if share_bump.get() == 1:
                int_err = 0
                omega_ref = share_omega_R.get()
            else:
                adjust = line_sensor.get_modifier()
                omega_ref = (V_romi - ((adjust*V_romi)/4)) / r_wheel
            state = 2

        elif state == 2:
            now = utime.ticks_us()
            t_step = utime.ticks_diff(now, old)
            old = now
            enc_R.update()
            omega_meas = enc_R.get_delta() * ((12500 * pi) / 9) // t_step
            int_err += (omega_ref - omega_meas) // t_step
            int_err = max(-50, min(int_err, 50))
            state = 3

        elif state == 3:
            duty = (K_p * (omega_ref - abs(omega_meas)) + K_i * int_err)/22
            mot_R.set_duty(duty)
            state = 1

        yield None

#----------------------------------------------------------------------------------#

# Task 3
def imu_control():
    '''
    @brief    Task 3:
    @details  
    '''

    state = 0
    while True:
        
        yield None

#----------------------------------------------------------------------------------#

# Task 4
def line_sense():
    '''
    @brief    Task 4:
    @details  
    '''

    state = 0
    while True:
        
        if state == 0:
            line_sensor.update()
            state = 1
        
        elif state  == 1:
            line_sensor.update()
        
        yield None

#----------------------------------------------------------------------------------#

# Task 5
def bump_sense(shares):
    '''
    @brief    Task 5:
    @details  
    '''

    share_bump, share_omega_L, share_omega_R = shares

    state = 0
    time_run = 0
    while True:
        
        if state == 0:
            share_bump.put(0)
            state =1

        elif state == 1:
            if bump_sensor.detect() < 2000:
                share_bump.put(1)
                share_omega_L.put(0)
                share_omega_R.put(0)
                mot_L.set_dir(False)
                mot_R.set_dir(False)
                old = utime.ticks_ms()
                state = 2
        
        elif state == 2:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            share_omega_L.put(8)
            share_omega_R.put(8)
            if time_run > 800:
                time_run = 0
                state = 3
        
        elif state == 3:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            mot_L.set_dir(True)
            share_omega_L.put(8)
            share_omega_R.put(8)
            if time_run > 1200:
                time_run = 0
                state = 4

        elif state == 4:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            mot_R.set_dir(True)
            share_omega_L.put(10)
            share_omega_R.put(10)
            if time_run > 1900:
                time_run = 0
                state = 5

        elif state == 5:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            mot_L.set_dir(False)
            share_omega_L.put(8)
            share_omega_R.put(8)
            if time_run > 1200:
                time_run = 0
                state = 6

        elif state == 6:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            mot_L.set_dir(True)
            share_omega_L.put(12)
            share_omega_R.put(12)
            if time_run > 1500:
                time_run = 0
                state = 7

        elif state == 7:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            mot_L.set_dir(True)
            share_omega_L.put(18)
            share_omega_R.put(24)
            if time_run > 1000:
                time_run = 0
                share_bump.put(0)
                state = 1

        yield None

#==================================================================================#
# Hardware Initialization and System Parameters                                    #
#==================================================================================#

# Motor objects
tim_L = Timer(1, freq=20_000)       # Timer for left motor
tim_R = Timer(4, freq=20_000)       # Timer for right motor
mot_L = Motor(tim_L, Pin.cpu.B5, Pin.cpu.B3, Pin.cpu.A8)    # Left motor object
mot_R = Motor(tim_R, Pin.cpu.A10, Pin.cpu.B10, Pin.cpu.B6)  # Right motor object

#----------------------------------------------------------------------------------#

# Encoder objects
enc_L = Encoder(2, 65535, 0, Pin.cpu.A0, Pin.cpu.A1)  # Left encoder object
enc_R = Encoder(3, 65535, 0, Pin.cpu.A6, Pin.cpu.A7)  # Right encoder object

#----------------------------------------------------------------------------------#

# Line sensor object
line_sensor = IRSensor([Pin(Pin.board.PC1), Pin(Pin.board.PB1), Pin(Pin.board.PC3), 
               Pin(Pin.board.PC0), Pin(Pin.board.PB0)])     # Line sensor object

#----------------------------------------------------------------------------------#

# Bump sensor object
bump_sensor = Bumpsensor(Pin(Pin.board.PC4))

#----------------------------------------------------------------------------------#

# IMU setup
res_pin = Pin(Pin.cpu.A4, mode=Pin.OUT_PP)      # Reset pin for IMU
res_pin.low()
utime.sleep_ms(10)
res_pin.high()
utime.sleep_ms(650)
i2c = I2C(1, I2C.CONTROLLER, baudrate=400000)   # I2C bus for IMU
imu = BNO055(i2c)                               # IMU object
utime.sleep_ms(100)
imu.op_mode("NDOF")
utime.sleep_ms(10)

#----------------------------------------------------------------------------------#

# Romi geometry
w_romi = 0.0705         # Romi radius
r_wheel = 0.035         # Radius of wheel
V_romi = .35            # Romi velocity

#==================================================================================#
# Scheduler Setup                                                                  #
#==================================================================================#

bump_triggered = task_share.Share('B', thread_protect=False, name="BumpTrigger")
bump_omega_L = task_share.Share('f', thread_protect=False, name="BumpLeftSpeed")
bump_omega_R = task_share.Share('f', thread_protect=False, name="BumpRightSpeed")

# Task setup
task1 = cotask.Task(mot_enc_L, name="Task_1", priority=1, period=10,
                    profile=True, trace=False, shares=(bump_triggered,bump_omega_L))
task2 = cotask.Task(mot_enc_R, name="Task_2", priority=1, period=10,
                    profile=True, trace=False, shares=(bump_triggered,bump_omega_R))
task3 = cotask.Task(imu_control, name="Task_3", priority=1, period=10,
                    profile=True, trace=False, shares=())
task4 = cotask.Task(line_sense, name="Task_4", priority=1, period=10,
                    profile=True, trace=False, shares=())
task5 = cotask.Task(bump_sense, name="Task_5", priority=1, period=10,
                    profile=True, trace=False, shares=(bump_triggered,
                                                       bump_omega_L,bump_omega_R))
cotask.task_list.append(task1)
cotask.task_list.append(task2)
cotask.task_list.append(task3)
cotask.task_list.append(task4)
cotask.task_list.append(task5)

#----------------------------------------------------------------------------------#

# Garbage collection and scheduler setup
gc.collect()
while True:
    try:
        cotask.task_list.pri_sched()  # Run the task scheduler
    except KeyboardInterrupt:
        break

# Print task information after termination
print('\n' + str(cotask.task_list))
print(task1.get_trace())
print('')
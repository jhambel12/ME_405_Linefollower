'''
@file          main.py
@brief         Main script for autonomous course navigation of a Romi robot
@details       This script implements the control necessary for a Romi differential
               drive robot to navigate a course using line sensors and a bump
               sensor to determine the path for the Romi to take. Each motor 
               operates with a PID control loop using feedback from encoders and an
               IMU as needed.
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
    @brief    Task 1: Control task for the left motor
    @details  This task intializes the left motor and encoder, performs the PID 
              control calculations, and sets the duty cycle.
    '''

    share_bump , share_omega_L , share_return , share_return_omega_L ,\
    share_heading_adjust = shares

    state = 0
    while True:
        
        # Initialize necessary variables and prepare the motor and encoder
        if state == 0:
            mot_L.enable()          # Enable the left motor
            enc_L.zero()            # Zero the encoder position
            old = utime.ticks_us()  # Store the current time
            mot_L.set_dir(True)     # Set motor direction
            K_p = 130               # Proportional gain constant
            K_i = 2                 # Integral gain constant
            K_d = 60               # Derivative gain constant
            err_old = 0             # Old error value for derivative control
            int_err = 0             # Integral error for integral control
            state = 1      

        # Detect if either the bump protocol or return protocol are active, and use
        # their prescribed velocities if they are. Otherwise, default to the line
        # sensor for control
        elif state == 1:
            if share_bump.get() == 1:
                int_err = 0
                omega_ref = share_omega_L.get()
            elif share_return.get() == 1:
                int_err = 0
                omega_ref = share_return_omega_L.get() + share_heading_adjust.get()
            else:
                adjust = line_sensor.get_modifier()
                omega_ref = (V_romi + ((adjust*V_romi)/4)) / r_wheel
            state = 2

        # Perform the encoder reading of velocity and calculation of errors for the
        # PID loop
        elif state == 2:
            now = utime.ticks_us()
            t_step = utime.ticks_diff(now, old)
            old = now
            enc_L.update()
            omega_meas = enc_L.get_delta() * ((12500 * pi) / 9) // t_step
            err = (omega_ref - omega_meas)
            int_err += err * t_step
            int_err = max(-20, min(int_err, 20))
            dif_err = (err - err_old) // t_step
            err_old = err
            state = 3

        # Calculate and set the duty cycle using the PID function
        elif state == 3:
            duty = (K_p * (omega_ref - abs(omega_meas)) + K_i * int_err
                    + K_d * dif_err)/28
            mot_L.set_duty(duty)
            state = 1

        yield None

#----------------------------------------------------------------------------------#

# Task 2
def mot_enc_R(shares):
    '''
    @brief    Task 2: Control task for the right motor
    @details  This task initializes the right motor and encoder, performs the PID
              control calculations, and sets the duty cycle.
    '''

    share_bump , share_omega_R , share_return , share_return_omega_R ,\
    share_heading_adjust = shares

    state = 0
    while True:
        
        # Initialize necessary variables and prepare the motor and encoder
        if state == 0:
            mot_R.enable()          # Enable the right motor
            enc_R.zero()            # Zero the encoder position
            old = utime.ticks_us()  # Store the current time
            mot_L.set_dir(True)     # Set motor direction
            K_p = 130               # Proportional gain constant
            K_i = 2                 # Integral gain constant
            K_d = 60               # Derivative gain constant
            err_old = 0             # Old error value for derivative control
            int_err = 0             # Integral error for integral control
            state = 1      

        # Detect if either the bump protocol or return protocol are active, and use
        # their prescribed velocities if they are. Otherwise, default to the line
        # sensor for control
        elif state == 1:
            if share_bump.get() == 1:
                int_err = 0
                omega_ref = share_omega_R.get()
            elif share_return.get() == 1:
                int_err = 0
                omega_ref = share_return_omega_R.get() - share_heading_adjust.get()
            else:
                adjust = line_sensor.get_modifier()
                omega_ref = (V_romi - ((adjust*V_romi)/4)) / r_wheel
            state = 2

        # Perform the encoder reading of velocity and calculation of errors for the
        # PID loop
        elif state == 2:
            now = utime.ticks_us()
            t_step = utime.ticks_diff(now, old)
            old = now
            enc_R.update()
            omega_meas = enc_R.get_delta() * ((12500 * pi) / 9) // t_step
            err = (omega_ref - omega_meas)
            int_err += err * t_step
            int_err = max(-20, min(int_err, 20))
            dif_err = (err - err_old) // t_step
            err_old = err
            state = 3

        # Calculate and set the duty cycle using the PID function
        elif state == 3:
            duty = (K_p * (omega_ref - abs(omega_meas)) + K_i * int_err
                    + K_d * dif_err)/23.34
            mot_R.set_duty(duty)
            state = 1

        yield None

#----------------------------------------------------------------------------------#

# Task 3
def imu_control(shares):
    '''
    @brief    Task 3: Control task for the IMU
    @details  This task saves the initial heading of the Romi to be used for the 
              return protocol. Once the return protocol is triggered, the IMU
              heading feedback controls the motors.
    '''

    share_return , share_return_omega_L , share_return_omega_R ,\
    share_heading_adjust = shares

    state = 0
    time_run = 0
    while True:

        # Take initial heading reading
        if state == 0:
            start_heading = imu.euler("head")
            state = 1

        # Detect of the return protocol needs to be executed
        elif state == 1:
            if share_return.get() == 1:
                share_return_omega_L.put(0)
                share_return_omega_R.put(0)
                old = utime.ticks_ms()
                state = 2
        
        # Rotate the Romi to generally face the starting box
        elif state == 2:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            mot_L.set_dir(False)
            return_omega_L.put(8)
            return_omega_R.put(8)
            if time_run > 1350:
                time_run = 0
                state = 3

        # Stop the Romi to ensure a smooth start to the return
        elif state == 3:
            mot_L.set_dir(True)
            return_omega_L.put(0)
            return_omega_R.put(0)
            state = 4

        # Set both wheels to a high speed to return quickly
        elif state == 4:
            return_omega_L.put(23)
            return_omega_R.put(23)
            old = utime.ticks_ms()
            state = 5

        # Use heading feeback to align the Romi and keep it on course to the start
        # box. After reaching the start box, disable the motors
        elif state == 5:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            current_heading = imu.euler("head")
            heading_error = current_heading - start_heading
            if heading_error > pi:
                share_heading_adjust.put(1)
            elif heading_error < pi:
                share_heading_adjust.put(-1)
            else:
                share_heading_adjust.put(0)
            if time_run > 2140:
                mot_L.disable()
                mot_R.disable()
                state = 1

        yield None

#----------------------------------------------------------------------------------#

# Task 4
def line_sense(shares):
    '''
    @brief    Task 4: Control task for the line sensors
    @details  This task updates the line sensor readings to prepare a new adjustment
              value for the motors to maintain orientation along the line. After
              cleairng the obstacle, the line sensor class prepares to detect the 
              trigger for the return protocol once entering the finish box.
    '''

    share_bump , share_return = shares

    state = 0
    time_run = 0
    while True:
        
        if state == 0:
            share_return.put(0)
            line_sensor.update()
            state = 1
        
        # Update the line sensor reading and detect if the obstacle has occured
        elif state  == 1:
            if share_bump.get() == 1:
                state = 2
            line_sensor.update()

        # Update the line sensor reading, and once the obstacle protocol has been
        # executed and the finish box is detected, begin the return protocol
        elif state == 2:
            line_sensor.update()
            if line_sensor.get_denominator() == 5 and share_bump.get() == 0:
                old = utime.ticks_ms()
                state = 3

        # Center in the finish box, and relinquish control to the IMU for the
        # return
        elif state == 3:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            if time_run > 1020:
                share_return.put(1)
                state = 1

        yield None

#----------------------------------------------------------------------------------#

# Task 5
def bump_sense(shares):
    '''
    @brief    Task 5: Control task for the bump sensor
    @details  The bump sensor contrl task monitors the bump sensor for triggering,
              and once triggered, it sends desired motor speeds to the motor 
              classes to navigate around the obstacle. Once clearing the obstacle
              and returning to the line, it reliquishes control to the line
              sensors.
    '''

    share_bump, share_omega_L, share_omega_R = shares

    state = 0
    time_run = 0
    while True:
        
        if state == 0:
            share_bump.put(0)
            state =1

        # Detect when a bump occurs and prepare the system to respond
        elif state == 1:
            if bump_sensor.detect() < 2000:
                share_bump.put(1)
                share_omega_L.put(0)
                share_omega_R.put(0)
                mot_L.set_dir(False)
                mot_R.set_dir(False)
                old = utime.ticks_ms()
                state = 2
        
        # Reverse from the obstacle
        elif state == 2:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            share_omega_L.put(8)
            share_omega_R.put(8)
            if time_run > 400:
                time_run = 0
                state = 3
        
        # Rotate right
        elif state == 3:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            mot_L.set_dir(True)
            share_omega_L.put(12)
            share_omega_R.put(12)
            if time_run > 350:
                time_run = 0
                state = 4

        # Drive forward to avoid the obstacle
        elif state == 4:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            mot_R.set_dir(True)
            share_omega_L.put(18)
            share_omega_R.put(18)
            if time_run > 600:
                time_run = 0
                state = 5

        # Rotate left
        elif state == 5:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            mot_L.set_dir(False)
            share_omega_L.put(12)
            share_omega_R.put(12)
            if time_run > 332:
                time_run = 0
                state = 6

        # Drive forward past the obstacle
        elif state == 6:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            mot_L.set_dir(True)
            share_omega_L.put(16)
            share_omega_R.put(16)
            if time_run > 880:
                time_run = 0
                state = 7

        # Turn back towards the path
        elif state == 7:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            share_omega_L.put(4)
            share_omega_R.put(18)
            if time_run > 500:
                time_run = 0
                state = 8

        # Continue straight until ready to relinquich control to the line sensors
        elif state == 8:
            now = utime.ticks_ms()
            time_run += utime.ticks_diff(now, old)
            old = utime.ticks_ms()
            share_omega_L.put(10)
            share_omega_R.put(10)
            if line_sensor.get_modifier() > 0:
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
bump_sensor = Bumpsensor(Pin(Pin.board.PC4))        # Bump sensor object

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

# Shared variable setup
#
# Flag to communicate to tasks when the bump sensor has been activated
bump_triggered = task_share.Share('B', thread_protect=False, name="BumpTrigger")
# Programmable speed for the left motor to set the obstacle avoidance response
bump_omega_L = task_share.Share('f', thread_protect=False, name="BumpLeftSpeed")
# Programmable speed for the left motor to set the obstacle avoidance response
bump_omega_R = task_share.Share('f', thread_protect=False, name="BumpRightSpeed")
# Flag to communicate to tasks when the return to start protocal is required
return_triggered = task_share.Share('B', thread_protect=False,
                                     name="ReturnTrigger")
# Programmable speed for the left motor to set the return response
return_omega_L = task_share.Share('f', thread_protect=False,
                                   name="ReturnLeftSpeed")
# Programmable speed for the left motor to set the return response
return_omega_R = task_share.Share('f', thread_protect=False,
                                   name="ReturnRightSpeed")
# Variable to inform the motors if the robot is off course during the return and
# correct for the error
return_adjustment = task_share.Share('f', thread_protect=False,
                                      name="ReturnAdjustment")

# Task setup
task1 = cotask.Task(mot_enc_L, name="Task_1", priority=1, period=6,
                    profile=True, trace=False, shares=(bump_triggered,bump_omega_L,
                                return_triggered,return_omega_L,return_adjustment))
task2 = cotask.Task(mot_enc_R, name="Task_2", priority=1, period=6,
                    profile=True, trace=False, shares=(bump_triggered,bump_omega_R,
                                return_triggered,return_omega_R,return_adjustment))
task3 = cotask.Task(imu_control, name="Task_3", priority=1, period=6,
                    profile=True, trace=False, shares=(return_triggered,
                                return_omega_L,return_omega_R,return_adjustment))
task4 = cotask.Task(line_sense, name="Task_4", priority=1, period=6,
                    profile=True, trace=False, shares=(bump_triggered,
                                return_triggered))
task5 = cotask.Task(bump_sense, name="Task_5", priority=1, period=6,
                    profile=True, trace=False, shares=(bump_triggered,
                                bump_omega_L,bump_omega_R))

# Append tasks to the task list
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
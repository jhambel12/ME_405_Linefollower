'''
@file          BNO055.py
@brief         A driver for the BNO055
@details       This file contains the class to setup the BNO055. It allows it to be set into any
               of its fusion modes, and perform multiple types of data reads necessary for feedback
               control.
@author        Jack Ellis, Jacob Hambel
@date          November 21, 2024
'''

import struct
from pyb import I2C
import utime

class BNO055:
    '''
    @brief       Interface with quadrature encoders
    @details     The functions in this class take inputs to intialize the pins to be used by the encoders, 
                 perform measurements of the rotation of the encoders, and perform calculations needed
                 to find motor speed.
    '''

    def __init__(self, myi2c):
        '''
        @brief    Initialize the IMU
        @details  This method initializes the IMU by accepting a pyb.I2C object 
                  preconfigured in CONTROLLER mode and the device address.
        @param    myi2c    A preconfigured pyb.I2C object in CONTROLLER mode
        '''
        # Store the I2C object and address
        self.i2c = myi2c

        # Create variables for the addresses of data registers
        self.adr_acc_x = 0x08
        self.adr_acc_y = 0x0A
        self.adr_acc_z = 0x0C
        self.adr_mag_x = 0x0E
        self.adr_mag_y = 0x10
        self.adr_mag_z = 0x12
        self.adr_gyr_x = 0x14
        self.adr_gyr_y = 0x16
        self.adr_gyr_z = 0x18
        self.adr_eul_head = 0x1A
        self.adr_eul_roll = 0x1C
        self.adr_eul_ptch = 0x1E

        # Create variables for the addreses of offset registers
        self.adr_acc_x_off = 0x55
        self.adr_acc_y_off = 0x57
        self.adr_acc_z_off = 0x59
        self.adr_mag_x_off = 0x5B
        self.adr_mag_y_off = 0x5D
        self.adr_mag_z_off = 0x5F
        self.adr_gyr_x_off = 0x61
        self.adr_gyr_y_off = 0x63
        self.adr_gyr_z_off = 0x65

        # Create variables for other useful registers
        self.adr_opr_mode = 0x3D
        self.adr_calib_stat = 0x35
        self.BNO055_ADDRESS = 0x28
        self.adr_unit_sel = 0x3B

        # Constant defining the desired unit system
        self.units = 4

        pass


    def op_mode(self,mode):
        '''
        @brief    Set fusion mode
        @details  This method changes the operating mode of the IMU to one of the many “fusion” modes
                  available from the BNO055.
        @param    mode: selected fusion mode to set the BNO055 to
        '''

        op_modes = {
            "IMU": 8,
            "COMPASS": 9,
            "M4G": 10,
            "NDOF_FMC_OFF": 11,
            "NDOF": 12
        }

        def select_op_mode(selected_mode):
            return op_modes.get(selected_mode)
        
        self.set_mode = select_op_mode(mode)
        self.op_buf = bytearray([0 for n in range(1)])
        self.i2c.mem_write(self.set_mode, self.BNO055_ADDRESS, self.adr_opr_mode)
        
        pass

    def status(self):
        '''
        @brief    Handle calibration status
        @details  This method retrieves and parses the calibration status byte from the IMU.
        '''

        self.stat_buf = bytearray([0 for n in range(1)])
        self.i2c.mem_read(self.stat_buf, self.BNO055_ADDRESS, self.adr_calib_stat)

        sys_calib = (self.stat_buf[0] >> 6) & 0b11    # Bits 6-7
        gyro_calib = (self.stat_buf[0] >> 4) & 0b11   # Bits 4-5
        accel_calib = (self.stat_buf[0] >> 2) & 0b11  # Bits 2-3
        mag_calib = self.stat_buf[0] & 0b11           # Bits 0-1

        # Display parsed calibration statuses
        print("System Calibration Status:", sys_calib)
        print("Gyroscope Calibration Status:", gyro_calib)
        print("Accelerometer Calibration Status:", accel_calib)
        print("Magnetometer Calibration Status:", mag_calib)
        return self.stat_buf

    def set_units(self):
        '''
        @brief    Sets the units for each measurement to the desired system
        @details  This method writes the constant defined for the desired unit system to the unit register
        '''
        self.i2c.mem_write(self.units, self.BNO055_ADDRESS, self.adr_unit_sel)

        pass

    def get_coeffs(self):
        '''
        @brief    Retrieve calibration coeffcients
        @details  This method retrieves the calibration coefficients from the IMU as a binary data.
        '''

        self.coeff_buf = bytearray(18)
    
        self.i2c.mem_read(self.coeff_buf, self.BNO055_ADDRESS, self.adr_acc_x_off)
        
        self.acc_x_off = struct.unpack('<h', self.coeff_buf[0:2])[0]
        self.acc_y_off = struct.unpack('<h', self.coeff_buf[2:4])[0]
        self.acc_z_off = struct.unpack('<h', self.coeff_buf[4:6])[0]
        self.mag_x_off = struct.unpack('<h', self.coeff_buf[6:8])[0]
        self.mag_y_off = struct.unpack('<h', self.coeff_buf[8:10])[0]
        self.mag_z_off = struct.unpack('<h', self.coeff_buf[10:12])[0]
        self.gyr_x_off = struct.unpack('<h', self.coeff_buf[12:14])[0]
        self.gyr_y_off = struct.unpack('<h', self.coeff_buf[14:16])[0]
        self.gyr_z_off = struct.unpack('<h', self.coeff_buf[16:18])[0]

        return self.coeff_buf

    def write_coeffs(self, a, b, c, d, e, f, g, h, i):
        '''
        @brief    Write calibration coefficients
        @details  This method writes calibration coefficients to the IMU based on inputs.
        @param    a-i: Calibration coefficients
        '''

        coeff_buf = struct.pack('<hhhhhhhhh',
            a, b, c,
            d, e, f,
            g, h, i
        )

        self.i2c.mem_write(coeff_buf, self.BNO055_ADDRESS, self.adr_acc_x_off)

        pass
    
    def euler(self, request):
        '''
        @brief    Read Euler angles
        @details  This method reads the Euler angles from the IMU to use as measurements for feedback. 
        @param    request: requested axis for euler angle
        '''

        self.eul_buf = bytearray(2)

        if request == "head":
            self.i2c.mem_read(self.eul_buf, self.BNO055_ADDRESS, self.adr_eul_head)
        
        if request == "roll":
            self.i2c.mem_read(self.eul_buf, self.BNO055_ADDRESS, self.adr_eul_roll)

        if request == "pitch":
            self.i2c.mem_read(self.eul_buf, self.BNO055_ADDRESS, self.adr_eul_ptch)
        
        self.eul = (struct.unpack('<h', self.eul_buf[0:2])[0]) / 900
        return self.eul

    def angular(self,request):
        '''
        @brief    Read angular velocity
        @details  This method reads the angular velocity from the IMU to use as measurements for feedback. 
        @param    request: requested axis for angular velocity
        '''

        self.gyr_buf = bytearray(2)

        if request == "x":
            self.i2c.mem_read(self.gyr_buf, self.BNO055_ADDRESS, self.adr_gyr_x)
        
        if request == "y":
            self.i2c.mem_read(self.gyr_buf, self.BNO055_ADDRESS, self.adr_gyr_y)

        if request == "z":
            self.i2c.mem_read(self.gyr_buf, self.BNO055_ADDRESS, self.adr_gyr_z)
        
        self.gyr = (struct.unpack('<h', self.gyr_buf[0:2])[0]) / 900
        return self.gyr
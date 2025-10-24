#! /usr/bin/env python3

'''
@author: Emerson Martins de Andrade
COPPE/UFRJ     2025

Based on: https://www.electronicwings.com/raspberry-pi/triple-axis-magnetometer-hmc5883l-interfacing-with-raspberry-pi
'''

from threading import Thread
import smbus
from time import sleep
import math


class Compass(Thread, object):

    def __init__(self):

        Thread.__init__(self)

        self.heading_angle = 0
        
        self._start_thread('main')
        
        self.heading_offset = -19.0

    def _start_thread(self, name):
        if name == 'main':
            self.main_loop = Thread(target=self._update)
            self.main_loop.daemon = True
            self.main_loop.start()

    def _update(self):
        
        try:
        
            #some MPU6050 Registers and their Address
            Register_A     = 0              #Address of Configuration register A
            Register_B     = 0x01           #Address of configuration register B
            Register_mode  = 0x02           #Address of mode register

            X_axis_H    = 0x03              #Address of X-axis MSB data register
            Z_axis_H    = 0x05              #Address of Z-axis MSB data register
            Y_axis_H    = 0x07              #Address of Y-axis MSB data register
            pi          = 3.14159265359     #define pi value

            def Magnetometer_Init():
                    #write to Configuration Register A
                    bus.write_byte_data(Device_Address, Register_A, 0x70)

                    #Write to Configuration Register B for gain
                    bus.write_byte_data(Device_Address, Register_B, 0xa0)

                    #Write to mode Register for selecting mode
                    bus.write_byte_data(Device_Address, Register_mode, 0)
                    
                    bus.write_byte_data(Device_Address, 0x00, 0xF8) # CRA 75Hz
                    
                    bus.write_byte_data(Device_Address, 0x02, 0x00) # Mode continuous reads

            def read_raw_data(addr):
                
                    #Read raw 16-bit value
                    high = bus.read_byte_data(Device_Address, addr)
                    low = bus.read_byte_data(Device_Address, addr+1)

                    #concatenate higher and lower value
                    value = ((high << 8) | low)

                    #to get signed value from module
                    if(value > 32768):
                        value = value - 65536
                    return value


            bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
            Device_Address = 0x1e   # HMC5883L magnetometer device address

            Magnetometer_Init()     # initialize HMC5883L magnetometer 

            while True:
                
                    #Read Accelerometer raw value
                    x = read_raw_data(X_axis_H)
                    z = read_raw_data(Z_axis_H)
                    y = read_raw_data(Y_axis_H)
                                        
                    # Offset and Scaling from calibration procedure
                    offset = [35.5, -88.5,  41.0]
                    scaling_factor = [1.0, 1.189, 1.0]
                    
                    # CalibratedData = ( unCalibratedData - Offset ) / Scaling Factor
                    x_calib = (x - offset[0]) / scaling_factor[0]
                    y_calib = (y - offset[1]) / scaling_factor[1]
                    #z_calib = (z - offset[2]) / scaling_factor[2]

                    heading = math.atan2(y_calib, x_calib)
                    
                    heading += math.radians(self.heading_offset)
                    
                    #Check for >360 degree
                    if(heading > 2*pi):
                            heading = heading - 2*pi

                    #check for sign
                    if(heading < 0):
                            heading = heading + 2*pi
                            
                    #convert into angle
                    heading_angle = int(heading * 180/pi)

                    self.heading_angle = heading_angle
                    sleep(1)
        except:
            pass
            sleep(1)

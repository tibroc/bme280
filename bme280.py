"""
This file contains the implementation of the bme280 class that let's you configure
the sensor and read-out the data.

Sources that served as guidance during implementation:
* https://github.com/robert-hh/BME280 (MIT License)

"""
import machine
from ustruct import unpack, unpack_from

import constants


class bme280_instance(i2c_connection=None, bme_address=None):
    
    def __init__(self):
        self.i2c = i2c_connection if i2c_connection else self._init_i2c()
        self.bme_i2c_addr = bme_address if bme_address else constants.BME_I2C_ADDR
        self.calibration_t = None
        self.calibration_p = None
        self.calibration_h = None
        self._start_up()

    def _start_up(self):
        self.check_sensor()
        print("Sensor found.")
        self.reset_sensor()
        print("Initiated soft-reset.")
        self.read_calibration_data()
        print("Read calibration data.")
    
    def _init_i2c(self):
        self.i2c = machine.I2C(scl=constants.SCL, sda=constants.SDA, freq=100000)

    def check_sensor(self):
        all_ics = self.i2c.scan()
        if self.bme_i2c_addr not in all_ics:
            raise OSError("Sensor not found at the provided address.")

    def reset_sensor(self):
        self.i2c.writeto_mem(self.bme_i2c_addr, constants.REG_RESET, b'\xB6')
    
    def read_calibration_data(self):
        calib00_25 = self.i2c.readfrom_mem(self.bme_i2c_addr, constants.REG_CALIB00_25[0], constants.REG_CALIB00_25[1])
        calib26_41 = self.i2c.readfrom_mem(self.bme_i2c_addr, constants.REG_CALIB26_41[0], constants.REG_CALIB26_41[1])
        dig00_25 = unpack("<HhhHhhhhhhhhBB", calib00_25)
        dig26_41 = unpack("<hBbhb", calib26_41)
        self.calibration_t = dig00_25[:3]
        self.calibration_p = dig00_25[3:12]
        h4 = (dig26_41[2] * 16) + (dig26_41[3] & 0xF)  # additional unpacking 
        h5 = dig26_41[3] // 16
        self.calibration_h = (dig00_25[13], dig26_41[0], dig26_41[1], dig26_41[2], h4, h5)



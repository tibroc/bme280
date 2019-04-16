"""
This file contains the implementation of the bme280 class that let's you configure
the sensor and read-out the data.

Sources that served as guidance during implementation:
* https://github.com/robert-hh/BME280 (MIT License)

"""
import machine
from ustruct import unpack

import constants


class bme280_instance:
    
    def __init__(self, i2c_connection=None, bme_address=None):
        self.i2c = i2c_connection if i2c_connection else self._init_i2c()
        self.bme_i2c_addr = bme_address if bme_address else constants.BME_I2C_ADDR
        self.mode = 'normal'
        self.time_sb = 500
        self.oversampling_t = 'x1'
        self.oversampling_p = 'x1'
        self.oversampling_h = 'x1'
        self.filter = 'off'
        self._config = 0x80  # config byte, default value
        self._ctrl_meas = 0x27  # control measure byte (os_t, os_p, mode), default value
        self._calibration_t = None  # init with empty tuple!?
        self._calibration_p = None
        self._calibration_h = None
        self._start_up()

    def _start_up(self):
        print("Initializing sensor.")
        self.check_sensor()
        print("Sensor found.")
        self.reset_sensor()
        print("Initiated soft-reset.")
        self._read_calibration_data()
        print("Read calibration data.")
    
    def _init_i2c(self):
        self.i2c = machine.I2C(scl=constants.SCL, sda=constants.SDA, freq=100000)

    def check_sensor(self):
        all_ics = self.i2c.scan()
        if self.bme_i2c_addr not in all_ics:
            raise OSError("Sensor not found at the provided address.")
        # check sensor id:
        chip = self.i2c.readfrom_mem(constants.BME_I2C_ADDR, constants.REG_ID, 2)
        chip_id = unpack("<h", chip)[0]
        if chip_id != 0x60:
            raise OSError("The i2c device does not return the correct chip id.")

    def reset_sensor(self):
        self.i2c.writeto_mem(self.bme_i2c_addr, constants.REG_RESET, bytearray([0xB6]))
    
    def _read_calibration_data(self):
        calib00_25 = self.i2c.readfrom_mem(self.bme_i2c_addr, constants.REG_CALIB00_25[0], constants.REG_CALIB00_25[1])
        calib26_41 = self.i2c.readfrom_mem(self.bme_i2c_addr, constants.REG_CALIB26_41[0], constants.REG_CALIB26_41[1])
        dig00_25 = unpack("<HhhHhhhhhhhhBB", calib00_25)
        dig26_41 = unpack("<hBbhb", calib26_41)
        self._calibration_t = dig00_25[:3]
        self._calibration_p = dig00_25[3:12]
        h4 = (dig26_41[2] * 16) + (dig26_41[3] & 0xF)  # additional unpacking 
        h5 = dig26_41[3] // 16
        self._calibration_h = (dig00_25[13], dig26_41[0], dig26_41[1], dig26_41[2], h4, h5)

    def set_temp_oversampling(self, rate):
        if rate not in ['x1', 'x2', 'x4', 'x8', 'x16']:
            raise TypeError("'rate' has to be one of ['x1', 'x2', 'x4', 'x8', 'x16']")
        bit_mask = 0x1F
        if rate == 'x1':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.OS_T_1
        if rate == 'x2':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.OS_T_2
        if rate == 'x4':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.OS_T_4
        if rate == 'x8':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.OS_T_8
        if rate == 'x16':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.OS_T_16
        self.i2c.writeto_mem(self.bme_i2c_addr, constants.REG_CTRL_MEAS, bytearray([self._ctrl_meas]))

    def get_temp_oversampling(self):
        ctrl_byte = self.i2c.readfrom_mem(self.bme_i2c_addr, constants.REG_CTRL_MEAS, 1)
        ctrl_byte = unpack('<b', ctrl_byte)[0]
        bit_mask = 0xE0
        value = ctrl_byte & bit_mask
        if value == constants.OS_T_1:
            return 'x1'
        if value == constants.OS_T_2:
            return 'x2'
        if value == constants.OS_T_4:
            return 'x4'
        if value == constants.OS_T_8:
            return 'x8'
        if value in [constants.OS_T_16, 0xE0, 0xC0]:
            return 'x16'
        raise RuntimeError("Something went wrong. Did you configure the sensor?")
        
    def set_press_oversampling(self, rate):
        if rate not in ['x1', 'x2', 'x4', 'x8', 'x16']:
            raise TypeError("'rate' has to be one of ['x1', 'x2', 'x4', 'x8', 'x16']")
        bit_mask = 0xE3
        if rate == 'x1':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.OS_P_1
        if rate == 'x2':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.OS_P_2
        if rate == 'x4':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.OS_P_4
        if rate == 'x8':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.OS_P_8
        if rate == 'x16':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.OS_P_16
        self.i2c.writeto_mem(self.bme_i2c_addr, constants.REG_CTRL_MEAS, bytearray([self._ctrl_meas]))

    def get_press_oversampling(self):
        ctrl_byte = self.i2c.readfrom_mem(self.bme_i2c_addr, constants.REG_CTRL_MEAS, 1)
        ctrl_byte = unpack('<b', ctrl_byte)[0]
        bit_mask = 0x1C
        value = ctrl_byte & bit_mask
        if value == constants.OS_P_1:
            return 'x1'
        if value == constants.OS_P_2:
            return 'x2'
        if value == constants.OS_P_4:
            return 'x4'
        if value == constants.OS_P_8:
            return 'x8'
        if value in [constants.OS_P_16, 0x1C, 0x18]:
            return 'x16'
        raise RuntimeError("Something went wrong. Did you configure the sensor?")

    def set_hum_oversampling(self, rate):
        if rate not in ['x1', 'x2', 'x4', 'x8', 'x16']:
            raise TypeError("'rate' has to be one of ['x1', 'x2', 'x4', 'x8', 'x16']")
        bit_mask = 0xF8
        if rate == 'x1':
            self._ctrl_hum = (self._ctrl_hum & bit_mask) | constants.OS_H_1
        if rate == 'x2':
            self._ctrl_hum = (self._ctrl_hum & bit_mask) | constants.OS_H_2
        if rate == 'x4':
            self._ctrl_hum = (self._ctrl_hum & bit_mask) | constants.OS_H_4
        if rate == 'x8':
            self._ctrl_hum = (self._ctrl_hum & bit_mask) | constants.OS_H_8
        if rate == 'x16':
            self._ctrl_hum = (self._ctrl_hum & bit_mask) | constants.OS_H_16
        self.i2c.writeto_mem(self.bme_i2c_addr, constants.REG_CTRL_HUM, bytearray([self._ctrl_hum]))
        # it is necessary to write to ctrl_meas for the changes in ctrl_hum to take place
        self.i2c.writeto_mem(self.bme_i2c_addr, constants.REG_CTRL_MEAS, bytearray([self._ctrl_meas]))

    def get_hum_oversampling(self):
        ctrl_byte = self.i2c.readfrom_mem(self.bme_i2c_addr, constants.REG_CTRL_HUM, 1)
        ctrl_byte = unpack('<b', ctrl_byte)[0]
        bit_mask = 0x07
        value = ctrl_byte & bit_mask
        if value == constants.OS_H_1:
            return 'x1'
        if value == constants.OS_H_2:
            return 'x2'
        if value == constants.OS_H_4:
            return 'x4'
        if value == constants.OS_H_8:
            return 'x8'
        if value in [constants.OS_H_16, 0x07, 0x06]:
            return 'x16'
        raise RuntimeError("Something went wrong. Did you configure the sensor?")

    def set_mode(self, mode):
        if mode not in ['sleep', 'force', 'normal']:
            raise TypeError("'mode' has to be one of ['sleep', 'force', 'normal']")
        bit_mask = 0xFC
        if mode == 'sleep':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.MODE_SLEEP
        if mode == 'force':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.MODE_FORCE
        if mode == 'normal':
            self._ctrl_meas = (self._ctrl_meas & bit_mask) | constants.MODE_NORMAL
        self.i2c.writeto_mem(self.bme_i2c_addr, constants.REG_CTRL_MEAS, bytearray([self._ctrl_meas]))

    def get_mode(self):
        ctrl_byte = self.i2c.readfrom_mem(self.bme_i2c_addr, constants.REG_CTRL_MEAS, 1)
        ctrl_byte = unpack('<b', ctrl_byte)[0]
        bit_mask = 0x03
        value = ctrl_byte & bit_mask
        if value == constants.MODE_SLEEP:
            return 'sleep'
        if value == constants.MODE_FORCE:
            return 'force'
        if value == constants.MODE_NORMAL:
            return 'normal'
        raise RuntimeError("Something went wrong. Did you configure the sensor?")


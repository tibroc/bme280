"""
BME280 trial and error script

"""

import machine
from ustruct import unpack

import constants
import bme280


constants.SDA = machine.Pin('X2', machine.Pin.OUT)
constants.SCL = machine.Pin('X3', machine.Pin.OUT)
constants.BME_I2C_ADDR = 0x76


# init the i2c interface
i2c = machine.I2C(scl=constants.SCL, sda=constants.SDA, freq=100000)

bme = bme280.bme280_instance(i2c)

bme.get_values


h = self._t_fine - 76800

h = (((
((self.adc_humidity << 14) - (self._calibration_h[3] << 20) - (self._calibration_h[4] * h)) + 
16384) >> 15) * (((((((h * self._calibration_h[5]) >> 10) * (((h * self._calibration_h[2]) >> 11) + 32768)) >> 10) + 2097152) * self._calibration_h[1] + 8192) >> 14))

h = h - (((((h >> 15) * (h >> 15)) >> 7) * self._calibration_h[0]) >> 4)
h = 0 if h < 0 else h
h = 419430400 if h > 419430400 else h
# only difference from data sheet is division by 1024 to get relative humidity
(h >> 12) / 1024

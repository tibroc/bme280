"""
BME280 trial and error script

"""

import machine
from ustruct import unpack

import constants
import bme280

# init the i2c interface
i2c = machine.I2C(scl=constants.SCL, sda=constants.SDA, freq=100000)

bme = bme280.bme280_instance(i2c)





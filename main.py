
import machine

import bme280
import constants

# init the i2c interface
i2c = machine.I2C(scl=constants.SCL, sda=constants.SDA, freq=100000)

# init the sensor
bme = bme280.bme280(i2c)



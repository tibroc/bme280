"""
BME280 trial and error script

"""

import machine
from ustruct import unpack, unpack_from

import constants
import bme280

# init the i2c interface
i2c = machine.I2C(scl=SCL, sda=SDA, freq=100000)

# somehow only returns 0x00
# i2c.readfrom_mem(bme_address, REG_ID, 2)


"""
BME280 constants

This file includes all constants and default configs of the sensor (e.g. pins, registers).
It is largely a transcription of the information in the data sheet:
https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280-DS002.pdf
"""

import machine

"""
Pin definitions
"""
SDA = machine.Pin(21, machine.Pin.OUT)
SCL = machine.Pin(22, machine.Pin.OUT)

"""
I2C adresses
"""
BME_I2C_ADDR = 0x77

"""
Memory register adresses
The names correspond to the memory map as described in the data sheet.
"""
REG_HUM_LSB = 0xFE  # read only
REG_HUM_MSB = 0xFD  # read only
REG_TEMP_XLSB = 0xFC   # read only
REG_TEMP_LSB = 0xFB  # read only
REG_TEMP_MSB = 0xFA  # read only
REG_PRESS_XLSB = 0xF9  # read only
REG_PRESS_LSB = 0xF8  # read only
REG_PRESS_MSB = 0xF7  # read only
REG_CONFIG = 0xF5  # read/write
REG_CTRL_MEAS = 0xF4  # read/write
REG_STATUS = 0xF3  # read/write
REG_CTRL_HUM = 0xF2  # read/write
REG_CALIB26_41 = (0xE1, 7)  # read 7 bytes until register 0xE7 - read only
REG_RESET = 0xE0  # write only
REG_ID = 0xD0  # read only
REG_CALIB00_25 = (0x88, 26)  # read 26 bytes until register 0xA1 - read only

"""
Oversampling rates
"""
OSRS_1 = 1
OSRS_2 = 2
OSRS_4 = 3
OSRS_8 = 4
OSRS_16 = 5

"""
modes
"""
MODE_SLEEP = 0
MODE_FORCE = 1
MODE_NORMAL = 3

"""
Additional constants
"""
SEALEVELPRESSURE_HPA = 1013.25


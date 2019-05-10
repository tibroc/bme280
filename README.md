
# MicroPython Interface to the Bosch BME280

This is a (MicroPython)[https://micropython.org/] interface to configure and read out the (Bosch BME280)[https://www.bosch-sensortec.com/bst/products/all_products/bme280] humidity, pressure and temperature sensor. It is supposed to provide an API to the sensor's full functionality and be cross-functional between boards. It was tested on the (ESP32)[https://www.espressif.com/en/products/hardware/esp32/overview] and the (PyBoard v1.1)[https://store.micropython.org/product/PYBv1.1].


## Hardware Set-Up
Connect the `Vin` pin of the sensor to a 3.3V outlet on the board and the `GND` to ground. You can connect the `SDA` and `SCL` pins of the sensor's I2C-bus basically to any general purpose pin on your board of choice. Then configure your I2C in your MicroPython script as described below.

## Usage

### I2C-connection
You first need to initialize a I2C-connection to your sensor. The sensor's default address on the bus is `0x76` and it is saved in `constants.py`. However, you can override this setting for each instance of the sensor-class when initializing it. An example to set-up your I2C-interface might look like this:

```python
import machine

# configure the I2C-pins (change the ids according to your set-up)
sda_pin = machine.Pin('X1', machine.Pin.OUT)
scl_pin = machine.Pin('X2', machine.Pin.OUT)

# init the i2c connection
i2c = machine.I2C(scl=scl_pin, sda=sda_pin, freq=100000)

```

### initialize sensor

```python
import bme280

# initialize a sensor
sensor = bme280.bme280_instance(i2c_connection=i2c, bme_address=None)

# read values
print(sensor.get_values)

```

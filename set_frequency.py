import time
import smbus2
from functions import *

# INIT SMBUS
i2c_bus = smbus2.SMBus(7)
address=0x40

# frequency to 330 Hz
frequency=340

set_frequency(i2c_bus,address,frequency)
# close the i2c bus
i2c_bus.close()
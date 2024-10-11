import numpy as np
from functions import *

i2c_bus = smbus2.SMBus(7)

left_turn(1.6,i2c_bus)

right_turn(1.6,i2c_bus)

i2c_bus.close()
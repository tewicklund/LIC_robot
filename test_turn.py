import numpy as np
from functions import *

i2c_bus = smbus2.SMBus(7)

left_turn(i2c_bus,0x08)

i2c_bus.close()
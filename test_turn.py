import numpy as np
from functions import *

i2c_bus = smbus2.SMBus(1)

left_turn(i2c_bus)

i2c_bus.close()
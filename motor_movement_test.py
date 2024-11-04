from functions import *


i2c_bus = smbus2.SMBus(7)

while True:
    drive_motor_exp('R',60,i2c_bus)
    drive_motor_exp('L',60,i2c_bus)
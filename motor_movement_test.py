from functions import *


i2c_bus = smbus2.SMBus(7)

while True:
    drive_motor_exp('R',32,i2c_bus)
    drive_motor_exp('L',-16,i2c_bus)
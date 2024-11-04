from functions import *


i2c_bus = smbus2.SMBus(7)

for x in range(100):
    drive_motor_exp('R',1,i2c_bus)
    drive_motor_exp('L',1,i2c_bus)
    time.sleep(0.01)

for x in range(100):
    drive_motor_exp('R',63,i2c_bus)
    drive_motor_exp('L',63,i2c_bus)
    time.sleep(0.01)
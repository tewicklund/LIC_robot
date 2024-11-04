from functions import *


i2c_bus = smbus2.SMBus(7)

speed=int(input("Enter speed, from 0 to 63"))

while True:
    for x in range(100):
        drive_motor_exp('R',speed,i2c_bus)
        drive_motor_exp('L',speed,i2c_bus)
        time.sleep(0.01)


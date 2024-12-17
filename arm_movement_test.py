import time
import smbus2
from functions import *

# INIT SMBUS
bus = smbus2.SMBus(7)
address=0x40

# frequency to 330 Hz
frequency=340
# NEED TO SET FREQUENCY BEFORE CONNECTING ANY SERVOS, SO THE ROBOT DOES NOT DESTROY ITSELF
set_frequency(bus,address,frequency)

option=0

while True:
    if option==0:
        print("Setting arm to position a")
        set_arm_position(bus,address,frequency,'a')
        

    elif option==1:
        print("Setting arm to position b")
        set_arm_position(bus,address,frequency,'b')
        

    elif option==2:
        print("Setting arm to position c")
        set_arm_position(bus,address,frequency,'c')
        


    option+=1
    if option==3:
        option=0
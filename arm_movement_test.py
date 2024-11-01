import time
import smbus
from functions import *

# INIT SMBUS
bus = smbus.SMBus(7)
address=0x40

# frequency to 330 Hz
frequency=340
set_frequency(bus,address,frequency)

option=0

while True:
    if option==0:
        set_arm_position(bus,address,frequency,'a')
        print("Setting arm to position a")

    elif option==1:
        set_arm_position(bus,address,frequency,'b')
        print("Setting arm to position b")

    elif option==2:
        set_arm_position(bus,address,frequency,'c')
        print("Setting arm to position c")

    else:
        set_arm_position(bus,address,frequency,'z')
        print("Setting arm to position home")

    option+=1
    if option==4:
        option=0
    time.sleep(1)
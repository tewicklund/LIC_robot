import time
import smbus

# PIN SELECTIONS
input1_pin = 11
input2_pin = 12
led_pin= 13

# INIT SMBUS
bus = smbus.SMBus(7)
address=0x40


def set_prescale(prescale):
    # Set sleep bit to 1
    mode1 = bus.read_byte_data(address, 0)
    mode1 = mode1 | 0x10
    bus.write_byte_data(address, 0, mode1)

    # Calculate prescale value
    bus.write_byte_data(address, 0xFE, prescale)

    # Reset back sleep bit to operational mode
    mode1 = mode1 & ~0x10
    bus.write_byte_data(address, 0, mode1)
    time.sleep(1)
    bus.write_byte_data(address, 1, 4)

set_prescale(60)

bus.write_byte_data(address, 7, 0)
bus.write_byte_data(address, 6, 0)

bus.write_byte_data(address, 11, 0)
bus.write_byte_data(address, 10, 0)

on_time = 0
duty_cycle = 0

option=0

while True:
    
    time.sleep(0.02)

    if option==0:
        # pwm1.5, duty cycle = 0
        on_time = 614
        duty_cycle = 0
        print("Detected in1 = 0, in2 = 0")

    elif option==1:
        # pwm1.1, duty cycle = 50
        on_time = 450
        duty_cycle = 2047
        print("Detected in1 = 0, in2 = 1")

    elif option==2:
        # pwm1.9, duty cycle = 50
        on_time = 778
        duty_cycle = 2047
        print("Detected in1 = 1, in2 = 0")

    else:
        #pwm1.5, duty cycle = 80
        on_time = 614
        duty_cycle = 3275
        print("Detected in1 = 1, in2 = 1")

    bus.write_byte_data(address, 9, on_time>>8)
    bus.write_byte_data(address, 8, on_time & 0xFF)
    bus.write_byte_data(address, 13, duty_cycle>>8)
    bus.write_byte_data(address, 12, duty_cycle & 0xFF)

    option+=1
    if option==4:
        option=0
    time.sleep(1)
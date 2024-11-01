import time
import smbus

# INIT SMBUS
bus = smbus.SMBus(7)
address=0x40

def set_servo_angle(bus,frequency,channel,angle):

    # compute the on time register values from the angle
    on_time_us=9.27*angle+1500
    on_time_register_value=int(on_time_us*4096/(1000000/frequency))

    # set on timestamp to zero
    bus.write_byte_data(address, 6+4*channel, 0)
    bus.write_byte_data(address, 7+4*channel, 0)

    # set off timestamp to on_time integer
    bus.write_byte_data(address, 8+4*channel, on_time_register_value & 0xFF)
    bus.write_byte_data(address, 9+4*channel, on_time_register_value>>8)
    
    

def set_frequency(frequency):

    prescale=int(25000000/(4096*frequency)+1)
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

# frequency to 330 Hz
frequency=340
set_frequency(frequency)

option=0

while True:
    if option==0:
        set_servo_angle(bus,frequency,0,-90)
        print("Setting servo to -90 degrees")

    elif option==1:
        set_servo_angle(bus,frequency,0,0)
        print("Setting servo to 0 degrees")

    elif option==2:
        set_servo_angle(bus,frequency,0,90)
        print("Setting servo to 90 degrees")

    else:
        set_servo_angle(bus,frequency,0,0)
        print("Setting servo to 0 degrees")

    option+=1
    if option==4:
        option=0
    time.sleep(1)
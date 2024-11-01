import time
import smbus

# INIT SMBUS
bus = smbus.SMBus(7)
address=0x40

def compute_on_time(time_on_us,frequency):
    on_time_points=int(time_on_us*4096/(1000000/frequency))
    return on_time_points

def set_frequency(frequency):

    prescale=25000000/(4096*frequency)+1
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

# set all register bits to zero for the first 2 channels
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
        on_time_1 = compute_on_time(666,340)
        on_time_2 = compute_on_time(666,340)
        print("Setting servo to -90 degrees")

    elif option==1:
        on_time_1 = compute_on_time(1500,340)
        on_time_2 = compute_on_time(1500,340)
        print("Setting servo to 0 degrees")

    elif option==2:
        on_time_1 = compute_on_time(2334,340)
        on_time_2 = compute_on_time(2334,340)
        print("Setting servo to 90 degrees")

    else:
        on_time_1 = compute_on_time(1500,340)
        on_time_2 = compute_on_time(1500,340)
        print("Setting servo to 0 degrees")

    bus.write_byte_data(address, 9, on_time>>8)
    bus.write_byte_data(address, 8, on_time & 0xFF)
    bus.write_byte_data(address, 13, duty_cycle>>8)
    bus.write_byte_data(address, 12, duty_cycle & 0xFF)

    option+=1
    if option==4:
        option=0
    time.sleep(1)
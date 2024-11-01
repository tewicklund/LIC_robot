from smbus2 import SMBus
import time

# PCA9685 setup
DEVICE_ADDRESS = 0x40  # I2C address of PCA9685
BUS_NUMBER = 7  # I2C bus number on Jetson Orin Nano

# Register addresses
MODE1 = 0x00
PRESCALE = 0xFE
ALL_LED_ON_L = 0xFA
ALL_LED_OFF_L = 0xFC

# Desired frequency
frequency = 330

def set_pwm_frequency(bus, device_address, frequency):
    # Calculate prescale value for desired frequency
    osc_clock = 25000000  # 25 MHz internal oscillator
    prescale_val = int(round(osc_clock / (4096 * frequency)) - 1)

    # Put device to sleep
    bus.write_byte_data(device_address, MODE1, 0x10)
    
    # Set prescale
    bus.write_byte_data(device_address, PRESCALE, prescale_val)
    
    # Wake up and enable auto-increment
    bus.write_byte_data(device_address, MODE1, 0x80)

def set_all_leds_50_duty_cycle(bus, device_address):
    # Set all LED channels to 50% duty cycle
    bus.write_byte_data(device_address, ALL_LED_ON_L, 0x00)
    bus.write_byte_data(device_address, ALL_LED_ON_L + 1, 0x00)
    bus.write_byte_data(device_address, ALL_LED_OFF_L, 0x00)
    bus.write_byte_data(device_address, ALL_LED_OFF_L + 1, 0x08)  # 2048 counts (50% of 4096)

# Initialize I2C bus
with SMBus(BUS_NUMBER) as bus:
    # Set PWM frequency
    set_pwm_frequency(bus, DEVICE_ADDRESS, frequency)
    
    # Set all LED channels to 50% duty cycle
    set_all_leds_50_duty_cycle(bus, DEVICE_ADDRESS)

    print("All LEDs set to 50% duty cycle at 330 Hz")

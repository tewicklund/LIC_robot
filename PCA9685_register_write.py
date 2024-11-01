from smbus2 import SMBus
import time

bus = SMBus(7)  # Use 1 for i2c-1

# address of PCA9685 on bus 7
device_address = 0x40  

# address and value of mode 1 register
mode_1_address=0x00
mode_1_value=0x00

# address and value of prescaler, used to set pwm frequency
prescale_address=0xfe
prescale_value=0x12

# addresses of PWM duty cycle
led_0_on_address_ls=0x06
led_0_on_address_ms=0x07
led_0_off_address_ls=0x08
led_0_off_address_ms=0x09

# values to write to led 0 as a test, expect 50% duty cycle
led_0_value_ls=0x00
led_0_value_ms=0x10

# Write byte to mode 1 register to put in normal mode
bus.write_byte_data(device_address,mode_1_address,mode_1_value)
time.sleep(0.01)

# Write byte to prescale address to set frequency to 330 Hz
bus.write_byte_data(device_address,prescale_address,prescale_value)
time.sleep(0.01)

# write bytes to led 0 addresses
bus.write_byte_data(device_address,led_0_on_address_ls,led_0_value_ls)
time.sleep(0.01)
bus.write_byte_data(device_address,led_0_on_address_ms,led_0_value_ms)
time.sleep(0.01)
bus.write_byte_data(device_address,led_0_off_address_ls,led_0_value_ls)
time.sleep(0.01)
bus.write_byte_data(device_address,led_0_off_address_ms,led_0_value_ms)
time.sleep(0.01)

# Close the bus connection if done
bus.close()

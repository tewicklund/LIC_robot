import smbus2 as SMBus
import time

bus = SMBus(7)  # Use 1 for i2c-1


device_address = 0x40  
prescale_address=0xfe
prescale_value=0x12

led_0_address_ls=0x06
led_0_address_ms=0x07

# values to write to led 0 as a test, expect 6.25% duty cycle
led_0_value_ls=0xff
led_0_value_ms=0x00


# Write byte to prescale address to set frequency to 330 Hz
bus.write_byte_data(device_address,prescale_address,prescale_value)
time.sleep(0.01)

# write bytes to led 0 addresses
bus.write_byte_data(device_address,led_0_address_ls,led_0_value_ls)
time.sleep(0.01)
bus.write_byte_data(device_address,led_0_address_ms,led_0_value_ms)
time.sleep(0.01)

# Close the bus connection if done
bus.close()

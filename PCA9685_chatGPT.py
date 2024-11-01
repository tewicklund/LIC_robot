from smbus2 import SMBus

# Create an SMBus instance for bus 7
bus = SMBus(7)

# Device and register information
device_address = 0x40  # I2C address of your device
register_address = 0x00  # Register address to write to (example)
data = 0xFF  # Example data to write

# Writing a byte to a register
bus.write_byte_data(device_address, register_address, data)

# Close the bus connection if done
bus.close()

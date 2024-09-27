import time
from smbus2 import SMBus

# I2C address of the BMM150 (default 0x13)
BMM150_I2C_ADDRESS = 0x13

# Register addresses
BMM150_DATA_X_LSB = 0x42
BMM150_DATA_X_MSB = 0x43
BMM150_DATA_Y_LSB = 0x44
BMM150_DATA_Y_MSB = 0x45
BMM150_DATA_Z_LSB = 0x46
BMM150_DATA_Z_MSB = 0x47
BMM150_OPMODE_REG = 0x4C
BMM150_POWER_CTRL_REG = 0x4B
BMM150_PRESET_MODE_REG = 0x4E

# Preset Mode Constants
BMM150_HIGH_ACCURACY_MODE = 0x02  # High accuracy mode setting for the OPMODE register

# Power and mode settings
BMM150_NORMAL_MODE = 0x00
BMM150_POWER_ON = 0x01

def initialize_bmm150(bus):
    # Enable the power control bit
    bus.write_byte_data(BMM150_I2C_ADDRESS, BMM150_POWER_CTRL_REG, BMM150_POWER_ON)
    time.sleep(0.01)  # Delay to allow power-up
    
    # Set to normal mode (OPMODE register)
    bus.write_byte_data(BMM150_I2C_ADDRESS, BMM150_OPMODE_REG, BMM150_NORMAL_MODE)
    time.sleep(0.01)  # Allow sensor to stabilize

    # Set sensor to high accuracy mode (Preset mode register)
    bus.write_byte_data(BMM150_I2C_ADDRESS, BMM150_PRESET_MODE_REG, BMM150_HIGH_ACCURACY_MODE)
    time.sleep(0.01)  # Allow sensor to stabilize in high accuracy mode

def read_bmm150(bus):
    # Read magnetometer data for X, Y, and Z axis
    try:
        x_lsb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_X_LSB)
        x_msb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_X_MSB)
        y_lsb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_Y_LSB)
        y_msb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_Y_MSB)
        z_lsb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_Z_LSB)
        z_msb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_Z_MSB)

        # Combine LSB and MSB to form 16-bit signed values
        x = (x_msb << 8) | x_lsb
        y = (y_msb << 8) | y_lsb
        z = (z_msb << 8) | z_lsb

        # Convert to signed 16-bit integers
        if x > 32767:
            x -= 65536
        if y > 32767:
            y -= 65536
        if z > 32767:
            z -= 65536

        return x, y, z

    except Exception as e:
        print(f"Error reading BMM150 data: {e}")
        return 0, 0, 0

def main():
    # Initialize I2C bus
    bus = SMBus(7)  # Bus 1 is typically used for I2C on Jetson Nano

    # Initialize the BMM150
    initialize_bmm150(bus)

    try:
        while True:
            # Read the sensor data
            x, y, z = read_bmm150(bus)

            # Print the data
            print(f"X: {x}, Y: {y}, Z: {z}")

            # Wait a bit before the next reading
            time.sleep(1)
    except KeyboardInterrupt:
        # Close the bus on exit
        bus.close()

if __name__ == '__main__':
    main()

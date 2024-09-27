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

# Initialization values
BMM150_SLEEP_MODE = 0x01
BMM150_NORMAL_MODE = 0x00
BMM150_POWER_ON = 0x01

def initialize_bmm150(bus):
    # Enable the power control bit
    bus.write_byte_data(BMM150_I2C_ADDRESS, BMM150_POWER_CTRL_REG, BMM150_POWER_ON)
    time.sleep(0.01)  # Delay to allow power-up
    
    # Set the device into normal mode (set OPMODE register)
    bus.write_byte_data(BMM150_I2C_ADDRESS, BMM150_OPMODE_REG, BMM150_NORMAL_MODE)
    time.sleep(0.01)  # Delay to allow sensor to stabilize

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

def take_average_readings(bus, num_samples=100):
    x_total, y_total, z_total = 0, 0, 0
    
    for _ in range(num_samples):
        x, y, z = read_bmm150(bus)
        x_total += x
        y_total += y
        z_total += z
        time.sleep(0.01)  # Short delay between readings to avoid overwhelming the sensor

    # Calculate the average for each axis
    x_avg = x_total / num_samples
    y_avg = y_total / num_samples
    z_avg = z_total / num_samples

    return x_avg, y_avg, z_avg

def main():
    # Initialize I2C bus
    bus = SMBus(1)  # Bus 1 is typically used for I2C on Jetson Nano

    # Initialize the BMM150
    initialize_bmm150(bus)

    try:
        while True:
            # Take average measurements from the sensor
            x_avg, y_avg, z_avg = take_average_readings(bus, num_samples=100)

            # Print the average data
            print(f"Averaged Readings -> X: {x_avg:.2f}, Y: {y_avg:.2f}, Z: {z_avg:.2f}")

            # Wait a bit before the next set of readings
            time.sleep(1)
    except KeyboardInterrupt:
        # Close the bus on exit
        bus.close()

if __name__ == '__main__':
    main()

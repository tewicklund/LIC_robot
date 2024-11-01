import smbus2
import time

# PCA9685 default settings
I2C_BUS = 7  # Jetson Nano's I2C bus
PCA9685_ADDR = 0x40
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE

# Servo settings
SERVO_MIN_PULSE = 666  # µs for -90 degrees
SERVO_MAX_PULSE = 2334  # µs for +90 degrees
SERVO_NEUTRAL_PULSE = 1500  # µs for 0 degrees
FREQUENCY = 330  # Refresh rate for the servo in Hz

# Initialize the I2C bus
bus = smbus2.SMBus(I2C_BUS)

# Functions to configure PCA9685
def set_pwm_freq(freq_hz):
    prescale_val = int(round(25000000.0 / (4096 * freq_hz)) - 1)
    old_mode = bus.read_byte_data(PCA9685_ADDR, MODE1)
    new_mode = (old_mode & 0x7F) | 0x10  # Sleep mode
    bus.write_byte_data(PCA9685_ADDR, MODE1, new_mode)
    bus.write_byte_data(PCA9685_ADDR, PRESCALE, prescale_val)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode)
    time.sleep(0.005)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode | 0xA1)

def set_servo_pulse(channel, pulse_us):
    pulse_length = 1000000 / FREQUENCY  # µs per cycle
    pulse = int(pulse_us / pulse_length * 4096)
    bus.write_byte_data(PCA9685_ADDR, 0x06 + 4 * channel, pulse & 0xFF)
    bus.write_byte_data(PCA9685_ADDR, 0x07 + 4 * channel, pulse >> 8)

def set_servo_angle(channel, angle):
    # Map angle to pulse width
    pulse_width = SERVO_MIN_PULSE + (angle + 90) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180
    set_servo_pulse(channel, pulse_width)

# Initialize PCA9685 and set frequency
set_pwm_freq(FREQUENCY)

# Example usage: Move servo to -90, 0, and +90 degrees
try:
    set_servo_angle(0, -90)  # Set to -90 degrees
    time.sleep(1)
    set_servo_angle(0, 0)  # Set to 0 degrees
    time.sleep(1)
    set_servo_angle(0, 90)  # Set to +90 degrees
    time.sleep(1)
finally:
    bus.close()

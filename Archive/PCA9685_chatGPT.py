import smbus2
import time

# PCA9685 default settings
I2C_BUS = 7  # Jetson Nano's I2C bus
PCA9685_ADDR = 0x40
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

# Servo settings
SERVO_MIN_PULSE = 666  # µs for -90 degrees
SERVO_MAX_PULSE = 2334  # µs for +90 degrees
FREQUENCY = 330  # Refresh rate for the servo in Hz

# Initialize the I2C bus
bus = smbus2.SMBus(I2C_BUS)

# Function to set PCA9685 frequency
def set_pwm_freq(freq_hz):
    prescale_val = int(round(25000000.0 / (4096 * freq_hz)) - 1)
    print(f"Setting frequency to {freq_hz} Hz, prescale value: {prescale_val}")
    old_mode = bus.read_byte_data(PCA9685_ADDR, MODE1)
    new_mode = (old_mode & 0x7F) | 0x10  # Sleep mode
    bus.write_byte_data(PCA9685_ADDR, MODE1, new_mode)  # Go to sleep
    bus.write_byte_data(PCA9685_ADDR, PRESCALE, prescale_val)  # Set prescale
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode)  # Wake up
    time.sleep(0.005)
    bus.write_byte_data(PCA9685_ADDR, MODE1, old_mode | 0xA1)  # Enable auto-increment

# Function to set the PWM pulse width on a given channel
def set_servo_pulse(channel, pulse_us):
    pulse_length = 1000000 / FREQUENCY  # µs per cycle
    pulse = int(pulse_us / pulse_length * 4096)  # Convert to 12-bit scale (0-4095)
    on = 0  # PWM on time (starts immediately)
    off = on + pulse  # PWM off time
    print(f"Setting pulse width: {pulse_us} µs on channel {channel} (on={on}, off={off})")
    bus.write_byte_data(PCA9685_ADDR, LED0_ON_L + 4 * channel, on & 0xFF)
    bus.write_byte_data(PCA9685_ADDR, LED0_ON_L + 4 * channel + 1, on >> 8)
    bus.write_byte_data(PCA9685_ADDR, LED0_ON_L + 4 * channel + 2, off & 0xFF)
    bus.write_byte_data(PCA9685_ADDR, LED0_ON_L + 4 * channel + 3, off >> 8)

# Function to map angle to pulse width and set servo angle
def set_servo_angle(channel, angle):
    pulse_width = SERVO_MIN_PULSE + (angle + 90) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180
    print(f"Setting angle: {angle}° to pulse width: {pulse_width} µs")
    set_servo_pulse(channel, pulse_width)

# Initialize PCA9685 and set frequency
print("Initializing PCA9685...")
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

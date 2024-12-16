import Jetson.GPIO as GPIO
import time

# Pin Definitions
input_pin = 11   # Physical pin 11 (BOARD numbering)
output_pin = 7   # Physical pin 7 (BOARD numbering)

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(input_pin, GPIO.IN)
GPIO.setup(output_pin, GPIO.OUT)

try:
    print("Monitoring input signal...")
    while True:
        # Read input pin state
        input_state = GPIO.input(input_pin)

        # Set output pin based on input state
        GPIO.output(output_pin, input_state)

        # Small delay to prevent excessive CPU usage
        time.sleep(0.01)
except KeyboardInterrupt:
    print("Exiting program.")
finally:
    GPIO.cleanup()
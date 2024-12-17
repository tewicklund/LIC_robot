import Jetson.GPIO as GPIO
import time
import os

# Pin Definitions
input_pin = 11   # Physical pin 11 (BOARD numbering)
output_pin = 7   # Physical pin 7 (BOARD numbering)

button_pressed_loops=0

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(input_pin, GPIO.IN)
GPIO.setup(output_pin, GPIO.OUT)

try:
    print("Monitoring input signal...")
    while True:
        # Read input pin state
        input_state = GPIO.input(input_pin)

        print(button_pressed_loops)
        if input_state:
            button_pressed_loops+=1
        else:
            button_pressed_loops=0
        
        if button_pressed_loops>=10:
            button_pressed_loops=0
            os.system('python3 /home/robotjetson/Code/LIC_robot/qr_grid_navigation.py')


        # Set output pin based on input state
        GPIO.output(output_pin, input_state)

        # Small delay to prevent excessive CPU usage
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting program.")
finally:
    GPIO.cleanup()
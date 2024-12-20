import Jetson.GPIO as GPIO
import time

input_pin = 7   # Physical pin 11 (BOARD numbering)

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(input_pin, GPIO.IN)

prev_switch_state=GPIO.input(input_pin)

while True:
    time.sleep(0.1)
    switch_state=GPIO.input(input_pin)
    print(switch_state)
    # time.sleep(0.1)

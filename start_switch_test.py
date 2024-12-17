import Jetson.GPIO as GPIO
import time

input_pin = 11   # Physical pin 11 (BOARD numbering)

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(input_pin, GPIO.IN)

prev_switch_state=GPIO.input(input_pin)

while True:
    switch_state=GPIO.input(input_pin)

    if switch_state:
        if switch_state != prev_switch_state:
            print("RUNNING TEST")
            prev_switch_state=switch_state
        
        #PUT RUN TEST CODE HERE
        for x in range(10):
            print(x)
            time.sleep(0.1)
    
    else:
        if switch_state != prev_switch_state:
            print("READY TO START TEST")
            prev_switch_state=switch_state


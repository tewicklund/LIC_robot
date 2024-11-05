from functions import *
import Jetson.GPIO as GPIO

i2c_bus = smbus2.SMBus(7)

left_encoder_pin=31
right_encoder_pin=33

num_edges_target=130

left_falling_edge_count=0
right_falling_edge_count=0

left_edges=0
right_edges=0

def count_left_edge(channel):
    global left_edges
    left_edges+=1
    #print(f"Falling edge detected! Count: {left_edges}")

def count_right_edge(channel):
    global right_edges
    right_edges+=1
    #print(f"Falling edge detected! Count: {right_edges}")

GPIO.setmode(GPIO.BOARD)
GPIO.setup(left_encoder_pin, GPIO.IN)
GPIO.setup(right_encoder_pin, GPIO.IN)

GPIO.add_event_detect(left_encoder_pin, GPIO.FALLING, callback=count_left_edge, bouncetime=10)
GPIO.add_event_detect(right_encoder_pin, GPIO.FALLING, callback=count_right_edge, bouncetime=10)




try:
    # Keep the program running to catch the interrupts
    print("Monitoring falling edges on GPIO pins")
    encoder_turn(left_edges,right_edges,num_edges_target,'R',i2c_bus)

except KeyboardInterrupt:
    print("Script terminated by user")

finally:
    # Clean up GPIO settings
    GPIO.cleanup()
    i2c_bus.close()



import pyrealsense2 as rs
import numpy as np
import cv2
import Jetson.GPIO as GPIO
from functions import *


i2c_bus = smbus2.SMBus(7)

#send speeds 0-63 to drive the motors on each side.

#pins for wheel encoders
left_encoder_pin=31
right_encoder_pin=33

#variables to track number of falling edges
left_falling_edge_count=0
right_falling_edge_count=0

#functions called by interrupt to count edges
def count_left_edge():
    global left_edges
    left_edges+=1
    #print(f"Falling edge detected! Count: {left_edges}")

def count_right_edge():
    global right_edges
    right_edges+=1
    #print(f"Falling edge detected! Count: {right_edges}")

#setup encoder pins as inputs
GPIO.setmode(GPIO.BOARD)
GPIO.setup(left_encoder_pin, GPIO.IN)
GPIO.setup(right_encoder_pin, GPIO.IN)

#attach interrupt functions to wheel encoder pins
GPIO.add_event_detect(left_encoder_pin, GPIO.FALLING, callback=count_left_edge, bouncetime=10)
GPIO.add_event_detect(right_encoder_pin, GPIO.FALLING, callback=count_right_edge, bouncetime=10)


# motor speed range, change to make robot run course faster or slower
base_speed=26
max_speed=63
min_speed=1

#pi control variables, set to 0 to disable
angle_p=1
centering_p=0.1
angle_i=0
centering_i=0

# maximum errors for i control
angle_error_sum=0
angle_error_sum_max=400
x_location_error_sum=0
x_location_error_max=400

# minimum ratio of horizontal lines to vertical lines that is recognized as a stop
ratio_limit=0.8

# boolean for loop logic, used for driving over previously acknowledged horizontal lines
horizontal_lines_acknowledged=False

# index variable for keeping track of stop in sequence
stop_num=0

# amount of time spent at each stop, in seconds
stop_time=2

# list of instructions, 'S' means stop at the line, 'R' means make a 90 degree right turn, and 'L' means make a 90 degree left turn
instruction_list=['S','S','S','S','S','S','S','S','S','S','R','S','R',
                  'S','S','S','S','S','S','S','S','S','S','L','S','L',
                  'S','S','S','S','S','S','S','S','S','S','R','S','R']

# list of arm positions to cycle through, just for demo purposes for now
#arm_position_list=['a','b',"c"]


# Initialize the RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline to stream the RGB camera at 640x480
frame_width=640
frame_height=480
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get timestamp for time since last stop
horiztonal_lines_time=time.time()

# Get timestamp for frame counter
frame_time=time.time()

# put sequence in try statement so if anything goes wrong, the finally statement will run
try:
    while True:
        # Wait for a coherent set of frames: color frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert RealSense frame to numpy array (BGR format for OpenCV)
        color_image = np.asanyarray(color_frame.get_data())

        # Apply gaussian blur to image
        kernel_size=(3,3)
        gauss_image=cv2.GaussianBlur(color_image,kernel_size,0)

        # Convert image to HSV
        hsv_image=cv2.cvtColor(gauss_image,cv2.COLOR_BGR2HSV)

        # Apply thresholds to only get blue color
        lower_blue=np.array([50,100,0])
        upper_blue=np.array([160,255,255])
        blue_threshold=cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Apply canny edge detection
        canny_low=200
        canny_high=400
        canny_image=cv2.Canny(blue_threshold,canny_low,canny_high)

        # create a blank copy of frame to overlay angle and hough lines
        line_image=np.copy(hsv_image)*0

        #get frame time to feed to image_to_angle function
        frame_time_elapsed=time.time()-frame_time
        frame_time=time.time()

        # interpret canny edges black and white image to return info about the angle and location of lines seen
        [avg_angle_deg,x_location_avg, horizontal_vertical_ratio,lines_seen]=image_to_angle(canny_image,line_image,frame_time_elapsed)

        # add overlay to frame
        output_image = cv2.addWeighted(hsv_image, 0.8, line_image, 1, 0) 

        # show the frame
        cv2.imshow('Robot Vision', output_image)


        
        #proportional control
        angle_error=avg_angle_deg
        x_location_error=(frame_width/2)-x_location_avg
        right_motor_speed+=int(angle_error*angle_p)
        left_motor_speed-=int(angle_error*angle_p)
        right_motor_speed+=int(x_location_error*centering_p)
        left_motor_speed-=int(x_location_error*centering_p)
        
        #integral control
        angle_error_sum+=angle_error
        x_location_error_sum+=x_location_error
        x_location_error_sum=clamp(x_location_error_sum,0,x_location_error_max)
        angle_error_sum=clamp(angle_error_sum,0,angle_error_sum_max)
        right_motor_speed+=int(angle_error_sum*angle_i)
        left_motor_speed-=int(angle_error_sum*angle_i)
        right_motor_speed+=int(x_location_error_sum*centering_i)
        left_motor_speed-=int(x_location_error_sum*centering_i)


        # clamp motor speeds to max_speed
        right_motor_speed=clamp(right_motor_speed,min_speed,max_speed)
        left_motor_speed=clamp(left_motor_speed,min_speed,max_speed)

        #stop if no lines
        if (not lines_seen):
            drive_motor("L",0,i2c_bus)
            drive_motor("R",0,i2c_bus)
            print("No lines, stopping motors")

        # if no horizontal lines in frame, drive as normal
        if (horizontal_vertical_ratio<ratio_limit and lines_seen):
            drive_motor("L",left_motor_speed,i2c_bus)
            drive_motor("R",right_motor_speed,i2c_bus)
            horizontal_lines_acknowledged=False

        # if new horizontal line encountered, stop for set amount of time
        elif(not horizontal_lines_acknowledged):
            horizontal_lines_acknowledged=True
            drive_motor("L",0,i2c_bus)
            drive_motor("R",0,i2c_bus)

            # let robot come to stop
            time.sleep(stop_time/2)

            # perform turn if instruction is 'R' or 'L'
            if instruction_list[stop_num]=='R' or instruction_list[stop_num]=='L':
                camera_assisted_turn(pipeline,instruction_list[stop_num],lower_blue,upper_blue,i2c_bus)

            # move arm to next position, for demo purposes only
            #move_arm(arm_position_list[stop_num%3],i2c_bus)
            #time.sleep(1)
            #move_arm('z',i2c_bus)

            # rest before continuing
            time.sleep(stop_time/2)

            # increment stop number
            stop_num+=1

            # reset stop time tracking variable
            horiztonal_lines_time=time.time()

            # exit the code if there are no more stops left
            if stop_num>=len(instruction_list):
                print("Course Complete")
                exit()
        
        # if old vertical lines still in frame, keep driving till they are out of frame
        else:
            drive_motor("L",left_motor_speed,i2c_bus)
            drive_motor("R",right_motor_speed,i2c_bus)


        # Break loop with 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()

    # Release the OpenCV window
    cv2.destroyAllWindows()

    # close the i2c bus
    i2c_bus.close()


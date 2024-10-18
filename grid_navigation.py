import pyrealsense2 as rs
import numpy as np
import cv2
from functions import *

i2c_bus = smbus2.SMBus(7)

initialize_bmm150(i2c_bus)

# motor driving and PID variables
base_speed=80
max_speed=90
min_speed=60
angle_p=1
centering_p=0.1
angle_i=0
centering_i=0
ratio_limit=0.8

horizontal_lines_acknowledged=False
angle_error_sum=0
angle_error_sum_max=400
x_location_error_sum=0
x_location_error_max=400
num_stops=2
num_turns=1
stop_num=0
turn_num=0
stop_time=2

# list of instructions, 'S' means stop at the line, and 'T' means turn at the line followed by the direction and the target magnetometer reading
instruction_list=['S','S','S','S','S','S','S','S','S','S']
#instruction_list=['S','S','L']
arm_position_list=['a','b',"c"]
# stop_list=[6,2,5,2,5]
# turn_list=["L","L","R","R","L"]
list_index=0
first_loop=True


# Initialize the RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline to stream the RGB camera at 640x480
frame_width=640
frame_height=480
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

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
        lower_blue=np.array([60,140,0])
        upper_blue=np.array([160,255,255])
        blue_threshold=cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Apply canny edge detection
        canny_low=200
        canny_high=400
        canny_image=cv2.Canny(blue_threshold,canny_low,canny_high)

        #split image into top half (looks for stop line) and bottom half (looks for follow line)
        #top_canny_image=canny_image[0:frame_height/2,:]
        #bottom_canny_image=canny_image[frame_height/2:frame_height,:]

        # create copy of frame to overlay angle and hough lines
        line_image=np.copy(hsv_image)*0

        # set line_image

        [avg_angle_deg,x_location_avg, horizontal_vertical_ratio,lines_seen]=image_to_angle(canny_image,line_image)
        #print("Average Angle:",avg_angle_deg)
        #print("Average X:",x_location_avg)
        #print("Horizontal Ratio:",horizontal_vertical_ratio)

        # add overlay to frame
        output_image = cv2.addWeighted(hsv_image, 0.8, line_image, 1, 0) 

        # show the frame
        cv2.imshow('Robot Vision', output_image)


        
        #control section
        right_motor_speed=base_speed
        left_motor_speed=base_speed

        #p control setup
        angle_error=avg_angle_deg
        x_location_error=(frame_width/2)-x_location_avg
        
        #i control setup
        angle_error_sum+=angle_error
        x_location_error_sum+=x_location_error
        x_location_error_sum=clamp(x_location_error_sum,0,x_location_error_max)
        angle_error_sum=clamp(angle_error_sum,0,angle_error_sum_max)
        
        #proportional control

        right_motor_speed+=int(angle_error*angle_p)
        left_motor_speed-=int(angle_error*angle_p)
        right_motor_speed+=int(x_location_error*centering_p)
        left_motor_speed-=int(x_location_error*centering_p)
        
        #integral control

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
            #print(left_motor_speed,":",right_motor_speed)
            horizontal_lines_acknowledged=False

        # if new horizontal line encountered, stop for set amount of time
        elif(not horizontal_lines_acknowledged):
            horizontal_lines_acknowledged=True
            drive_motor("L",0,i2c_bus)
            drive_motor("R",0,i2c_bus)

            time.sleep(stop_time/2)

            if instruction_list[stop_num]=='R':
                #right_turn(1.4,i2c_bus)
                camera_assisted_turn(pipeline,'R',i2c_bus)
            elif instruction_list[stop_num]=='L':
                camera_assisted_turn(pipeline,'L',i2c_bus)
                #left_turn(1.3,i2c_bus)

            #move_arm(arm_position_list[stop_num%3],i2c_bus)
            #time.sleep(1)
            #move_arm('z',i2c_bus)

            
            time.sleep(stop_time/2)
            stop_num+=1
            if stop_num>=len(instruction_list):
                print("Course Complete")
                exit()
        
        # if old vertical lines still in frame, keep driving as usual till they are out of frame
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

    i2c_bus.close()


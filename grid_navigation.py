import pyrealsense2 as rs
import numpy as np
import cv2
from functions import *

i2c_bus = smbus2.SMBus(1)

# motor driving and PID variables
base_speed=80
max_speed=90
min_speed=60
angle_p=1
centering_p=0.15
angle_i=0.05
p_control=True
i_control=False
ratio_limit=0.8

vertical_lines_seen=False
angle_error_sum=0
angle_error_sum_max=400
num_stops=2
num_turns=1
stop_num=0
turn_num=0
stop_time=5

stop_list=[5,2,5,2,5]
turn_list=["L","L","R","R","L"]
list_index=0


# Initialize the RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline to stream the RGB camera at 640x480
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

time.sleep(5)

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
        lower_blue=np.array([100,150,0])
        upper_blue=np.array([140,255,255])
        blue_threshold=cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Apply canny edge detection
        canny_low=200
        canny_high=400
        canny_image=cv2.Canny(blue_threshold,canny_low,canny_high)

        #split image into top half (looks for stop line) and bottom half (looks for follow line)
        #top_canny_image=canny_image[0:240,:]
        #bottom_canny_image=canny_image[240:480,:]

        # create copy of frame to overlay angle and hough lines
        line_image=np.copy(canny_image)*0

        [avg_angle_deg,x_location_avg, horizontal_vertical_ratio,lines_seen]=image_to_angle(canny_image,line_image)
        #print("Average Angle:",avg_angle_deg)
        #print("Average X:",x_location_avg)
        #print("Horizontal Ratio:",horizontal_vertical_ratio)

        # add overlay to frame
        canny_image = cv2.addWeighted(canny_image, 0.8, line_image, 1, 0) 

        # show the frame
        cv2.imshow('Hough Transform', canny_image)


        
        #proportional control
        right_motor_speed=base_speed
        left_motor_speed=base_speed
        x_location_error=240-x_location_avg
        if p_control:
            right_motor_speed+=int(avg_angle_deg*angle_p)
            left_motor_speed-=int(avg_angle_deg*angle_p)
            if (not vertical_lines_seen):
                right_motor_speed+=int(x_location_error*centering_p)
                left_motor_speed-=int(x_location_error*centering_p)
        
        #integral control
        if i_control:
            right_motor_speed+=int(angle_error_sum*angle_i)
            left_motor_speed-=int(angle_error_sum*angle_i)
        
        #spin motors backward if below threshold (only needed for sharp turns, pursuing different approach)
        # if left_motor_speed<min_speed:
        #     left_motor_speed=-min_speed-(min_speed-left_motor_speed)
        # if right_motor_speed<min_speed:
        #     right_motor_speed=-min_speed-(min_speed-right_motor_speed)


        # clamp motor speeds to max_speed
        if right_motor_speed>max_speed:
            right_motor_speed=max_speed
        if right_motor_speed<-max_speed:
            right_motor_speed=-max_speed

        if left_motor_speed>max_speed:
            left_motor_speed=max_speed
        if left_motor_speed<-max_speed:
            left_motor_speed=-max_speed

        #stop if no lines
        if (not lines_seen):
            drive_motor("L",0,i2c_bus)
            drive_motor("R",0,i2c_bus)
            print("No lines, stopping motors")


        # if no horizontal lines in frame, drive as normal
        if (horizontal_vertical_ratio<ratio_limit):
            drive_motor("L",left_motor_speed,i2c_bus)
            drive_motor("R",right_motor_speed,i2c_bus)
            print(left_motor_speed,":",right_motor_speed)
            vertical_lines_seen=False

        # if new vertical line encountered, stop for set amount of time
        elif(not vertical_lines_seen):
            stop_num+=1
            vertical_lines_seen=True
            drive_motor("L",0,i2c_bus)
            drive_motor("R",0,i2c_bus)
            if stop_num>=stop_list[list_index]:
                if turn_list[list_index]=="L":
                    left_turn(i2c_bus,1.6)
                else:
                    right_turn(i2c_bus,1.6)
                list_index+=1
                if list_index>=len(stop_list):
                    exit()
                else:
                    stop_num=0
            time.sleep(stop_time)
        
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


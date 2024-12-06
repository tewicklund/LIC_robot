import pyrealsense2 as rs
import numpy as np
import cv2
from functions import *

test_name="LIC 1 Nov 18 2024"

# use I2C1 interface on Jetson nano, pins 3 and 5
i2c_bus = smbus2.SMBus(7)

# set to true if you would like the arm to actuate at each stop
minor_motion_control=False
pca_address=0x40
frequency=340

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
ratio_limit=1.6

# boolean for loop logic, used for driving over previously acknowledged horizontal lines
horizontal_lines_acknowledged=False

# index variable for keeping track of stop in sequence
stop_num=0

# amount of time spent at each stop, in seconds
stop_time=3

# limit of ratio of white to black pixels in mask that counts as qr code in frame
#white_ratio_limit=0.02

# exposure time of the camera in microseconds
#exposure_time_us=800

# list of instructions, 'S' means stop at the line, 'R' means make a 90 degree right turn, and 'L' means make a 90 degree left turn
instruction_list=['S','S','S','S','S','S','S','S','S','S','S','R','S','R']#,
                  #'S','S','S','S','S','S','S','S','S','S','L','S','L',
                  #'S','S','S','S','S','S','S','S','S','S','R','S','R']


frame_width, frame_height,pipeline=init_camera(400)

######------MANUAL EXPOSURE ADJUST------######
# # Get the camera device from the pipeline
# device = pipeline.get_active_profile().get_device()

# # Get the RGB camera sensor
# sensor = device.query_sensors()[1]  # Assumes the RGB camera is the second sensor
# if not sensor.supports(rs.option.exposure):
#     print("The connected device does not support manual exposure.")
    

# # Set the exposure time
# sensor.set_option(rs.option.exposure, exposure_time_us)
# print(f"Exposure time set to {exposure_time_us} microseconds.")


# Get timestamp for frame counter
frame_time=time.time()

# get timestamp for accel/decel
#timestamp=time.time()

# assign timestamps for speed changing, in seconds
# accel_time=1
# cruise_time=1.5
# decel_time=2

# # set speeds used by the robot during straightforward navigation
# max_speed=63
# min_speed=1
# cruise_speed=20
# slow_speed=15

# qr_not_found="No QR code found"
# qr_string=qr_not_found
# qr_stop_number=0

# put sequence in try statement so if anything goes wrong, the finally statement will run
try:
    while True:
        
        #get color image from camera
        color_image = get_color_image(pipeline)
        
        # Apply gaussian blur to image
        kernel_size=(3,3)
        gauss_image=cv2.GaussianBlur(color_image,kernel_size,0)

        # Convert image to HSV
        hsv_image=cv2.cvtColor(gauss_image,cv2.COLOR_BGR2HSV)

        # Adjust thresholds to isolate bright (almost white) blue
        lower_blue = np.array([100, 150, 200])  # H: 100-120 for blue, S: High, V: High
        upper_blue = np.array([130, 255, 255])  # Narrow range for bright blue

        #Apply the adjusted thresholds
        blue_threshold = cv2.inRange(hsv_image, lower_blue, upper_blue)


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
        print(f"Horizontal/Vertical Ratio: {horizontal_vertical_ratio}")
        # add overlay to frame
        output_image = cv2.addWeighted(hsv_image, 0.8, line_image, 1, 0) 

        # show the frame
        cv2.imshow('Robot Vision', output_image)
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


import pyrealsense2 as rs
import numpy as np
import cv2
from functions import *

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
ratio_limit=0.8

# boolean for loop logic, used for driving over previously acknowledged horizontal lines
horizontal_lines_acknowledged=False

# index variable for keeping track of stop in sequence
stop_num=0

# amount of time spent at each stop, in seconds
stop_time=2

# exposure time of the camera in microseconds
exposure_time_us=800

# list of instructions, 'S' means stop at the line, 'R' means make a 90 degree right turn, and 'L' means make a 90 degree left turn
instruction_list=['S','S','S','S','S','S','S','S','S','S','S','R','S','R']#,
                  #'S','S','S','S','S','S','S','S','S','S','L','S','L',
                  #'S','S','S','S','S','S','S','S','S','S','R','S','R']


# Initialize the RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline to stream the RGB camera at 640x480
frame_width=640
frame_height=480
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.gyro)

# Start streaming
pipeline.start(config)

# Get the camera device from the pipeline
device = pipeline.get_active_profile().get_device()

# Get the RGB camera sensor
sensor = device.query_sensors()[1]  # Assumes the RGB camera is the second sensor
if not sensor.supports(rs.option.exposure):
    print("The connected device does not support manual exposure.")
    

# Set the exposure time
sensor.set_option(rs.option.exposure, exposure_time_us)
print(f"Exposure time set to {exposure_time_us} microseconds.")


# Get timestamp for frame counter
frame_time=time.time()

# get timestamp for accel/decel
timestamp=time.time()

# assign timestamps for speed changing, in seconds
accel_time=1
cruise_time=1.5
decel_time=2

# set speeds used by the robot during straightforward navigation
max_speed=63
min_speed=1
cruise_speed=40
slow_speed=8

qr_not_found="No QR code found"
qr_string=qr_not_found
qr_stop_number=0

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
        qr_string = read_qr_code(color_image)
        cv2.imshow('Color Image', color_image)


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
        #print(f"Horizontal/Vertical Ratio: {horizontal_vertical_ratio}")
        # add overlay to frame
        output_image = cv2.addWeighted(hsv_image, 0.8, line_image, 1, 0) 

        # show the frame
        #cv2.imshow('Robot Vision', output_image)

        # base speed control, based on elapsed time
        elapsed_time=time.time()-timestamp
        if (elapsed_time<accel_time):
            base_speed=slow_speed+(cruise_speed-slow_speed)*elapsed_time/accel_time
        elif(elapsed_time<cruise_time):
            base_speed=cruise_speed
        elif(elapsed_time<decel_time):
            base_speed=cruise_speed-(cruise_speed-slow_speed)*elapsed_time/decel_time
        else:
            base_speed=slow_speed

        # run at slow speed if next command is a turn
        try:
            next_instruction=instruction_list[stop_num+1]
            if next_instruction != 'S':
                base_speed=slow_speed
        except:
            base_speed=slow_speed

        #start motors turning
        right_motor_speed=base_speed
        left_motor_speed=base_speed

        
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
            drive_motor_exp("L",0,i2c_bus)
            drive_motor_exp("R",0,i2c_bus)
            print("No lines, stopping motors")

        # if no qr code in frame, drive as normal
        elif (qr_string==qr_not_found):
            drive_motor_exp("L",left_motor_speed,i2c_bus)
            drive_motor_exp("R",right_motor_speed,i2c_bus)
            #print(f"Left motor throttle: {left_motor_speed}")
            #print(f"Right motor throttle: {right_motor_speed}\n")
            horizontal_lines_acknowledged=False

        # if new qr code encountered, stop for set amount of time
        elif(elapsed_time>cruise_time):
            
            #set flag for horizontal lines high
            horizontal_lines_acknowledged=True

            #stop motors
            drive_motor_exp("L",0,i2c_bus)
            drive_motor_exp("R",0,i2c_bus)

            #send POST request to database letting it know the robot has arrived at a stop
            epoch_timestamp=int(time.time())
            if qr_string != 'R' and qr_string != 'L' and qr_string != 'S':
                qr_stop_number=int(qr_string)
            else:
                qr_stop_number=0
            arrive_depart="arrive"
            send_POST_request(epoch_timestamp,qr_stop_number,arrive_depart)


            # let robot come to stop
            time.sleep(stop_time/2)

            # perform turn if instruction is 'R' or 'L'
            if qr_string=='R' or qr_string=='L':
                gyro_turn(pipeline,qr_string,i2c_bus)
            
            elif minor_motion_control:
                set_arm_position(i2c_bus,pca_address,frequency,'a')
                time.sleep(2)
                set_arm_position(i2c_bus,pca_address,frequency,'b')
                time.sleep(2)

            # rest before continuing
            time.sleep(stop_time/2)

            #send POST request to database letting it know the robot has departed a stop
            epoch_timestamp=int(time.time())
            qr_stop_number=int(qr_string)
            arrive_depart="depart"
            send_POST_request(epoch_timestamp,qr_stop_number,arrive_depart)

            # increment stop number
            stop_num+=1

            #reset timestamp
            timestamp=time.time()

            # exit the code if there are no more stops left
            if qr_string=='S':
                print("Course Complete")
                exit()
        
        # if old qr still in frame, keep driving till they are out of frame
        else:
            drive_motor_exp("L",left_motor_speed,i2c_bus)
            drive_motor_exp("R",right_motor_speed,i2c_bus)


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


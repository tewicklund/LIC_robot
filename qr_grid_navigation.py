import pyrealsense2 as rs
import numpy as np
import cv2
import os
from functions import *
import Jetson.GPIO as GPIO




####--------------ADJUSTABLE VARIABLES--------------####
test_name="LIC 2 Dec 16 2024"

max_time=10

#pid control variables, set to 0 to disable
angle_p=0.6
centering_p=0.06
angle_i=0
centering_i=0

# minimum ratio of horizontal lines to vertical lines that is recognized as a stop
ratio_limit=0.8

# amount of time spent at each stop, in seconds
stop_time=5
overtime_flag=False
exit_flag=False


# assign timestamps for speed changing, in seconds
accel_time=1
cruise_time=1.5
decel_time=2
go_slow=False

# set speeds used by the robot during straightforward navigation
max_speed=63
min_speed=1
cruise_speed=40
slow_speed=10
left_bias=2

# give the camera a second to initialize before beginning drive sequence
camera_delay_flag=True
resync_NTP=True
####------------END ADJUSTABLE VARIABLES------------####

# Pin Definitions
switch_pin = 11   # Physical pin 11 (BOARD numbering)
major_minor_pin=7

#variable to track how long the button is pressed
prev_start_switch_state=True

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(switch_pin, GPIO.IN)
GPIO.setup(major_minor_pin,GPIO.IN)



# use I2C1 interface on Jetson nano, pins 3 and 5
i2c_bus = smbus2.SMBus(7)

# init servo controller
servo_address=0x40
servo_frequency=340
set_frequency(i2c_bus,servo_address,servo_frequency)

# maximum errors for i control
angle_error_sum=0
angle_error_sum_max=400
x_location_error_sum=0
x_location_error_max=400

# Get timestamp for frame counter
frame_time=time.time()

# get accel_timestamp for accel/decel
accel_timestamp=time.time()

# message to print if no QR code found in frame
qr_not_found="No QR code found"
qr_string=qr_not_found

# put sequence in try statement so if anything goes wrong, the finally statement will run
try:
    while True:
        start_switch_state=GPIO.input(switch_pin)
        major_motion_on=GPIO.input(major_minor_pin)
        if start_switch_state:

            #set flag to show switch was flipped on
            prev_start_switch_state=True

            #update time since departure
            time_since_departure=time.time()-stop_time
                
            
            if camera_delay_flag:
                camera_delay_flag=False
                # init camera
                exposure_time_us=200
                frame_width, frame_height,pipeline=init_camera(exposure_time_us)
                accel_timestamp=time.time()
                time.sleep(1)
            # delay to let camera power on and adjust exposure and sync ntp
            if resync_NTP:
                resync_NTP=False
                # force sync with same NTP server as arduino and HTTP server
                os.system('cat /var/log/syslog | grep systemd-timesyncd')
                time.sleep(1)

                

            #get color image from camera
            color_image = get_color_image(pipeline)

            cropped_image=crop_image(color_image)
            cropped_frame_width=800
            
            # Apply gaussian blur to image
            kernel_size=(3,3)
            gauss_image=cv2.GaussianBlur(cropped_image,kernel_size,0)

            # Convert image to HSV
            hsv_image=cv2.cvtColor(gauss_image,cv2.COLOR_BGR2HSV)

            # Apply thresholds to only get blue color
            lower_blue = np.array([90, 80, 50])
            upper_blue = np.array([130, 255, 255])
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
            elapsed_time=time.time()-accel_timestamp
            if elapsed_time>max_time:
                exit_flag=True
                overtime_flag=True
            if (elapsed_time<accel_time and not go_slow):
                base_speed=slow_speed+(cruise_speed-slow_speed)*elapsed_time/accel_time
            elif(elapsed_time<cruise_time and not go_slow):
                base_speed=cruise_speed
            elif(elapsed_time<decel_time and not go_slow):
                base_speed=cruise_speed-(cruise_speed-slow_speed)*elapsed_time/decel_time
            else:
                base_speed=slow_speed

            #start motors turning
            right_motor_speed=base_speed
            left_motor_speed=base_speed+left_bias

            #proportional control
            angle_error=avg_angle_deg
            x_location_error=(cropped_frame_width/2)-x_location_avg
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



            if exit_flag:
                pass

            #stop if no lines
            elif (not lines_seen):
                drive_motor_exp("L",0,i2c_bus)
                drive_motor_exp("R",0,i2c_bus)
                print("No lines, stopping motors")

            # if no horizontal lines in frame and max time hasn't elapsed, drive as normal
            elif horizontal_vertical_ratio<ratio_limit:
                drive_motor_exp("L",left_motor_speed,i2c_bus)
                drive_motor_exp("R",right_motor_speed,i2c_bus)

            # if new lines code encountered, stop for set amount of time
            elif elapsed_time>cruise_time:

                #stop motors
                drive_motor_exp("L",0,i2c_bus)
                drive_motor_exp("R",0,i2c_bus)

                #repeatedly check for qr code
                qr_int=99
                qr_string=qr_not_found
                qr_timestamp=time.time()
                while qr_string==qr_not_found:
                    print("Looking for QR code...")
                    color_image = get_color_image(pipeline)
                    #cv2.imshow('Robot Vision', color_image)
                    qr_string=read_qr_code(color_image)
                    qr_elapsed_time=time.time()-qr_timestamp
                    if qr_elapsed_time>max_time:
                        exit_flag=True
                        overtime_flag=True
                        qr_string='99'

                    # Break loop with 'q' key
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
                
                print("QR String:",qr_string)

                #parse QR code as an integer
                try:
                    qr_int=int(qr_string)
                    # go slow if stop is right before a turn (multiple of ten) but go normal speed if stop number is zero
                    go_slow=qr_int%10==0 and qr_int !=0
                except:
                    pass

                #send POST request to database letting it know the robot has arrived at a stop
                epoch_timestamp=int(time.time())
                arrive_depart="arrive"
                send_POST_request(test_name,epoch_timestamp,qr_string,arrive_depart)
                    
                
                # perform turn if instruction is 'R' or 'L', go slow on next one
                if qr_string=='R' or qr_string=='L':
                    turn_complete=gyro_turn(pipeline,qr_string,i2c_bus,max_time)
                    go_slow=True

                    if not turn_complete:
                        exit_flag=True
                        overtime_flag=True
                
                elif qr_string=='S':
                    exit_flag=True

                
                if not exit_flag:
                    time.sleep(stop_time/2)

                    if not major_motion_on and qr_int != 99:
                        print('starting arm movement')
                        epoch_timestamp=int(time.time())
                        arrive_depart="start_arm"
                        send_POST_request(test_name,epoch_timestamp,qr_string,arrive_depart)
                        set_arm_position(i2c_bus,servo_address,servo_frequency,'a')
                        time.sleep(2)
                        set_arm_position(i2c_bus,servo_address,servo_frequency,'b')
                        print('arm movement done')
                        time.sleep(2)
                        epoch_timestamp=int(time.time())
                        arrive_depart="end_arm"
                        send_POST_request(test_name,epoch_timestamp,qr_string,arrive_depart)

                    # rest before continuing
                    time.sleep(stop_time/2)

                    #send POST request to database letting it know the robot has departed a stop
                    epoch_timestamp=int(time.time())
                    arrive_depart="depart"
                    send_POST_request(test_name,epoch_timestamp,qr_string,arrive_depart)
                    
                    #reset accel_timestamp
                    accel_timestamp=time.time()


            # if old lines still in frame, keep driving till they are out of frame
            else:
                drive_motor_exp("L",left_motor_speed,i2c_bus)
                drive_motor_exp("R",right_motor_speed,i2c_bus)

            if exit_flag:
                exit_flag=False
                #resync_NTP=True
                if overtime_flag:
                    print("ERROR: robot got lost")
                    overtime_flag=False
                else:
                    print("Course Complete")
                print('Return switch to STOP position')
                #cv2.destroyAllWindows()
                while start_switch_state:
                    start_switch_state=GPIO.input(switch_pin)
                    time.sleep(0.1)


            # Break loop with 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        #if the switch in the STOP position, print something once then wait
        else:
            time.sleep(0.1)
            #reset accel_timestamp
            go_slow=True
            accel_timestamp=time.time()
            if prev_start_switch_state==True:
                prev_start_switch_state=False
                print('Ready to begin next test...')
            

finally:
    # Stop streaming
    pipeline.stop()

    # Release the OpenCV window
    cv2.destroyAllWindows()

    # close the i2c bus
    i2c_bus.close()
import cv2
import numpy as np
import smbus2
import time
import pyrealsense2 as rs
from pyzbar.pyzbar import decode
import requests

def crop_image(color_image):
    # Original image dimensions
    image_height, image_width, _ = color_image.shape

    # Desired crop dimensions
    crop_width = 800
    crop_height = 400

    # Calculate the center of the image
    center_x = image_width // 2
    center_y = image_height // 2

    # Calculate the crop box
    start_x = center_x - crop_width // 2
    end_x = center_x + crop_width // 2
    start_y = center_y - crop_height // 2
    end_y = center_y + crop_height // 2

    # Perform the crop
    cropped_image = color_image[start_y:end_y, start_x:end_x]
    return cropped_image

def init_camera(exposure_time_us):
    # Initialize the RealSense pipeline
    pipeline = rs.pipeline()
    frame_width=1280
    frame_height=800
    config = rs.config()
    config.enable_stream(rs.stream.gyro)
    config.enable_stream(rs.stream.color, frame_width, frame_height, rs.format.bgr8, 30)
    pipeline.start(config)

    # Get the camera device from the pipeline
    device = pipeline.get_active_profile().get_device()

    # Get the RGB camera sensor
    sensor = device.query_sensors()[1]

    # Set the exposure time
    sensor.set_option(rs.option.exposure, exposure_time_us)
    return frame_width, frame_height, pipeline

def get_color_image(pipeline):
    color_frame=None
    while not color_frame:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

    # Convert RealSense frame to numpy array (BGR format for OpenCV)
    color_image = np.asanyarray(color_frame.get_data())
    return color_image

def send_POST_request(test_name,epoch_timestamp,stop_number,arrive_depart):
    # Set the URL for the HTTP POST request
    url = "http://192.168.11.123:8080/robot"  # office ip

    # Define the data to send in the POST request
    data = {
        "test_name": test_name,
        "epoch_timestamp": epoch_timestamp,  
        "stop_number": stop_number,                 
        "arrive_depart": arrive_depart
    }

    try:
        # Send the POST request
        response = requests.post(url, json=data)
        
        # Check for a successful response
        if response.status_code == 200:
            print("Request successful:", response.json())
        else:
            print("Request failed with status code:", response.status_code)
            print("Response:", response.text)

    except requests.exceptions.RequestException as e:
        print("An error occurred:", e)

# function to read qr codes
def read_qr_code(color_image: np.ndarray) -> str:
    # Convert the image to grayscale (QR detection works better in grayscale)
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
    # Decode any QR codes found in the image
    decoded_objects = decode(gray_image)
    
    # If QR codes were found, return the text of the first one
    if decoded_objects:
        qr_text = decoded_objects[0].data.decode("utf-8")
        return qr_text
    else:
        return "No QR code found"

def set_arm_position(bus,address,frequency,pos_char):

    angle_deg=75

    if pos_char=='a':

        set_servo_angle(bus,address,frequency,1,0)

        for x in range(angle_deg+1):
            set_servo_angle(bus,address,frequency,0,-x)
            print(-x)
            time.sleep(0.01)
        
        time.sleep(2)

        for x in range(angle_deg+1):
            set_servo_angle(bus,address,frequency,0,x-angle_deg)
            print(x-angle_deg)
            time.sleep(0.01)
        time.sleep(2)


    elif pos_char=='b':
        set_servo_angle(bus,address,frequency,0,0)
        
        for x in range(angle_deg+1):
            set_servo_angle(bus,address,frequency,1,-x)
            print(-x)
            time.sleep(0.01)
        
        time.sleep(2)
        
        for x in range(angle_deg+1):
            set_servo_angle(bus,address,frequency,1,x-angle_deg)
            print(x-angle_deg)
            time.sleep(0.01)
        time.sleep(2)

    elif pos_char=='c':
        set_servo_angle(bus,address,frequency,1,0)

        for x in range(angle_deg+1):
            set_servo_angle(bus,address,frequency,0,x)
            print(x)
            time.sleep(0.01)
        
        time.sleep(2)
        
        for x in range(angle_deg+1):
            set_servo_angle(bus,address,frequency,0,angle_deg-x)
            print(angle_deg-x)
            time.sleep(0.01)
        time.sleep(2)

    else:
        set_servo_angle(bus,address,frequency,0,0)
        set_servo_angle(bus,address,frequency,1,0)
    
def set_servo_angle(bus,address,frequency,channel,angle):

    # compute the on time register values from the angle
    on_time_us=9.27*angle+1500
    on_time_register_value=int(on_time_us*4096/(1000000/frequency))

    # set on timestamp to zero
    bus.write_byte_data(address, 6+4*channel, 0)
    bus.write_byte_data(address, 7+4*channel, 0)

    # set off timestamp to on_time integer
    bus.write_byte_data(address, 8+4*channel, on_time_register_value & 0xFF)
    bus.write_byte_data(address, 9+4*channel, on_time_register_value>>8)
    
    

def set_frequency(bus,address,frequency):

    prescale=int(25000000/(4096*frequency)+1)
    # Set sleep bit to 1
    mode1 = bus.read_byte_data(address, 0)
    mode1 = mode1 | 0x10
    bus.write_byte_data(address, 0, mode1)

    # Calculate prescale value
    bus.write_byte_data(address, 0xFE, prescale)

    # Reset back sleep bit to operational mode
    mode1 = mode1 & ~0x10
    bus.write_byte_data(address, 0, mode1)
    time.sleep(1)
    bus.write_byte_data(address, 1, 4)

def image_to_angle(image, overlay, frame_time_elapsed):
    # parameters for hough lines transform
    min_intersections=10
    min_line_length=5
    max_line_gap=5
    hough_lines = cv2.HoughLinesP(image, 1, np.pi / 180, min_intersections, np.array([]), min_line_length, max_line_gap)

    # variables for hough_lines loop
    angle_total_deg=0
    angle_avg_deg=0
    total_length=0
    x_location_sum=0
    x_location_avg=0
    horizontal_sum=0
    vertical_sum=0
    num_horizontal_lines=0
    num_vertical_lines=0
    if hough_lines is not None:
        for line in hough_lines:
            for x1,y1,x2,y2 in line:

                #determine line angle
                if (y2!=y1):
                    line_angle=np.arctan((x2-x1)/(y2-y1))
                else:
                    line_angle=np.pi/2

                #convert line angle to degrees
                line_angle_deg=line_angle*180/np.pi
                line_angle_deg_mag=abs(line_angle_deg)
                line_length=np.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

                #if the line is horizontal or close to horizontal, only add length to horizontal sum
                if line_angle_deg_mag>60:
                    horizontal_sum+=line_length
                    num_horizontal_lines+=1

                    #add red line to overlay to represent horizontal stop line
                    cv2.line(overlay,(x1,y1),(x2,y2),(0,0,255),10)

                # if lane is close to vertical, recognize it as a guide line
                else: 
                    num_vertical_lines+=1
                    #add green line to overlay to represent guide line
                    cv2.line(overlay,(x1,y1),(x2,y2),(0,255,0),10)

                    line_angle_weighted=line_angle_deg*line_length
                    x_location_sum+=x1+x2
                    vertical_sum+=line_length
                    angle_total_deg+=line_angle_weighted
                    total_length=total_length+line_length
                
                
        #prevent average angle from being infinity
        if total_length==0:
            total_length=1
        
        #compute average angle
        angle_avg_deg=angle_total_deg/total_length
        angle_avg_deg_rounded=round(angle_avg_deg,2)
        if num_vertical_lines==0:
            x_location_avg=240
        else:
            x_location_avg=x_location_sum/(2*num_vertical_lines)
        horizontal_vertical_ratio=horizontal_sum/vertical_sum

        #display this average angle value on frame
        font = cv2.FONT_HERSHEY_SIMPLEX
        angle_text=str(angle_avg_deg_rounded)
        location_text=str(x_location_avg)
        cv2.putText(overlay, angle_text, (50, 50), font, 1, (255,255,255), 2)
        cv2.putText(overlay, location_text, (50, 100), font, 1, (255,255,255), 2)

        #compute and display framerate
        if frame_time_elapsed != 0:
            framerate=1/frame_time_elapsed
        else:
            framerate=0
        framerate_text=str(framerate)+" fps"
        cv2.putText(overlay, framerate_text, (50, 150), font, 1, (255,255,255), 2)

        #return 4 variables: angle, x location of line, horizontal/vertical ratio, and lines_seen boolean
        return angle_avg_deg, x_location_avg, horizontal_vertical_ratio,True
    
    #if no lines seen, return some default values
    else:
        return 0.1234,240,0.1,False
    

def drive_motor_exp(side,speed,bus):
    i2c_address=0x08
    side_bit=(side=='R')
    forward_bit=(speed>0)
    if not (0 <= abs(speed) <= 63):
        raise ValueError("The number must be a 6-bit integer (0-63).")
    #print("Value is good!")
    byte_to_send=(side_bit<<7) | (forward_bit<<6) | (abs(int(speed)) & 0x3F)
    #print("Byte to send ready")
    bus.write_byte(i2c_address, byte_to_send)
    #print("byte sent on i2c bus")
    time.sleep(0.01)  # Small delay between bytes for stability
    
def clamp(variable, min_value, max_value):
    if variable>=0:
        if variable>max_value:
            variable=max_value
        elif variable<min_value:
            variable=min_value
    
    else:
        if variable<-max_value:
            variable=-max_value
        elif variable>-min_value:
            variable=-min_value
    return variable

def gyro_turn(pipeline,direction,i2c_bus,max_time,angle_magnitude_radians):
    # timing used for integration
    start_time=time.time()
    time_since_start=0
    timestamp=start_time

    turn_complete=False

    # variables to store angle position info
    radians_turned=0
    error=0


    # speed settings for the motors
    base_speed=40
    min_speed=15
    p=12.5

    try:
        if direction=='R':
            target_radians=(angle_magnitude_radians)
        else:
            target_radians=-1*(angle_magnitude_radians)

        while abs(radians_turned)<abs(target_radians) and time_since_start<=max_time:
            time_since_start=time.time()-start_time
            # Get frameset of motion data
            frames = pipeline.wait_for_frames()

            # Get gyroscope data
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            

            if gyro_frame:
                # Extract gyroscope data (x, y, z)
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                gx, gy, gz = gyro_data.x, gyro_data.y, gyro_data.z

                # Print the accelerometer and gyroscope values
                # print(f"Gyroscope: x={gx:.3f}, y={gy:.3f}, z={gz:.3f}")

                # update the amount of radians turned so far
                time_elapsed=time.time()-timestamp
                timestamp=time.time()
                radians_turned+=gz*time_elapsed
                #print(f"Radians turned: {radians_turned:.3f}")

                #adjust motor speeds based on radians turned
                error=target_radians-radians_turned
                motor_speed=error*p
                
                #make sure motor speed doesn't drop too low
                motor_speed=clamp(motor_speed,min_speed,base_speed)
                print(f"Speed: {motor_speed}")
                
                #drive motors to make the turn
                drive_motor_exp('L',motor_speed,i2c_bus)
                drive_motor_exp('R',-motor_speed,i2c_bus)
            
            # Delay to reduce CPU load
            # time.sleep(0.1

        if time_since_start<max_time:
            print("Turn Complete")
            turn_complete=True
        else:
            print("ERROR: turn could not be completed")
            turn_complete=False
            drive_motor_exp('L',0,i2c_bus)
            drive_motor_exp('R',0,i2c_bus)
        
        
    except Exception as e:
        print(f"Error occurred: {e}")
    
    return turn_complete
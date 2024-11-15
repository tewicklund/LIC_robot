import cv2
import numpy as np
import smbus2
import time
import pyrealsense2 as rs
from pyzbar.pyzbar import decode
import requests

def calculate_white_ratio(mask):
    """
    Calculate the ratio of white pixels in a binary mask.
    
    Parameters:
        mask (numpy.ndarray): Binary image from cv2.inRange(), with white pixels (255) and black pixels (0).
    
    Returns:
        float: Ratio of white pixels to total pixels (value between 0 and 1).
    """
    if not isinstance(mask, np.ndarray):
        raise ValueError("Input mask must be a numpy array.")

    # Ensure the mask is binary (values should be 0 or 255)
    unique_values = np.unique(mask)
    if not np.all(np.isin(unique_values, [0, 255])):
        raise ValueError("Input mask must be binary (values 0 or 255).")

    # Count white pixels (value 255)
    white_pixels = np.sum(mask == 255)
    total_pixels = mask.size

    # Calculate and return the ratio
    white_ratio = white_pixels / total_pixels
    return white_ratio

def get_color_image(pipeline):
    color_frame=None
    while not color_frame:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

    # Convert RealSense frame to numpy array (BGR format for OpenCV)
    color_image = np.asanyarray(color_frame.get_data())
    return color_image

def send_POST_request(epoch_timestamp,stop_number,arrive_depart):
    # Set the URL for the HTTP POST request
    url = "http://192.168.4.6:8080/robot"  # Replace with the actual endpoint

    # Define the data to send in the POST request
    data = {
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

    angle_deg=80

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
    
def move_arm(character,bus):
    i2c_address=0x09
    data_byte=ord(character)
    bus.write_byte(i2c_address, data_byte)
    time.sleep(1) #delay to allow arm time to move

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
    
    

def drive_motor(side,speed,bus):
    i2c_address=0x08
    message="U"+side+":"
    if speed<0:
        message+="-"
    else:
        message+="+"
    if abs(speed)<100:
        message+="0"
    if abs(speed)<10:
        message+="0"
    message+=str(abs(speed))+"V"


    data_bytes = [ord(char) for char in message]

    # Send each byte
    for byte in data_bytes:
        bus.write_byte(i2c_address, byte)
        time.sleep(0.01)  # Small delay between bytes for stability

#turning code, needs to be replaced when magnetometer comes in
def left_turn(turn_time,bus):
    start_time=time.time()

    while time.time()-start_time<turn_time:
        drive_motor('L',-80,bus)
        drive_motor('R',80,bus)


def camera_assisted_turn(pipeline,direction,lower_blue,upper_blue,bus):
    #start the turn with 2 seconds of blind turning
    turn_power=80
    turn_time=2
    start_time=time.time()
    while time.time()-start_time<turn_time:
        if direction=='L':
            drive_motor('L',-1*turn_power,bus)
            drive_motor('R',turn_power,bus)
        elif direction=='R':
            drive_motor('L',turn_power,bus)
            drive_motor('R',-1*turn_power,bus)

    line_straight=False
    while not line_straight:
        if direction=='L':
            drive_motor('L',-1*turn_power,bus)
            drive_motor('R',turn_power,bus)
        elif direction=='R':
            drive_motor('L',turn_power,bus)
            drive_motor('R',-1*turn_power,bus)
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
        blue_threshold=cv2.inRange(hsv_image, lower_blue, upper_blue)
        #cv2.imshow("blue mask",blue_threshold)
        
        # Apply canny edge detection
        canny_low=200
        canny_high=400
        canny_image=cv2.Canny(blue_threshold,canny_low,canny_high)

        # create copy of frame to overlay angle and hough lines
        line_image=np.copy(hsv_image)*0

        [avg_angle_deg,x_location_avg, horizontal_vertical_ratio,lines_seen]=image_to_angle(canny_image,line_image)

        # add overlay to frame
        output_image = cv2.addWeighted(color_image, 0.8, line_image, 1, 0) 

        # show the frame
        cv2.imshow('Robot Vision', output_image)

        if avg_angle_deg>-5 and avg_angle_deg<5 and avg_angle_deg != 0.1234:
            line_straight=True



        # Break loop with 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    drive_motor("L",0,bus)
    drive_motor("R",0,bus)

#turning code, needs to be replaced when magnetometer comes in
def right_turn(turn_time,bus):
    start_time=time.time()

    while time.time()-start_time<turn_time:
        drive_motor('L',80,bus)
        drive_motor('R',-80,bus)

# def perform_turn(turn_string,bus):
#     turn_direction=turn_string[1]
#     target_angle=int(turn_string[3:6])
#     print("Turning",turn_direction," till angle is",target_angle)
#     direction_reading=read_bmm150(bus)

#     while abs(direction_reading-target_angle)>10:
#         print(direction_reading)
#         if turn_direction=='L':
#             drive_motor('L',-80,bus)
#             drive_motor('R',80,bus)
#         elif turn_direction=='R':
#             drive_motor('L',80,bus)
#             drive_motor('R',-80,bus)
#         direction_reading=read_bmm150(bus)


def initialize_bmm150(bus):
    # I2C address of the BMM150 (default 0x13)
    BMM150_I2C_ADDRESS = 0x13
    BMM150_OPMODE_REG = 0x4C
    BMM150_POWER_CTRL_REG = 0x4B
    BMM150_XY_REP_REG = 0x51
    BMM150_Z_REP_REG = 0x52

    # Preset Mode Constants
    BMM150_HIGH_ACCURACY_XY = 0x17  # High accuracy mode setting for the OPMODE register
    BMM150_HIGH_ACCURACY_Z = 0x29  # High accuracy mode setting for the OPMODE register

    # Power and mode settings
    BMM150_NORMAL_MODE = 0x00
    BMM150_POWER_ON = 0x01
    # Enable the power control bit
    bus.write_byte_data(BMM150_I2C_ADDRESS, BMM150_POWER_CTRL_REG, BMM150_POWER_ON)
    time.sleep(0.01)  # Delay to allow power-up
    
    # Set to normal mode (OPMODE register)
    bus.write_byte_data(BMM150_I2C_ADDRESS, BMM150_OPMODE_REG, BMM150_NORMAL_MODE)
    time.sleep(0.01)  # Allow sensor to stabilize

    # Set sensor to high accuracy mode (Preset mode register)
    bus.write_byte_data(BMM150_I2C_ADDRESS, BMM150_XY_REP_REG, BMM150_HIGH_ACCURACY_XY)
    bus.write_byte_data(BMM150_I2C_ADDRESS, BMM150_Z_REP_REG, BMM150_HIGH_ACCURACY_Z)
    time.sleep(0.01)  # Allow sensor to stabilize in high accuracy mode

def read_bmm150(bus):
    # I2C address of the BMM150 (default 0x13)
    BMM150_I2C_ADDRESS = 0x13

    # Register addresses
    BMM150_DATA_X_LSB = 0x42
    BMM150_DATA_X_MSB = 0x43
    BMM150_DATA_Y_LSB = 0x44
    BMM150_DATA_Y_MSB = 0x45
    BMM150_DATA_Z_LSB = 0x46
    BMM150_DATA_Z_MSB = 0x47
    
    # Read magnetometer data for X, Y, and Z axis
    try:
        x_lsb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_X_LSB)
        x_msb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_X_MSB)
        y_lsb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_Y_LSB)
        y_msb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_Y_MSB)
        z_lsb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_Z_LSB)
        z_msb = bus.read_byte_data(BMM150_I2C_ADDRESS, BMM150_DATA_Z_MSB)

        # Combine LSB and MSB to form 16-bit signed values
        x = (x_msb << 8) | x_lsb
        y = (y_msb << 8) | y_lsb
        z = (z_msb << 8) | z_lsb

        # Convert to signed 16-bit integers
        if x > 32767:
            x -= 65536
        if y > 32767:
            y -= 65536
        if z > 32767:
            z -= 65536

        return z

    except Exception as e:
        print(f"Error reading BMM150 data: {e}")
        return 0
    
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

def gyro_turn(pipeline,direction,i2c_bus):
    # timing used for integration
    timestamp=time.time()

    # variables to store angle position info
    radians_turned=0
    error=0


    # speed settings for the motors
    base_speed=20
    min_speed=5
    p=12.5

    try:
        if direction=='R':
            target_radians=(np.pi/2)
        else:
            target_radians=-1*(np.pi/2)

        while abs(radians_turned)<abs(target_radians):
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

        print("Turn Complete")
        drive_motor_exp('L',0,i2c_bus)
        drive_motor_exp('R',0,i2c_bus)
        
    except Exception as e:
        print(f"Error occurred: {e}")
        

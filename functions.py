import cv2
import numpy as np
import smbus2
import time

def image_to_angle(image, overlay):
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

        #return 4 variables: angle, x location of line, horizontal/vertical ratio, and lines_seen boolean
        return angle_avg_deg, x_location_avg, horizontal_vertical_ratio,True
    
    #if no lines seen, return some default values
    else:
        return 0,240,0.1,False

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
def left_turn(bus,i2c_address):
    message="Uturn:lV"
    data_bytes = [ord(char) for char in message]

    # Send each byte
    for byte in data_bytes:
        bus.write_byte(i2c_address, byte)
        time.sleep(0.01)  # Small delay between bytes for stability

#turning code, needs to be replaced when magnetometer comes in
def right_turn(bus,i2c_address):
    message="Uturn:rV"
    data_bytes = [ord(char) for char in message]

    # Send each byte
    for byte in data_bytes:
        bus.write_byte(i2c_address, byte)
        time.sleep(0.01)  # Small delay between bytes for stability

def perform_turn(turn_string,bus):
    turn_direction=turn_string[1]
    target_angle=int(turn_string[3:6])
    print("Turning",turn_direction," till angle is",target_angle)
    direction_reading=read_bmm150(bus)

    while abs(direction_reading-target_angle)>10:
        print(direction_reading)
        if turn_direction=='L':
            drive_motor('L',-80,bus)
            drive_motor('R',80,bus)
        elif turn_direction=='R':
            drive_motor('L',80,bus)
            drive_motor('R',-80,bus)
        direction_reading=read_bmm150(bus)


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
            variable=min_value
    return variable

        
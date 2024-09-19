import cv2
import numpy as np
import smbus2
import time

def image_to_angle(image, overlay):
    # parameters for hough lines transform
    min_intersections=10
    min_line_length=10
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
    if hough_lines is not None:
        for line in hough_lines:
            for x1,y1,x2,y2 in line:
                #add line to overlay for debugging
                cv2.line(overlay,(x1,y1),(x2,y2),(255,0,0),10)

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

                # if lane is close to vertical, recognize it as a guide line
                else: 
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
        x_location_avg=x_location_sum/(2*len(hough_lines))
        horizontal_vertical_ratio=horizontal_sum/vertical_sum

        #display this average angle value on frame
        font = cv2.FONT_HERSHEY_SIMPLEX
        Text=str(angle_avg_deg)
        cv2.putText(overlay, Text, (50, 50), font, 1, (255,255,255), 2)

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
def left_turn(bus,turn_time):
    start_time=time.time()
    while(time.time()-start_time<turn_time):
        drive_motor("L",-80,bus)
        drive_motor("R",80,bus)
    drive_motor("L",0,bus)
    drive_motor("R",0,bus)

#turning code, needs to be replaced when magnetometer comes in
def right_turn(bus,turn_time):
    start_time=time.time()
    while(time.time()-start_time<turn_time):
        drive_motor("L",80,bus)
        drive_motor("R",-80,bus)
    drive_motor("L",0,bus)
    drive_motor("R",0,bus)


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

        
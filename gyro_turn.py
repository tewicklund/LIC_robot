import pyrealsense2 as rs
import time
import numpy as np
from functions import *

i2c_bus = smbus2.SMBus(7)

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable the motion sensor stream
config.enable_stream(rs.stream.gyro)

# Start streaming with the specified configuration
pipeline.start(config)

# timing used for integration
timestamp=time.time()

# variables to store angle position info
radians_turned=0
error=0
target_radians=np.pi/2

# speed settings for the motors
current_speed=0
base_speed=20
min_speed=5
p=12.5
direction=input("Enter direction char (L or R): ")

try:
    if direction=="R":
        while radians_turned<target_radians:
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
                
                #drive motors to make the turn
                drive_motor_exp('L',motor_speed,i2c_bus)
                drive_motor_exp('R',-motor_speed,i2c_bus)
            
            # Delay to reduce CPU load
            # time.sleep(0.1)
        
        drive_motor_exp('L',0,i2c_bus)
        drive_motor_exp('R',0,i2c_bus)
    else:
        while radians_turned>target_radians:
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
                
                #drive motors to make the turn
                drive_motor_exp('L',motor_speed,i2c_bus)
                drive_motor_exp('R',-motor_speed,i2c_bus)
            
            # Delay to reduce CPU load
            # time.sleep(0.1)
        
        drive_motor_exp('L',0,i2c_bus)
        drive_motor_exp('R',0,i2c_bus)
    
except KeyboardInterrupt:
    print("Stopping data capture...")

finally:
    # Stop the pipeline
    pipeline.stop()

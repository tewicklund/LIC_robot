import pyrealsense2 as rs
import time
from functions import *

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable the motion sensor stream
#config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)

# Start streaming with the specified configuration
pipeline.start(config)

timestamp=time.time()
radians_turned=0

try:
    while True:
        # Get frameset of motion data
        frames = pipeline.wait_for_frames()

        # Get accelerometer and gyroscope data
        #accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        if gyro_frame:# and accel_frame:
            # Extract accelerometer data (x, y, z)
            #accel_data = accel_frame.as_motion_frame().get_motion_data()
            #ax, ay, az = accel_data.x, accel_data.y, accel_data.z

            # Extract gyroscope data (x, y, z)
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()
            gx, gy, gz = gyro_data.x, gyro_data.y, gyro_data.z

            # Print the accelerometer and gyroscope values
            #print(f"Accelerometer: x={ax:.3f}, y={ay:.3f}, z={az:.3f}")
            print(f"Gyroscope: x={gx:.3f}, y={gy:.3f}, z={gz:.3f}")

            time_elapsed=time.time()-timestamp
            radians_turned+=gz*time_elapsed
            print(f"Radians turned: {radians_turned:.3f}")

        
        # Delay to reduce CPU load
        #time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping data capture...")

finally:
    # Stop the pipeline
    pipeline.stop()

import pyrealsense2 as rs
import time

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable the motion sensor stream
config.enable_stream(rs.stream.accel)

# Start streaming with the specified configuration
pipeline.start(config)

timestamp=time.time()

velocity=0

position=0

frame_num=0

num_cal_frames=100

cal_frames=[]

try:
    while True:
        # Get frameset of motion data
        frames = pipeline.wait_for_frames()
        frame_num+=1

        # Get accelerometer and gyroscope data
        accel_frame = frames.first_or_default(rs.stream.accel)

        if accel_frame:
            # Extract accelerometer data (x, y, z)
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            ax, ay, az = accel_data.x, accel_data.y, accel_data.z

            time_elapsed=time.time()-timestamp

            #add readings to cal list if applicable
            if frame_num<num_cal_frames:
                cal_frames.append(ay)
                print(f"Calibrating, {frame_num}/{num_cal_frames}")
            else:
                velocity+=(ay-sum(cal_frames)/len(cal_frames))*time_elapsed
                position+=velocity*time_elapsed
                print(f"Acceleration: {ay}")
                print(f"Velocity: {velocity}")
                print(f"Position: {position}")
            
            timestamp=time.time()
            
            

            # Print the accelerometer and gyroscope values
            #print(f"Accelerometer: x={ax:.3f}, y={ay:.3f}, z={az:.3f}")


        # Delay to reduce CPU load
        #time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping data capture...")

finally:
    # Stop the pipeline
    pipeline.stop()

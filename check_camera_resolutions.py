import pyrealsense2 as rs

# Create a context and get a list of connected devices
pipeline = rs.pipeline()
config = rs.config()

# Enumerate all supported streams and configurations
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

# Loop through available sensors on the device
for sensor in device.query_sensors():
    if sensor.is_streaming():
        sensor.stop()  # Stop the sensor if it's currently streaming

    print(f"Sensor: {sensor.get_info(rs.camera_info.name)}")
    for profile in sensor.get_stream_profiles():
        # Filter for color stream
        if profile.stream_type() == rs.stream.color:
            v = profile.as_video_stream_profile()
            print(f"Resolution: {v.width()}x{v.height()}, Format: {v.format()}, FPS: {v.fps()}")

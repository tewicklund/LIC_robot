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

gyro_turn(pipeline,'R',i2c_bus)
time.sleep(2)
gyro_turn(pipeline,'L',i2c_bus)
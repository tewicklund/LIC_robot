import pyrealsense2 as rs
import numpy as np
import cv2

# Initialize the RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline to stream the RGB camera at 640x480
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
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
        lower_blue=np.array([100,150,0])
        upper_blue=np.array([140,255,255])
        blue_threshold=cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Apply canny edge detection
        canny_low=200
        canny_high=400
        canny_image=cv2.Canny(blue_threshold,canny_low,canny_high)
        

        # Display the RGB image
        cv2.imshow('Canny Video Stream', canny_image)

        # Break loop with 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()

# Release the OpenCV window
cv2.destroyAllWindows()

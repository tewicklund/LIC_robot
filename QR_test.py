import pyrealsense2 as rs
import numpy as np
import cv2
from pyzbar.pyzbar import decode

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

# Initialize the RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline to stream the RGB camera at 640x480
config = rs.config()
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

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
        print(read_qr_code(color_image))

        # Display the RGB image
        cv2.imshow('Color Video Stream', color_image)

        # Break loop with 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()

    # Release the OpenCV window
    cv2.destroyAllWindows()
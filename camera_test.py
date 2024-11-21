from functions import *

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
frame_width=1280
frame_height=800
config = rs.config()
config.enable_stream(rs.stream.color, frame_width, frame_height, rs.format.bgr8, 30)
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
        cv2.imshow("Color Image",color_image)

        # Break loop with 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()

    # Release the OpenCV window
    cv2.destroyAllWindows()
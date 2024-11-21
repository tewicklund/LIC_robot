from functions import *

# Initialize camera
frame_width,frame_height,pipeline=init_camera(400)

try:
    while True:
        # get color image from camera
        color_image = get_color_image(pipeline)

        # show color image in a window
        cv2.imshow("Color Image",color_image)

        # Break loop with 'q' key, required for camera
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()

    # Release the OpenCV window
    cv2.destroyAllWindows()

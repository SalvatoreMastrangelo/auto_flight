# import pyzed.sl as sl
import cv2
import numpy as np
from pymavlink import mavutil
import time

def main():
    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720   # or sl.RESOLUTION.HD1080
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL    # FASTEST + OK quality
    init_params.coordinate_units = sl.UNIT.MILLIMETER

    # Open the camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("Error:", status)
        exit(1)

    # Prepare containers
    imageL = sl.Mat()
    imageR = sl.Mat()
    depth = sl.Mat()

    runtime = sl.RuntimeParameters()

    print("Press 'q' to exit.")

    while True:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left RGB image
            zed.retrieve_image(imageL, sl.VIEW.LEFT)
            frameL = imageL.get_data()

            # Retrieve right RGB image
            zed.retrieve_image(imageR, sl.VIEW.RIGHT)
            frameR = imageR.get_data()

            # Retrieve depth map
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            depth_map = depth.get_data()

            # Normalize depth for display
            # Apply logarithmic scaling for better depth visualization
            depth_clipped = np.clip(depth_map, 1, np.inf)  # Avoid log(0)
            depth_log = np.log(depth_clipped)
            depth_display = cv2.normalize(depth_log, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

            # Show images
            cv2.imshow("ZED | Left Image", frameL)
            cv2.imshow("ZED | Right Image", frameR)
            # apply a blue->red heatmap to the normalized depth and show it
            # Invert normalized depth so near=hot and far=cool
            depth_inverted = 255 - depth_display
            depth_colored = cv2.applyColorMap(depth_inverted, cv2.COLORMAP_JET)
            cv2.imshow("ZED | Depth", depth_colored)

            # print(cv2.waitKey(1), ord('q'))
            if cv2.waitKey(1) == ord('q'):
                break

    # Close
    cv2.destroyAllWindows()
    zed.close()

if __name__ == "__main__":
    main()

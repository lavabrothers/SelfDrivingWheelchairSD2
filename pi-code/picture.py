#
# Kinect V2 Closest Point Detector for Raspberry Pi 5 (using freenect2-python)
#
# This script uses the 'freenect2' Python library to continuously capture
# depth frames from a Kinect V2, identify the closest point, and print its
# distance in inches.
#
# This version is based on the user-provided snippet and assumes the
# 'freenect2' library is installed and working.
#
# How to run:
# 1. Make sure the 'freenect2' and 'numpy' Python libraries are installed.
#    pip install freenect2 numpy
# 2. Run the script from your terminal:
#    python your_script_name.py
#

from freenect2 import Device, FrameType
import numpy as np
import time

# Conversion factor from millimeters to inches
MM_TO_INCHES = 0.0393701

def main():
    """
    Initializes the Kinect device and enters a loop to process depth frames.
    """
    try:
        # --- 1. Initialize the freenect2 device ---
        device = Device()
    except Exception as e:
        print("Error: Could not initialize Kinect V2 device.")
        print(f"Please ensure the device is connected and drivers are set up correctly.")
        print(f"Underlying error: {e}")
        return

    print("Kinect V2 device initialized. Starting stream...")

    # --- 2. Start the device and loop through incoming frames ---
    # The 'with device.running()' context manager handles starting and
    # stopping the device stream.
    # The device object acts as a generator, yielding frames as they arrive.
    with device.running():
        for frame_type, frame in device:
            # We are only interested in the depth frames for this task
            if frame_type == FrameType.Depth:

                # --- 3. Process the depth frame ---
                # The frame object's to_array() method returns a NumPy array.
                # The data type is float32 and values are distances in millimeters.
                depth_data = frame.to_array()

                # Find all non-zero depth values. A value of 0 means the Kinect
                # could not determine the depth for that pixel (e.g., too close,
                # too far, or a material that absorbs infrared light).
                non_zero_depths = depth_data[depth_data > 0]

                if non_zero_depths.size > 0:
                    # Find the minimum distance among all valid points
                    min_depth_mm = np.min(non_zero_depths)

                    # Convert the distance to inches
                    min_depth_inches = min_depth_mm * MM_TO_INCHES

                    # --- 4. Print the result ---
                    # Print the distance to the console, overwriting the previous line.
                    # The '\r' character moves the cursor to the start of the line.
                    # The 'end=""' prevents it from printing a newline.
                    print(f"Closest point is {min_depth_inches:.2f} inches away.   \r", end="")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # This allows you to stop the script gracefully with Ctrl+C
        print("\nProgram stopped by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")

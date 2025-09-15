#
# Kinect V2 Image Capture for Raspberry Pi 5 (using freenect2-python)
#
# This script adapts the working depth-sensing code to capture a single
# high-resolution color image and save it to a file.
#
# How to run:
# 1. Make sure 'freenect2', 'numpy', and 'opencv-python' are installed.
#    pip install freenect2 numpy opencv-python
# 2. Run the script from your terminal:
#    python capture_image.py
#

from freenect2 import Device, FrameType
import numpy as np
import cv2 # Import OpenCV for image processing

def main():
    """
    Initializes the Kinect device, captures a color frame, and saves it as an image.
    """
    try:
        # --- 1. Initialize the freenect2 device ---
        device = Device()
    except Exception as e:
        print("Error: Could not initialize Kinect V2 device.")
        print("Please ensure the device is connected and drivers are set up correctly.")
        print(f"Underlying error: {e}")
        return

    print("Kinect V2 device initialized. Starting stream to capture an image...")
    output_filename = "kinect_capture.jpg"

    # --- 2. Start the device and look for a color frame ---
    with device.running():
        # The device object is a generator. We'll loop until we get the frame we want.
        for frame_type, frame in device:
            # We only want the high-resolution color frame for this task
            if frame_type == FrameType.Color:

                # --- 3. Process the color frame ---
                # to_array() gives us a NumPy array representing the image.
                # The data is in 4-channel BGRA format (Blue, Green, Red, Alpha).
                color_image_bgra = frame.to_array()

                # --- 4. Convert and Save the Image ---
                # OpenCV's imwrite function expects a 3-channel BGR image.
                # We can easily convert by slicing off the alpha (A) channel.
                color_image_bgr = color_image_bgra[:, :, :3]

                # Save the image to the specified file
                cv2.imwrite(output_filename, color_image_bgr)
                
                print(f"✅ Image successfully captured and saved to '{output_filename}'")

                # --- 5. Exit the loop ---
                # Since we only need one picture, we break the loop after saving.
                break

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Allows you to stop the script gracefully with Ctrl+C
        print("\nProgram stopped by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
"""
pylibfreenecttest.py

This script serves as a basic test and demonstration for interfacing with the
Kinect V2 sensor using the `pylibfreenect2` library. It captures both color
and depth frames, processes them, and displays various visualizations using OpenCV.

The script demonstrates:
- Initialization of the Kinect V2 sensor.
- Synchronous capture of color and depth frames.
- Display of raw color and depth images.
- Application of Kinect's built-in registration to align color and depth frames.
- Display of the registered (aligned) color image.

This tool is useful for verifying Kinect V2 connectivity, driver installation,
and basic functionality of the `pylibfreenect2` library.

Dependencies:
- numpy: For numerical operations on image data.
- cv2 (OpenCV): For image processing, resizing, and display.
- sys: For system exit in case of critical initialization failures.
- pylibfreenect2: Python wrapper for libfreenect2, used to interface with the Kinect V2.
"""

import numpy as np
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame, FrameMap

try:
    # Attempt to disable OpenCV's OpenGL usage, which can sometimes cause issues
    # in certain environments or with specific GPU configurations.
    cv2.ogl.setUseOpenGl(False)
except:
    pass # Ignore if OpenGL is not available or setting fails.

# 1. Initialize Freenect2 and enumerate devices.
fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("Error: No Kinect v2 devices found!")
    sys.exit(1)

# 2. Open the first detected Kinect V2 device.
serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial)

# 3. Configure frame listeners for both Color and Depth frames.
types = FrameType.Color | FrameType.Depth
listener = SyncMultiFrameListener(types)

device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

print("Starting device...")
# 4. Start the Kinect V2 stream.
device.start()

# 5. Initialize the Registration object.
# This is used to align the color and depth frames.
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

# 6. Create a FrameMap object to hold incoming frames.
frames = FrameMap()

print("Device started. Press 'q' in the video windows to quit.")

# 7. Main loop for capturing and displaying frames.
while True:
    # Wait for a new set of color and depth frames (with a 10-second timeout).
    if listener.waitForNewFrame(frames, 10 * 1000):
        
        color_frame = frames[FrameType.Color]
        depth_frame = frames[FrameType.Depth]

        # --- Get the RAW Color Image (Large Resolution) ---
        # Convert the color frame to a NumPy array (BGRA format).
        color_image = color_frame.asarray()[:,:,:3] # Slice to get BGR channels.
        color_image_small = cv2.resize(color_image, (960, 540)) # Resize for display.
        cv2.imshow("Color (Full)", color_image_small) # Display the raw color image.

        # --- Get the RAW Depth Image (Smaller Resolution) ---
        # Convert the depth frame to a NumPy array (float32).
        raw_depth_image = depth_frame.asarray(np.float32)
        # Normalize the raw depth image to 0-255 for visualization.
        raw_depth_viz = cv2.normalize(raw_depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        cv2.imshow("Raw Depth", raw_depth_viz) # Display the raw depth image.

        # --- Get the REGISTERED (Aligned) Image ---
        # Create output frames for the registration process.
        undistorted = Frame(512, 424, 4) # Undistorted depth map.
        registered = Frame(512, 424, 4) # Registered color frame, aligned with depth.
        
        # Apply registration to populate 'undistorted' and 'registered' frames.
        registration.apply(color_frame, depth_frame, undistorted, registered)

        # Convert the REGISTERED color frame to a NumPy array (BGR format).
        registered_image = registered.asarray(np.uint8)[:,:,:3] # Slice to get BGR channels.
        cv2.imshow("Registered Color (Aligned with Depth)", registered_image) # Display the aligned color image.

        # Release the frames to free up resources for the next capture.
        listener.release(frames)
    
    else:
        print("Timeout! Waiting for new frames...") # Print message on timeout.
        
    # Check for 'q' key press in any OpenCV window to quit the program.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print("Stopping device...")
# 8. Stop and close the Kinect V2 device.
device.stop()
device.close()
# 9. Close all OpenCV display windows.
cv2.destroyAllWindows()
print("Script finished.")

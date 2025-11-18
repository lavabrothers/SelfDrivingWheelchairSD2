"""
kinectslice.py

This module provides an interface for the Kinect V2 sensor specifically tailored for
2D environmental mapping tasks. It replaces the functionality of `kinect_sensor.py`
for mapping by providing a "scan slice" – a 2D point cloud representing the
horizontal cross-section of the Kinect's view.

It leverages `pylibfreenect2` for sensor interaction and `numpy` for efficient
data processing. A key feature is the use of Kinect's built-in registration
to obtain undistorted and aligned depth data, which is crucial for accurate mapping.

Dependencies:
- pylibfreenect2: Python wrapper for libfreenect2, used to interface with the Kinect V2.
- numpy: For numerical operations and efficient array manipulation.
- math: For mathematical functions, specifically for converting degrees to radians.
"""
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, Registration, Frame, FrameMap
import numpy as np
import time
import sys
import math

# Kinect V2 sensor properties
KINECT_H_FOV_DEG = 70.6 # Horizontal Field of View of the Kinect V2 depth sensor in degrees.
KINECT_H_FOV_RAD = math.radians(KINECT_H_FOV_DEG) # Horizontal Field of View in radians.
KINECT_WIDTH = 512  # Width of the depth map in pixels.
KINECT_HEIGHT = 424 # Height of the depth map in pixels.

# --- Global variables for the Kinect device and listener ---
freenect2 = None    # Freenect2 object for managing Kinect devices.
device = None       # Represents the opened Kinect V2 device.
listener = None     # Listener for receiving frames from the Kinect.
serial = ""         # Serial number of the connected Kinect device.
frames = None       # FrameMap object to hold received frames.
registration = None # Registration object for depth-to-color alignment and undistortion.

# Pre-calculate the horizontal angles for each pixel column.
# This array stores the angle (in radians) for each of the 512 columns,
# from the far left (-KINECT_H_FOV_RAD / 2.0) to the far right (+KINECT_H_FOV_RAD / 2.0).
pixel_angles_rad = np.linspace(-KINECT_H_FOV_RAD / 2.0, KINECT_H_FOV_RAD / 2.0, KINECT_WIDTH)

def initialize_kinect() -> bool:
    """
    Initializes and starts the Kinect V2 sensor, including setting up frame listeners
    and the registration pipeline.

    This function performs the necessary steps to get the Kinect V2 ready for
    capturing and processing frames, particularly for obtaining calibrated depth data.

    Returns:
        bool: True if the Kinect V2 sensor was successfully initialized and started,
              False otherwise.
    """
    global freenect2, device, listener, serial, frames, registration
    try:
        freenect2 = Freenect2()
        num_devices = freenect2.enumerateDevices()
        if num_devices == 0:
            print("Error: No Kinect V2 devices found!")
            return False
            
        serial = freenect2.getDeviceSerialNumber(0)
        print(f"Kinect V2 found with serial: {serial}")
        
        device = freenect2.openDevice(serial)
        
        # We need both color and depth frames for the registration process.
        types = FrameType.Color | FrameType.Depth
        listener = SyncMultiFrameListener(types)
        frames = FrameMap() # Initialize FrameMap to store incoming frames.

        device.setColorFrameListener(listener)
        device.setIrAndDepthFrameListener(listener)
        
        print("Starting Kinect V2 stream...")
        device.start()
        
        # Initialize the Registration object using the camera parameters.
        # This is crucial for getting undistorted depth maps aligned with the color camera.
        print("Initializing sensor registration...")
        registration = Registration(device.getIrCameraParams(),
                                    device.getColorCameraParams())
        print("Registration complete.")
        
        print("Kinect V2 sensor initialized and started.")
        return True
        
    except Exception as e:
        print(f"Error initializing Kinect V2: {e}")
        if "LIBUSB_ERROR_ACCESS" in str(e):
             print("\n--- PERMISSION ERROR (Udev rules) ---")
             print("This is likely a USB permission issue. You may need to set up udev rules for the Kinect.")
        return False

def get_scan_slice() -> list[tuple[float, float]] | None:
    """
    Captures a new depth frame, applies registration, and processes it to generate
    a 2D point cloud "slice" of the environment.

    For each column in the depth map, it finds the minimum depth value, effectively
    flattening the 3D depth data into a 2D horizontal scan. These depths are then
    converted into (x, y) coordinates relative to the sensor using trigonometry.

    Returns:
        list of (x, y) tuples | None: A list of 2D points (x, y) in millimeters,
                                      where 'y' represents the forward distance
                                      and 'x' represents the horizontal distance
                                      from the sensor's center. Returns None on failure or timeout.
    """
    global device, listener, frames, registration
    
    if listener is None or device is None or frames is None or registration is None:
        print("Error: Kinect V2 not fully initialized.")
        return None

    try:
        # Wait for a new set of color and depth frames.
        if not listener.waitForNewFrame(frames, 10 * 1000): # 10 second timeout.
            print("Error: Timeout waiting for new Kinect frame.")
            # Do NOT release frames here if timeout occurs, as they might not be valid.
            return None
            
        color_frame = frames[FrameType.Color]
        depth_frame = frames[FrameType.Depth]
        
        # Create output frames for the registration process.
        undistorted = Frame(KINECT_WIDTH, KINECT_HEIGHT, 4) # Undistorted depth map.
        registered = Frame(KINECT_WIDTH, KINECT_HEIGHT, 4) # Registered color frame (not used for this module).

        # Apply registration to get the corrected depth map in 'undistorted'.
        registration.apply(color_frame, depth_frame, undistorted, registered)
        
        # Convert the undistorted depth frame to a NumPy array of float32.
        depth_map = undistorted.asarray(np.float32)
        
        # Release the frames to free up resources.
        listener.release(frames)
        
        # --- 2D "Flattening" of the depth map ---
        # Replace 0s (indicating no depth reading) with a very large number
        # so they don't interfere with the minimum depth calculation.
        depth_map[depth_map == 0] = 99999.0
        
        # Find the minimum depth value in each column (across axis=0).
        # This effectively creates a 1D array of 512 minimum depths, one for each horizontal pixel.
        min_depths_mm = np.min(depth_map, axis=0)
        
        # --- Convert to 2D Point Cloud (relative to sensor) ---
        # Use trigonometry to convert polar coordinates (depth, angle) to Cartesian (x, y).
        # x = depth * sin(angle) (horizontal distance from center)
        # y = depth * cos(angle) (forward distance from sensor)
        x_coords_mm = min_depths_mm * np.sin(pixel_angles_rad)
        y_coords_mm = min_depths_mm * np.cos(pixel_angles_rad)
        
        # Combine into a list of (x, y) tuples and filter out invalid points
        # (i.e., points that were originally 0 depth and replaced with 99999.0).
        point_cloud = [
            (x, y) for x, y in zip(x_coords_mm, y_coords_mm) 
            if y < 90000.0 # Filter out points with very large 'y' values.
        ]
        
        return point_cloud

    except Exception as e:
        print(f"Error in get_scan_slice: {e}")
        return None

def shutdown_kinect():
    """
    Stops the Kinect V2 stream and closes the device.

    This function is crucial for releasing hardware resources and should always
    be called when the Kinect is no longer needed to prevent resource leaks or
    issues with subsequent sensor initialization.
    """
    global device
    print("Shutting down Kinect V2...")
    if device:
        try:
            device.stop()
            device.close()
            print("Kinect V2 shut down successfully.")
        except Exception as e:
            print(f"Error during Kinect V2 shutdown: {e}")


if __name__ == "__main__":
    """
    Main execution block for a simple test of the mapping sensor.

    This block initializes the Kinect, continuously retrieves 2D scan slices,
    and prints information about the number of points and the left-most/right-most
    points in the slice. Press Ctrl+C to stop the test.
    """
    print("Initializing Mapping Sensor...")
    if initialize_kinect():
        print("Testing sensor. Press Ctrl+C to stop.")
        try:
            while True:
                scan_slice = get_scan_slice()
                
                if scan_slice:
                    print(f"Got scan slice with {len(scan_slice)} points.")
                    # Print first and last point as a sample for horizontal extent.
                    if len(scan_slice) > 0:
                        print(f"  -> Left-most point (x,y): ({scan_slice[0][0]:.0f}, {scan_slice[0][1]:.0f}) mm")
                        print(f"  -> Right-most point (x,y): ({scan_slice[-1][0]:.0f}, {scan_slice[-1][1]:.0f}) mm")
                    else:
                        print("  -> Scan slice was empty (all points filtered).")
                else:
                    print("No scan slice returned...")
                
                time.sleep(0.1) # Small delay to control loop frequency.
        except KeyboardInterrupt:
            print("\nStopping test due to KeyboardInterrupt...")
        finally:
            shutdown_kinect() # Ensure Kinect is properly shut down.
    else:
        print("Sensor initialization failed. Exiting.")

# File: mapping_sensor.py
"""
A new sensor module designed for 2D mapping.
This module replaces kinect_sensor.py for mapping tasks.

It provides a function `get_scan_slice()` which returns a 2D point-cloud
"slice" of what the Kinect sees, rather than just the single nearest point.
"""
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, Registration, Frame, FrameMap
import numpy as np
import time
import sys
import math

# Kinect V2 sensor properties
# Horizontal Field of View (approx. 70.6 degrees)
KINECT_H_FOV_DEG = 70.6
KINECT_H_FOV_RAD = math.radians(KINECT_H_FOV_DEG)
# Depth map width (512 pixels)
KINECT_WIDTH = 512
KINECT_HEIGHT = 424

# --- Global variables for the device ---
freenect2 = None
device = None
listener = None
serial = ""
frames = None
registration = None # <-- ADDED: Registration object

# We can pre-calculate the angles for each pixel column
# This saves computation in the main loop
pixel_angles_rad = np.linspace(-KINECT_H_FOV_RAD / 2.0, KINECT_H_FOV_RAD / 2.0, KINECT_WIDTH)

def initialize_kinect():
    """
    Initializes and starts the Kinect V2 sensor.
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
        
        # We need BOTH color and depth for registration
        types = FrameType.Color | FrameType.Depth
        listener = SyncMultiFrameListener(types)
        frames = FrameMap()

        device.setColorFrameListener(listener)
        device.setIrAndDepthFrameListener(listener)
        
        print("Starting Kinect V2 stream...")
        device.start()
        
        # --- ADDED: Initialize Registration ---
        # This is the critical step to get calibrated depth data
        print("Initializing sensor registration...")
        registration = Registration(device.getIrCameraParams(),
                                    device.getColorCameraParams())
        print("Registration complete.")
        # --- END ADDED ---
        
        print("Kinect V2 sensor initialized and started.")
        return True
        
    except Exception as e:
        print(f"Error initializing Kinect V2: {e}")
        if "LIBUSB_ERROR_ACCESS" in str(e):
             print("\n--- PERMISSION ERROR (Udev rules) ---")
        return False

def get_scan_slice():
    """
    Gets a depth frame, processes it, and returns a 2D point cloud "slice".

    Returns:
        list of (x, y) tuples: A list of 2D points relative to the sensor.
                                'y' is forward distance, 'x' is horizontal.
                                Returns None on failure.
    """
    global device, listener, frames, registration
    
    if listener is None or device is None or frames is None or registration is None:
        print("Error: Kinect V2 not fully initialized.")
        return None

    try:
        if not listener.waitForNewFrame(frames, 10 * 1000): # 10 sec timeout
            print("Error: Timeout waiting for new Kinect frame.")
            # Do NOT release frames here, it can cause issues
            return None # Return None on timeout
            
        # --- MODIFIED: Get both frames ---
        color_frame = frames[FrameType.Color]
        depth_frame = frames[FrameType.Depth]
        
        # --- ADDED: Apply Registration ---
        # These are the output frames
        undistorted = Frame(512, 424, 4)
        registered = Frame(512, 424, 4) # Not used, but required by .apply()

        # This call populates 'undistorted' with the corrected depth map
        registration.apply(color_frame, depth_frame, undistorted, registered)
        
        # --- MODIFIED: Use the corrected 'undistorted' frame ---
        depth_map = undistorted.asarray(np.float32)
        
        listener.release(frames)
        # --- END MODIFICATIONS ---
        
        
        # --- 2D "Flattening" ---
        # We want to find the nearest object in each of the 512 columns.
        
        # Replace 0s (no reading) with a very large number
        depth_map[depth_map == 0] = 99999.0
        
        # Find the minimum depth value in each column (axis=0)
        # This "flattens" the 424 rows into a single 1D array of 512 depths.
        min_depths_mm = np.min(depth_map, axis=0)
        
        # --- Convert to 2D Point Cloud (relative to sensor) ---
        
        # Use trigonometry with our pre-calculated angles
        # x = depth * sin(angle)
        # y = depth * cos(angle)  (y is forward distance)
        x_coords_mm = min_depths_mm * np.sin(pixel_angles_rad)
        y_coords_mm = min_depths_mm * np.cos(pixel_angles_rad)
        
        # Combine into a list of (x, y) tuples
        # We also filter out any "no reading" points
        point_cloud = [
            (x, y) for x, y in zip(x_coords_mm, y_coords_mm) 
            if y < 90000.0 # Filter out the 99999.0 we added
        ]
        
        return point_cloud

    except Exception as e:
        print(f"Error in get_scan_slice: {e}")
        return None

def shutdown_kinect():
    """
    Stops and closes the Kinect V2 device.
    This is CRITICAL to run on exit.
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
    # A simple test for the new mapping sensor
    print("Initializing Mapping Sensor...")
    if initialize_kinect():
        print("Testing sensor. Press Ctrl+C to stop.")
        try:
            while True:
                scan_slice = get_scan_slice()
                
                if scan_slice:
                    print(f"Got scan slice with {len(scan_slice)} points.")
                    # Print first and last point as a sample
                    if len(scan_slice) > 0:
                        print(f"  -> Left-most point (x,y): ({scan_slice[0][0]:.0f}, {scan_slice[0][1]:.0f}) mm")
                        print(f"  -> Right-most point (x,y): ({scan_slice[-1][0]:.0f}, {scan_slice[-1][1]:.0f}) mm")
                    else:
                        print("  -> Scan slice was empty (all points filtered).")
                else:
                    print("No scan slice returned...")
                
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopping test...")
        finally:
            shutdown_kinect()
    else:
        print("Sensor initialization failed. Exiting.")
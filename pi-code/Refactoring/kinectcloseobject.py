# File: kinect_sensor.py
# This is the UPDATED version for Kinect V2 and pylibfreenect2
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, FrameMap
import numpy as np
import time
import sys

# Kinect V2 sensor properties
# Horizontal Field of View (approx. 70.6 degrees)
KINECT_H_FOV = 70.6
# Depth map width (512 pixels)
KINECT_WIDTH = 512

# --- ADDED: Smoothing / Damping factor ---
# 1.0 = no smoothing (raw data)
# 0.1 = very heavy smoothing (slow to react)
# 0.7 is a good starting point
SMOOTHING_FACTOR = 0.7
# --- End of addition ---


# --- Global variables for the device ---
# The pylibfreenect2 API is stateful
freenect2 = None
device = None
listener = None
serial = ""

# --- State variables for smoothing ---
last_known_depth = None
last_known_angle = None

def initialize_kinect():
# ... (rest of the function is unchanged) ...
    """
    Initializes and starts the Kinect V2 sensor.
    This is now a stateful operation.
    """
    global freenect2, device, listener, serial
    try:
        freenect2 = Freenect2()
        # --- FIX: Use camelCase ---
        num_devices = freenect2.enumerateDevices()
        if num_devices == 0:
            print("Error: No Kinect V2 devices found!")
            return False
            
        # --- FIX: Use camelCase ---
        serial = freenect2.getDeviceSerialNumber(0)
        print(f"Kinect V2 found with serial: {serial}")
        
        # --- FIX: Use camelCase ---
        device = freenect2.openDevice(serial)
        
        # Based on your working script, we will request both
        # Color and Depth, as this seems more reliable.
        types = FrameType.Color | FrameType.Depth
        listener = SyncMultiFrameListener(types)
        
        # --- FIX: Use camelCase ---
        device.setColorFrameListener(listener)
        device.setIrAndDepthFrameListener(listener)
        
        # Start the device
        print("Starting Kinect V2 stream...")
        device.start()
        
        print("Kinect V2 sensor initialized and started.")
        return True
        
    except Exception as e:
        print(f"Error initializing Kinect V2: {e}")
        if "LIBUSB_ERROR_ACCESS" in str(e):
             print("\n--- PERMISSION ERROR ---")
             print("This is likely a USB permission issue.")
             print("You may need to set up udev rules for the Kinect.")
             print("See the pylibfreenect2 installation instructions.")
        return False

def get_nearest_object_angle():
# ... (function definition is unchanged) ...
    """
    Gets a depth frame from the Kinect V2 and finds the closest point.
    
    Returns:
        (float, float): A tuple of (minimum_depth, angle)
                       'minimum_depth' is the depth in **millimeters**.
                       'angle' is the horizontal angle in degrees from center.
                       (Negative = left, Positive = right, 0 = center)
                       Returns (None, None) on failure *if no value has ever been seen*.
                       Returns the *last known* value on a temporary failure.
    """
    global device, listener, last_known_depth, last_known_angle
    
    if listener is None or device is None:
        print("Error: Kinect V2 not initialized.")
        return None, None

    try:
        # --- FIX: Use FrameMap() instead of a dict {} ---
        frames = FrameMap()
        
        # --- FIX: Use camelCase ---
        # The timeout (10*1000) is in milliseconds
        if not listener.waitForNewFrame(frames, 10 * 1000):
            print("Timeout waiting for new frames. Returning last known value.")
            listener.release(frames)
            # --- MODIFIED: Return last known value ---
            return last_known_depth, last_known_angle
            
        # Get the depth frame (we just ignore the color frame)
        depth_frame = frames[FrameType.Depth]
        
        # Convert the frame data to a numpy array
        # The data is in millimeters (mm)
        depth_map = depth_frame.asarray()
        
        # --- IMPORTANT ---
        # We MUST release the frames, or the sensor will hang
        # --- FIX: Use camelCase 'release' ---
        listener.release(frames)
        
        # Filter out "0" values, as these indicate no reading
        valid_depths = depth_map[depth_map > 0]
        
        if valid_depths.size == 0:
            # No valid depth readings in this frame
            # --- MODIFIED: Return last known value ---
            return last_known_depth, last_known_angle

        # --- MODIFIED: Use percentile instead of min ---
        # This is much more robust to single-pixel noise
        new_depth = np.percentile(valid_depths, 1)
        # --- End of modification ---
        
        # Find the (x, y) coordinates of this minimum depth
        # We still search for the *absolute* min for the angle,
        # but we use the *percentile* for the depth value.
        # This is a good compromise.
        search_map = np.where(depth_map == 0, 999999, depth_map)
        y, x = np.unravel_index(np.argmin(search_map), search_map.shape)

        # Now, convert the horizontal pixel coordinate 'x' to an angle
        # 0px -> -35.3 deg
        # 256px -> 0 deg
        # 512px -> +35.3 deg
        
        # Calculate pixel's normalized position (-1.0 to +1.0)
        normalized_x = (x - (KINECT_WIDTH / 2.0)) / (KINECT_WIDTH / 2.0)
        
        # Convert normalized position to angle
        new_angle = normalized_x * (KINECT_H_FOV / 2.0)
        
        # --- MODIFIED: Apply temporal smoothing ---
        if last_known_depth is None:
            # This is the first frame, just set the values
            last_known_depth = new_depth
            last_known_angle = new_angle
        else:
            # Apply weighted average (low-pass filter)
            last_known_depth = (new_depth * SMOOTHING_FACTOR) + \
                               (last_known_depth * (1.0 - SMOOTHING_FACTOR))
            last_known_angle = (new_angle * SMOOTHING_FACTOR) + \
                               (last_known_angle * (1.0 - SMOOTHING_FACTOR))
        # --- End of modification ---
        
        return last_known_depth, last_known_angle

    except Exception as e:
        print(f"Error in get_nearest_object_angle: {e}")
        # --- MODIFIED: Return last known value ---
        return last_known_depth, last_known_angle

def shutdown_kinect():
# ... (rest of the function is unchanged) ...
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
    
    # freenect2.shutdown() # This can sometimes cause a segfault
                         # Closing the device is usually sufficient.


if __name__ == "__main__":
# ... (rest of the main test loop is unchanged) ...
    # A simple test to run if you execute this file directly
    print("Initializing Kinect V2...")
    if initialize_kinect():
        print("Testing Kinect V2. Point it at something.")
        print("Press Ctrl+C to stop.")
        try:
            while True:
                depth, angle = get_nearest_object_angle()
                if depth is not None:
                    print(f"Nearest object found!")
                    print(f"  -> Depth: {depth:.0f} mm  ({depth/1000.0:.2f} m)")
                    print(f"  -> Angle: {angle:.2f} degrees from center")
                else:
                    print("No object detected (still searching for first lock)...")
                
                time.sleep(0.1) # V2 can run faster
        except KeyboardInterrupt:
            print("\nStopping test...")
        finally:
            shutdown_kinect()
    else:
        print("Kinect V2 initialization failed. Exiting.")










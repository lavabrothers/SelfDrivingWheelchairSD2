#!/home/lavabrothers/Documents/SelfDrivingWheelchairSD2/pi-code/.venv/bin/python3
import board
import adafruit_mcp4728
import time
import numpy as np
import cv2

# Import freenect2 (assuming pylibfreenect2 is installed)
try:
    from freenect2 import Freenect2, SyncMultiFrameListener
    from freenect2 import FrameType, Registration, setLogger
    from freenect2.libfreenect2 import Logger, IrCameraParams, ColorCameraParams
except ImportError:
    print("Error: pylibfreenect2 not found. Please ensure it is installed.")
    exit()

# --- Configuration ---
# Distance threshold in meters (3 feet = 0.9144 meters)
DISTANCE_THRESHOLD_METERS = 0.9144
# Area of interest for obstacle detection (e.g., a central rectangle in the depth frame)
# These values might need adjustment based on Kinect placement and desired detection zone.
# (y_start, y_end, x_start, x_end)
ROI_Y_START = 100
ROI_Y_END = 380
ROI_X_START = 200
ROI_X_END = 440

# --- DAC Setup ---
try:
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    exit()

def drive_forward():
    """Sets DAC channels to drive the wheelchair forward."""
    mcp.channel_a.normalized_value = 0.75
    mcp.channel_b.normalized_value = 0.25
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    print("\rDriving Forward...", end='')

def stop_wheelchair():
    """Sets DAC channels to stop the wheelchair."""
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    print("\rStopping Wheelchair.", end='')

def initialize_kinect():
    """Initializes Freenect2 and returns the device and listener."""
    setLogger(Logger()) # Suppress freenect2 verbose output if desired

    fn = Freenect2()
    num_devices = fn.enumerateDevices()
    if num_devices == 0:
        print("No Kinect v2 device found!")
        return None, None

    serial = fn.getDeviceSerialNumber(0)
    device = fn.openDevice(serial)

    listener = SyncMultiFrameListener(FrameType.Depth | FrameType.Ir | FrameType.Color)

    device.setColorFrameListener(listener)
    device.setIrAndDepthFrameListener(listener)

    device.start()
    print(f"Kinect v2 device '{serial}' started.")
    return device, listener

def process_depth_frame(depth_frame):
    """
    Processes the depth frame to find the minimum distance in the ROI.
    Returns the minimum distance in meters.
    """
    # Convert depth frame to meters (depth values are typically in mm)
    # The depth frame from freenect2 is usually already in mm, so divide by 1000
    depth_data_meters = depth_frame.asarray(np.float32) / 1000.0

    # Extract ROI
    roi = depth_data_meters[ROI_Y_START:ROI_Y_END, ROI_X_START:ROI_X_END]

    # Filter out invalid depth values (0 or very large values)
    # Kinect v2 depth range is typically 0.5m to 4.5m
    valid_depths = roi[(roi > 0.5) & (roi < 4.5)]

    if valid_depths.size > 0:
        min_distance = np.min(valid_depths)
        return min_distance
    else:
        return float('inf') # No valid objects in ROI

def main():
    device, listener = initialize_kinect()
    if device is None:
        return

    try:
        stop_wheelchair() # Ensure wheelchair is stopped initially

        while True:
            frames = listener.waitForNewFrame()
            depth_frame = frames["depth"]

            min_distance = process_depth_frame(depth_frame)

            if min_distance < DISTANCE_THRESHOLD_METERS:
                stop_wheelchair()
                print(f"Object too close! Distance: {min_distance:.2f}m. Stopping.")
            else:
                drive_forward()
                print(f"Clear. Driving forward. Min distance: {min_distance:.2f}m.", end='')

            listener.release(frames)
            time.sleep(0.1) # Small delay to prevent busy-waiting

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        print("\nStopping Kinect and cleaning up.")
        if device:
            device.stop()
            device.close()
        stop_wheelchair() # Ensure wheelchair is stopped on exit

if __name__ == "__main__":
    main()

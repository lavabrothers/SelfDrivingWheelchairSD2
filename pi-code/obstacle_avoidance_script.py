import board
import adafruit_mcp4728
import time
from freenect2 import Device, FrameType
import numpy as np

# --- Configuration ---
OBSTACLE_DISTANCE_THRESHOLD_MM = 1000  # 1000 mm = 1 meter
FORWARD_SPEED_VALUE_A = 0.75
FORWARD_SPEED_VALUE_B = 0.25
NEUTRAL_SPEED_VALUE = 0.5

# --- Setup DAC ---
try:
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    exit()

# --- Setup Kinect v2 Device ---
try:
    device = Device()
    print("Kinect v2 device found and initialized.")
except Exception as e:
    print(f"Error: Could not find Kinect v2 device. Please check connection. {e}")
    exit()

# --- Wheelchair Control Functions ---
def drive_forward():
    """Sets DAC channels to drive the wheelchair forward."""
    mcp.channel_a.normalized_value = FORWARD_SPEED_VALUE_A
    mcp.channel_b.normalized_value = FORWARD_SPEED_VALUE_B
    mcp.channel_c.normalized_value = NEUTRAL_SPEED_VALUE
    mcp.channel_d.normalized_value = NEUTRAL_SPEED_VALUE
    print(f"\rDriving Forward (A:{FORWARD_SPEED_VALUE_A:.2f}, B:{FORWARD_SPEED_VALUE_B:.2f})", end='')

def stop_wheelchair():
    """Sets DAC channels to stop the wheelchair."""
    mcp.channel_a.normalized_value = NEUTRAL_SPEED_VALUE
    mcp.channel_b.normalized_value = NEUTRAL_SPEED_VALUE
    mcp.channel_c.normalized_value = NEUTRAL_SPEED_VALUE
    mcp.channel_d.normalized_value = NEUTRAL_SPEED_VALUE
    print(f"\rStopped (A:{NEUTRAL_SPEED_VALUE:.2f}, B:{NEUTRAL_SPEED_VALUE:.2f})", end='')

# --- Depth Sensing Function ---
def is_obstacle_too_close(depth_frame):
    """
    Analyzes the depth frame to determine if an obstacle is too close.
    Returns True if an obstacle is detected within the threshold, False otherwise.
    """
    # The depth frame contains depth values in millimeters.
    # We can take a region of interest (e.g., the center of the frame)
    # to check for obstacles directly in front of the wheelchair.
    
    # For simplicity, let's consider the minimum depth in the entire frame.
    # A more robust solution would involve a specific ROI.
    
    # Convert depth frame to a numpy array for easier processing
    depth_data = depth_frame.to_array()

    # Filter out invalid depth values (0 usually means no depth data)
    valid_depths = depth_data[depth_data > 0]

    if valid_depths.size > 0:
        min_depth = np.min(valid_depths)
        # print(f"Min depth: {min_depth} mm") # For debugging
        return min_depth < OBSTACLE_DISTANCE_THRESHOLD_MM
    
    return False # No valid depth data, assume no obstacle

# --- Main Loop ---
def main():
    print("Starting obstacle avoidance script. Press Ctrl+C to exit.")
    stop_wheelchair() # Start in a stopped state

    with device.running():
        try:
            while True:
                frames = {}
                # Capture a single depth frame
                for type_, frame in device:
                    frames[type_] = frame
                    if FrameType.Depth in frames:
                        break
                
                depth = frames[FrameType.Depth]
                
                if is_obstacle_too_close(depth):
                    stop_wheelchair()
                    print(f" - OBSTACLE DETECTED! Stopping. (Threshold: {OBSTACLE_DISTANCE_THRESHOLD_MM}mm)", end='')
                else:
                    drive_forward()
                    print(f" - Clear. Driving forward. (Threshold: {OBSTACLE_DISTANCE_THRESHOLD_MM}mm)", end='')
                
                # Release the frames to prevent the queue from filling up
                device.release(frames)
                
                time.sleep(0.1) # Small delay to prevent busy-waiting

        except KeyboardInterrupt:
            print("\nExiting script.")
        finally:
            stop_wheelchair() # Ensure wheelchair stops on exit
            device.stop()
            device.close()
            print("Kinect v2 device closed. DAC set to neutral. Program terminated.")

if __name__ == "__main__":
    main()

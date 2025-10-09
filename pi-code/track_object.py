# track_object.py
# This script uses the Kinect V2's depth camera to find the closest object
# in its field of view and turns the wheelchair to keep that object centered.

import time
import board
import numpy as np

# --- Adafruit Libraries for Wheelchair Control ---
import adafruit_mpu6050
import adafruit_mcp4728

# --- Freenect2 Library for Kinect V2 ---
from freenect2 import Device, FrameType

# --- Constants ---
# --- Proportional Control ---
# This gain value determines how aggressively the wheelchair turns to correct its angle.
# A higher value means faster, more aggressive turning. Start low and tune as needed.
PROPORTIONAL_GAIN = 0.002
# The maximum speed at which the wheelchair will turn.
MAX_TURN_SPEED = 0.30
# The horizontal zone (in pixels) in the center of the view where no turning will occur.
# This prevents jerky movements when the target is nearly centered.
CENTER_DEAD_ZONE_PX = 40

# --- Noise Reduction ---
# The size of the square region (in pixels) around the closest point to average.
# A larger value provides more stability but might not track very small objects well.
ROI_SIZE = 20

# --- Performance ---
# To improve performance on the Raspberry Pi, we can skip frames.
# A value of 1 processes every frame. A value of 5 processes every 5th frame.
PROCESS_EVERY_NTH_FRAME = 2

# --- Global State Variables ---
current_fwd_bwd = 0.0
current_left_right = 0.0

# --- Hardware Setup ---
try:
    print("Initializing I2C and sensors...")
    i2c = board.I2C()
    mpu = adafruit_mpu6050.MPU6050(i2c)
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MPU6050 and MCP4728 found and initialized. ✅")
except Exception as e:
    print(f"Error: Could not find a required I2C device. Please check connections.")
    print(f"Details: {e}")
    exit()

try:
    print("Initializing Kinect V2 device...")
    kinect = Device()
    print("Kinect V2 found and initialized. ✅")
except Exception as e:
    print(f"Error: Could not initialize Kinect V2. Is it connected and powered?")
    print(f"Details: {e}")
    exit()

# --- DAC Control Functions (for wheelchair movement) ---
def set_movement(fwd_bwd, left_right):
    """
    Sets the wheelchair's motor speeds.
    fwd_bwd: -1.0 (full reverse) to 1.0 (full forward)
    left_right: -1.0 (turn left) to 1.0 (turn right)
    """
    global current_fwd_bwd, current_left_right
    # Clamp values to the expected range [-1.0, 1.0]
    fwd_bwd = max(-1.0, min(1.0, fwd_bwd))
    left_right = max(-1.0, min(1.0, left_right))
    
    # The MCP4728 DAC uses a normalized value from 0.0 to 1.0.
    # 0.5 is neutral, 1.0 is full forward/right, 0.0 is full backward/left.
    mcp.channel_a.normalized_value = 0.5 + (fwd_bwd / 2.0)
    mcp.channel_b.normalized_value = 0.5 - (fwd_bwd / 2.0)
    mcp.channel_c.normalized_value = 0.5 - (left_right / 2.0) # Inverted for intuitive turning
    mcp.channel_d.normalized_value = 0.5 + (left_right / 2.0) # Inverted for intuitive turning
    
    current_fwd_bwd = fwd_bwd
    current_left_right = left_right

def stop_all_movement():
    """Stops all wheelchair movement."""
    print("\nStopping movement...")
    set_movement(0.0, 0.0)
    print("Stopped.")

# --- Main Tracking Logic ---
def execute_depth_tracking():
    """
    Finds the closest object using the depth camera and turns to center it.
    """
    print(f"\n--- Starting Depth Tracking Mode ---")
    print(f"Will turn to keep the closest object in the center of the view.")
    print("Press CTRL+C to stop tracking and exit.")

    frame_count = 0
    try:
        with kinect.running():
            # Loop through frames from the Kinect
            for frame_type, frame in kinect:
                # We only care about depth frames for this task
                if frame_type == FrameType.Depth:
                    frame_count += 1
                    # Skip frames to save processing power
                    if frame_count % PROCESS_EVERY_NTH_FRAME != 0:
                        continue

                    # Get depth data as a NumPy array (values are in millimeters)
                    depth_data = frame.to_array()
                    
                    # Get the dimensions of the depth frame
                    height, width = depth_data.shape

                    # Find the closest point. We ignore zero values, as they represent invalid readings.
                    valid_depths = depth_data[depth_data > 0]
                    
                    if valid_depths.size > 0:
                        min_depth = np.min(valid_depths)
                        
                        # --- Noise Reduction: Averaging a Region of Interest (ROI) ---
                        # Find the initial coordinate of the closest point
                        min_depth_coords = np.where(depth_data == min_depth)
                        initial_y = int(np.median(min_depth_coords[0]))
                        initial_x = int(np.median(min_depth_coords[1]))

                        # Define a square ROI around the initial point
                        roi_half = ROI_SIZE // 2
                        y1 = max(0, initial_y - roi_half)
                        y2 = min(height, initial_y + roi_half)
                        x1 = max(0, initial_x - roi_half)
                        x2 = min(width, initial_x + roi_half)
                        
                        roi = depth_data[y1:y2, x1:x2]
                        
                        # Get all valid points in the ROI that are close to the minimum depth
                        # This prevents averaging in background objects if the ROI is on an edge
                        roi_points = np.where((roi > 0) & (roi < min_depth + 150)) # 150mm tolerance
                        
                        if roi_points[0].size > 0:
                            # Calculate the center of mass (average coordinate) of these points
                            # This is our new, more stable target
                            target_x = int(np.mean(roi_points[1])) + x1
                            target_y = int(np.mean(roi_points[0])) + y1

                            # --- Proportional Turning Logic ---
                            frame_center_x = width // 2
                            
                            # Calculate the error (how far the target is from the center)
                            error = target_x - frame_center_x
                            
                            status = ""
                            # If the error is outside the dead zone, calculate a turning speed
                            if abs(error) > (CENTER_DEAD_ZONE_PX // 2):
                                # The turning speed is proportional to the error
                                turn_speed = error * PROPORTIONAL_GAIN
                                # Clamp the turn speed to the maximum allowed value
                                turn_speed = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, turn_speed))
                                
                                set_movement(0.0, turn_speed)
                                status = "TRACKING"
                            else:
                                # If inside the dead zone, stop turning
                                set_movement(0.0, 0.0)
                                status = "CENTERED"
                            
                            # Display the status in the console
                            print(f"Status: {status} | Target X: {target_x} | Error: {error} | Turn Speed: {turn_speed:.2f} | Distance: {min_depth / 1000:.2f}m", end='\r')
                        else:
                            # If ROI has no valid points, stop as a safety measure
                            set_movement(0.0, 0.0)
                            print(f"Status: SEARCHING... (ROI lost)                                       ", end='\r')

                    else:
                        # If there's no valid depth data, stop moving as a safety measure
                        set_movement(0.0, 0.0)
                        print(f"Status: SEARCHING... (No valid depth data found)                                       ", end='\r')

    except KeyboardInterrupt:
        print("\nTracking mode interrupted by user.")
    finally:
        # Ensure the wheelchair is stopped when the program ends
        stop_all_movement()

# --- Main Program Execution ---
if __name__ == "__main__":
    # Set motors to neutral before starting
    set_movement(0.0, 0.0)
    
    # Start the main tracking function
    execute_depth_tracking()
    
    print("Program terminated.")

# Import necessary libraries
import time
import board
import math
import numpy as np

# --- Adafruit Libraries for Wheelchair Control ---
import adafruit_mpu6050
import adafruit_mcp4728

# --- Freenect2 Library for Kinect V2 ---
from freenect2 import Device, FrameType

# --- Constants ---
# --- ❗ YOU MUST CALIBRATE THESE VALUES ❗ ---
MOVE_SPEED_OFFSET = 0.1
TURN_SPEED_OFFSET = 0.25
MM_TO_FEET = 0.00328084 # Conversion for Kinect data

# --- NEW: Constants for Follow Mode ---
FOLLOW_DEAD_ZONE_FT = 0.5 # The buffer zone (in feet) to prevent jittering

# --- Constants for Kinect Guidance ---
ROI_WIDTH = 100
ROI_HEIGHT = 50

# --- Ramping Configuration ---
RAMP_STEP = 0.05
RAMP_DELAY = 0.05

# --- Global State Variables ---
current_fwd_bwd = 0.0
current_left_right = 0.0

# --- Hardware Setup (Initialize I2C, MPU, MCP, and Kinect) ---
# (This section is unchanged from your previous script)
try:
    print("Initializing I2C and sensors...")
    i2c = board.I2C()
    mpu = adafruit_mpu6050.MPU6050(i2c)
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MPU6050 and MCP4728 found and initialized. ✅")
except ValueError as e:
    print(f"Error: Could not find a required I2C device. Please check connections.")
    print(f"Details: {e}")
    exit()

try:
    print("Initializing Kinect V2 device...")
    kinect = Device()
    print("Kinect V2 found and initialized. ✅")
except Exception as e:
    print(f"Error: Could not initialize Kinect V2 device.")
    exit()

# --- DAC Control Functions (Unchanged) ---
def set_movement(fwd_bwd, left_right):
    global current_fwd_bwd, current_left_right
    fwd_bwd = max(-1.0, min(1.0, fwd_bwd))
    left_right = max(-1.0, min(1.0, left_right))
    mcp.channel_a.normalized_value = 0.5 + (fwd_bwd / 2.0)
    mcp.channel_b.normalized_value = 0.5 - (fwd_bwd / 2.0)
    mcp.channel_c.normalized_value = 0.5 + (left_right / 2.0)
    mcp.channel_d.normalized_value = 0.5 - (left_right / 2.0)
    current_fwd_bwd = fwd_bwd
    current_left_right = left_right

def ramp_to_speed(target_fwd_bwd, target_left_right):
    # This function is now mostly used for turns and initial stop
    while abs(target_fwd_bwd - current_fwd_bwd) > 0.01 or \
          abs(target_left_right - current_left_right) > 0.01:
        new_fwd_bwd = current_fwd_bwd; new_left_right = current_left_right
        if target_fwd_bwd > current_fwd_bwd: new_fwd_bwd += RAMP_STEP
        elif target_fwd_bwd < current_fwd_bwd: new_fwd_bwd -= RAMP_STEP
        if target_left_right > current_left_right: new_left_right += RAMP_STEP
        elif target_left_right < current_left_right: new_left_right -= RAMP_STEP
        set_movement(new_fwd_bwd, new_left_right)
        time.sleep(RAMP_DELAY)
    set_movement(target_fwd_bwd, target_left_right)

def stop_all_movement():
    print("\nStopping movement...")
    set_movement(0.0, 0.0) # Use a direct stop for responsiveness
    print("Stopped.")

# --- Core Logic Functions ---

# --- NEW: Continuous Follow Mode Function ---
def execute_follow_mode(target_distance_ft):
    """
    Continuously adjusts movement to maintain a target distance from an object.
    """
    print(f"\n--- Starting Follow Mode ---")
    print(f"Target distance: {target_distance_ft:.1f} ft (Dead Zone: +/- {FOLLOW_DEAD_ZONE_FT} ft)")
    print("Press CTRL+C to stop following and return to the main menu.")

    target_distance_mm = target_distance_ft / MM_TO_FEET
    upper_bound_mm = target_distance_mm + (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)
    lower_bound_mm = target_distance_mm - (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)
    
    try:
        with kinect.running():
            for frame_type, frame in kinect:
                if frame_type == FrameType.Depth:
                    depth_data = frame.to_array()

                    # Define and extract the Region of Interest (ROI)
                    height, width = depth_data.shape
                    roi_y1 = (height - ROI_HEIGHT) // 2; roi_y2 = roi_y1 + ROI_HEIGHT
                    roi_x1 = (width - ROI_WIDTH) // 2; roi_x2 = roi_x1 + ROI_WIDTH
                    center_roi = depth_data[roi_y1:roi_y2, roi_x1:roi_x2]
                    
                    valid_depths = center_roi[center_roi > 0]

                    if valid_depths.size > 0:
                        closest_point_mm = np.min(valid_depths)
                        
                        # --- THE CORE FOLLOW LOGIC ---
                        if closest_point_mm > upper_bound_mm:
                            # Object is too far, move forward
                            if current_fwd_bwd <= 0:
                                set_movement(MOVE_SPEED_OFFSET, 0.0)
                            status = "MOVING FORWARD"
                        elif closest_point_mm < lower_bound_mm:
                            # Object is too close, stop
                            if current_fwd_bwd > 0:
                                set_movement(0.0, 0.0)
                            status = "STOPPED (Too Close)"
                        else:
                            # Object is in the dead zone, stay stopped
                            if current_fwd_bwd > 0:
                                set_movement(0.0, 0.0)
                            status = "HOLDING (In Zone)"
                        
                        current_dist_ft = closest_point_mm * MM_TO_FEET
                        print(f"Target: {target_distance_ft:.1f}ft | Current: {current_dist_ft:.1f}ft | Status: {status}   ", end='\r')

                    else:
                        # Path is clear, stop for safety
                        if current_fwd_bwd > 0:
                            set_movement(0.0, 0.0)
                        print(f"Target: {target_distance_ft:.1f}ft | Current: --- | Status: STOPPED (Path Clear) ", end='\r')

    except KeyboardInterrupt:
        print("\nFollow mode interrupted by user.")
        stop_all_movement()

def execute_turn(direction, target_angle_deg):
    """Turns left or right by a specific angle using the MPU6050 gyro."""
    # (This function is unchanged)
    print(f"Executing turn: {direction} {target_angle_deg}°...")
    turn_value = TURN_SPEED_OFFSET * 2
    if direction == 'left': turn_value *= -1
    ramp_to_speed(0.0, turn_value)
    total_angle_turned = 0.0; last_time = time.monotonic()
    while total_angle_turned < target_angle_deg:
        current_time = time.monotonic(); time_delta = current_time - last_time; last_time = current_time
        total_angle_turned += abs(math.degrees(mpu.gyro[2] * time_delta))
        print(f"  -> Progress: {total_angle_turned:.2f}° / {target_angle_deg}°", end='\r')
        time.sleep(0.01)
    print(f"\n  -> Target angle reached.")
    stop_all_movement()

# --- Main Program Loop (Modified for Follow Mode) ---
if __name__ == "__main__":
    set_movement(0.0, 0.0)
    
    print("\n--- Kinect-Guided Wheelchair Control ---")
    print("Commands:")
    print("  'follow [feet]'   - Maintain a distance of [feet] from an object.")
    print("  'left [degrees]'  - e.g., 'left 90'")
    print("  'right [degrees]' - e.g., 'right 45'")
    print("  'stop'            - Halts any current movement")
    print("  'exit'            - Closes the program")
    print("-----------------------------------------")

    while True:
        command_str = input("Enter command > ").lower().strip()
        parts = command_str.split()
        if not parts: continue

        if len(parts) == 2:
            try:
                value = float(parts[1])
                if value <= 0:
                    print("Error: Distance or angle must be a positive number."); continue
                
                command = parts[0]
                if command == 'follow':
                    execute_follow_mode(target_distance_ft=value)
                elif command in ('left', 'right'):
                    execute_turn(direction=command, target_angle_deg=value)
                else:
                    print("Invalid command.")
            except ValueError:
                print("Error: Invalid distance/angle. Please enter a number.")
        
        elif parts[0] == "stop":
            stop_all_movement()
        elif parts[0] == "exit":
            print("Setting to neutral and exiting program.")
            stop_all_movement()
            break
        else:
            print("Invalid command format.")
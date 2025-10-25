# StableFollow_v4.py
# Autonomous Wheelchair Control Script
# - Person Following Mode
# - Cruise Control with Obstacle Avoidance

import time
import board
import numpy as np
import cv2
import adafruit_mcp4728
from freenect2 import Device, FrameType

# --- Constants ---
MOVE_SPEED_OFFSET = 0.15
TURN_SPEED_OFFSET = 0.25
MM_TO_FEET = 0.00328084
FOLLOW_DEAD_ZONE_FT = 1.0
TRACKING_CENTER_DEAD_ZONE_PX = 30

# --- PERFORMANCE TUNING ---
PROCESS_EVERY_NTH_FRAME = 15
DETECTION_WIDTH_PX = 560

# --- Hardware Setup ---
try:
    print("Initializing I2C and sensors...")
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MCP4728 found and initialized. ✅")
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
    print(f"Details: {e}")
    exit()

# --- Load the pre-trained person detector ---
try:
    print("Loading person detection model...")
    upper_body_cascade = cv2.CascadeClassifier('haarcascade_upperbody.xml')
    if upper_body_cascade.empty():
        raise IOError("Could not load haarcascade_upperbody.xml")
    print("Person detection model loaded successfully. ✅")
except Exception as e:
    print(f"Fatal Error: {e}")
    print("Please ensure 'haarcascade_upperbody.xml' is in the same directory.")
    exit()

# --- DAC Control Functions ---
def set_movement(fwd_bwd, left_right):
    fwd_bwd = max(-1.0, min(1.0, fwd_bwd))
    left_right = max(-1.0, min(1.0, left_right))
    mcp.channel_a.normalized_value = 0.5 + (fwd_bwd / 2.0)
    mcp.channel_b.normalized_value = 0.5 - (fwd_bwd / 2.0)
    mcp.channel_c.normalized_value = 0.5 + (left_right / 2.0)
    mcp.channel_d.normalized_value = 0.5 - (left_right / 2.0)

def stop_all_movement():
    print("\nStopping movement...")
    set_movement(0.0, 0.0)
    print("Stopped.")


# --- NEW: Cruise Control Mode ---
def execute_cruise_control_mode(stop_distance_ft):
    print(f"\n--- Starting Cruise Control Mode ---")
    print(f"Will stop if an object is within {stop_distance_ft:.1f} ft.")
    print("Press CTRL+C to stop.")

    stop_distance_mm = stop_distance_ft / MM_TO_FEET
    latest_depth_frame_obj = None

    try:
        with kinect.running():
            # We only need the depth frame for this mode
            # --- BUG FIX: Changed from iter_frames to the correct iteration ---
            for frame_type, frame in kinect:
                # Only process depth frames
                if frame_type != FrameType.Depth:
                    continue
                
                depth_image = frame.to_array()
                
                # To avoid stopping for objects far to the side, we focus on a central region
                depth_h, depth_w = depth_image.shape
                center_w_start = depth_w // 4
                center_w_end = 3 * depth_w // 4
                
                # This ROI is the middle half of the screen's width
                center_roi = depth_image[:, center_w_start:center_w_end]

                # Find all valid depth points (non-zero) in our central view
                valid_depths = center_roi[center_roi > 0]

                if valid_depths.size > 0:
                    closest_point_mm = np.min(valid_depths)
                    current_dist_ft = closest_point_mm * MM_TO_FEET

                    # If the closest point is beyond our stop distance, move forward
                    if closest_point_mm > stop_distance_mm:
                        set_movement(MOVE_SPEED_OFFSET, 0.0)
                        status = "CRUISING"
                    # Otherwise, stop
                    else:
                        set_movement(0.0, 0.0)
                        status = "OBJECT TOO CLOSE"
                    
                    print(f"Closest Object: {current_dist_ft:.1f}ft | Status: {status}    ", end='\r')
                else:
                    # If there's no depth data in front, cruise forward cautiously
                    set_movement(MOVE_SPEED_OFFSET, 0.0)
                    print(f"Status: No obstacle data, cruising cautiously...             ", end='\r')
                    
    except KeyboardInterrupt:
        print("\nCruise control mode interrupted by user.")
    finally:
        stop_all_movement()

# --- Core Logic: Person Following Mode (Optimized) ---
def execute_person_follow_mode(target_distance_ft):
    print(f"\n--- Starting Person Follow Mode ---")
    print(f"Target distance: {target_distance_ft:.1f} ft (Dead Zone: +/- {FOLLOW_DEAD_ZONE_FT} ft)")
    print("Press CTRL+C to stop following.")

    target_distance_mm = target_distance_ft / MM_TO_FEET
    upper_bound_mm = target_distance_mm + (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)
    lower_bound_mm = target_distance_mm - (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)

    frame_count = 0
    latest_color_frame_obj = None
    latest_depth_frame_obj = None

    try:
        with kinect.running():
            for frame_type, frame in kinect:
                if frame_type == FrameType.Color:
                    latest_color_frame_obj = frame
                elif frame_type == FrameType.Depth:
                    latest_depth_frame_obj = frame

                frame_count += 1
                if frame_count % PROCESS_EVERY_NTH_FRAME != 0:
                    continue

                if latest_color_frame_obj is None or latest_depth_frame_obj is None:
                    continue

                color_image = latest_color_frame_obj.to_array()
                depth_image = latest_depth_frame_obj.to_array()

                h, w, _ = color_image.shape
                scale_factor = DETECTION_WIDTH_PX / w
                resized_w = DETECTION_WIDTH_PX
                resized_h = int(h * scale_factor)
                frame_resized = cv2.resize(color_image, (resized_w, resized_h))
                
                gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)
                bodies = upper_body_cascade.detectMultiScale(gray, 1.1, 4)

                if len(bodies) > 0:
                    largest_body = max(bodies, key=lambda rect: rect[2] * rect[3])
                    x, y, w, h = largest_body

                    depth_h, depth_w = depth_image.shape
                    depth_scale_x = depth_w / resized_w
                    depth_scale_y = depth_h / resized_h
                    color_cX = x + w // 2
                    
                    depth_roi_x = int((x * depth_scale_x))
                    depth_roi_y = int((y * depth_scale_y))
                    depth_roi_w = int((w * depth_scale_x))
                    depth_roi_h = int((h * depth_scale_y))

                    person_roi = depth_image[depth_roi_y : depth_roi_y + depth_roi_h,
                                             depth_roi_x : depth_roi_x + depth_roi_w]
                    
                    valid_depths = person_roi[person_roi > 0]
                    if valid_depths.size > 0:
                        closest_point_mm = np.min(valid_depths)
                        current_dist_ft = closest_point_mm * MM_TO_FEET

                        fwd_bwd_speed = 0.0
                        if closest_point_mm > upper_bound_mm:
                            fwd_bwd_speed = MOVE_SPEED_OFFSET
                            status_dist = "MOVING FWD"
                        elif closest_point_mm < lower_bound_mm:
                            fwd_bwd_speed = -MOVE_SPEED_OFFSET
                            status_dist = "BACKING UP"
                        else:
                            fwd_bwd_speed = 0.0
                            status_dist = "IN ZONE"
                        
                        left_right_speed = 0.0
                        frame_center_x = resized_w // 2
                        left_bound = frame_center_x - TRACKING_CENTER_DEAD_ZONE_PX
                        right_bound = frame_center_x + TRACKING_CENTER_DEAD_ZONE_PX

                        if color_cX < left_bound:
                            left_right_speed = TURN_SPEED_OFFSET
                            status_turn = "TURN RIGHT"
                        elif color_cX > right_bound:
                            left_right_speed = -TURN_SPEED_OFFSET
                            status_turn = "TURN LEFT"
                        else:
                            left_right_speed = 0.0
                            status_turn = "CENTERED"
                        
                        set_movement(fwd_bwd_speed, left_right_speed)
                        print(f"Dist: {current_dist_ft:.1f}ft ({status_dist}) | Turn: {status_turn} | Target Found ✅ ", end='\r')
                else:
                    set_movement(0.0, 0.0)
                    print(f"Status: SEARCHING for person... ❌                                           ", end='\r')

    except KeyboardInterrupt:
        print("\nPerson following mode interrupted by user.")
    finally:
        stop_all_movement()

# --- Main Program Loop ---
if __name__ == "__main__":
    stop_all_movement()
    
    print("\n--- Kinect-Guided Wheelchair Control ---")
    print("Commands:")
    print("  'follow [feet]'   - Follow a person at a set distance.")
    # --- CHANGE 1: Added cruise command to help text ---
    print("  'cruise [feet]'   - Drive forward, stopping [feet] from any object.")
    print("  'stop'            - Halts any current movement.")
    print("  'exit'            - Closes the program.")
    print("-----------------------------------------")

    while True:
        command_str = input("Enter command > ").lower().strip()
        parts = command_str.split()
        if not parts: continue

        command = parts[0]
        
        # --- CHANGE 2: Added logic to handle the 'cruise' command ---
        if command == "cruise" and len(parts) == 2:
            try:
                value = float(parts[1])
                if value <= 0:
                    print("Error: Stop distance must be a positive number."); continue
                execute_cruise_control_mode(stop_distance_ft=value)
            except ValueError:
                print("Error: Invalid distance. Please enter a number.")

        elif command == "follow" and len(parts) == 2:
            try:
                value = float(parts[1])
                if value <= 0:
                    print("Error: Follow distance must be a positive number."); continue
                execute_person_follow_mode(target_distance_ft=value)
            except ValueError:
                print("Error: Invalid distance. Please enter a number.")
        
        elif command == "stop":
            stop_all_movement()
        elif command == "exit":
            print("Setting to neutral and exiting program.")
            stop_all_movement()
            break
        else:
            print("Invalid command. Try 'follow 5', 'cruise 3', 'stop', or 'exit'.")
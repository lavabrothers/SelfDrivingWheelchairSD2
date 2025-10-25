# StableFollow_v3.py
# Autonomous Wheelchair Control Script with Person Following
# - Flipped L/R controls
# - Backs up when too close to target

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
FOLLOW_DEAD_ZONE_FT = 0.4
TRACKING_CENTER_DEAD_ZONE_PX = 30

# --- PERFORMANCE TUNING ---
PROCESS_EVERY_NTH_FRAME = 3
DETECTION_WIDTH_PX = 360

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

def release_frame(frame_obj):
    """Helper to safely release a freenect2 frame if it exists."""

# --- Core Logic: Person Following Mode (More Responsive) ---
def execute_person_follow_mode(target_distance_ft):
    print(f"\n--- Starting Person Follow Mode (Responsive) ---")
    print(f"Target distance: {target_distance_ft:.1f} ft (Dead Zone: +/- {FOLLOW_DEAD_ZONE_FT} ft)")
    print("Press CTRL+C to stop following.")

    cv2.ocl.setUseOpenCL(True)

    target_distance_mm = target_distance_ft / MM_TO_FEET
    upper_bound_mm = target_distance_mm + (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)
    lower_bound_mm = target_distance_mm - (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)

    # State variables for responsive turning
    last_detection_time = 0
    last_turn_direction = 0.0

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

                color_image_cpu = latest_color_frame_obj.to_array()
                depth_image_cpu = latest_depth_frame_obj.to_array()
                frame_umat = cv2.UMat(color_image_cpu)

                h, w, _ = color_image_cpu.shape
                scale_factor = DETECTION_WIDTH_PX / w
                resized_w = DETECTION_WIDTH_PX
                resized_h = int(h * scale_factor)
                
                frame_resized_umat = cv2.resize(frame_umat, (resized_w, resized_h))
                gray_umat = cv2.cvtColor(frame_resized_umat, cv2.COLOR_BGR2GRAY)
                bodies = upper_body_cascade.detectMultiScale(gray_umat, 1.1, 4)

                if len(bodies) > 0:
                    # --- TARGET FOUND ---
                    last_detection_time = time.monotonic() # Update timestamp
                    largest_body = max(bodies, key=lambda rect: rect[2] * rect[3])
                    x, y, w, h = largest_body

                    depth_h, depth_w = depth_image_cpu.shape
                    depth_scale_x = depth_w / resized_w
                    depth_scale_y = depth_h / resized_h
                    color_cX = x + w // 2
                    
                    depth_roi_x = int(x * depth_scale_x)
                    depth_roi_y = int(y * depth_scale_y)
                    depth_roi_w = int(w * depth_scale_x)
                    depth_roi_h = int(h * depth_scale_y)

                    person_roi = depth_image_cpu[depth_roi_y : depth_roi_y + depth_roi_h,
                                                 depth_roi_x : depth_roi_x + depth_roi_w]
                    
                    valid_depths = person_roi[person_roi > 0]
                    if valid_depths.size > 0:
                        closest_point_mm = np.min(valid_depths)
                        current_dist_ft = closest_point_mm * MM_TO_FEET

                        # --- CHANGE 1: No more backwards movement ---
                        fwd_bwd_speed = 0.0
                        if closest_point_mm > upper_bound_mm:
                            fwd_bwd_speed = MOVE_SPEED_OFFSET
                            status_dist = "MOVING FWD"
                        elif closest_point_mm < lower_bound_mm:
                            fwd_bwd_speed = 0.0 # Stop instead of reversing
                            status_dist = "TOO CLOSE"
                        else:
                            fwd_bwd_speed = 0.0
                            status_dist = "IN ZONE"
                        
                        # --- CHANGE 2: Flipped turning controls back to original ---
                        left_right_speed = 0.0
                        frame_center_x = resized_w // 2
                        left_bound = frame_center_x - TRACKING_CENTER_DEAD_ZONE_PX
                        right_bound = frame_center_x + TRACKING_CENTER_DEAD_ZONE_PX

                        if color_cX < left_bound:
                            left_right_speed = TURN_SPEED_OFFSET # Turn LEFT
                            status_turn = "TURN LEFT"
                        elif color_cX > right_bound:
                            left_right_speed = -TURN_SPEED_OFFSET # Turn RIGHT
                            status_turn = "TURN RIGHT"
                        else:
                            left_right_speed = 0.0
                            status_turn = "CENTERED"
                        
                        last_turn_direction = left_right_speed # Remember this turn
                        set_movement(fwd_bwd_speed, left_right_speed)
                        print(f"Dist: {current_dist_ft:.1f}ft ({status_dist}) | Turn: {status_turn} | Target Found ✅ ", end='\r')
                else:
                    # --- CHANGE 3: TARGET LOST - New responsive logic ---
                    time_since_last_seen = time.monotonic() - last_detection_time
                    if time_since_last_seen < TARGET_LOST_TIMEOUT_S:
                        # If recently lost, stop forward/back and continue last turn
                        set_movement(0.0, last_turn_direction)
                        print(f"Status: RE-ACQUIRING target... searching... ❓                              ", end='\r')
                    else:
                        # If truly lost, stop everything
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
    print("  'follow [feet]'   - Follow a person, maintaining a distance (e.g., 'follow 4').")
    print("  'stop'            - Halts any current movement.")
    print("  'exit'            - Closes the program.")
    print("-----------------------------------------")

    while True:
        command_str = input("Enter command > ").lower().strip()
        parts = command_str.split()
        if not parts: continue

        command = parts[0]
        if command == "follow" and len(parts) == 2:
            try:
                value = float(parts[1])
                if value <= 0:
                    print("Error: Distance must be a positive number."); continue
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
            print("Invalid command format. Try 'follow 5', 'stop', or 'exit'.")

# MPUFullControl.py
# Autonomous Wheelchair Control Script with Person Following

# Import necessary libraries
import time
import board
import math
import numpy as np
import cv2

# --- Adafruit Libraries for Wheelchair Control ---
import adafruit_mpu6050
import adafruit_mcp4728

# --- Freenect2 Library for Kinect V2 ---
from freenect2 import Device, FrameType

# --- Constants ---
MOVE_SPEED_OFFSET = 0.15
TURN_SPEED_OFFSET = 0.25
MM_TO_FEET = 0.00328084

# --- Constants for Person Follow Mode ---
FOLLOW_DEAD_ZONE_FT = 1.0
TRACKING_CENTER_DEAD_ZONE_PX = 50

# --- PERFORMANCE TUNING ---
# This is the key to preventing crashes. The code will only process every Nth frame.
# - Increase this number if the script crashes (e.g., to 7 or 10).
# - Decrease this number for faster responsiveness if the system is stable (e.g., to 3 or 4).
PROCESS_EVERY_NTH_FRAME = 5

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
    print("Please ensure 'haarcascade_upperbody.xml' is in the same directory as the script.")
    exit()


# --- DAC Control Functions ---
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

def stop_all_movement():
    print("\nStopping movement...")
    set_movement(0.0, 0.0)
    print("Stopped.")

# --- Core Logic: Person Following Mode ---
def execute_person_follow_mode(target_distance_ft):
    print(f"\n--- Starting Person Follow Mode ---")
    print(f"Target distance: {target_distance_ft:.1f} ft (Dead Zone: +/- {FOLLOW_DEAD_ZONE_FT} ft)")
    print("Press CTRL+C to stop following.")

    target_distance_mm = target_distance_ft / MM_TO_FEET
    upper_bound_mm = target_distance_mm + (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)
    lower_bound_mm = target_distance_mm - (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)

    # Variables for frame skipping logic
    frame_count = 0
    latest_color_frame = None
    latest_depth_frame = None

    try:
        with kinect.running():
            for frame_type, frame in kinect:
                # Step 1: Always grab the latest frames to keep the input queue clear
                if frame_type == FrameType.Color:
                    latest_color_frame = frame.to_array()
                elif frame_type == FrameType.Depth:
                    latest_depth_frame = frame.to_array()

                frame_count += 1

                # Step 2: Skip the expensive processing on most frames
                if frame_count % PROCESS_EVERY_NTH_FRAME != 0:
                    continue

                # Step 3: Only proceed if we have a valid pair of frames
                if latest_color_frame is None or latest_depth_frame is None:
                    continue

                # The expensive processing only happens here, periodically
                color_image = latest_color_frame
                depth_image = latest_depth_frame

                # 1. Person Detection on the Color Image
                h, w, _ = color_image.shape
                scale_factor = 640 / w
                resized_w = 640
                resized_h = int(h * scale_factor)
                frame_resized = cv2.resize(color_image, (resized_w, resized_h))
                
                gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)
                bodies = upper_body_cascade.detectMultiScale(gray, 1.1, 4)

                if len(bodies) > 0:
                    largest_body = max(bodies, key=lambda rect: rect[2] * rect[3])
                    x, y, w, h = largest_body

                    # 2. Map Color Coordinates to Depth Coordinates
                    depth_h, depth_w = depth_image.shape
                    depth_scale_x = depth_w / resized_w
                    depth_scale_y = depth_h / resized_h
                    color_cX = x + w // 2
                    
                    depth_roi_x = int((x * depth_scale_x))
                    depth_roi_y = int((y * depth_scale_y))
                    depth_roi_w = int((w * depth_scale_x))
                    depth_roi_h = int((h * depth_scale_y))

                    # 3. Get Depth from the Person's Location
                    person_roi = depth_image[depth_roi_y : depth_roi_y + depth_roi_h,
                                             depth_roi_x : depth_roi_x + depth_roi_w]
                    
                    valid_depths = person_roi[person_roi > 0]
                    if valid_depths.size > 0:
                        closest_point_mm = np.min(valid_depths)
                        current_dist_ft = closest_point_mm * MM_TO_FEET

                        # 4. Control Logic (Forward/Backward)
                        fwd_bwd_speed = 0.0
                        if closest_point_mm > upper_bound_mm:
                            fwd_bwd_speed = MOVE_SPEED_OFFSET
                            status_dist = "MOVING FWD"
                        elif closest_point_mm < lower_bound_mm:
                            fwd_bwd_speed = 0.0
                            status_dist = "TOO CLOSE"
                        else:
                            fwd_bwd_speed = 0.0
                            status_dist = "IN ZONE"
                        
                        # 5. Control Logic (Turning)
                        left_right_speed = 0.0
                        frame_center_x = resized_w // 2
                        left_bound = frame_center_x - TRACKING_CENTER_DEAD_ZONE_PX
                        right_bound = frame_center_x + TRACKING_CENTER_DEAD_ZONE_PX

                        if color_cX < left_bound:
                            left_right_speed = -TURN_SPEED_OFFSET
                            status_turn = "TURN LEFT"
                        elif color_cX > right_bound:
                            left_right_speed = TURN_SPEED_OFFSET
                            status_turn = "TURN RIGHT"
                        else:
                            left_right_speed = 0.0
                            status_turn = "CENTERED"
                        
                        # 6. Set Movement
                        set_movement(fwd_bwd_speed, left_right_speed)
                        print(f"Dist: {current_dist_ft:.1f}ft ({status_dist}) | Turn: {status_turn} | Target Found ✅ ", end='\r')
                else:
                    # If no person is found, stop movement but continue searching
                    set_movement(0.0, 0.0)
                    print(f"Status: SEARCHING for person... ❌                                           ", end='\r')

    except KeyboardInterrupt:
        print("\nPerson following mode interrupted by user.")
    finally:
        # IMPORTANT: Always stop movement when the function ends for any reason
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
#!/usr/bin/env python3

# StableFollow_v3.py
# Autonomous Wheelchair Control Script with Person Following
# - Flipped L/R controls
# - Backs up when too close to target
#
# CONVERTED TO PYLIBFREENECT2
# - ADDED CV2 VIDEO STREAM FOR DEBUGGING

import time
import board
import numpy as np
import cv2
import adafruit_mcp4728
import sys

# --- New Imports for pylibfreenect2 ---
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, FrameMap

# --- Constants ---
MOVE_SPEED_OFFSET = 0.15
TURN_SPEED_OFFSET = 0.25
MM_TO_FEET = 0.00328084
FOLLOW_DEAD_ZONE_FT = 0.4
TRACKING_CENTER_DEAD_ZONE_PX = 30
# Added missing constant from original file's logic
TARGET_LOST_TIMEOUT_S = 5.0 

# --- PERFORMANCE TUNING ---
PROCESS_EVERY_NTH_FRAME = 3
DETECTION_WIDTH_PX = 360

# --- Module-level Globals for Kinect ---
freenect2 = None
kinect = None
listener = None

# --- Hardware Setup ---
try:
    print("Initializing I2C and sensors...")
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MCP4728 found and initialized. ✅")
except Exception as e:
    print(f"Error: Could not find a required I2C device. Please check connections.")
    print(f"Details: {e}")
    sys.exit(1)

# --- REPLACED: Kinect V2 Hardware Setup (pylibfreenect2) ---
try:
    print("Initializing Kinect V2 (pylibfreenect2)...")
    freenect2 = Freenect2()
    num_devices = freenect2.enumerateDevices()
    if num_devices == 0:
        print("Error: No Kinect V2 devices found!")
        sys.exit(1)
        
    serial = freenect2.getDeviceSerialNumber(0)
    kinect = freenect2.openDevice(serial)
    
    # Request both Color and Depth frames
    types = FrameType.Color | FrameType.Depth
    listener = SyncMultiFrameListener(types)
    
    kinect.setColorFrameListener(listener)
    kinect.setIrAndDepthFrameListener(listener)
    
    print(f"Starting Kinect V2 stream (Serial: {serial})...")
    kinect.start()
    print("Kinect V2 found and initialized. ✅")
    
except Exception as e:
    print(f"Error initializing Kinect V2: {e}")
    if "LIBUSB_ERROR_ACCESS" in str(e):
         print("\n--- PERMISSION ERROR ---")
         print("This is likely a USB permission issue.")
    sys.exit(1)
# --- END OF REPLACEMENT ---

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
    if kinect: # Shut down kinect if it started
        kinect.stop()
        kinect.close()
    sys.exit(1)

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

# --- Core Logic: Person Following Mode (More Responsive) ---
def execute_person_follow_mode(target_distance_ft):
    print(f"\n--- Starting Person Follow Mode (Responsive) ---")
    print(f"Target distance: {target_distance_ft:.1f} ft (Dead Zone: +/- {FOLLOW_DEAD_ZONE_FT} ft)")
    print("Press CTRL+C to stop following. (Or 'q' in the video window)")

    cv2.ocl.setUseOpenCL(True)

    target_distance_mm = target_distance_ft / MM_TO_FEET
    upper_bound_mm = target_distance_mm + (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)
    lower_bound_mm = target_distance_mm - (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)

    # State variables for responsive turning
    last_detection_time = 0
    last_turn_direction = 0.0

    frame_count = 0
    
    # Create FrameMap for pylibfreenect2
    frames = FrameMap()
    
    # --- ADDED: Create a window for the video feed ---
    window_name = "Kinect V2 Detection"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    try:
        # --- REPLACED: Main Loop Structure ---
        while True:
            # 1. Wait for a new frame pair
            if not listener.waitForNewFrame(frames, 10 * 1000): # 10 sec timeout
                print("Kinect timeout, retrying...", end='\r')
                continue
            
            color_frame = frames[FrameType.Color]
            depth_frame = frames[FrameType.Depth]

            # 2. Get data using the stable pipeline
            # Get as uint8 (from dev-tools)
            color_image_bgra_1080p = color_frame.asarray(dtype=np.uint8)
            depth_image_cpu = depth_frame.asarray()
            
            # 3. Release frames *immediately*
            listener.release(frames)

            # 4. Convert 1080p BGRA -> 1080p BGR (from dev-tools)
            color_image_bgr_1080p = cv2.cvtColor(color_image_bgra_1080p, cv2.COLOR_BGRA2BGR)
            # --- END OF REPLACEMENT ---

            frame_count += 1
            if frame_count % PROCESS_EVERY_NTH_FRAME != 0:
                continue

            # The rest of your original logic is preserved
            # It now uses the 1080p BGR frame, just like the original code
            frame_umat = cv2.UMat(color_image_bgr_1080p)

            h, w, _ = color_image_bgr_1080p.shape # Get shape from BGR frame
            scale_factor = DETECTION_WIDTH_PX / w
            resized_w = DETECTION_WIDTH_PX
            resized_h = int(h * scale_factor)
            
            frame_resized_umat = cv2.resize(frame_umat, (resized_w, resized_h))
            
            # --- ADDED: Get a numpy array copy for drawing ---
            frame_resized_display = frame_resized_umat.get()
            
            gray_umat = cv2.cvtColor(frame_resized_umat, cv2.COLOR_BGR2GRAY)
            bodies = upper_body_cascade.detectMultiScale(gray_umat, 1.1, 4)

            # --- ADDED: Draw the turning dead-zone ---
            frame_center_x = resized_w // 2
            left_bound = frame_center_x - TRACKING_CENTER_DEAD_ZONE_PX
            right_bound = frame_center_x + TRACKING_CENTER_DEAD_ZONE_PX
            # Draw as faint blue lines
            cv2.line(frame_resized_display, (left_bound, 0), (left_bound, resized_h), (255, 100, 100), 1)
            cv2.line(frame_resized_display, (right_bound, 0), (right_bound, resized_h), (255, 100, 100), 1)

            if len(bodies) > 0:
                # --- TARGET FOUND ---
                last_detection_time = time.monotonic() # Update timestamp
                largest_body = max(bodies, key=lambda rect: rect[2] * rect[3])
                x, y, w, h = largest_body

                # --- ADDED: Draw rectangle on the display frame ---
                # Draw a bright green box around the detected torso
                cv2.rectangle(frame_resized_display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # ---

                depth_h, depth_w = depth_image_cpu.shape
                depth_scale_x = depth_w / resized_w
                depth_scale_y = depth_h / resized_h
                color_cX = x + w // 2
                
                depth_roi_x = int(x * depth_scale_x)
                depth_roi_y = int(y * depth_scale_y)
                depth_roi_w = int(w * depth_scale_x)
                depth_roi_h = int(h * depth_scale_y)
                
                # Clamp ROI values to be within depth map bounds
                depth_roi_x = max(0, depth_roi_x)
                depth_roi_y = max(0, depth_roi_y)
                depth_roi_w = min(depth_w - depth_roi_x, depth_roi_w)
                depth_roi_h = min(depth_h - depth_roi_y, depth_roi_h)

                person_roi = depth_image_cpu[depth_roi_y : depth_roi_y + depth_roi_h,
                                             depth_roi_x : depth_roi_x + depth_roi_w]
                
                valid_depths = person_roi[person_roi > 0]
                if valid_depths.size > 0:
                    closest_point_mm = np.min(valid_depths)
                    current_dist_ft = closest_point_mm * MM_TO_FEET

                    # --- Original logic from your file ---
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
                    
                    left_right_speed = 0.0
                    
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
                # --- TARGET LOST ---
                time_since_last_seen = time.monotonic() - last_detection_time
                if time_since_last_seen < TARGET_LOST_TIMEOUT_S:
                    # If recently lost, stop forward/back and continue last turn
                    set_movement(0.0, last_turn_direction)
                    print(f"Status: RE-ACQUIRING target... searching... ❓                              ", end='\r')
                else:
                    # If truly lost, stop everything
                    set_movement(0.0, 0.0)
                    print(f"Status: SEARCHING for person... ❌                                           ", end='\r')

            # --- ADDED: Display the video stream ---
            # Show the frame (with or without a box)
            cv2.imshow(window_name, frame_resized_display)
            
            # Check for 'q' key press to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n'q' pressed in video window. Stopping follow mode.")
                break # Exit the while loop

    except KeyboardInterrupt:
        print("\nPerson following mode interrupted by user.")
    finally:
        # Stop motors first
        stop_all_movement()
        
        # --- ADDED: Clean up the OpenCV window ---
        cv2.destroyWindow(window_name)
        
        print("Follow mode stopped.")

# --- Main Program Loop ---
if __name__ == "__main__":
    stop_all_movement()
    
    print("\n--- Kinect-Guided Wheelchair Control ---")
    print("Commands:")
    print("  'follow [feet]'   - Follow a person, maintaining a distance (e.g., 'follow 4').")
    print("  'stop'            - Halts any current movement.")
    print("  'exit'            - Closes the program.")
    print("-----------------------------------------")

    try:
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
                
                # --- ADDED: Explicit shutdown ---
                if kinect:
                    kinect.stop()
                    kinect.close()
                print("Kinect stopped.")
                # ---
                
                break
            else:
                print("Invalid command format. Try 'follow 5', 'stop', or 'exit'.")
    except KeyboardInterrupt:
        print("\nUser exit detected.")
    finally:
        # Final safety cleanup
        print("Ensuring motors are stopped and Kinect is closed.")
        stop_all_movement()
        if kinect:
            kinect.stop()
            kinect.close()
            
        # --- ADDED: Final safety cleanup for any OpenCV windows ---
        cv2.destroyAllWindows()
        print("Shutdown complete.")
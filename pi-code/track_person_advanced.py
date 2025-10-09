# track_person_advanced.py
# This script uses a combination of the Kinect V2's RGB and Depth cameras
# to robustly detect and track a person.

import time
import board
import numpy as np
import cv2

# --- Adafruit Libraries for Wheelchair Control ---
import adafruit_mpu6050
import adafruit_mcp4728

# --- Freenect2 Library for Kinect V2 ---
from freenect2 import Device, FrameType

# --- Constants ---
# --- Proportional Control ---
PROPORTIONAL_GAIN = 0.0015
MAX_TURN_SPEED = 0.30
CENTER_DEAD_ZONE_PX = 60

# --- Performance ---
# Downscale width for the color image to speed up processing
RGB_PROCESS_WIDTH = 640

# --- Person Detection ---
CASCADE_PATH = "pi-code/haarcascade_upperbody.xml"
# The size of the ROI in the depth map to search for the person
DEPTH_ROI_SIZE = 80

# --- Global State Variables ---
current_fwd_bwd = 0.0
current_left_right = 0.0
# Store the latest frames
latest_frames = {
    FrameType.Color: None,
    FrameType.Depth: None
}

# --- Hardware Setup ---
try:
    print("Initializing I2C and sensors...")
    i2c = board.I2C()
    mpu = adafruit_mpu6050.MPU6050(i2c)
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MPU6050 and MCP4728 found and initialized. ✅")
except Exception as e:
    print(f"Error: Could not find a required I2C device: {e}")
    exit()

try:
    print("Initializing Kinect V2 device...")
    kinect = Device()
    print("Kinect V2 found and initialized. ✅")
except Exception as e:
    print(f"Error: Could not initialize Kinect V2: {e}")
    exit()

# --- Person Detection Setup ---
try:
    print(f"Loading Haar Cascade classifier from '{CASCADE_PATH}'...")
    body_cascade = cv2.CascadeClassifier(CASCADE_PATH)
    if body_cascade.empty():
        raise IOError("Failed to load cascade file.")
    print("Classifier loaded successfully. ✅")
except Exception as e:
    print(f"Error: Could not load the Haar Cascade classifier: {e}")
    exit()

# --- DAC Control Functions ---
def set_movement(fwd_bwd, left_right):
    global current_fwd_bwd, current_left_right
    fwd_bwd = max(-1.0, min(1.0, fwd_bwd))
    left_right = max(-1.0, min(1.0, left_right))
    mcp.channel_a.normalized_value = 0.5 + (fwd_bwd / 2.0)
    mcp.channel_b.normalized_value = 0.5 - (fwd_bwd / 2.0)
    mcp.channel_c.normalized_value = 0.5 - (left_right / 2.0)
    mcp.channel_d.normalized_value = 0.5 + (left_right / 2.0)
    current_fwd_bwd = fwd_bwd
    current_left_right = left_right

def stop_all_movement():
    print("\nStopping movement...")
    set_movement(0.0, 0.0)
    print("Stopped.")

# --- Main Tracking Logic ---
def execute_advanced_tracking():
    print(f"\n--- Starting Depth-Assisted Person Tracking ---")
    print("Press CTRL+C to stop.")

    try:
        with kinect.running():
            # This listener function will be called by the freenect2 library
            # whenever a new frame is available.
            def frame_listener(frame_type, frame):
                latest_frames[frame_type] = frame

            # This listener function will be called by the freenect2 library
            # whenever a new frame is available.
            def frame_listener(frame_type, frame):
                latest_frames[frame_type] = frame

            # Explicitly set listeners for both color and depth frames
            kinect.color_frame_listener = frame_listener
            kinect.ir_and_depth_frame_listener = frame_listener

            # Start both streams
            kinect.start()

            while True:
                # Get the most recent color and depth frames
                color_frame = latest_frames[FrameType.Color]
                depth_frame = latest_frames[FrameType.Depth]

                # Wait until we have at least one of each frame
                if color_frame is None or depth_frame is None:
                    time.sleep(0.05) # Shorter delay for faster frame acquisition
                    continue

                # --- Step 1: Process the Color Frame for Person Detection ---
                color_image_bgra = color_frame.to_array()
                color_image_bgr = color_image_bgra[:, :, :3]

                scale = RGB_PROCESS_WIDTH / color_image_bgr.shape[1]
                new_height = int(color_image_bgr.shape[0] * scale)
                resized_frame = cv2.resize(color_image_bgr, (RGB_PROCESS_WIDTH, new_height))
                gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)

                bodies = body_cascade.detectMultiScale(gray, 1.1, 5, minSize=(50, 100))

                if len(bodies) > 0:
                    largest_body = max(bodies, key=lambda rect: rect[2] * rect[3])
                    x, y, w, h = largest_body
                    
                    # --- Step 2: Map RGB Detection to Depth Space ---
                    # Normalize the coordinates of the detected body's center
                    norm_x = (x + w / 2) / RGB_PROCESS_WIDTH
                    norm_y = (y + h / 2) / new_height

                    depth_data = depth_frame.to_array()
                    depth_h, depth_w = depth_data.shape

                    # Find the approximate center in the depth image
                    depth_target_x = int(norm_x * depth_w)
                    depth_target_y = int(norm_y * depth_h)

                    # --- Step 3: Lock-On with Depth Data ---
                    roi_half = DEPTH_ROI_SIZE // 2
                    y1 = max(0, depth_target_y - roi_half)
                    y2 = min(depth_h, depth_target_y + roi_half)
                    x1 = max(0, depth_target_x - roi_half)
                    x2 = min(depth_w, depth_target_x + roi_half)
                    
                    depth_roi = depth_data[y1:y2, x1:x2]
                    valid_depths = depth_roi[depth_roi > 0]

                    if valid_depths.size > 0:
                        # Find the actual closest point within the targeted ROI
                        min_depth = np.min(valid_depths)
                        min_depth_coords = np.where(depth_roi == min_depth)
                        
                        # The final target X is relative to the full depth frame
                        final_target_x = int(np.median(min_depth_coords[1])) + x1

                        # --- Step 4: Proportional Turning ---
                        frame_center_x = depth_w // 2
                        error = final_target_x - frame_center_x
                        
                        turn_speed = 0.0
                        if abs(error) > (CENTER_DEAD_ZONE_PX // 2):
                            turn_speed = error * PROPORTIONAL_GAIN
                            turn_speed = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, turn_speed))
                            set_movement(0.0, turn_speed)
                            status = "TRACKING"
                        else:
                            set_movement(0.0, 0.0)
                            status = "CENTERED"
                        
                        print(f"Status: {status} | Target X: {final_target_x} | Error: {error} | Turn: {turn_speed:.2f} | Dist: {min_depth/1000:.2f}m", end='\r')
                    else:
                        set_movement(0.0, 0.0)
                        print(f"Status: LOCKED (No depth in ROI)                                       ", end='\r')
                else:
                    set_movement(0.0, 0.0)
                    print(f"Status: SEARCHING... (No person detected)                                       ", end='\r')
                
                # Small delay to prevent this loop from running too fast
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nTracking mode interrupted by user.")
    finally:
        stop_all_movement()
        kinect.stop()
        kinect.close()

# --- Main Program Execution ---
if __name__ == "__main__":
    set_movement(0.0, 0.0)
    execute_advanced_tracking()
    print("\nProgram terminated.")

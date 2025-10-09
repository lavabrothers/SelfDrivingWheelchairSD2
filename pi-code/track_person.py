# track_person.py
# This script uses the Kinect V2's RGB camera and a Haar Cascade classifier
# to detect and track a person's upper body, keeping them centered in the view.

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
PROCESS_EVERY_NTH_FRAME = 5

# --- Person Detection ---
# Path to the Haar Cascade XML file for upper body detection
CASCADE_PATH = "pi-code/haarcascade_upperbody.xml"

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

# --- Person Detection Setup ---
try:
    print(f"Loading Haar Cascade classifier from '{CASCADE_PATH}'...")
    body_cascade = cv2.CascadeClassifier(CASCADE_PATH)
    if body_cascade.empty():
        raise IOError("Failed to load cascade file.")
    print("Classifier loaded successfully. ✅")
except Exception as e:
    print(f"Error: Could not load the Haar Cascade classifier.")
    print(f"Details: {e}")
    exit()


# --- DAC Control Functions (for wheelchair movement) ---
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
def execute_person_tracking():
    print(f"\n--- Starting Person Tracking Mode ---")
    print(f"Will turn to keep the largest detected person in the center of the view.")
    print("Press CTRL+C to stop tracking and exit.")

    frame_count = 0
    try:
        with kinect.running():
            for frame_type, frame in kinect:
                if frame_type == FrameType.Color:
                    frame_count += 1
                    if frame_count % PROCESS_EVERY_NTH_FRAME != 0:
                        continue

                    # Get color image data (BGRA format)
                    color_image_bgra = frame.to_array()
                    # Convert to 3-channel BGR for OpenCV
                    color_image_bgr = color_image_bgra[:, :, :3]

                    # --- PERFORMANCE OPTIMIZATION ---
                    # Resize the image to a smaller size for faster processing.
                    new_width = 640
                    scale = new_width / color_image_bgr.shape[1]
                    new_height = int(color_image_bgr.shape[0] * scale)
                    resized_frame = cv2.resize(color_image_bgr, (new_width, new_height))

                    # Convert to grayscale for the classifier
                    gray = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
                    
                    # Get dimensions from the resized frame
                    height, width, _ = resized_frame.shape

                    # Detect upper bodies in the grayscale image
                    bodies = body_cascade.detectMultiScale(
                        gray,
                        scaleFactor=1.1,
                        minNeighbors=5,
                        minSize=(50, 100) # Min size of a detectable body
                    )

                    if len(bodies) > 0:
                        # Find the largest detected body by area (w * h)
                        largest_body = max(bodies, key=lambda rect: rect[2] * rect[3])
                        x, y, w, h = largest_body
                        
                        # Calculate the center of the detected body
                        target_x = x + w // 2

                        # --- Proportional Turning Logic ---
                        frame_center_x = width // 2
                        error = target_x - frame_center_x
                        
                        status = ""
                        turn_speed = 0.0
                        if abs(error) > (CENTER_DEAD_ZONE_PX // 2):
                            turn_speed = error * PROPORTIONAL_GAIN
                            turn_speed = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, turn_speed))
                            set_movement(0.0, turn_speed)
                            status = "TRACKING"
                        else:
                            set_movement(0.0, 0.0)
                            status = "CENTERED"
                        
                        print(f"Status: {status} | Target X: {target_x} | Error: {error} | Turn Speed: {turn_speed:.2f}", end='\r')

                    else:
                        # If no bodies are found, stop moving
                        set_movement(0.0, 0.0)
                        print(f"Status: SEARCHING... (No person detected)                                       ", end='\r')

    except KeyboardInterrupt:
        print("\nTracking mode interrupted by user.")
    finally:
        stop_all_movement()

# --- Main Program Execution ---
if __name__ == "__main__":
    set_movement(0.0, 0.0)
    execute_person_tracking()
    print("Program terminated.")

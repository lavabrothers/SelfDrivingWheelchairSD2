"""
depth-local-control.py

This script provides a local, keyboard-controlled interface for the self-driving
wheelchair, integrating real-time motor control with live Kinect V2 depth and
color camera feeds. It is primarily intended as a development tool for testing
motor responses and basic depth perception in a visual environment.

The script allows direct manual control of the wheelchair's movement using
keyboard inputs ('w', 'a', 's', 'd') and displays two OpenCV windows:
1. A flipped RGB feed from the Kinect's color camera (driver's view).
2. A depth visualization with overlaid information about the nearest object
   and visual indicators for cropped regions (floor and sides).

Key Features:
- Initializes the MCP4728 DAC for motor control.
- Initializes the Kinect V2 sensor and its frame listeners.
- Uses `pynput` for non-blocking keyboard input to control movement.
- Displays live RGB and depth camera feeds using OpenCV.
- Processes depth data to find the nearest object within a cropped region,
  displaying its distance and angle.
- Provides visual cues for areas of the depth frame that are ignored.
- Ensures graceful shutdown of all hardware and display windows upon exit.

Dependencies:
- board, adafruit_mcp4728: For I2C communication with the DAC.
- time: For delays.
- pynput: For keyboard input handling.
- sys, os: For system and path operations.
- cv2 (OpenCV): For image processing, resizing, and display.
- numpy: For numerical operations on image and depth data.
- pylibfreenect2: Python wrapper for libfreenect2, used to interface with the Kinect V2.
- person_detector: Custom module (from Refactoring) providing Kinect initialization
                   and constants for depth processing.
"""

# --- Original Motor Control Imports ---
import board
import adafruit_mcp4728
import time
from pynput import keyboard

# --- Imports for Kinect v2 Camera ---
import sys
import cv2
import numpy as np
from pylibfreenreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame, FrameMap

# --- ADDED: Import for OS path manipulation ---
import os 

# --- Setup: DAC (Motor Controller) ---
try:
    i2c = board.I2C() # Initialize I2C bus.
    mcp = adafruit_mcp4728.MCP4728(i2c) # Create MCP4728 DAC object.
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    sys.exit(1) # Exit if DAC is not found, as it's critical for control.

# --- State Variables ---
keys_pressed = {
    'w': False, # Forward
    's': False, # Backward
    'a': False, # Turn Left
    'd': False  # Turn Right
}

running = True # Global flag to signal threads to stop.

# --- Motor Control Functions ---
def update_dac_channels():
    """
    Reads the `keys_pressed` state and sets the DAC values accordingly to control
    the wheelchair's movement.

    This function translates keyboard input into normalized DAC values (0.0-1.0)
    for forward/backward and left/right movement.
    """
    val_a = 0.5  # Neutral for Forward
    val_b = 0.5  # Neutral for Backward
    val_c = 0.5  # Neutral for Right
    val_d = 0.5  # Neutral for Left

    # Adjust forward/backward channels based on 'w' and 's' keys.
    if keys_pressed['w'] and not keys_pressed['s']:
        val_a = 0.75 # Move forward
        val_b = 0.25
    elif keys_pressed['s'] and not keys_pressed['w']:
        val_a = 0.25
        val_b = 0.75 # Move backward

    # Adjust left/right channels based on 'd' and 'a' keys.
    if keys_pressed['d'] and not keys_pressed['a']:
        val_c = 0.75 # Turn right
        val_d = 0.25
    elif keys_pressed['a'] and not keys_pressed['d']:
        val_c = 0.25
        val_d = 0.75 # Turn left

    # Apply the calculated values to the DAC channels.
    mcp.channel_a.normalized_value = val_a
    mcp.channel_b.normalized_value = val_b
    mcp.channel_c.normalized_value = val_c
    mcp.channel_d.normalized_value = val_d
    
    print_status(val_a, val_b, val_c, val_d)

def print_status(a: float, b: float, c: float, d: float):
    """
    Prints the current normalized values of all DAC channels to the console
    in a readable, single-line format.

    Args:
        a (float): Normalized value for channel A.
        b (float): Normalized value for channel B.
        c (float): Normalized value for channel C.
        d (float): Normalized value for channel D.
    """
    print(
        f"\rStatus -> Fwd(A): {a:.2f}, Bwd(B): {b:.2f} | "
        f"Right(C): {c:.2f}, Left(D): {d:.2f}",
        end='' # Keep output on the same line.
    )

def show_instructions():
    """
    Displays the keyboard control instructions and program information to the user.
    """
    print("\n--- Real-time Wheelchair Control & Camera ---")
    print("Press and HOLD keys to move:")
    print("  'w' - Move forward")
    print("  's' - Move backward")
    print("  'd' - Move right")
    print("  'a' - Move left")
    print("\n  Press 'ESC' (in this terminal or camera window) to stop.")
    print("  You can also press 'q' in any camera window to quit.")
    print("-------------------------------------------------")
    print("Starting control listener and camera feed...")

# --- Keyboard Listener Functions ---

def on_press(key):
    """
    Callback function for `pynput` when a key is pressed.
    Updates the `keys_pressed` dictionary and triggers DAC update.
    """
    try:
        if key.char in keys_pressed:
            if not keys_pressed[key.char]: # Only update if key was not already pressed.
                keys_pressed[key.char] = True
                update_dac_channels()
    except AttributeError:
        pass # Ignore special keys (e.g., Shift, Ctrl) that don't have a .char attribute.

def on_release(key):
    """
    Callback function for `pynput` when a key is released.
    Updates the `keys_pressed` dictionary and triggers DAC update.
    Handles 'ESC' key to stop the program.
    """
    global running 
    try:
        if key.char in keys_pressed:
            keys_pressed[key.char] = False
            update_dac_channels()
    except AttributeError:
        pass # Ignore special keys.
    
    if key == keyboard.Key.esc:
        print("\n\nEscape key pressed. Stopping and exiting.")
        running = False      # Signal the main loop to stop.
        return False         # Stops the keyboard listener thread.

# --- Main Program ---
if __name__ == "__main__":

    # --- Path correction for importing `person_detector` module ---
    # Get the directory of the current script (e.g., `pi-code/dev-tools/`).
    script_dir = os.path.dirname(os.path.realpath(__file__))
    # Get the parent directory (project root, e.g., `pi-code/`).
    project_root = os.path.join(script_dir, '..')
    # Add the 'Refactoring' directory to Python's system path.
    refactoring_dir = os.path.join(project_root, 'Refactoring')
    sys.path.append(refactoring_dir)
    
    try:
        # Import the `person_detector` module from the 'Refactoring' directory.
        import person_detector as kinect_sensor
    except ImportError:
        print(f"Error: Could not import 'person_detector.py' from {refactoring_dir}")
        print("Please check the file path and ensure the module exists.")
        sys.exit(1)
    # --- End of path correction ---
    
    # 1. Set initial DAC state to neutral (stop).
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    
    show_instructions()

    # 2. Initialize the Kinect sensor using the `person_detector` module.
    if not kinect_sensor.initialize_detector():
        print("Error: Kinect initialization failed. Exiting.")
        sys.exit(1)
    
    print("Kinect v2 camera and depth sensor started.")

    # 3. Start `pynput` keyboard listener in a non-blocking thread.
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # 4. Main Camera and Control Loop.
    
    # Create the FrameMap object ONCE outside the loop to reuse it.
    frames = FrameMap()

    while running: # Loop continues as long as the `running` flag is True.
        # Acquire the Kinect lock to safely access frames, as `person_detector` also uses it.
        with kinect_sensor.kinect_lock:
            # Wait for new frames from the Kinect sensor.
            if not kinect_sensor.listener.waitForNewFrame(frames, 10 * 1000): # 10-second timeout.
                print("Timeout waiting for frames! Skipping frame processing.", end='\r')
                continue # Continue to the next iteration if timeout occurs.
                
            color_frame = frames[FrameType.Color]
            depth_frame = frames[FrameType.Depth]

            # Get the raw depth map as a NumPy array.
            depth_map = depth_frame.asarray()
            
            # Release the frames immediately after copying their data.
            kinect_sensor.listener.release(frames)

        # --- Perform sensor logic locally for visualization ---
        # This logic is adapted from `person_detector.get_nearest_object_angle`
        # to provide real-time feedback in this development tool.
        
        depth, angle, coords = None, None, None # Initialize default values.
        
        try:
            # Apply cropping logic using constants from `kinect_sensor` module.
            height, width = depth_map.shape # Expected: 424, 512.
            crop_row_bottom = int(height * (1.0 - kinect_sensor.CROP_BOTTOM_RATIO))
            crop_col_left = int(width * kinect_sensor.CROP_LEFT_RATIO)
            crop_col_right = int(width * (1.0 - kinect_sensor.CROP_RIGHT_RATIO))

            depth_map_roi = depth_map[0:crop_row_bottom, crop_col_left:crop_col_right]
            
            # Find valid (non-zero) depths within the Region of Interest.
            valid_depths = depth_map_roi[depth_map_roi > 0]
            
            if valid_depths.size > 0:
                # Object was found, calculate its depth, angle, and coordinates.
                depth = np.percentile(valid_depths, 1) # Use 1st percentile for robust minimum.

                # Find pixel coordinates of the minimum depth within the ROI.
                search_map = np.where(depth_map_roi == 0, 999999, depth_map_roi) # Replace 0s with large number.
                y_roi, x_roi = np.unravel_index(np.argmin(search_map), search_map.shape)

                # Convert ROI coordinates back to full frame coordinates.
                x_full_frame = x_roi + crop_col_left
                y_full_frame = y_roi 
                coords = (x_full_frame, y_full_frame)

                # Calculate horizontal angle from the center.
                normalized_x = (x_full_frame - (kinect_sensor.KINECT_WIDTH / 2.0)) / (kinect_sensor.KINECT_WIDTH / 2.0)
                angle = normalized_x * (kinect_sensor.KINECT_H_FOV / 2.0)
                
        except Exception as e:
            print(f"Error during depth analysis: {e}", end='\r')
            depth, angle, coords = None, None, None # Reset on error.
        
        # --- Process and Display the RGB Driver View (Window 1) ---
        img = color_frame.asarray(dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR) # Convert from BGRA to BGR.
        img_small = cv2.resize(img, (960, 540)) # Resize for display.
        img_flipped = cv2.flip(img_small, 1) # Flip horizontally for a "mirror" view.
        cv2.imshow("Kinect v2 RGB Feed (Driver View)", img_flipped)

        # --- Process and Display the Depth Object View (Window 2) ---
        # Normalize depth map for visual display.
        depth_viz = np.clip(depth_map, 500, 4500) # Clip depths between 0.5m and 4.5m.
        depth_viz = (depth_viz - 500) / (4000.0)  # Remap to 0-1 range.
        depth_viz = (255 * (1.0 - depth_viz)).astype(np.uint8) # Invert colors (closer = brighter) and scale to 0-255.
        depth_viz[depth_map == 0] = 0 # Make "no-reading" (0) pixels black.
        
        image_color = cv2.cvtColor(depth_viz, cv2.COLOR_GRAY2BGR) # Convert to 3-channel BGR for drawing.

        # Get frame dimensions for drawing.
        viz_height = image_color.shape[0] # 424.
        viz_width = kinect_sensor.KINECT_WIDTH # 512 (Use constant for accuracy).

        # Draw the BOTTOM crop line and label.
        crop_row_viz = int(viz_height * (1.0 - kinect_sensor.CROP_BOTTOM_RATIO))
        cv2.line(image_color, (0, crop_row_viz), (viz_width - 1, crop_row_viz), 
                 (255, 255, 0), 1) # Cyan line.
        cv2.putText(image_color, "IGNORING THIS AREA (FLOOR)", 
                    (10, crop_row_viz + 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Draw the LEFT peripheral crop line and label.
        crop_col_left_viz = int(viz_width * kinect_sensor.CROP_LEFT_RATIO)
        cv2.line(image_color, (crop_col_left_viz, 0), (crop_col_left_viz, viz_height - 1),
                 (255, 255, 0), 1) # Cyan line.
        cv2.putText(image_color, "IGNORING", (5, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Draw the RIGHT peripheral crop line and label.
        crop_col_right_viz = int(viz_width * (1.0 - kinect_sensor.CROP_RIGHT_RATIO))
        cv2.line(image_color, (crop_col_right_viz, 0), (crop_col_right_viz, viz_height - 1),
                 (255, 255, 0), 1) # Cyan line.
        cv2.putText(image_color, "IGNORING", (crop_col_right_viz + 5, 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Draw the detected object (red circle, depth, and angle) if found and within active area.
        if coords is not None and depth is not None:
            x, y = int(coords[0]), int(coords[1])
            # Only draw the circle if it's within the *active* (non-cropped) area.
            if (y < crop_row_viz and 
                x > crop_col_left_viz and 
                x < crop_col_right_viz):
                
                cv2.circle(image_color, (x, y), 10, (0, 0, 255), 2) # Red circle.
                text = f"{depth/1000.0:.2f} m, {angle:.1f} deg"
                cv2.putText(image_color, text, (x + 15, y + 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        cv2.imshow("Kinect Depth Test", image_color) # Display the depth visualization.
        
        # 5. Check for 'q' key press in OpenCV windows to quit.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False # Signal the keyboard listener thread to stop.
            listener.stop() # Force the keyboard listener to stop.
            break # Exit the camera loop.

    # --- Cleanup after exiting the loop ---
    print("\nShutting down...")
    
    # Ensure the keyboard listener thread is stopped.
    if listener.is_alive():
        listener.stop()
        
    # Shut down the Kinect sensor using the `person_detector` module.
    kinect_sensor.shutdown_detector()
    
    # Close all OpenCV display windows.
    cv2.destroyAllWindows()

    # Ensure the wheelchair motors are set to neutral (stopped).
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    
    print("\nDAC set to neutral. Camera and listeners stopped.")
    print("Program terminated.")

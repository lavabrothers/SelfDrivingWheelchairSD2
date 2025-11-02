#!/usr/bin/env python3

# --- Original Motor Control Imports ---
import board
import adafruit_mcp4728
import time
from pynput import keyboard

# --- Imports for Kinect v2 Camera ---
import sys
import cv2
import numpy as np
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame, FrameMap

# --- ADDED: Import for OS path manipulation ---
import os 

# --- Setup: DAC (Motor Controller) ---
try:
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    exit()

# --- State Variables ---
keys_pressed = {
    'w': False,
    's': False,
    'a': False,
    'd': False
}

running = True # Global flag to signal threads to stop

# --- Motor Control Functions (Unchanged) ---
def update_dac_channels():
    """
    Reads the keys_pressed state and sets the DAC values accordingly.
    """
    val_a = 0.5  # Forward
    val_b = 0.5  # Backward
    val_c = 0.5  # Right
    val_d = 0.5  # Left

    if keys_pressed['w'] and not keys_pressed['s']:
        val_a = 0.75
        val_b = 0.25
    elif keys_pressed['s'] and not keys_pressed['w']:
        val_a = 0.25
        val_b = 0.75

    if keys_pressed['d'] and not keys_pressed['a']:
        val_c = 0.75
        val_d = 0.25
    elif keys_pressed['a'] and not keys_pressed['d']:
        val_c = 0.25
        val_d = 0.75

    mcp.channel_a.normalized_value = val_a
    mcp.channel_b.normalized_value = val_b
    mcp.channel_c.normalized_value = val_c
    mcp.channel_d.normalized_value = val_d
    
    print_status(val_a, val_b, val_c, val_d)

def print_status(a, b, c, d):
    """
    Prints the current status of all channels in a readable format.
    """
    print(
        f"\rStatus -> Fwd(A): {a:.2f}, Bwd(B): {b:.2f} | "
        f"Right(C): {c:.2f}, Left(D): {d:.2f}",
        end=''
    )

def show_instructions():
    """
    Displays the new controls to the user.
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

# --- Keyboard Listener Functions (Unchanged) ---

def on_press(key):
    try:
        if key.char in keys_pressed:
            if not keys_pressed[key.char]:
                keys_pressed[key.char] = True
                update_dac_channels()
    except AttributeError:
        pass

def on_release(key):
    global running 
    try:
        if key.char in keys_pressed:
            keys_pressed[key.char] = False
            update_dac_channels()
    except AttributeError:
        pass
    
    if key == keyboard.Key.esc:
        print("\n\nEscape key pressed. Stopping and exiting.")
        running = False 
        return False     # Stops the listener thread

# --- Main Program ---
if __name__ == "__main__":

    # --- ADDED: Path correction to import from ../Refactoring ---
    # Get the directory of the current script (dev-tools)
    script_dir = os.path.dirname(os.path.realpath(__file__))
    # Get the parent directory (project root)
    project_root = os.path.join(script_dir, '..')
    # Add the 'Refactoring' directory to the path
    refactoring_dir = os.path.join(project_root, 'Refactoring')
    sys.path.append(refactoring_dir)
    
    try:
        # Import the object detection script
        import kinectcloseobject as kinect_sensor
    except ImportError:
        print(f"Error: Could not import 'kinectcloseobject.py' from {refactoring_dir}")
        print("Please check the file path.")
        sys.exit(1)
    # --- END OF PATH CORRECTION ---
    
    # 1. Set initial DAC state
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    
    show_instructions()

    # --- MODIFIED: Use the kinect_sensor module to initialize ---
    # This single initialization requests BOTH Color and Depth frames.
    if not kinect_sensor.initialize_kinect():
        print("Error: Kinect initialization failed. Exiting.")
        sys.exit(1)
    
    print("Kinect v2 camera and depth sensor started.")
    # We will use the listener from the kinect_sensor module
    # kinect_sensor.listener
    # --- END OF MODIFICATION ---

    # --- Start pynput listener in a non-blocking thread ---
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # --- MODIFIED: Main Camera Loop ---
    
    # Create the FrameMap object ONCE outside the loop
    frames = FrameMap()

    while running:
        # 1. Get new frames from the sensor
        if not kinect_sensor.listener.waitForNewFrame(frames, 10 * 1000):
            print("Timeout waiting for frames!")
            continue
            
        color_frame = frames[FrameType.Color]
        depth_frame = frames[FrameType.Depth]

        # 2. Get the raw depth map
        depth_map = depth_frame.asarray()
        
        # 3. Call the object detection function
        # We pass the frame we already grabbed to avoid re-grabbing
        depth, angle, coords = kinect_sensor.get_nearest_object_angle(depth_frame_obj=depth_frame)
        
        # --- 4. Process and Display the RGB Driver View (Window 1) ---
        img = color_frame.asarray(dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        img_small = cv2.resize(img, (960, 540)) 
        img_flipped = cv2.flip(img_small, 1) # Flip horizontally
        cv2.imshow("Kinect v2 RGB Feed (Driver View)", img_flipped)

        # --- 5. Process and Display the Depth Object View (Window 2) ---
        # (This visualization code is from kinect_sensor.py's test mode)
        depth_viz = np.clip(depth_map, 500, 4500)
        depth_viz = (depth_viz - 500) / (4000.0)
        depth_viz = (255 * (1.0 - depth_viz)).astype(np.uint8)
        depth_viz[depth_map == 0] = 0
        image_color = cv2.cvtColor(depth_viz, cv2.COLOR_GRAY2BGR)

        # Draw the crop line
        viz_height = image_color.shape[0]
        crop_row_viz = int(viz_height * (1.0 - kinect_sensor.CROP_BOTTOM_RATIO))
        cv2.line(image_color, (0, crop_row_viz), (kinect_sensor.KINECT_WIDTH - 1, crop_row_viz), 
                 (255, 255, 0), 1) # Cyan line
        cv2.putText(image_color, "IGNORING THIS AREA (FLOOR)", 
                    (10, crop_row_viz + 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Draw the detected object
        if coords is not None and depth is not None:
            x, y = coords
            if y < crop_row_viz:
                cv2.circle(image_color, (x, y), 10, (0, 0, 255), 2) # Red circle
                text = f"{depth/1000.0:.2f} m, {angle:.1f} deg"
                cv2.putText(image_color, text, (x + 15, y + 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        cv2.imshow("Kinect Depth Test", image_color)
        
        # --- 6. Release the frames ---
        kinect_sensor.listener.release(frames)

        # --- 7. Check for quit key ---
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False # Signal the listener thread to stop
            listener.stop() # Force the listener to stop
            break # Exit the camera loop

    # --- Cleanup after exiting the loop ---
    print("\nShutting down...")
    
    # Stop the listener thread
    if listener.is_alive():
        listener.stop()
        
    # --- MODIFIED: Use the kinect_sensor module to shut down ---
    kinect_sensor.shutdown_kinect()
    
    # Close all OpenCV windows
    cv2.destroyAllWindows()

    # Ensure the wheelchair is stopped
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    
    print("\nDAC set to neutral. Camera and listeners stopped.")
    print("Program terminated.")
"""
local-control.py

This script provides a local, keyboard-controlled interface for the self-driving
wheelchair, integrating real-time motor control with a live Kinect V2 color
camera feed. It is intended as a development tool for testing basic motor
responses and visual feedback in a graphical environment.

The script allows direct manual control of the wheelchair's movement using
keyboard inputs ('w', 'a', 's', 'd') and displays an OpenCV window showing
a flipped RGB feed from the Kinect's color camera (driver's view).

Key Features:
- Initializes the MCP4728 DAC for motor control via I2C.
- Initializes the Kinect V2 sensor and its color frame listener.
- Uses `pynput` for non-blocking keyboard input to control movement.
- Displays a live RGB camera feed using OpenCV.
- Provides real-time console feedback on the current DAC channel values.
- Ensures graceful shutdown of all hardware and display windows upon exit.

Dependencies:
- board, adafruit_mcp4728: For I2C communication with the DAC.
- time: For potential delays.
- pynput: For keyboard input handling.
- sys: For system exit in case of critical initialization failures.
- cv2 (OpenCV): For image processing, resizing, and display.
- numpy: For numerical operations on image data.
- pylibfreenect2: Python wrapper for libfreenect2, used to interface with the Kinect V2.
"""

# --- Motor Control Imports ---
import board
import adafruit_mcp4728
import time
from pynput import keyboard

# --- Imports for Kinect v2 Camera ---
import sys
import cv2
import numpy as np
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame, FrameMap # FrameMap is not used here, but kept for consistency with other Kinect modules.

# --- Setup: DAC (Motor Controller) ---
try:
    i2c = board.I2C() # Initialize I2C bus.
    mcp = adafruit_mcp4728.MCP4728(i2c) # Create MCP4728 DAC object.
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    sys.exit(1) # Exit if DAC is not found, as it's critical for control.

# --- State Variables ---
# A dictionary to keep track of which control keys are currently being held down.
keys_pressed = {
    'w': False, # Forward
    's': False, # Backward
    'a': False, # Turn Left
    'd': False  # Turn Right
}

# Global flag to signal threads to stop.
# This will be set to False by the `on_release` function when 'ESC' is pressed.
running = True

# --- Motor Control Functions ---
def update_dac_channels():
    """
    Reads the `keys_pressed` state and sets the DAC values accordingly to control
    the wheelchair's movement.

    This function is the central point for translating keyboard input into
    normalized DAC values (0.0-1.0) for forward/backward and left/right movement.
    """
    # Initialize DAC values to a neutral (stop) state (0.5).
    val_a = 0.5  # Forward channel
    val_b = 0.5  # Backward channel
    val_c = 0.5  # Right turn channel
    val_d = 0.5  # Left turn channel

    # --- Forward/Backward Control ---
    if keys_pressed['w'] and not keys_pressed['s']:
        # Move Forward: Increase channel A, decrease channel B.
        val_a = 0.75
        val_b = 0.25
    elif keys_pressed['s'] and not keys_pressed['w']:
        # Move Backward: Decrease channel A, increase channel B.
        val_a = 0.25
        val_b = 0.75
    # If both 'w' and 's' are pressed, or neither, channels A and B remain neutral (0.5).

    # --- Left/Right Control ---
    if keys_pressed['d'] and not keys_pressed['a']:
        # Turn Right: Increase channel C, decrease channel D.
        val_c = 0.75
        val_d = 0.25
    elif keys_pressed['a'] and not keys_pressed['d']:
        # Turn Left: Decrease channel C, increase channel D.
        val_c = 0.25
        val_d = 0.75
    # If both 'a' and 'd' are pressed, or neither, channels C and D remain neutral (0.5).

    # Send the calculated values to the DAC channels.
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
    # Using \r (carriage return) and `end=''` makes the output update on a single line.
    print(
        f"\rStatus -> Fwd(A): {a:.2f}, Bwd(B): {b:.2f} | "
        f"Right(C): {c:.2f}, Left(D): {d:.2f}",
        end=''
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
    print("  You can also press 'q' in the camera window to quit.")
    print("-------------------------------------------------")
    print("Starting control listener and camera feed...")

# --- Keyboard Listener Functions ---

def on_press(key):
    """
    Callback function for `pynput` when a key is PRESSED.

    Updates the `keys_pressed` dictionary for the corresponding key and
    triggers an update to the DAC channels if the key state has changed.
    """
    try:
        # Check if the pressed key is one of our defined control keys.
        if key.char in keys_pressed:
            # Only update if the key state wasn't already "pressed" to avoid redundant calls.
            if not keys_pressed[key.char]:
                keys_pressed[key.char] = True
                update_dac_channels()
    except AttributeError:
        # Ignore special keys (e.g., Shift, Ctrl, Alt) that do not have a `.char` attribute.
        pass

def on_release(key):
    """
    Callback function for `pynput` when a key is RELEASED.

    Updates the `keys_pressed` dictionary for the corresponding key and
    triggers an update to the DAC channels. Handles the 'ESC' key to
    stop the keyboard listener and exit the program.
    """
    global running  # We need to modify the global `running` flag.

    try:
        # Check if the released key is one of our defined control keys.
        if key.char in keys_pressed:
            # Update the key's state to "not pressed" and trigger DAC update.
            keys_pressed[key.char] = False
            update_dac_channels()
    except AttributeError:
        # Ignore special keys.
        pass
    
    # If the 'ESC' key is released, signal to stop the program.
    if key == keyboard.Key.esc:
        print("\n\nEscape key pressed. Stopping and exiting.")
        running = False  # Signal the main loop to stop.
        return False     # This stops the `pynput` listener thread itself.

# --- Main Program ---
if __name__ == "__main__":
    
    # 1. Set initial DAC state to neutral (stop).
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    
    show_instructions()

    # 2. Initialize Kinect v2 Camera.
    freenect2 = None
    device = None
    kinect_listener = None
    try:
        freenect2 = Freenect2()
        num_devices = freenect2.enumerateDevices()
        if num_devices == 0:
            print("Error: No Kinect v2 devices found!")
            sys.exit(1)

        serial = freenect2.getDeviceSerialNumber(0)
        device = freenect2.openDevice(serial)

        # We only need the Color feed for visual driving.
        types = FrameType.Color
        kinect_listener = SyncMultiFrameListener(types)
        device.setColorFrameListener(kinect_listener)
        device.start()
        print("Kinect v2 camera started.")

    except Exception as e:
        print(f"Error initializing Kinect v2: {e}")
        # Ensure Kinect is shut down if it partially initialized before exiting.
        if device:
            try:
                device.stop()
                device.close()
            except Exception as shutdown_e:
                print(f"Error during Kinect shutdown after init failure: {shutdown_e}")
        sys.exit(1)

    # 3. Start `pynput` keyboard listener in a non-blocking thread.
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # 4. Main Camera Loop.
    # This loop will run in the main thread until `running` is set to False.
    frames = FrameMap() # Create the FrameMap object ONCE outside the loop to reuse it.

    while running: # Loop continues as long as the `running` flag is True.
        # Wait for new frames from the Kinect.
        if not kinect_listener.waitForNewFrame(frames, 10 * 1000): # 10-second timeout.
            print("Timeout waiting for frames! Skipping frame processing.", end='\r')
            continue # Continue to the next iteration if timeout occurs.
            
        color_frame = frames[FrameType.Color]

        # Convert frame to a format OpenCV can use (BGRA to BGR).
        img = color_frame.asarray(dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        # Resize for better performance (Kinect v2 is 1920x1080).
        img_small = cv2.resize(img, (960, 540)) 
        img_flipped = cv2.flip(img_small, 1) # Flip horizontally for a "mirror" view.

        # Display the image.
        cv2.imshow("Kinect v2 RGB Feed (Driver View)", img_flipped)

        # Release the frames for the next iteration.
        kinect_listener.release(frames)

        # `cv2.waitKey()` is needed to update the OpenCV window.
        # We also check if 'q' is pressed in the OpenCV window as a backup exit mechanism.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False # Signal the keyboard listener thread to stop (if it's still running).
            listener.stop() # Force the keyboard listener to stop.
            break # Exit the camera loop.

    # --- Cleanup after exiting the loop ---
    print("\nShutting down...")
    
    # Stop the keyboard listener thread (if it hasn't stopped already).
    if listener.is_alive():
        listener.stop()
        
    # Stop and close the Kinect sensor.
    if device:
        try:
            device.stop()
            device.close()
        except Exception as e:
            print(f"Error during Kinect shutdown: {e}")
    
    # Close all OpenCV display windows.
    cv2.destroyAllWindows()

    # Ensure the wheelchair motors are set to neutral (stopped).
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    
    print("\nDAC set to neutral. Camera and listeners stopped.")
    print("Program terminated.")

#!/usr/bin/env python3

# --- Original Motor Control Imports ---
import board
import adafruit_mcp4728
import time
from pynput import keyboard

# --- ADDED: Imports for Kinect v2 Camera ---
import sys
import cv2
import numpy as np
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame

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

# --- ADDED: Global flag to signal threads to stop ---
# This will be set to False by the on_release function
running = True

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
    print("  You can also press 'q' in the camera window to quit.")
    print("-------------------------------------------------")
    print("Starting control listener and camera feed...")

# --- Keyboard Listener Functions ---

def on_press(key):
    """
    This function is called by the listener whenever a key is PRESSED.
    """
    try:
        if key.char in keys_pressed:
            if not keys_pressed[key.char]:
                keys_pressed[key.char] = True
                update_dac_channels()
    except AttributeError:
        pass

# --- MODIFIED: on_release() ---
def on_release(key):
    """
    This function is called by the listener whenever a key is RELEASED.
    It now also controls the global 'running' flag.
    """
    global running  # <-- MODIFIED: We need to change the global flag

    try:
        if key.char in keys_pressed:
            keys_pressed[key.char] = False
            update_dac_channels()
    except AttributeError:
        pass
    
    if key == keyboard.Key.esc:
        print("\n\nEscape key pressed. Stopping and exiting.")
        running = False  # <-- MODIFIED: Signal the main loop to stop
        return False     # <-- This stops the listener thread itself

# --- Main Program ---
if __name__ == "__main__":
    
    # 1. Set initial DAC state
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    
    show_instructions()

    # --- ADDED: Kinect v2 Initialization ---
    try:
        freenect2 = Freenect2()
        num_devices = freenect2.enumerateDevices()
        if num_devices == 0:
            print("Error: No Kinect v2 devices found!")
            sys.exit(1)

        serial = freenect2.getDeviceSerialNumber(0)
        device = freenect2.openDevice(serial)

        # We only need the Color feed for driving
        types = FrameType.Color
        kinect_listener = SyncMultiFrameListener(types)
        device.setColorFrameListener(kinect_listener)
        device.start()
        print("Kinect v2 camera started.")

    except Exception as e:
        print(f"Error initializing Kinect v2: {e}")
        sys.exit(1)

    # --- MODIFIED: Start pynput listener in a non-blocking thread ---
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()  # <-- This is the key change!

    # --- ADDED: Main Camera Loop ---
    # This loop will run in the main thread until 'running' is set to False
    while running:
        # Get new frames from the Kinect
        frames = kinect_listener.waitForNewFrame()
        color_frame = frames["color"]

        # Convert frame to a format OpenCV can use (BGRA to BGR)
        img = color_frame.asarray(dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        # Resize for better performance (Kinect v2 is 1920x1080)
        img_small = cv2.resize(img, (960, 540)) 
        img_flipped = cv2.flip(img_small, 1)

        # Display the image
        cv2.imshow("Kinect v2 RGB Feed (Driver View)", img_flipped)

        # Release the frames for the next iteration
        kinect_listener.release(frames)

        # cv2.waitKey() is needed to update the window.
        # We also check if 'q' is pressed in the CV window as a backup exit.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False # Signal the listener thread to stop (if it's still running)
            listener.stop() # Force the listener to stop
            break # Exit the camera loop

    # --- Cleanup after exiting the loop ---
    print("\nShutting down...")
    
    # Stop the listener thread (if it hasn't stopped already)
    if listener.is_alive():
        listener.stop()
        
    # Stop the Kinect
    device.stop()
    device.close()
    
    # Close the OpenCV window
    cv2.destroyAllWindows()

    # Ensure the wheelchair is stopped
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    
    print("\nDAC set to neutral. Camera and listeners stopped.")
    print("Program terminated.")
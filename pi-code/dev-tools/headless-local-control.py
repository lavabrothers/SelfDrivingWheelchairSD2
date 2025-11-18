"""
headless-local-control.py

This script provides a local, keyboard-controlled interface for the self-driving
wheelchair, focusing solely on motor control without any visual feedback from
cameras. It is intended as a development tool for testing motor responses
in a "headless" environment, where a graphical display might not be available
or desired.

The script allows direct manual control of the wheelchair's movement using
keyboard inputs ('w', 'a', 's', 'd'). It continuously monitors key presses
and releases to adjust the DAC outputs, which in turn control the wheelchair's
motors.

Key Features:
- Initializes the MCP4728 DAC for motor control via I2C.
- Uses `pynput` for non-blocking keyboard input handling.
- Translates keyboard commands into proportional motor control signals.
- Provides real-time console feedback on the current DAC channel values.
- Ensures graceful shutdown and stops the wheelchair upon 'ESC' key press.

Dependencies:
- board, adafruit_mcp4728: For I2C communication with the DAC.
- time: For potential delays (though not heavily used in this version).
- pynput: For keyboard input handling.
"""

import board
import adafruit_mcp4728
import time
from pynput import keyboard # Import the pynput library for keyboard input.

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

# --- Functions ---
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
        # Returning False from this callback stops the `pynput` listener thread.
        return False

def show_instructions():
    """
    Displays the keyboard control instructions and program information to the user.
    """
    print("\n--- Headless Real-time Wheelchair Control ---")
    print("Press and HOLD keys to move:")
    print("  'w' - Move forward")
    print("  's' - Move backward")
    print("  'd' - Move right")
    print("  'a' - Move left")
    print("\n  Press 'ESC' to stop and exit.")
    print("------------------------------------")
    print("Starting control listener...")


# --- Main Program ---
if __name__ == "__main__":
    # Set the initial neutral state on the DAC before starting the control loop.
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    
    show_instructions()

    # Start the `pynput` keyboard listener in its own non-blocking thread.
    # The `with` statement ensures the listener is properly managed.
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        # `listener.join()` blocks the main thread, keeping the script running
        # until the listener is explicitly stopped (by `on_release` returning False).
        listener.join()
        
    # --- Cleanup after exiting the listener loop ---
    # Ensure the wheelchair is stopped when the program terminates.
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    print("\nDAC set to neutral. Program terminated.")

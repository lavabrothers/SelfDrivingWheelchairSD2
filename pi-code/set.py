import board
import adafruit_mcp4728
import time
from pynput import keyboard # <-- Import the new library

# --- Setup ---
try:
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    exit()

# --- State Variables ---
# We now use a dictionary to keep track of which keys are currently being held down.
keys_pressed = {
    'w': False,
    's': False,
    'a': False,
    'd': False
}

# --- Functions ---
def update_dac_channels():
    """
    Reads the keys_pressed state and sets the DAC values accordingly.
    This function is now the central point for controlling the motors.
    """
    # These are local variables for this function call.
    # Start with a neutral state (0.5).
    val_a = 0.5  # Forward
    val_b = 0.5  # Backward
    val_c = 0.5  # Right
    val_d = 0.5  # Left

    # --- Forward/Backward Control ---
    if keys_pressed['w'] and not keys_pressed['s']:
        # Go Forward
        val_a = 0.75
        val_b = 0.25
    elif keys_pressed['s'] and not keys_pressed['w']:
        # Go Backward
        val_a = 0.25
        val_b = 0.75
    # If both or neither are pressed, they remain at the neutral 0.5

    # --- Left/Right Control ---
    if keys_pressed['d'] and not keys_pressed['a']:
        # Go Right
        val_c = 0.75
        val_d = 0.25
    elif keys_pressed['a'] and not keys_pressed['d']:
        # Go Left
        val_c = 0.25
        val_d = 0.75
    # If both or neither are pressed, they remain at the neutral 0.5

    # Send the calculated values to the DAC
    mcp.channel_a.normalized_value = val_a
    mcp.channel_b.normalized_value = val_b
    mcp.channel_c.normalized_value = val_c
    mcp.channel_d.normalized_value = val_d
    
    print_status(val_a, val_b, val_c, val_d)

def print_status(a, b, c, d):
    """
    Prints the current status of all channels in a readable format.
    Using \r (carriage return) and end='' makes it update on a single line.
    """
    print(
        f"\rStatus -> Fwd(A): {a:.2f}, Bwd(B): {b:.2f} | "
        f"Right(C): {c:.2f}, Left(D): {d:.2f}",
        end=''
    )

def on_press(key):
    """
    This function is called by the listener whenever a key is PRESSED.
    """
    try:
        # Check if the pressed key is one of our control keys
        if key.char in keys_pressed:
            # If the key state isn't already "pressed", update it and send new values
            if not keys_pressed[key.char]:
                keys_pressed[key.char] = True
                update_dac_channels()
    except AttributeError:
        # This handles special keys (like Shift, Ctrl, etc.) which don't have a '.char' attribute
        pass

def on_release(key):
    """
    This function is called by the listener whenever a key is RELEASED.
    """
    try:
        # Check if the released key is one of our control keys
        if key.char in keys_pressed:
            # Update the key's state to "not pressed" and send the new neutral values
            keys_pressed[key.char] = False
            update_dac_channels()
    except AttributeError:
        pass
    
    # To stop the listener and exit the program, press the 'ESC' key
    if key == keyboard.Key.esc:
        print("\n\nEscape key pressed. Stopping and exiting.")
        # Return False from a callback to stop the listener
        return False

def show_instructions():
    """
    Displays the new controls to the user.
    """
    print("\n--- Real-time Wheelchair Control ---")
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
    # Set the initial neutral state on the DAC before starting
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    
    show_instructions()

    # The pynput listener runs in its own thread.
    # We provide our on_press and on_release functions as callbacks.
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        # listener.join() will block the main thread, keeping the script
        # running until the listener is stopped (by returning False from a callback).
        listener.join()
        
    # --- Cleanup after exiting the listener loop ---
    # Ensure the wheelchair is stopped when the program ends.
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    print("\nDAC set to neutral. Program terminated.")

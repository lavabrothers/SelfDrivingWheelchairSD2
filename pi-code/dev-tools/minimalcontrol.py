"""
minimalcontrol.py

This script provides a minimal, SSH-friendly keyboard-controlled interface
for the self-driving wheelchair's motors. It is designed for environments
where a graphical display is not available (e.g., SSH terminal) and uses
the `curses` library for non-blocking keyboard input.

The script allows direct manual control of the wheelchair's movement using
keyboard inputs ('w', 'a', 's', 'd'). It implements a "last key wins" logic
to prevent issues with simultaneous key presses over SSH and includes an
auto-stop timeout for safety if no keys are pressed.

Key Features:
- Initializes the MCP4728 DAC for motor control via I2C.
- Uses the `curses` library for robust, non-blocking keyboard input in a terminal.
- Translates keyboard commands into proportional motor control signals.
- Provides real-time status updates directly within the curses terminal window.
- Implements an auto-stop mechanism for safety if no input is received.
- Ensures graceful shutdown and stops the wheelchair upon 'q' key press or program exit.

Dependencies:
- board, adafruit_mcp4728: For I2C communication with the DAC.
- time: For timing operations and auto-stop timeout.
- sys: For system exit in case of critical initialization failures.
- curses: For terminal-based keyboard input and display management.
"""

# --- Motor Control Imports ---
import board
import adafruit_mcp4728
import time
import sys
import curses  # Import the curses library for terminal interaction.

# --- Setup: DAC (Motor Controller) ---
try:
    i2c = board.I2C() # Initialize I2C bus.
    mcp = adafruit_mcp4728.MCP4728(i2c) # Create MCP4728 DAC object.
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    sys.exit(1) # Exit if DAC is not found, as it's critical for control.
except Exception as e:
    print(f"An unexpected error occurred during I2C setup: {e}")
    sys.exit(1)

# --- State Variables ---
# This dictionary tracks the desired state of control keys, driving the DAC logic.
keys_pressed = {
    'w': False, # Forward
    's': False, # Backward
    'a': False, # Turn Left
    'd': False  # Turn Right
}

# --- Motor Control Functions ---
def update_dac_channels(stdscr):
    """
    Reads the `keys_pressed` state and sets the DAC values accordingly.
    Prints the current status to the `curses` window.

    Args:
        stdscr: The `curses` window object for displaying output.
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
    
    # Print status to the curses window.
    status_line = 7 # Line number to display status.
    stdscr.move(status_line, 0) # Move cursor to the start of the status line.
    stdscr.clrtoeol() # Clear the rest of the line.
    stdscr.addstr(
        f"Status -> Fwd(A): {val_a:.2f}, Bwd(B): {val_b:.2f} | "
        f"Right(C): {val_c:.2f}, Left(D): {val_d:.2f}"
    )

def set_neutral(mcp_instance: adafruit_mcp4728.MCP4728):
    """
    Sets all DAC channels to their neutral (0.5) position, effectively stopping the wheelchair.

    Args:
        mcp_instance (adafruit_mcp4728.MCP4728): The MCP4728 DAC object.
    """
    mcp_instance.channel_a.normalized_value = 0.5
    mcp_instance.channel_b.normalized_value = 0.5
    mcp_instance.channel_c.normalized_value = 0.5
    mcp_instance.channel_d.normalized_value = 0.5
    print("\nAll channels set to neutral (0.5).")

def show_instructions(stdscr):
    """
    Displays the keyboard control instructions and program information
    within the `curses` window.

    Args:
        stdscr: The `curses` window object for displaying output.
    """
    stdscr.addstr("--- Real-time SSH Wheelchair Control ---\n")
    stdscr.addstr("Press and HOLD keys to move:\n")
    stdscr.addstr("  'w' - Move forward\n")
    stdscr.addstr("  's' - Move backward\n")
    stdscr.addstr("  'd' - Move right\n")
    stdscr.addstr("  'a' - Move left\n")
    stdscr.addstr("\n  Press 'q' to stop and exit.\n")
    stdscr.addstr("----------------------------------------\n")

# --- Main Program (rewritten for curses) ---
def main(stdscr):
    """
    Main application function, designed to run within the `curses` environment.

    This function handles non-blocking keyboard input, updates motor controls,
    and manages the display of status information. It includes an auto-stop
    timeout for safety.

    Args:
        stdscr: The main `curses` window object.
    """
    global keys_pressed # We need to modify the global state of `keys_pressed`.

    # --- Curses Setup ---
    stdscr.nodelay(True)  # Make `getch()` non-blocking, so it returns immediately if no key is pressed.
    stdscr.clear()        # Clear the terminal screen.
    show_instructions(stdscr) # Display instructions.
    stdscr.refresh()      # Update the physical screen.

    # --- State Variables for Main Loop ---
    running = True
    last_key_press_time = time.time()
    auto_stop_timeout = 0.2 # If no key is pressed for this duration (in seconds), motors stop.
    needs_update = True     # Flag to force an initial DAC update and subsequent updates when state changes.

    while running:
        now = time.time()
        
        # 1. Check for keyboard input.
        char = stdscr.getch() # Returns -1 (curses.ERR) if no key is pressed.

        if char != curses.ERR:
            # A key was pressed!
            last_key_press_time = now
            needs_update = True
            key = chr(char) # Convert character code to string.

            # Logic: "Last key press wins". This prevents "stuck keys" over SSH
            # and simplifies control by ensuring only one directional input is active.
            new_keys_pressed = {
                'w': False, 's': False, 'a': False, 'd': False
            }

            if key == 'w':
                new_keys_pressed['w'] = True
            elif key == 's':
                new_keys_pressed['s'] = True
            elif key == 'a':
                new_keys_pressed['a'] = True
            elif key == 'd':
                new_keys_pressed['d'] = True
            elif key == 'q':
                running = False # Signal to exit the main loop.
            
            keys_pressed = new_keys_pressed # Update the global state.

        # 2. Check for "key release" (timeout).
        # If no keys have been pressed for longer than `auto_stop_timeout`.
        elif (now - last_key_press_time > auto_stop_timeout):
            # If any control key was previously pressed, set all to False to stop motors.
            if any(keys_pressed.values()): # Only update if not already stopped.
                keys_pressed = {k: False for k in keys_pressed}
                needs_update = True # Flag for DAC update.

        # 3. Update DAC if the control state has changed.
        if needs_update:
            update_dac_channels(stdscr) # Send new values to DAC and update status display.
            stdscr.refresh()            # Update the physical terminal screen.
            needs_update = False        # Reset flag.
        
        # Loop at approximately 50Hz (20ms delay).
        time.sleep(0.02)


if __name__ == "__main__":
    try:
        # Set initial DAC state to neutral before starting curses.
        set_neutral(mcp)
        # `curses.wrapper` handles all terminal setup and cleanup,
        # ensuring the terminal is restored correctly on exit or error.
        curses.wrapper(main)
    
    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        # This block runs after `curses.wrapper` has completed and restored the terminal.
        print("\nShutting down...")
        set_neutral(mcp) # Ensure motors are stopped one final time.
        print("Program terminated.")

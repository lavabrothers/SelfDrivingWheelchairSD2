#!/usr/bin/env python3

# --- Motor Control Imports ---
import board
import adafruit_mcp4728
import time
import sys
import curses  # <-- ADDED: For reading keys over SSH

# --- Setup: DAC (Motor Controller) ---
try:
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    sys.exit(1)
except Exception as e:
    print(f"An unexpected error occurred during I2C setup: {e}")
    sys.exit(1)

# --- State Variables ---
# This dictionary still drives the DAC logic
keys_pressed = {
    'w': False,
    's': False,
    'a': False,
    'd': False
}

# --- Motor Control Functions (Unchanged from original) ---
def update_dac_channels(stdscr):
    """
    Reads the keys_pressed state and sets the DAC values accordingly.
    Prints status to the curses window.
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
    
    # Print status to the curses window
    status_line = 7
    stdscr.move(status_line, 0)
    stdscr.clrtoeol() # Clear the line
    stdscr.addstr(
        f"Status -> Fwd(A): {val_a:.2f}, Bwd(B): {val_b:.2f} | "
        f"Right(C): {val_c:.2f}, Left(D): {val_d:.2f}"
    )

def set_neutral(mcp_instance):
    """
    Sets all DAC channels to their neutral (0.5) position.
    """
    mcp_instance.channel_a.normalized_value = 0.5
    mcp_instance.channel_b.normalized_value = 0.5
    mcp_instance.channel_c.normalized_value = 0.5
    mcp_instance.channel_d.normalized_value = 0.5
    print("\nAll channels set to neutral (0.5).")

def show_instructions(stdscr):
    """
    Displays the controls in the curses window.
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
    Main application function, wrapped by curses.
    stdscr is the main window object.
    """
    global keys_pressed # We need to modify the global state

    # --- Curses Setup ---
    stdscr.nodelay(True)  # Make getch() non-blocking
    stdscr.clear()
    show_instructions(stdscr)
    stdscr.refresh()

    # --- State Variables ---
    running = True
    last_key_press_time = time.time()
    # If no key is pressed for this long (in seconds), stop the motors.
    auto_stop_timeout = 0.2 
    needs_update = True # Flag to force DAC update

    while running:
        now = time.time()
        
        # 1. Check for keyboard input
        char = stdscr.getch() # Returns -1 (curses.ERR) if no key is pressed

        if char != curses.ERR:
            # A key was pressed!
            last_key_press_time = now
            needs_update = True
            key = chr(char)

            # Logic: Last key press wins. This prevents "stuck keys"
            # over SSH and avoids diagonal control issues.
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
                running = False
            
            keys_pressed = new_keys_pressed

        # 2. Check for "key release" (timeout)
        elif (now - last_key_press_time > auto_stop_timeout):
            # No keys have been pressed for a while, so stop.
            if any(keys_pressed.values()): # Only update if not already stopped
                keys_pressed = {k: False for k in keys_pressed}
                needs_update = True

        # 3. Update DAC if state changed
        if needs_update:
            update_dac_channels(stdscr)
            stdscr.refresh()
            needs_update = False
        
        # Loop at ~50Hz
        time.sleep(0.02)


if __name__ == "__main__":
    try:
        # Set initial DAC state
        set_neutral(mcp)
        # curses.wrapper handles all terminal setup and cleanup
        curses.wrapper(main)
    
    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        # This runs after curses has restored the terminal
        print("\nShutting down...")
        set_neutral(mcp)
        print("Program terminated.")
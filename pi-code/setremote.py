import board
import adafruit_mcp4728
import time
import sys
import tty
import termios
import threading

# --- IMPORTANT HARDWARE NOTE ---
# For best performance, ensure your I2C bus speed is increased on the Pi.
# Add 'dtparam=i2c_arm_baudrate=400000' to /boot/firmware/config.txt and reboot.

# --- Setup ---
try:
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    exit()

# --- Shared State (for communication between threads) ---
shared_state = {
    'last_key': ' ',
    'last_press_time': 0,
    'running': True
}

# --- Constants ---
RELEASE_TIMEOUT = 0.2         # Time in seconds to consider a key "released"
CONTROL_LOOP_PERIOD = 0.02    # Loop runs at 50 Hz (1 / 0.02)
PRINT_LOOP_PERIOD = 10        # Print status every 10 loops (~5 times per second)

# --- Functions ---

def getch():
    """Gets a single character from stdin without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def keyboard_listener():
    """
    Runs in a separate thread to listen for keypresses without blocking the main loop.
    """
    while shared_state['running']:
        char = getch()
        shared_state['last_key'] = char
        shared_state['last_press_time'] = time.time()
        if char == 'q' or ord(char) == 3: # 'q' or Ctrl+C ASCII value
            shared_state['running'] = False

def set_movement(direction):
    """
    Sets the DAC values based on direction. Does NOT print, to keep it fast.
    """
    val_a, val_b, val_c, val_d = 0.5, 0.5, 0.5, 0.5 # Neutral state

    if direction == 'w': val_a, val_b = 0.75, 0.25
    elif direction == 's': val_a, val_b = 0.25, 0.75
    elif direction == 'd': val_c, val_d = 0.75, 0.25
    elif direction == 'a': val_c, val_d = 0.25, 0.75

    mcp.channel_a.normalized_value = val_a
    mcp.channel_b.normalized_value = val_b
    mcp.channel_c.normalized_value = val_c
    mcp.channel_d.normalized_value = val_d

def print_status(mcp_device):
    """
    Handles all printing to the terminal. Separated to avoid slowing down motor control.
    """
    # Reads the current values directly from the hardware object
    a = mcp_device.channel_a.normalized_value
    b = mcp_device.channel_b.normalized_value
    c = mcp_device.channel_c.normalized_value
    d = mcp_device.channel_d.normalized_value
    print(
        f"\rStatus -> Fwd(A): {a:.2f}, Bwd(B): {b:.2f} | "
        f"Right(C): {c:.2f}, Left(D): {d:.2f}   ",
        end='',
        flush=True # Ensure output is sent immediately
    )

def show_instructions():
    """Displays controls to the user."""
    print("\n--- Real-Time 'Press and Hold' Wheelchair Control (Optimized) ---")
    print("HOLD a key to move:")
    print("  'w'      - Move forward")
    print("  's'      - Move backward")
    print("  'd'      - Move right")
    print("  'a'      - Move left")
    print("  'q'      - To QUIT the program.")
    print("----------------------------------------------------------------")
    sys.stdout.flush()

# --- Main Program ---
if __name__ == "__main__":
    show_instructions()

    listener_thread = threading.Thread(target=keyboard_listener, daemon=True)
    listener_thread.start()

    active_command = ' '
    print_counter = 0
    set_movement(active_command) # Set initial stop state
    print_status(mcp)            # Print initial status

    try:
        while shared_state['running']:
            time_since_last_press = time.time() - shared_state['last_press_time']
            
            new_command = ' ' # Default to stop
            if time_since_last_press < RELEASE_TIMEOUT:
                new_command = shared_state['last_key']

            if new_command != active_command:
                set_movement(new_command)
                print_status(mcp) # Print immediately on change
                active_command = new_command
            
            # Also print periodically for continuous feedback
            print_counter += 1
            if print_counter >= PRINT_LOOP_PERIOD:
                print_status(mcp)
                print_counter = 0

            time.sleep(CONTROL_LOOP_PERIOD)

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down.")
    finally:
        shared_state['running'] = False
        set_movement(' ')
        print_status(mcp)
        print("\nDAC set to neutral. Program terminated.")
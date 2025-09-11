import board
import adafruit_mcp4728
import time

# --- Setup ---
try:
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    exit()

# --- Initial State Variables ---
# These variables hold the current normalized values for each channel.
# We start at 0.5 for a neutral, stopped position.

# val_a (Forward) + val_b (Backward) should always equal 1.0
val_a = 0.5  # Channel A: Forward
val_b = 0.5  # Channel B: Backward

# val_c (Right) + val_d (Left) should always equal 1.0
val_c = 0.5  # Channel C: Right
val_d = 0.5  # Channel D: Left

# The amount to change the value with each command (e.g., how fast the chair reacts)
STEP = 0.05

# --- Functions ---
def update_dac_channels():
    """
    Sends the current values of our variables to the corresponding DAC channels.
    """
    mcp.channel_a.normalized_value = val_a
    mcp.channel_b.normalized_value = val_b
    mcp.channel_c.normalized_value = val_c
    mcp.channel_d.normalized_value = val_d
    print_status()

def print_status():
    """
    Prints the current status of all channels in a readable format.
    """
    print(
        f"Status -> Fwd(A): {val_a:.2f}, Bwd(B): {val_b:.2f} | "
        f"Right(C): {val_c:.2f}, Left(D): {val_d:.2f}"
    )

def show_instructions():
    """
    Displays the available commands to the user.
    """
    print("\n--- Wheelchair Control ---")
    print("Enter a command to move the chair:")
    print("  'forward' or 'w' - Move forward")
    print("  'backward' or 's' - Move backward")
    print("  'right' or 'd'    - Move right")
    print("  'left' or 'a'     - Move left")
    print("  'stop'            - Halt all movement (return to neutral)")
    print("  'exit'            - Stop and close the program")
    print("--------------------------")

# --- Main Program Loop ---
if __name__ == "__main__":
    # Set the initial neutral state on the DAC
    update_dac_channels()
    show_instructions()

    while True:
        try:
            command = input("> ").lower().strip()

            if command in ("forward", "w"):
                # Increase forward value, not exceeding 1.0
                val_a = min(1.0, val_a + STEP)
                # The backward value is automatically calculated to maintain the 1.0 ratio
                val_b = 1.0 - val_a

            elif command in ("backward", "s"):
                # Increase backward value, not exceeding 1.0
                val_b = min(1.0, val_b + STEP)
                # The forward value is automatically calculated
                val_a = 1.0 - val_b

            elif command in ("right", "d"):
                # Increase right value, not exceeding 1.0
                val_c = min(1.0, val_c + STEP)
                # The left value is automatically calculated
                val_d = 1.0 - val_c

            elif command in ("left", "a"):
                # Increase left value, not exceeding 1.0
                val_d = min(1.0, val_d + STEP)
                # The right value is automatically calculated
                val_c = 1.0 - val_d

            elif command == "stop":
                print("Stopping movement...")
                val_a, val_b = 0.5, 0.5
                val_c, val_d = 0.5, 0.5

            elif command == "exit":
                print("Setting to neutral and exiting program.")
                val_a, val_b = 0.5, 0.5
                val_c, val_d = 0.5, 0.5
                update_dac_channels()
                break

            else:
                print("Invalid command. Please try again.")
                # Skip the hardware update for an invalid command
                continue

            # Send the new values to the hardware after a valid command
            update_dac_channels()

        except KeyboardInterrupt:
            # Handle Ctrl+C gracefully
            print("\nKeyboard interrupt detected. Stopping and exiting.")
            val_a, val_b = 0.5, 0.5
            val_c, val_d = 0.5, 0.5
            update_dac_channels()
            break

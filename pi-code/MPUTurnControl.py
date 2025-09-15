# Import necessary libraries
import time
import board
import math
import adafruit_mpu6050
import adafruit_mcp4728

# --- Constants ---
# This is how "fast" the wheelchair will turn. It represents the offset from the
# neutral 0.5 value. 0.5 is full speed in one direction.
# You can adjust this value to make turns faster or slower.
TURN_SPEED_OFFSET = 0.25 

# --- Hardware Setup ---
try:
    print("Initializing I2C and sensors...")
    i2c = board.I2C()  # uses board.SCL and board.SDA
    mpu = adafruit_mpu6050.MPU6050(i2c)
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MPU6050 and MCP4728 found and initialized. ✅")
except ValueError as e:
    print(f"Error: Could not find a required I2C device. Please check connections.")
    print(f"Details: {e}")
    exit()

# --- DAC Control Functions ---

def set_movement(fwd_bwd, left_right):
    """
    Sets the DAC channels to specific normalized values.
    - fwd_bwd:  -1.0 (full backward) to +1.0 (full forward)
    - left_right: -1.0 (full left) to +1.0 (full right)
    """
    # Forward/Backward control (Channels A & B)
    # val_a (Fwd) + val_b (Bwd) must equal 1.0
    # A value of +1.0 means val_a=1.0, val_b=0.0
    # A value of -1.0 means val_a=0.0, val_b=1.0
    # A value of 0.0 means val_a=0.5, val_b=0.5 (neutral)
    mcp.channel_a.normalized_value = 0.5 + (fwd_bwd / 2.0)
    mcp.channel_b.normalized_value = 0.5 - (fwd_bwd / 2.0)

    # Left/Right control (Channels D & C)
    # val_d (Left) + val_c (Right) must equal 1.0
    # A value of +1.0 means val_c=1.0, val_d=0.0
    # A value of -1.0 means val_c=0.0, val_d=1.0
    # A value of 0.0 means val_c=0.5, val_d=0.5 (neutral)
    mcp.channel_c.normalized_value = 0.5 + (left_right / 2.0)
    mcp.channel_d.normalized_value = 0.5 - (left_right / 2.0)

def stop_all_movement():
    """Brings the wheelchair to a complete stop."""
    print("Stopping all movement.")
    set_movement(0.0, 0.0)

# --- Gyro-based Turn Function ---

def execute_turn(direction, target_angle_deg):
    """
    Turns the wheelchair left or right by a specific angle using the gyro.
    """
    print(f"Executing turn: {direction} {target_angle_deg}°...")
    
    total_angle_turned = 0.0
    last_time = time.monotonic()

    # Determine turn direction and start the physical turn
    if direction == 'left':
        turn_value = -1.0 * TURN_SPEED_OFFSET * 2 # Set to move left
    else: # right
        turn_value = 1.0 * TURN_SPEED_OFFSET * 2  # Set to move right
        
    set_movement(0.0, turn_value) # No forward/backward, just turn

    # This loop runs until the target angle is reached
    while total_angle_turned < target_angle_deg:
        current_time = time.monotonic()
        # time_delta is the small amount of time that has passed since the last loop
        time_delta = current_time - last_time
        last_time = current_time

        # Read the gyro's rotational speed on the Z-axis (in radians/sec)
        # mpu.gyro[2] corresponds to the Z-axis (yaw)
        gyro_z_rad_s = mpu.gyro[2]
        
        # NOTE on Gyro Drift: For higher accuracy, you would first measure the gyro's
        # reading while it's completely still and subtract that "zero-rate offset"
        # from `gyro_z_rad_s` in every calculation. For simple turns, this is okay.
        
        # Calculate the small angle turned during this tiny time slice
        # Angle = Rotational Speed * Time
        angle_turned_rad = gyro_z_rad_s * time_delta
        angle_turned_deg = abs(math.degrees(angle_turned_rad)) # Use absolute value

        # Add it to our total
        total_angle_turned += angle_turned_deg

        # Optional: Print progress to the screen without creating new lines
        print(f"  -> Progress: {total_angle_turned:.2f}° / {target_angle_deg}°", end='\r')
        
        # A tiny delay to ensure the loop doesn't run too fast
        time.sleep(0.01)

    # Stop the physical turn once the loop is finished
    stop_all_movement()
    print(f"\nTurn complete. Final angle: {total_angle_turned:.2f}° ✨")


# --- Main Program Loop ---
if __name__ == "__main__":
    # Ensure wheelchair is stopped at the beginning
    stop_all_movement()

    print("\n--- MPU-based Wheelchair Control ---")
    print("Commands:")
    print("  'left [degrees]'  - e.g., 'left 90'")
    print("  'right [degrees]' - e.g., 'right 45'")
    print("  'stop'            - Halts any current movement")
    print("  'exit'            - Closes the program")
    print("------------------------------------")

    while True:
        try:
            # Get user input and split it into words
            command_str = input("Enter command > ").lower().strip()
            parts = command_str.split()

            if not parts: # If user just hits Enter
                continue

            # --- Command Parsing ---
            if len(parts) == 2 and parts[0] in ('left', 'right'):
                try:
                    # The second part should be the angle
                    angle = float(parts[1])
                    if angle <= 0:
                        print("Error: Angle must be a positive number.")
                        continue
                    
                    # Call the main turning function
                    execute_turn(direction=parts[0], target_angle_deg=angle)

                except ValueError:
                    print("Error: Invalid angle. Please enter a number (e.g., '90', '45.5').")
            
            elif parts[0] == "stop":
                stop_all_movement()

            elif parts[0] == "exit":
                print("Setting to neutral and exiting program.")
                stop_all_movement()
                break # Exit the while loop

            else:
                print("Invalid command format. Please try again.")

        except KeyboardInterrupt:
            # Handle Ctrl+C gracefully
            print("\nKeyboard interrupt detected.")
            stop_all_movement()
            break
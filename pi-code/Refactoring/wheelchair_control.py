# File: wheelchair_control.py
import board
import adafruit_mcp4728

# --- Variables ---
# We store the mcp object globally in this module after it's initialized
mcp = None

# We also store the current values to avoid sending redundant I2C commands
val_a, val_b = 0.5, 0.5
val_c, val_d = 0.5, 0.5

# --- Functions ---
def initialize_dac():
    """
    Initializes the I2C connection and the MCP4728.
    Returns the mcp object on success or None on failure.
    """
    global mcp
    try:
        i2c = board.I2C()
        mcp = adafruit_mcp4728.MCP4728(i2c)
        print("MCP4728 DAC found and initialized.")
        stop()  # Ensure chair is stopped on init
        return mcp
    except Exception as e:
        print(f"Error: Could not find MCP4728. Check I2C connections.")
        print(f"Details: {e}")
        mcp = None
        return None

def _update_dac_channels():
    """Internal function to send the current values to the hardware."""
    if mcp is None:
        print("Error: DAC not initialized.")
        return

    # Check if values have actually changed before sending
    if mcp.channel_a.normalized_value != val_a:
        mcp.channel_a.normalized_value = val_a
    if mcp.channel_b.normalized_value != val_b:
        mcp.channel_b.normalized_value = val_b
    if mcp.channel_c.normalized_value != val_c:
        mcp.channel_c.normalized_value = val_c
    if mcp.channel_d.normalized_value != val_d:
        mcp.channel_d.normalized_value = val_d

def stop():
    """Halts all movement by returning to neutral (0.5)."""
    global val_a, val_b, val_c, val_d
    print("Command: STOP")
    val_a, val_b = 0.5, 0.5
    val_c, val_d = 0.5, 0.5
    _update_dac_channels()

def set_rotation(speed):
    """
    Sets the chair to rotate in place.
    :param speed: Controls direction and magnitude.
                  -0.5 (max left) to +0.5 (max right).
                  0.0 is stop.
    """
    global val_a, val_b, val_c, val_d
    
    # Ensure forward/backward are neutral for a point turn
    val_a, val_b = 0.5, 0.5

    # Clamp speed to be between -0.5 and 0.5
    speed = max(-0.5, min(0.5, speed))

    if speed > 0:  # Rotate Right
        val_c = 0.5 + speed
        val_d = 1.0 - val_c
    elif speed < 0:  # Rotate Left
        val_d = 0.5 - speed  # speed is negative, so this adds
        val_c = 1.0 - val_d
    else:  # Stop
        val_c, val_d = 0.5, 0.5
        
    print(f"Command: Rotate ({speed:.2f}) -> Right(C): {val_c:.2f}, Left(D): {val_d:.2f}")
    _update_dac_channels()

if __name__ == "__main__":
    # A simple test to run if you execute this file directly
    import time
    if initialize_dac():
        print("Testing left turn for 2 seconds...")
        set_rotation(-0.3) # 30% speed left
        time.sleep(2)
        
        print("Testing right turn for 2 seconds...")
        set_rotation(0.3) # 30% speed right
        time.sleep(2)
        
        print("Testing stop.")
        stop()
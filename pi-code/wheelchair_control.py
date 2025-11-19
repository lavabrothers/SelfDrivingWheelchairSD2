"""
wheelchair_control.py

This module provides an interface for controlling the wheelchair's motors
via an Adafruit MCP4728 Digital-to-Analog Converter (DAC). It allows for
setting motor speeds for forward/backward movement and left/right rotation,
as well as a full stop.

The DAC outputs are typically connected to motor controllers that interpret
analog voltage signals to drive the wheelchair's motors. The module stores
current DAC values to avoid sending redundant I2C commands, optimizing
communication efficiency.

Dependencies:
- board: Provides access to I2C bus.
- adafruit_mcp4728: Library for interfacing with the MCP4728 DAC.
"""

import board
import adafruit_mcp4728

# --- Global Variables ---
# The MCP4728 DAC object, initialized upon successful connection.
mcp: adafruit_mcp4728.MCP4728 | None = None

# Current normalized values (0.0 to 1.0) for each DAC channel.
# These represent the desired output voltage for the motor controllers.
# Channels A and B typically control forward/backward movement.
# Channels C and D typically control left/right rotation.
val_a, val_b = 0.5, 0.5 # Neutral position for forward/backward.
val_c, val_d = 0.5, 0.5 # Neutral position for left/right.

# --- Functions ---
def initialize_dac() -> adafruit_mcp4728.MCP4728 | None:
    """
    Initializes the I2C connection and attempts to connect to the MCP4728 DAC.

    Upon successful initialization, it sets all DAC channels to a neutral (stop)
    position to ensure the wheelchair is stationary.

    Returns:
        adafruit_mcp4728.MCP4728 | None: The initialized MCP4728 object on success,
                                         or None if initialization fails.
    """
    global mcp
    try:
        i2c = board.I2C() # Get the default I2C bus.
        mcp = adafruit_mcp4728.MCP4728(i2c) # Create the MCP4728 object.
        print("MCP4728 DAC found and initialized.")
        stop()  # Ensure chair is stopped on initialization.
        return mcp
    except Exception as e:
        print(f"Error: Could not find MCP4728. Check I2C connections and address.")
        print(f"Details: {e}")
        mcp = None # Ensure mcp is None if initialization fails.
        return None

def _update_dac_channels():
    """
    Internal function to send the current `val_a`, `val_b`, `val_c`, `val_d`
    values to the hardware DAC channels.

    This function only sends commands if the target value for a channel has
    actually changed, reducing unnecessary I2C communication.
    """
    if mcp is None:
        print("Error: DAC not initialized. Cannot update channels.")
        return

    # Update each channel only if its value has changed.
    if mcp.channel_a.normalized_value != val_a:
        mcp.channel_a.normalized_value = val_a
    if mcp.channel_b.normalized_value != val_b:
        mcp.channel_b.normalized_value = val_b
    if mcp.channel_c.normalized_value != val_c:
        mcp.channel_c.normalized_value = val_c
    if mcp.channel_d.normalized_value != val_d:
        mcp.channel_d.normalized_value = val_d

def stop():
    """
    Halts all wheelchair movement by setting all DAC channels to their neutral (0.5) position.
    """
    global val_a, val_b, val_c, val_d
    print("Command: STOP")
    val_a, val_b = 0.5, 0.5
    val_c, val_d = 0.5, 0.5
    _update_dac_channels()

def set_rotation(speed: float):
    """
    Sets the wheelchair to rotate in place (point turn).

    This function assumes channels C and D control the differential speed for rotation.
    Forward/backward channels (A and B) are set to neutral.

    Args:
        speed (float): Controls the direction and magnitude of rotation.
                       -0.5 (maximum left rotation) to +0.5 (maximum right rotation).
                       0.0 results in no rotation.
    """
    global val_a, val_b, val_c, val_d
    
    # Ensure forward/backward movement is neutral for a pure rotation.
    val_a, val_b = 0.5, 0.5

    # Clamp the speed value to the valid range [-0.5, 0.5].
    speed = max(-0.5, min(0.5, speed))

    if speed > 0:  # Rotate Right
        val_c = 0.5 + speed # Increase right motor speed.
        val_d = 1.0 - val_c # Decrease left motor speed proportionally.
    elif speed < 0:  # Rotate Left
        val_d = 0.5 - speed # Increase left motor speed (speed is negative, so this adds).
        val_c = 1.0 - val_d # Decrease right motor speed proportionally.
    else:  # Stop rotation
        val_c, val_d = 0.5, 0.5
        
    print(f"Command: Rotate ({speed:.2f}) -> Right(C): {val_c:.2f}, Left(D): {val_d:.2f}")
    _update_dac_channels()

def set_joystick_values(fwd: float, back: float, left: float, right: float):
    """
    Directly sets the DAC channel values based on raw joystick inputs.

    This function is intended for direct mapping of joystick axis values (0.0-1.0)
    to DAC channels.

    Args:
        fwd (float): Normalized value for forward movement (0.0-1.0).
        back (float): Normalized value for backward movement (0.0-1.0).
        left (float): Normalized value for left rotation (0.0-1.0).
        right (float): Normalized value for right rotation (0.0-1.0).
    """
    global val_a, val_b, val_c, val_d
    val_a = fwd
    val_b = back
    val_c = right
    val_d = left
    _update_dac_channels()

def set_movement(fwd_bwd: float, left_right: float):
    """
    Sets wheelchair movement based on a centered range for forward/backward and left/right.

    This function translates intuitive control inputs (e.g., -1.0 for full backward,
    1.0 for full forward) into the appropriate DAC normalized values (0.0-1.0).

    Args:
        fwd_bwd (float): Controls forward/backward movement.
                         -1.0 (full backward) to 1.0 (full forward).
        left_right (float): Controls left/right turning.
                            -1.0 (full left) to 1.0 (full right).
    """
    global val_a, val_b, val_c, val_d
    
    # Clamp input values to the valid range [-1.0, 1.0].
    fwd_bwd = max(-1.0, min(1.0, fwd_bwd))
    left_right = max(-1.0, min(1.0, left_right))
    
    # Convert centered range to DAC's 0.0-1.0 range.
    val_a = 0.5 + (fwd_bwd / 2.0)
    val_b = 0.5 - (fwd_bwd / 2.0)
    val_c = 0.5 + (left_right / 2.0)
    val_d = 0.5 - (left_right / 2.0)
    _update_dac_channels()


if __name__ == "__main__":
    """
    Main execution block for testing the wheelchair control module.

    When run directly, this script initializes the DAC and performs a sequence
    of test movements (left turn, right turn, stop) with delays.
    """
    import time
    if initialize_dac():
        print("Testing left turn for 2 seconds...")
        set_rotation(-0.3) # 30% speed left.
        time.sleep(2)
        
        print("Testing right turn for 2 seconds...")
        set_rotation(0.3) # 30% speed right.
        time.sleep(2)
        
        print("Testing stop.")
        stop()

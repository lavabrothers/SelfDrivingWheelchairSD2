# File: imu_sensor.py
"""
This module provides a tilt-compensated compass heading from an
MPU-9250 IMU by talking directly to the sensor's I2C registers
using the 'smbus2' library.

This avoids problematic high-level libraries that fail to install.

--- CRITICAL SETUP ---

1.  INSTALL THE LIBRARY:
    pip install smbus2

2.  ENABLE I2C ON YOUR RASPBERRY PI:
    Use 'sudo raspi-config' -> Interface Options -> I2C -> Enable.

--- ! WARNING: NO CALIBRATION ! ---

This low-level script does NOT include a calibration step.
The magnetometer readings will be "raw" and highly sensitive
to magnetic interference from the wheelchair's motors or
any nearby metal.

For a final project, a manual calibration routine would
be needed, but this script WILL run and give you a
tilt-compensated heading.
"""
import smbus2
import math
import time

# --- Global variables ---
bus = None
# Store the last known heading to return on a failed read
last_known_heading = 0.0

# Smoothing factor for the low-pass filter (0.0 - 1.0)
# Lower values = smoother but more "laggy"
# Higher values = more responsive but "twitchier"
SMOOTHING_FACTOR = 1

# MPU9250 Registers
MPU9250_ADDR = 0x68
PWR_MGMT_1   = 0x6B
INT_PIN_CFG  = 0x37
ACCEL_XOUT_H = 0x3B

# AK8963 Magnetometer Registers
AK8963_ADDR  = 0x0C
AK8963_CNTL  = 0x0A
AK8963_ST1   = 0x02
AK8963_HXL   = 0x03
AK8963_ST2   = 0x09

def _read_word_2c(addr, reg_start):
    """Reads two 8-bit bytes and combines them into one 16-bit signed value."""
    high = bus.read_byte_data(addr, reg_start)
    low  = bus.read_byte_data(addr, reg_start + 1)
    val = (high << 8) + low
    
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def _read_mag_word_le(addr, reg_start):
    """Reads two 8-bit bytes (little-endian) for the magnetometer."""
    low  = bus.read_byte_data(addr, reg_start)
    high = bus.read_byte_data(addr, reg_start + 1)
    val = (high << 8) + low
    
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def initialize_imu():
    """
    Initializes the connection to the IMU using smbus2.
    """
    global bus
    
    print("Initializing IMU (smbus2 direct access)...")
    
    try:
        # Assumes I2C bus 1
        bus = smbus2.SMBus(1)
        
        # 1. Wake up MPU-9250
        bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        
        # 2. Enable I2C Bypass mode to talk to magnetometer
        bus.write_byte_data(MPU9250_ADDR, INT_PIN_CFG, 0x02)
        time.sleep(0.1)
        
        # 3. Configure Magnetometer (AK8963)
        #    Set to 16-bit output, 100Hz continuous mode 2
        bus.write_byte_data(AK8963_ADDR, AK8963_CNTL, 0x16)
        time.sleep(0.1)
        
    except Exception as e:
        print("Error: Could not initialize MPU9250 on I2C bus 1.")
        print("Please ensure the sensor is connected and I2C is enabled.")
        print(f"Details: {e}")
        return False

    print("IMU is stable and providing data.")
    return True

def get_heading():
    """
    Gets the current tilt-compensated compass heading.
    
    Returns:
        (float): A compass heading from 0.0 to 359.9 degrees.
                 Returns the last known good heading on a read failure.
    """
    global last_known_heading, bus
    
    if bus is None:
        print("Error: IMU not initialized.")
        return 0.0  # Return 0 if not ready

    try:
        # 1. Read Accelerometer
        # Read raw 16-bit values
        ax_raw = _read_word_2c(MPU9250_ADDR, ACCEL_XOUT_H)
        ay_raw = _read_word_2c(MPU9250_ADDR, ACCEL_XOUT_H + 2)
        az_raw = _read_word_2c(MPU9250_ADDR, ACCEL_XOUT_H + 4)
        
        # Normalize to G's (default scale is +/- 2g)
        ax = ax_raw / 16384.0
        ay = ay_raw / 16384.0
        
        # 2. Read Magnetometer
        # Check if data is ready
        status = bus.read_byte_data(AK8963_ADDR, AK8963_ST1)
        if not (status & 0x01):
            return last_known_heading # Data not ready, return old value
            
        # Read raw 16-bit values (little-endian)
        mx = _read_mag_word_le(AK8963_ADDR, AK8963_HXL)
        my = _read_mag_word_le(AK8963_ADDR, AK8963_HXL + 2)
        mz = _read_mag_word_le(AK8963_ADDR, AK8963_HXL + 4)
        
        # Read ST2 register to mark data as read
        bus.read_byte_data(AK8963_ADDR, AK8963_ST2)

        # 3. Calculate roll and pitch from accelerometer
        
        # Clamp ax to avoid math domain errors
        if ax > 1.0:  ax = 1.0
        if ax < -1.0: ax = -1.0
        
        pitch = math.asin(-ax)
        
        # Clamp ay/cos(pitch) to avoid math domain errors
        cos_pitch = math.cos(pitch)
        if cos_pitch == 0: cos_pitch = 0.0001 # Avoid divide by zero
        roll_arg = ay / cos_pitch
        if roll_arg > 1.0:  roll_arg = 1.0
        if roll_arg < -1.0: roll_arg = -1.0
        
        roll = math.asin(roll_arg)
        
        # 4. Apply tilt compensation to magnetometer data
        mag_x_comp = mx * math.cos(pitch) + mz * math.sin(pitch)
        mag_y_comp = mx * math.sin(roll) * math.sin(pitch) + \
                     my * math.cos(roll) - \
                     mz * math.sin(roll) * math.cos(pitch)
        
        # 5. Calculate heading
        heading = 180 * math.atan2(mag_y_comp, mag_x_comp) / math.pi
        
        # Convert to 0-360
        heading = (heading + 360) % 360
        
        # --- NEW: Apply smoothing filter ---
        # This handles the 360/0 degree "wraparound"
        angle_diff = heading - last_known_heading
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
            
        smoothed_heading = (last_known_heading + angle_diff * SMOOTHING_FACTOR) % 360
        
        last_known_heading = smoothed_heading
        return smoothed_heading

    except Exception as e:
        # On any read error (e.g., I2C disconnect), return last value
        print(f"IMU read error: {e}. Returning last known heading.")
        return last_known_heading

if __name__ == "__main__":
    # A simple test to run if you execute this file directly
    if initialize_imu():
        print("IMU Test Started. Press Ctrl+C to stop.")
        print("Rotate the sensor to see the heading change.")
        try:
            while True:
                heading = get_heading()
                print(f"Current Compass Heading: {heading: >7.2f} degrees", end="\r")
                time.sleep(0.05) # Poll at 20Hz
                
        except KeyboardInterrupt:
            print("\nStopping IMU test.")
    else:
        print("Could not initialize IMU. Exiting.")

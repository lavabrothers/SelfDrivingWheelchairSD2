# capture_scan.py
# This script performs a 360-degree scan using the MPU-9250 gyroscope for angle data
# and saves the raw sensor data to a CSV file.

import time
import board
import math
import csv
import smbus2  # For MPU-9250
import adafruit_mcp4728
from freenect2 import Device, FrameType

# --- Scan Configuration ---
TURN_SPEED = 0.2
RAW_DATA_FILENAME = "raw_scan_data_gyro.csv"

# --- I2C Addresses and Registers (MPU-9250) ---
MPU9250_ADDRESS = 0x68
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
GYRO_ZOUT_H = 0x47
GYRO_FS_250DPS = 0x00 # Gyro full-scale range: +/- 250 degrees/second

# --- Hardware Setup ---
bus = None
mcp = None
kinect = None

# MPU-9250 Init
try:
    print("Initializing MPU-9250...")
    bus = smbus2.SMBus(1)
    # Wake up MPU-9250
    bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    # Configure Gyroscope
    bus.write_byte_data(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FS_250DPS)
    print("MPU-9250 OK. ✅")
except Exception as e:
    print(f"Error: Could not initialize MPU-9250. Check I2C connections. Details: {e}")
    exit()

# MCP4728 (DAC) Init
try:
    print("Initializing DAC...")
    i2c_board = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c_board)
    print("MCP4728 DAC OK. ✅")
except Exception as e:
    print(f"Error: Could not find MCP4728. Check I2C connections: {e}")
    exit()

# Kinect V2 Init
try:
    print("Initializing Kinect V2 device...")
    kinect = Device()
    print("Kinect OK. ✅")
except Exception as e:
    print(f"Error: Could not initialize Kinect: {e}")
    exit()

# --- Helper Functions ---

def read_word_2c(addr, reg):
    """Reads a 16-bit signed word from the I2C bus."""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) + low
    # Convert from 2's complement
    return val - 65536 if val >= 0x8000 else val

def get_gyro_z():
    """Reads the Z-axis gyro value and returns it in degrees per second."""
    gyro_z_raw = read_word_2c(MPU9250_ADDRESS, GYRO_ZOUT_H)
    # The scale factor for GYRO_FS_250DPS is 131 LSB/(dps)
    return gyro_z_raw / 131.0

def set_turn(speed):
    """Sets the wheelchair motors to turn in place."""
    speed = max(-1.0, min(1.0, speed))
    mcp.channel_c.normalized_value = 0.5 + (speed / 2.0)
    mcp.channel_d.normalized_value = 0.5 - (speed / 2.0)
    # Ensure forward/backward motors are neutral
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5

# --- Main Capture Logic ---

def perform_360_capture():
    """
    Rotates the wheelchair 360 degrees and writes raw depth data to a CSV file,
    using the gyroscope for angle measurement.
    """
    print(f"\n--- Starting 360° Data Capture ---")
    print(f"Data will be saved to '{RAW_DATA_FILENAME}'")
    total_angle_turned = 0.0

    # Open the CSV file to write to
    with open(RAW_DATA_FILENAME, 'w', newline='') as f:
        csv_writer = csv.writer(f)

        try:
            with kinect.running():
                print("Beginning rotation...")
                set_turn(TURN_SPEED)
                last_time = time.monotonic()

                # Loop until we've turned a full 360 degrees
                for frame_type, frame in kinect:
                    if total_angle_turned >= 360.0:
                        break

                    # Time-based angle calculation
                    current_time = time.monotonic()
                    time_delta = current_time - last_time
                    last_time = current_time

                    # Get turn rate in degrees/sec and multiply by time to get degrees turned
                    gyro_z_dps = get_gyro_z()
                    total_angle_turned += abs(gyro_z_dps * time_delta)

                    # Process the Kinect frame if available
                    if frame_type == FrameType.Depth:
                        depth_data = frame.to_array()
                        # Get a vertical slice of pixels at the horizontal center
                        center_u = depth_data.shape[1] // 2
                        depth_slice = depth_data[:, center_u]

                        # Create a row with the accumulated angle and all depth pixels
                        row_data = [total_angle_turned] + depth_slice.tolist()
                        csv_writer.writerow(row_data)

                    print(f"Capturing... Angle: {total_angle_turned:.1f}° / 360°", end='\r')

        except Exception as e:
            print(f"\nAn error occurred during capture: {e}")
        finally:
            print("\nCapture rotation complete. Stopping motors.")
            set_turn(0.0)

# --- Main Execution ---
if __name__ == "__main__":
    set_turn(0.0) # Ensure motors are stopped initially
    input("Press Enter to begin the 360-degree data capture...")
    perform_360_capture()
    print(f"\n✅ Raw data capture complete.")
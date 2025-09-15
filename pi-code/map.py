# capture_scan.py
# This script performs a 360-degree scan and saves the raw sensor data to a CSV file.

import time
import board
import math
import csv # Library for writing CSV files

import adafruit_mpu6050
import adafruit_mcp4728
from freenect2 import Device, FrameType

# --- Scan Configuration ---
TURN_SPEED = 0.2
RAW_DATA_FILENAME = "raw_scan_data.csv"

# --- Hardware Setup (Unchanged) ---
try:
    print("Initializing I2C and sensors...")
    i2c = board.I2C()
    mpu = adafruit_mpu6050.MPU6050(i2c)
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("Hardware OK. ✅")
except Exception as e: print(f"Error: I2C device not found: {e}"); exit()

try:
    print("Initializing Kinect V2 device...")
    kinect = Device()
    print("Kinect OK. ✅")
except Exception as e: print(f"Error: Could not initialize Kinect: {e}"); exit()

def set_turn(speed):
    """Sets the wheelchair motors to turn in place."""
    speed = max(-1.0, min(1.0, speed))
    mcp.channel_c.normalized_value = 0.5 + (speed / 2.0)
    mcp.channel_d.normalized_value = 0.5 - (speed / 2.0)
    mcp.channel_a.normalized_value = 0.5; mcp.channel_b.normalized_value = 0.5

def perform_360_capture():
    """
    Rotates the wheelchair 360 degrees and writes raw depth data to a CSV file.
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

                for frame_type, frame in kinect:
                    if total_angle_turned >= 360.0:
                        break

                    current_time = time.monotonic()
                    time_delta = current_time - last_time
                    last_time = current_time

                    gyro_z_rad_s = mpu.gyro[2]
                    total_angle_turned += abs(math.degrees(gyro_z_rad_s * time_delta))

                    if frame_type == FrameType.Depth:
                        depth_data = frame.to_array()
                        center_u = depth_data.shape[1] // 2
                        depth_slice = depth_data[:, center_u]
                        
                        # Create a row with the angle and all depth pixels
                        row_data = [total_angle_turned] + depth_slice.tolist()
                        
                        # Write the raw data directly to the file
                        csv_writer.writerow(row_data)

                    print(f"Capturing... Angle: {total_angle_turned:.1f}° / 360°", end='\r')

        except Exception as e:
            print(f"\nAn error occurred during capture: {e}")
        finally:
            print("\nCapture rotation complete. Stopping motors.")
            set_turn(0.0)
            
if __name__ == "__main__":
    set_turn(0.0)
    input("Press Enter to begin the 360-degree data capture...")
    perform_360_capture()
    print(f"\n✅ Raw data capture complete.")
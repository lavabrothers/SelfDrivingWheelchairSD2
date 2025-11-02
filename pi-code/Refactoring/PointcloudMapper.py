# File: PointcloudMapper.py
"""
A controller script to perform a 360-degree scan and save the raw sensor data to a CSV file.

This version is modeled after the original 'map.py' but uses the modern hardware 
libraries ('pylibfreenect2' and the new MPU driver) for consistency.

Core Logic:
1.  Initializes all hardware (DAC, IMU, Kinect).
2.  Starts a 360-degree rotation.
3.  Continuously measures the angle turned using the IMU.
4.  For each captured depth frame, it takes a vertical slice of data.
5.  Saves the angle and the raw depth slice to a new row in a CSV file.
6.  Stops and shuts down after the scan is complete.
"""

import time
import sys
import math
import csv
import datetime
import wheelchair_control as wc
import mpu as imu
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, FrameMap

# --- Configuration ---
SCAN_SPEED = 0.2
RAW_DATA_FILENAME_PREFIX = "raw_scan_data"

# --- Global variables for Kinect ---
freenect2 = None
device = None
listener = None

def initialize_kinect():
    """Initializes and starts the Kinect V2 sensor using pylibfreenect2."""
    global freenect2, device, listener
    try:
        print("Initializing Kinect V2 device...")
        freenect2 = Freenect2()
        if freenect2.enumerateDevices() == 0:
            print("FATAL: No Kinect V2 devices found!")
            return False
            
        serial = freenect2.getDeviceSerialNumber(0)
        device = freenect2.openDevice(serial)
        
        listener = SyncMultiFrameListener(FrameType.Depth)
        device.setIrAndDepthFrameListener(listener)
        
        print(f"Starting Kinect V2 stream (Serial: {serial})...")
        device.start()
        print("Kinect OK. ✅")
        return True
        
    except Exception as e:
        print(f"FATAL: Could not initialize Kinect: {e}")
        return False

def shutdown_kinect():
    """Stops and closes the Kinect V2 device."""
    global device
    if device:
        print("Shutting down Kinect V2...")
        try:
            device.stop()
            device.close()
            print("Kinect shutdown complete.")
        except Exception as e:
            print(f"Error during Kinect shutdown: {e}")

def perform_360_capture(csv_writer):
    """
    Rotates the wheelchair 360 degrees and writes raw depth data to the provided CSV writer.
    """
    global listener
    print("\n--- Starting 360° Raw Data Capture ---")
    total_angle_turned = 0.0
    frames = FrameMap()

    try:
        print("Beginning rotation...")
        wc.set_rotation(SCAN_SPEED)
        last_time = time.monotonic()

        while total_angle_turned < 360.0:
            if not listener.waitForNewFrame(frames, 10 * 1000): # 10-second timeout
                print("\nTimeout waiting for new Kinect frame. Aborting scan.")
                break

            depth_frame = frames[FrameType.Depth]
            
            current_time = time.monotonic()
            time_delta = current_time - last_time
            last_time = current_time

            gyro_data = imu.mpu.readGyroscopeMaster()
            gyro_z_dps = gyro_data[2]
            total_angle_turned += abs(gyro_z_dps * time_delta)

            depth_data = depth_frame.asarray()
            # Take a vertical slice from the center of the frame, similar to map.py
            center_u = depth_data.shape[1] // 2
            depth_slice = depth_data[:, center_u]
            
            # Create a row with the angle and all depth pixels
            row_data = [total_angle_turned] + depth_slice.tolist()
            
            # Write the raw data directly to the file
            csv_writer.writerow(row_data)

            print(f"Capturing... Angle: {total_angle_turned:.1f}° / 360°", end='\r')
            
            listener.release(frames)

    except Exception as e:
        print(f"\nAn error occurred during capture: {e}")
    finally:
        print("\nCapture rotation complete. Stopping motors.")
        wc.stop()

def main():
    """Main program flow."""
    print("--- Raw Data Capture (like map.py) ---")
    
    if not wc.initialize_dac() or not imu.initialize_imu() or not initialize_kinect():
        print("FATAL: A required hardware module failed to initialize. Exiting.")
        shutdown_kinect()
        sys.exit(1)
        
    print("\nAll modules initialized.")
    
    filename = f"{RAW_DATA_FILENAME_PREFIX}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"Output will be saved to '{filename}'")
    input("Press Enter to begin the 360-degree data capture...")

    try:
        with open(filename, 'w', newline='') as f:
            csv_writer = csv.writer(f)
            perform_360_capture(csv_writer)
        print("\n--- Raw data capture complete ---")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Stopping program.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        print("Ensuring wheelchair is stopped and shutting down.")
        wc.stop()
        shutdown_kinect()
        print("Shutdown complete. Exiting.")

if __name__ == "__main__":
    print("="*50)
    print("!! SAFETY WARNING !!")
    print("This script will move the wheelchair in a full circle.")
    print("Ensure the area is clear and wheels are OFF THE GROUND for initial testing.")
    print("="*50)
    
    try:
        for i in range(5, 0, -1):
            print(f"Starting in {i}...", end="\r")
            time.sleep(1)
        print("Starting now.                ")
        main()
    except KeyboardInterrupt:
        print("\nProgram start cancelled by user.")
        wc.stop()
        shutdown_kinect()

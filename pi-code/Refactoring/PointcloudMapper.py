"""
PointcloudMapper.py

This module provides a controller script to perform a 360-degree environmental scan
and save the raw sensor data to a CSV file. It is designed to capture a "slice"
of depth data at various rotational angles, which can then be processed offline
to construct a 2D map.

This script is an updated version of the original `map.py`, utilizing modern
hardware libraries (`pylibfreenect2` for Kinect and the refactored MPU driver)
for consistency and improved performance.

Key Features:
- Initializes necessary hardware: DAC for wheelchair rotation, IMU for angle tracking,
  and Kinect V2 for depth data.
- Starts a controlled 360-degree rotation of the wheelchair.
- Continuously measures the angle turned using the IMU's gyroscope.
- For each captured depth frame, it extracts a vertical slice of depth data from the center.
- Saves the current rotational angle and the raw depth slice to a new row in a timestamped CSV file.
- Ensures proper shutdown of all hardware components after the scan is complete or interrupted.

Dependencies:
- time: For timing operations.
- sys: For system exit in case of fatal initialization errors.
- math: For mathematical functions (though less prominent in this raw data capture).
- csv: For writing raw data to CSV files.
- datetime: For generating timestamped filenames.
- wheelchair_control (wc): Custom module for controlling wheelchair movement.
- mpu (imu): Custom module for reading IMU data (gyroscope).
- pylibfreenect2: Python wrapper for libfreenect2, used to interface with the Kinect V2.
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
SCAN_SPEED = 0.2                    # Speed at which the wheelchair rotates during the 360-degree scan.
RAW_DATA_FILENAME_PREFIX = "raw_scan_data" # Prefix for the output CSV file containing raw scan data.

# --- Global variables for Kinect ---
freenect2: Freenect2 | None = None  # Freenect2 object for managing Kinect devices.
device = None                       # Represents the opened Kinect V2 device.
listener: SyncMultiFrameListener | None = None # Listener for receiving frames from the Kinect.

def initialize_kinect() -> bool:
    """
    Initializes and starts the Kinect V2 sensor using the `pylibfreenect2` library.

    This function sets up the Freenect2 context, enumerates devices, opens the
    first detected Kinect, configures the frame listener for depth frames only,
    and starts the Kinect V2 stream.

    Returns:
        bool: True if the Kinect V2 sensor was successfully initialized and started,
              False otherwise.
    """
    global freenect2, device, listener
    try:
        print("Initializing Kinect V2 device...")
        freenect2 = Freenect2()
        if freenect2.enumerateDevices() == 0:
            print("FATAL: No Kinect V2 devices found!")
            return False
            
        serial = freenect2.getDeviceSerialNumber(0)
        device = freenect2.openDevice(serial)
        
        # For raw data capture, only depth frames are needed.
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
    """
    Stops and closes the Kinect V2 device.

    This function is critical for releasing hardware resources and should always
    be called when the Kinect is no longer needed to prevent resource leaks or
    issues with subsequent sensor initialization.
    """
    global device
    if device:
        print("Shutting down Kinect V2...")
        try:
            device.stop()
            device.close()
            print("Kinect shutdown complete.")
        except Exception as e:
            print(f"Error during Kinect shutdown: {e}")

def perform_360_capture(csv_writer: csv.writer):
    """
    Rotates the wheelchair 360 degrees and continuously captures raw depth data
    along with the current rotational angle, writing them to the provided CSV writer.

    The rotation continues until 360 degrees are covered or an error occurs.

    Args:
        csv_writer (csv.writer): A CSV writer object to write the captured data.
    """
    global listener
    print("\n--- Starting 360° Raw Data Capture ---")
    total_angle_turned = 0.0    # Accumulator for the total angle turned.
    frames = FrameMap()         # FrameMap to hold incoming Kinect frames.

    try:
        print("Beginning rotation...")
        wc.set_rotation(SCAN_SPEED) # Start the wheelchair rotating.
        last_time = time.monotonic() # Timestamp for calculating time delta.

        while total_angle_turned < 360.0:
            # Wait for a new depth frame from the Kinect.
            if not listener.waitForNewFrame(frames, 10 * 1000): # 10-second timeout.
                print("\nTimeout waiting for new Kinect frame. Aborting scan.")
                break

            depth_frame = frames[FrameType.Depth]
            
            current_time = time.monotonic()
            time_delta = current_time - last_time
            last_time = current_time

            # Read gyroscope data from the IMU to track rotation.
            gyro_data = imu.mpu.readGyroscopeMaster()
            gyro_z_dps = gyro_data[2] # Z-axis gyroscope reading in degrees per second.
            
            # Accumulate the total angle turned based on gyroscope data.
            total_angle_turned += abs(gyro_z_dps * time_delta)

            depth_data = depth_frame.asarray()
            # Take a vertical slice from the center of the frame.
            center_u = depth_data.shape[1] // 2
            depth_slice = depth_data[:, center_u]
            
            # Create a row with the current angle and all depth pixels in the slice.
            row_data = [total_angle_turned] + depth_slice.tolist()
            
            # Write the raw data directly to the file.
            csv_writer.writerow(row_data)

            print(f"Capturing... Angle: {total_angle_turned:.1f}° / 360°", end='\r')
            
            listener.release(frames) # Release the frames immediately after processing.

    except Exception as e:
        print(f"\nAn error occurred during capture: {e}")
    finally:
        print("\nCapture rotation complete. Stopping motors.")
        wc.stop() # Ensure motors are stopped after capture.

def main():
    """
    Main program flow for the raw data capture.

    This function initializes all required hardware (DAC, IMU, Kinect),
    prompts the user to start the scan, performs the 360-degree capture,
    saves the raw data to a CSV file, and ensures proper shutdown of all modules.
    """
    print("--- Raw Data Capture (like map.py) ---")
    
    # Initialize all critical hardware modules. Exit if any fail.
    if not wc.initialize_dac() or not imu.initialize_imu() or not initialize_kinect():
        print("FATAL: A required hardware module failed to initialize. Exiting.")
        shutdown_kinect() # Ensure Kinect is shut down even on partial failure.
        sys.exit(1)
        
    print("\nAll modules initialized.")
    
    # Generate a unique filename with a timestamp for the raw data.
    filename = f"{RAW_DATA_FILENAME_PREFIX}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"Output will be saved to '{filename}'")
    input("Press Enter to begin the 360-degree data capture...")

    try:
        with open(filename, 'w', newline='') as f:
            csv_writer = csv.writer(f)
            perform_360_capture(csv_writer) # Perform the data capture.
        print("\n--- Raw data capture complete ---")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Stopping program.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        print("Ensuring wheelchair is stopped and shutting down.")
        wc.stop() # Ensure wheelchair motors are stopped.
        shutdown_kinect() # Ensure Kinect is properly shut down.
        print("Shutdown complete. Exiting.")

if __name__ == "__main__":
    """
    Entry point for the script when executed directly.

    Includes a safety warning and a countdown before starting the main mapping
    process, allowing the user to prepare or cancel.
    """
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
        wc.stop() # Ensure wheelchair stops if cancelled during countdown.
        shutdown_kinect() # Ensure Kinect is shut down if cancelled during countdown.

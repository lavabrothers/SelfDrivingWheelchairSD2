"""
mapping_module.py

This module is responsible for performing a 360-degree environmental scan using the
Kinect V2 sensor and an IMU (Inertial Measurement Unit). It captures raw depth data
along with rotational information, saves it to a CSV file, and then triggers an
automatic processing step to convert this raw data into a 2D point cloud map.

The module is designed to integrate with other parts of the wheelchair control system,
sharing the Kinect listener and lock with the `person_detector` module to ensure
synchronized access to sensor data.

Key Features:
- Initializes necessary hardware: DAC for wheelchair rotation and IMU for angle tracking.
- Acquires and shares the Kinect V2 listener and frame map from the `person_detector` module.
- Performs a controlled 360-degree rotation of the wheelchair while capturing depth data.
- Saves raw depth data and corresponding rotational angles to a timestamped CSV file.
- Supports interruption of the mapping process via a `threading.Event`.
- Automatically triggers an external script (`process_pointcloud.py`) to generate
  the final point cloud map.

Dependencies:
- time: For timing operations.
- sys, os: For system and path operations.
- csv: For writing raw data to CSV files.
- datetime: For generating timestamped filenames.
- subprocess: For executing the point cloud processing script.
- threading: For managing the stop event to interrupt mapping.
- wheelchair_control (wc): Custom module for controlling wheelchair movement.
- mpu (imu): Custom module for reading IMU data (gyroscope).
- person_detector (vision): Custom module that manages the Kinect V2 listener and lock.
- pylibfreenect2: Python wrapper for libfreenect2, used for Kinect V2 frame types.
"""

import time
import sys
import os
import csv
import datetime
import subprocess
import threading
import wheelchair_control as wc
import mpu as imu
import person_detector as vision # Import the vision module to get its lock/listener
from pylibfreenect2 import FrameType, FrameMap

# --- Configuration ---
SCAN_SPEED = 0.15                   # Speed at which the wheelchair rotates during a 360-degree scan.
RAW_DATA_FILENAME_PREFIX = "raw_scan_data" # Prefix for the raw CSV data files.

# --- Global variables ---
listener = None                     # Reference to the shared Kinect V2 SyncMultiFrameListener.
frames = None                       # Reference to the shared Kinect V2 FrameMap.
latest_csv_filename = None          # Stores the filename of the most recently captured raw data.

def initialize() -> bool:
    """
    Initializes all hardware and shared resources required for the mapping module.

    This includes initializing the DAC for wheelchair control, the IMU for rotation
    tracking, and acquiring the shared Kinect V2 listener and frame map from the
    `person_detector` module.

    Returns:
        bool: True if all necessary components are successfully initialized, False otherwise.
    """
    global listener, frames
    print("--- Initializing Mapping Module ---")
    
    # Initialize the Digital-to-Analog Converter for motor control.
    if not wc.initialize_dac():
        print("FATAL: DAC initialization failed.")
        return False
    
    # Initialize the IMU for gyroscope readings.
    if not imu.initialize_imu():
        print("FATAL: IMU initialization failed.")
        return False
    
    # Acquire the Kinect listener and frame map from the `person_detector` module.
    # This ensures that both mapping and person detection can share the same Kinect stream.
    if 'person_detector' not in sys.modules or vision.listener is None or vision.frames is None:
        print("FATAL: Could not get Kinect listener from vision module.")
        print("Ensure vision.initialize_detector() is called first in the main flow.")
        return False
    
    print("Mapping module acquiring listener from person_detector... ✅")
    listener = vision.listener # Use the *exact same* listener instance.
    frames = vision.frames     # Use the *exact same* FrameMap instance.
    
    print("--- Mapping Module Initialized OK ✅ ---")
    return True

def shutdown():
    """
    Shuts down all hardware and releases resources used by the mapping module.

    This primarily involves stopping the wheelchair motors. The Kinect V2 sensor
    is managed and shut down by the `person_detector` module in the main program flow.
    """
    print("\n--- Shutting Down Mapping Module ---")
    wc.stop() # Ensure wheelchair motors are stopped.
    # The Kinect V2 sensor is not shut down here, as it's shared with person_detector.
    # The main program is responsible for calling vision.shutdown_detector().
    print("--- Mapping Module Shutdown Complete ---")

def perform_mapping(stop_event: threading.Event):
    """
    Executes a full 360-degree mapping scan.

    This function orchestrates the data capture, saves the raw data to a CSV file,
    and then triggers the point cloud processing. The process can be gracefully
    interrupted by setting the provided `stop_event`.

    Args:
        stop_event (threading.Event): An event object that, when set, signals
                                      the mapping process to halt.
    """
    global latest_csv_filename
    
    # Determine the output directory for the raw data file.
    script_dir = os.path.dirname(__file__)
    output_dir = os.path.abspath(os.path.join(script_dir, '..')) # Parent directory of Refactoring.
    
    # Generate a unique filename with a timestamp.
    filename = os.path.join(output_dir, f"{RAW_DATA_FILENAME_PREFIX}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    
    latest_csv_filename = filename
    print(f"Raw scan data will be saved to '{filename}'")

    try:
        with open(filename, 'w', newline='') as f:
            csv_writer = csv.writer(f)
            # Perform the 360-degree data capture.
            _perform_360_capture(csv_writer, stop_event)
        
        # Only proceed with data processing if the scan was not interrupted.
        if not stop_event.is_set():
            print("\n--- Raw data capture complete ---")
            _process_data()
        else:
            print("\n--- Raw data capture interrupted. Skipping processing. ---")

    except Exception as e:
        print(f"\nAn unexpected error occurred during mapping: {e}")
    finally:
        wc.stop() # Ensure motors are stopped even if an error occurs.

def _perform_360_capture(csv_writer: csv.writer, stop_event: threading.Event):
    """
    Rotates the wheelchair 360 degrees, continuously capturing Kinect depth data
    and IMU gyroscope readings, and writes them to the provided CSV writer.

    The rotation continues until 360 degrees are covered or the `stop_event` is set.

    Args:
        csv_writer (csv.writer): A CSV writer object to write the captured data.
        stop_event (threading.Event): An event object to check for interruption signals.
    """
    global listener, frames
    print("\n--- Starting 360° Raw Data Capture ---")
    total_angle_turned = 0.0 # Accumulator for the total angle turned.

    try:
        print("Beginning rotation...")
        wc.set_rotation(SCAN_SPEED) # Start the wheelchair rotating.
        last_time = time.monotonic() # Timestamp for calculating time delta.

        while total_angle_turned < 360.0:
            
            # Check if a stop signal has been received.
            if stop_event.is_set():
                print("\nMapping scan: Stop signal received. Halting rotation.")
                break

            depth_frame = None
            
            # Acquire the Kinect lock from the vision module to safely access frames.
            with vision.kinect_lock:
                if not listener.waitForNewFrame(frames, 10 * 1000): # Wait for a new depth frame.
                    print("\nTimeout waiting for new Kinect frame. Aborting scan.")
                    break

                depth_frame = frames[FrameType.Depth]
                depth_data = depth_frame.asarray() # Get the depth data as a NumPy array.
                
                listener.release(frames) # Release the frames immediately after use.
            # The Kinect lock is released here.

            current_time = time.monotonic()
            time_delta = current_time - last_time
            last_time = current_time

            # Read gyroscope data from the IMU.
            gyro_data = imu.mpu.readGyroscopeMaster()
            gyro_z_dps = gyro_data[2] # Z-axis gyroscope reading in degrees per second.
            
            # Accumulate the total angle turned based on gyroscope data.
            total_angle_turned += abs(gyro_z_dps * time_delta)

            # Extract a central slice of the depth map.
            center_u = depth_data.shape[1] // 2 # Center column index.
            depth_slice = depth_data[:, center_u] # All rows for the center column.
            
            # Write the current angle and depth slice to the CSV file.
            row_data = [total_angle_turned] + depth_slice.tolist()
            csv_writer.writerow(row_data)

            print(f"Capturing... Angle: {total_angle_turned:.1f}° / 360°", end='\r')
            
    finally:
        print("\nCapture rotation complete. Stopping motors.")
        wc.stop() # Ensure motors are stopped after capture.

def _process_data():
    """
    Invokes the `process_pointcloud.py` script to convert the raw CSV data
    into a point cloud map.

    This function uses `subprocess` to run the external script, ensuring that
    the processing happens automatically after data capture.
    """
    global latest_csv_filename
    if latest_csv_filename:
        print("\n--- Starting Point Cloud Processing ---")
        try:
            script_dir = os.path.dirname(__file__)
            script_path = os.path.join(script_dir, "process_pointcloud.py")
            
            # Run the processing script, checking for successful execution.
            # The current working directory for the subprocess is set to the parent
            # directory of the script_dir (i.e., pi-code/).
            subprocess.run(["python3", script_path], check=True, cwd=os.path.dirname(script_dir))
            print("--- Point Cloud Processing Complete ✅ ---")
        except FileNotFoundError:
            print(f"FATAL: Could not find the processing script at '{script_path}'")
        except subprocess.CalledProcessError as e:
            print(f"FATAL: Error occurred while running processing script: {e}")
    else:
        print("No data file was captured, skipping processing.")

if __name__ == "__main__":
    """
    Main execution block for testing the mapping module.

    NOTE: This test mode is highly dependent on `person_detector.py` being
    initialized first to provide the Kinect listener and lock. It is generally
    recommended to run the full `visual_main_flow.py` or `main_flow.py` for
    integrated testing.
    """
    print("WARNING: Test mode for mapping_module.py is now dependent on")
    print("person_detector.py being present and its models.")
    print("It is recommended to run the main visual_main_flow.py instead.")
    # Example of how to run this in a test scenario (requires manual setup of vision):
    # import person_detector
    # if person_detector.initialize_detector():
    #     if initialize():
    #         try:
    #             stop_event = threading.Event()
    #             perform_mapping(stop_event)
    #         except KeyboardInterrupt:
    #             stop_event.set()
    #             print("\nMapping interrupted by user.")
    #         finally:
    #             shutdown()
    #             person_detector.shutdown_detector()
    #     else:
    #         print("Mapping module initialization failed.")
    # else:
    #     print("Person detector initialization failed.")

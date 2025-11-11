# File: mapping_module.py
"""
A module to perform a 360-degree scan, save the raw sensor data,
and then automatically process it into a point cloud.
"""

import time
import sys
import os
import csv
import datetime
import subprocess
import wheelchair_control as wc
import mpu as imu
# <--- MODIFIED: Import the vision module to get its lock/listener ---
import person_detector as vision
from pylibfreenect2 import FrameType, FrameMap

# --- Configuration ---
SCAN_SPEED = 0.15
RAW_DATA_FILENAME_PREFIX = "raw_scan_data"

# --- Global variables ---
# <--- MODIFIED: We no longer manage the kinect device, just use its listener ---
listener = None
frames = None
latest_csv_filename = None

def initialize():
    """Initializes all hardware required for mapping."""
    global listener, frames # <--- ADDED globals
    print("--- Initializing Mapping Module ---")
    if not wc.initialize_dac():
        print("FATAL: DAC initialization failed.")
        return False
    if not imu.initialize_imu():
        print("FATAL: IMU initialization failed.")
        return False
    
    # <--- MODIFIED: Get listener from person_detector ---
    if 'person_detector' not in sys.modules or vision.listener is None or vision.frames is None:
        print("FATAL: Could not get Kinect listener from vision module.")
        print("Ensure vision.initialize_detector() is called first.")
        return False
    
    print("Mapping module acquiring listener from person_detector... ✅")
    listener = vision.listener # Use the *exact same* listener
    frames = vision.frames     # Use the *exact same* FrameMap
    
    print("--- Mapping Module Initialized OK ✅ ---")
    return True

# <--- REMOVED: initialize_kinect() is no longer needed ---

def shutdown():
    """Shuts down all hardware used by the mapping module."""
    print("\n--- Shutting Down Mapping Module ---")
    wc.stop()
    # <--- MODIFIED: We do not shut down the kinect here. ---
    # The main program will call vision.shutdown_detector()
    print("--- Mapping Module Shutdown Complete ---")

# <--- REMOVED: shutdown_kinect() is no longer needed ---

def perform_mapping():
    """
    Performs a 360-degree scan, saves the data, and processes it.
    """
    global latest_csv_filename
    
    script_dir = os.path.dirname(__file__)
    output_dir = os.path.abspath(os.path.join(script_dir, '..'))
    filename = os.path.join(output_dir, f"{RAW_DATA_FILENAME_PREFIX}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    
    latest_csv_filename = filename
    print(f"Output will be saved to '{filename}'")

    try:
        with open(filename, 'w', newline='') as f:
            csv_writer = csv.writer(f)
            _perform_360_capture(csv_writer)
        print("\n--- Raw data capture complete ---")
        
        _process_data()

    except Exception as e:
        print(f"\nAn unexpected error occurred during mapping: {e}")
    finally:
        wc.stop()

def _perform_360_capture(csv_writer):
    """
    Rotates the wheelchair 360 degrees and writes raw depth data.
    """
    global listener, frames # <--- MODIFIED
    print("\n--- Starting 360° Raw Data Capture ---")
    total_angle_turned = 0.0
    # <--- REMOVED: Do not create a new FrameMap, use the global one ---
    # frames = FrameMap()

    try:
        print("Beginning rotation...")
        wc.set_rotation(SCAN_SPEED)
        last_time = time.monotonic()

        while total_angle_turned < 360.0:
            depth_frame = None
            
            # <--- MODIFIED: Use the lock from the vision module ---
            with vision.kinect_lock:
                if not listener.waitForNewFrame(frames, 10 * 1000):
                    print("\nTimeout waiting for new Kinect frame. Aborting scan.")
                    break

                depth_frame = frames[FrameType.Depth]
                depth_data = depth_frame.asarray() # Get data *inside* the lock
                
                listener.release(frames) # Release *inside* the lock
            # <--- MODIFIED: Lock is released here ---

            current_time = time.monotonic()
            time_delta = current_time - last_time
            last_time = current_time

            gyro_data = imu.mpu.readGyroscopeMaster()
            gyro_z_dps = gyro_data[2]
            total_angle_turned += abs(gyro_z_dps * time_delta)

            center_u = depth_data.shape[1] // 2
            depth_slice = depth_data[:, center_u]
            
            row_data = [total_angle_turned] + depth_slice.tolist()
            csv_writer.writerow(row_data)

            print(f"Capturing... Angle: {total_angle_turned:.1f}° / 360°", end='\r')
            
    finally:
        print("\nCapture rotation complete. Stopping motors.")
        wc.stop()

def _process_data():
    """
    Calls the process_pointcloud.py script to process the latest CSV file.
    """
    global latest_csv_filename
    if latest_csv_filename:
        print("\n--- Starting Point Cloud Processing ---")
        try:
            script_dir = os.path.dirname(__file__)
            script_path = os.path.join(script_dir, "process_pointcloud.py")
            
            subprocess.run(["python3", script_path], check=True, cwd=os.path.dirname(script_dir))
            print("--- Point Cloud Processing Complete ✅ ---")
        except FileNotFoundError:
            print(f"FATAL: Could not find the processing script at '{script_path}'")
        except subprocess.CalledProcessError as e:
            print(f"FATAL: Error occurred while running processing script: {e}")
    else:
        print("No data file was captured, skipping processing.")

if __name__ == "__main__":
    # ... (Test mode remains the same, but now requires person_detector.py) ...
    print("WARNING: Test mode for mapping_module.py is now dependent on")
    print("person_detector.py being present and its models.")
    print("It is recommended to run the main visual_main_flow.py instead.")

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
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, FrameMap

# --- Configuration ---
SCAN_SPEED = 0.2
RAW_DATA_FILENAME_PREFIX = "raw_scan_data"

# --- Global variables for Kinect ---
freenect2 = None
device = None
listener = None
latest_csv_filename = None
_kinect_initialized_by_this_module = False

def initialize():
    """Initializes all hardware required for mapping."""
    print("--- Initializing Mapping Module ---")
    if not wc.initialize_dac():
        print("FATAL: DAC initialization failed.")
        return False
    if not imu.initialize_imu():
        print("FATAL: IMU initialization failed.")
        return False
    if not initialize_kinect():
        print("FATAL: Kinect initialization failed.")
        return False
    print("--- Mapping Module Initialized OK ✅ ---")
    return True

def initialize_kinect():
    """Initializes and starts the Kinect V2 sensor using pylibfreenect2."""
    global freenect2, device, listener, _kinect_initialized_by_this_module
    
    # If another module (like person_detector) has already initialized freenect2,
    # we can likely re-use the instance. This is a simplification to avoid
    # complex hardware sharing logic for now.
    if 'person_detector' in sys.modules and sys.modules['person_detector'].freenect2 is not None:
        print("Kinect already initialized by another module. Re-using instance.")
        freenect2 = sys.modules['person_detector'].freenect2
        device = sys.modules['person_detector'].kinect
        # We still need our own listener for the depth frames
        listener = SyncMultiFrameListener(FrameType.Depth)
        device.setIrAndDepthFrameListener(listener)
        device.start() # Ensure it's started
        _kinect_initialized_by_this_module = False # Mark that we didn't init it
        return True

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
        _kinect_initialized_by_this_module = True # Mark that we initialized it
        print("Kinect OK. ✅")
        return True
        
    except Exception as e:
        print(f"FATAL: Could not initialize Kinect: {e}")
        return False

def shutdown():
    """Shuts down all hardware used by the mapping module."""
    print("\n--- Shutting Down Mapping Module ---")
    wc.stop()
    shutdown_kinect()
    print("--- Mapping Module Shutdown Complete ---")

def shutdown_kinect():
    """Stops and closes the Kinect V2 device."""
    global device, _kinect_initialized_by_this_module
    if device and _kinect_initialized_by_this_module:
        print("Shutting down Kinect V2 (initialized by mapping_module)...")
        try:
            device.stop()
            device.close()
            print("Kinect shutdown complete.")
        except Exception as e:
            print(f"Error during Kinect shutdown: {e}")
    elif device:
        print("Skipping Kinect shutdown (not initialized by this module).")

def perform_mapping():
    """
    Performs a 360-degree scan, saves the data, and processes it.
    """
    global latest_csv_filename
    
    # Save the CSV in the parent directory ('pi-code')
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
        
        # Automatically process the captured data
        _process_data()

    except Exception as e:
        print(f"\nAn unexpected error occurred during mapping: {e}")
    finally:
        wc.stop()

def _perform_360_capture(csv_writer):
    """
    Rotates the wheelchair 360 degrees and writes raw depth data.
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
            if not listener.waitForNewFrame(frames, 10 * 1000):
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
            center_u = depth_data.shape[1] // 2
            depth_slice = depth_data[:, center_u]
            
            row_data = [total_angle_turned] + depth_slice.tolist()
            csv_writer.writerow(row_data)

            print(f"Capturing... Angle: {total_angle_turned:.1f}° / 360°", end='\r')
            
            listener.release(frames)

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
            # Construct a robust path to the processing script
            script_dir = os.path.dirname(__file__)
            script_path = os.path.join(script_dir, "process_pointcloud.py")
            
            # The processing script finds the latest CSV in the parent dir,
            # which is where we saved it.
            subprocess.run(["python3", script_path], check=True, cwd=os.path.dirname(script_dir))
            print("--- Point Cloud Processing Complete ✅ ---")
        except FileNotFoundError:
            print(f"FATAL: Could not find the processing script at '{script_path}'")
        except subprocess.CalledProcessError as e:
            print(f"FATAL: Error occurred while running processing script: {e}")
    else:
        print("No data file was captured, skipping processing.")

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
        print("Starting test mapping sequence...")
        
        if initialize():
            perform_mapping()
    except KeyboardInterrupt:
        print("\nProgram cancelled by user.")
    finally:
        shutdown()
        print("\nTest complete.")

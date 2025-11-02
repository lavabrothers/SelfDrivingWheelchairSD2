# File: PointcloudMapper_3D.py
"""
A controller script to perform a 360-degree scan and save the resulting 
3D point cloud to a CSV file.

This version merges the 3D processing logic (from patches) with the 
hardware control framework (from PointcloudMapper.py).

Core Logic:
1.  Initializes all hardware (DAC, IMU, Kinect).
2.  Gets the Kinect's intrinsic camera parameters.
3.  Starts a 360-degree rotation.
4.  Continuously measures the angle turned using the IMU.
5.  For each captured depth frame, it converts the *entire frame* to 
    3D (x,y,z) sensor-relative points.
6.  It then transforms those points into world-relative (x,y,z) coordinates
    based on the current IMU heading.
7.  All world-relative points are collected into a global list.
8.  After the scan is complete, the full point cloud is saved to a CSV file.
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
RAW_DATA_FILENAME_PREFIX = "point_cloud_map" # Changed from raw_scan_data
SCAN_RANGE_MM = 8000 # Ignore points further than 8 meters
PIXEL_STEP = 15 # Process every 15th pixel to keep point cloud manageable

# --- Global variables for Kinect ---
freenect2 = None
device = None
listener = None
ir_params = None # To store camera intrinsics

def initialize_kinect():
    """Initializes and starts the Kinect V2 sensor using pylibfreenect2."""
    global freenect2, device, listener, ir_params
    try:
        print("Initializing Kinect V2 device...")
        freenect2 = Freenect2()
        if freenect2.enumerateDevices() == 0:
            print("FATAL: No Kinect V2 devices found!")
            return False
            
        serial = freenect2.getDeviceSerialNumber(0)
        device = freenect2.openDevice(serial)
        
        # We only need the depth frames for this mapping task
        listener = SyncMultiFrameListener(FrameType.Depth)
        device.setIrAndDepthFrameListener(listener)
        
        print(f"Starting Kinect V2 stream (Serial: {serial})...")
        device.start()

        # Get camera intrinsic parameters
        ir_params = device.getIrCameraParams()
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

def convert_frame_to_3d_points(depth_frame, params):
    """Converts a full depth frame into a list of 3D (x, y, z) points."""
    points = []
    depth_data = depth_frame.asarray()
    
    # Iterate over the depth image with a step to reduce point density
    for v in range(0, depth_frame.height, PIXEL_STEP):
        for u in range(0, depth_frame.width, PIXEL_STEP):
            depth_mm = depth_data[v, u]
            
            if depth_mm == 0 or depth_mm > SCAN_RANGE_MM:
                continue

            # Convert to meters for calculation
            z_cam = depth_mm / 1000.0
            # Use intrinsic parameters for accurate 3D conversion
            x_cam = (u - params.cx) * z_cam / params.fx
            y_cam = (v - params.cy) * z_cam / params.fy
            
            # We return coordinates in mm
            points.append((x_cam * 1000.0, y_cam * 1000.0, z_cam * 1000.0))
            
    return points

def transform_points(point_slice, heading_deg):
    """Rotates sensor-relative 3D (x,y,z) points to world-relative 3D points."""
    heading_rad = math.radians(heading_deg)
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    world_points = []
    # The sensor's (x, z) plane is the floor plane, y is height.
    # We rotate around the sensor's y-axis.
    for sensor_x, sensor_y, sensor_z in point_slice:
        # Rotation is applied to the horizontal plane (x and z)
        world_x = (sensor_x * cos_h) - (sensor_z * sin_h)
        world_z = (sensor_x * sin_h) + (sensor_z * cos_h)
        # Height (y) remains unchanged by this rotation
        world_y = sensor_y
        world_points.append((world_x, world_y, world_z))
    return world_points

def perform_360_capture():
    """
    Rotates the wheelchair 360 degrees and captures a full 3D point cloud.
    Returns the completed point cloud.
    """
    global listener, ir_params
    print("\n--- Starting 360° 3D Point Cloud Capture ---")
    total_angle_turned = 0.0
    frames = FrameMap()
    global_point_cloud = []

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

            # Process the entire frame for 3D points
            sensor_points = convert_frame_to_3d_points(depth_frame, ir_params)
            world_points = transform_points(sensor_points, total_angle_turned)
            global_point_cloud.extend(world_points)

            print(f"Scanning... Angle: {total_angle_turned:.1f}° / 360°, Points this frame: {len(world_points): >4}", end='\r')
            
            listener.release(frames)

    except Exception as e:
        print(f"\nAn error occurred during capture: {e}")
    finally:
        print("\nCapture rotation complete. Stopping motors.")
        wc.stop()
        return global_point_cloud

def save_point_cloud_to_csv(point_cloud_data):
    """Saves the collected (world_x, world_y, world_z) points to a CSV file."""
    if not point_cloud_data:
        print("No point cloud data to save.")
        return
    filename = f"{RAW_DATA_FILENAME_PREFIX}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    try:
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["world_x_mm", "world_y_mm", "world_z_mm"])
            for row in point_cloud_data:
                # row[0]=x, row[1]=y (height), row[2]=z
                writer.writerow([f"{row[0]:.0f}", f"{row[1]:.0f}", f"{row[2]:.0f}"])
        print(f"\n--- 3D Point cloud map saved successfully to {filename} ---")
    except Exception as e:
        print(f"\n--- Error saving CSV file: {e} ---")

def main():
    """Main program flow."""
    print("--- 3D Point Cloud Capture ---")
    
    if not wc.initialize_dac() or not imu.initialize_imu() or not initialize_kinect():
        print("FATAL: A required hardware module failed to initialize. Exiting.")
        shutdown_kinect()
        sys.exit(1)
        
    print("\nAll modules initialized.")
    
    print(f"Output will be prefixed with '{RAW_DATA_FILENAME_PREFIX}'")
    input("Press Enter to begin the 360-degree data capture...")

    try:
        # Perform the scan and get the collected data
        point_cloud_data = perform_360_capture()
        
        print("\n--- 3D point cloud capture complete ---")
        
        # Save the data to a file
        save_point_cloud_to_csv(point_cloud_data)

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
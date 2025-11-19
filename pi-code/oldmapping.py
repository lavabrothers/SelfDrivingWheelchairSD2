"""
oldmapping.py

This module implements a legacy 3D point cloud mapping routine for the self-driving
wheelchair. It performs a 360-degree scan, capturing full depth frames from the
Kinect V2 sensor, converting them into 3D points, and transforming these points
into a world-relative coordinate system using IMU heading data. The collected
3D point cloud is then saved to a CSV file.

This script represents an earlier approach to 3D mapping, where the entire depth
frame is processed into a point cloud at each step of the rotation.

Key Features:
- Initializes necessary hardware: DAC for wheelchair rotation, IMU for angle tracking,
  and Kinect V2 for depth data.
- Retrieves Kinect's intrinsic camera parameters for accurate 3D point conversion.
- Rotates the wheelchair 360 degrees, continuously capturing depth frames and IMU data.
- Converts each depth frame into a list of 3D (x, y, z) points relative to the sensor.
- Transforms sensor-relative points to world-relative coordinates based on the current
  rotational angle measured by the IMU.
- Collects all world-relative points into a global point cloud.
- Saves the final 3D point cloud to a timestamped CSV file.

Dependencies:
- time: For timing operations.
- sys: For system exit in case of fatal initialization errors.
- math: For trigonometric calculations during point transformation.
- csv: For writing point cloud data to CSV files.
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
SCAN_SPEED = 0.2            # Speed at which the wheelchair rotates during the 360-degree scan.
RAW_DATA_FILENAME_PREFIX = "point_cloud_map" # Prefix for the output CSV file.
SCAN_RANGE_MM = 8000        # Maximum depth distance (in mm) to consider for points (8 meters).
PIXEL_STEP = 15             # Processes every Nth pixel to reduce point cloud density and processing load.

# --- Global variables for Kinect ---
freenect2 = None    # Freenect2 object for managing Kinect devices.
device = None       # Represents the opened Kinect V2 device.
listener = None     # Listener for receiving frames from the Kinect.
ir_params = None    # Stores Kinect's infrared camera intrinsic parameters for 3D conversion.

def initialize_kinect() -> bool:
    """
    Initializes and starts the Kinect V2 sensor using the `pylibfreenect2` library.

    This function sets up the Freenect2 context, enumerates devices, opens the
    first detected Kinect, configures the frame listener for depth frames only,
    starts the Kinect V2 stream, and retrieves the intrinsic camera parameters.

    Returns:
        bool: True if the Kinect V2 sensor was successfully initialized and started,
              False otherwise.
    """
    global freenect2, device, listener, ir_params
    try:
        print("Initializing Kinect V2 device...")
        freenect2 = Freenect2()
        if freenect2.enumerateDevices() == 0:
            print("FATAL: No Kinect V2 devices found!")
            return False
            
        serial = freenect2.getDeviceSerialNumber(0)
        device = freenect2.openDevice(serial)
        
        # For 3D mapping, only depth frames are needed.
        listener = SyncMultiFrameListener(FrameType.Depth)
        device.setIrAndDepthFrameListener(listener)
        
        print(f"Starting Kinect V2 stream (Serial: {serial})...")
        device.start()

        # Get camera intrinsic parameters, essential for converting 2D depth pixels to 3D coordinates.
        ir_params = device.getIrCameraParams()
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

def convert_frame_to_3d_points(depth_frame: Frame, params) -> list[tuple[float, float, float]]:
    """
    Converts a full depth frame into a list of 3D (x, y, z) points relative to the camera.

    It iterates through the depth image, applying a `PIXEL_STEP` to reduce density,
    and uses the camera's intrinsic parameters to project each valid depth pixel
    into 3D space. Points outside the `SCAN_RANGE_MM` or with zero depth are ignored.

    Args:
        depth_frame (Frame): The Kinect depth frame object.
        params: The Kinect's infrared camera intrinsic parameters.

    Returns:
        list[tuple[float, float, float]]: A list of 3D points, where each tuple
                                          represents (x, y, z) coordinates in millimeters
                                          relative to the Kinect sensor.
    """
    points = []
    depth_data = depth_frame.asarray()
    
    # Iterate over the depth image with a step to reduce point density.
    for v in range(0, depth_frame.height, PIXEL_STEP):
        for u in range(0, depth_frame.width, PIXEL_STEP):
            depth_mm = depth_data[v, u]
            
            # Ignore invalid depth readings (0) or points beyond the scan range.
            if depth_mm == 0 or depth_mm > SCAN_RANGE_MM:
                continue

            # Convert depth from millimeters to meters for intrinsic parameter calculations.
            z_cam = depth_mm / 1000.0
            # Calculate X and Y coordinates in camera space using intrinsic parameters.
            x_cam = (u - params.cx) * z_cam / params.fx
            y_cam = (v - params.cy) * z_cam / params.fy
            
            # Store coordinates in millimeters.
            points.append((x_cam * 1000.0, y_cam * 1000.0, z_cam * 1000.0))
            
    return points

def transform_points(point_slice: list[tuple[float, float, float]], heading_deg: float) -> list[tuple[float, float, float]]:
    """
    Rotates a list of sensor-relative 3D (x, y, z) points to world-relative 3D points
    based on a given heading (yaw angle).

    The rotation is applied around the sensor's vertical (y) axis, transforming
    the horizontal (x and z) components of each point.

    Args:
        point_slice (list[tuple[float, float, float]]): A list of 3D points
                                                         relative to the sensor.
        heading_deg (float): The current yaw angle of the sensor in degrees
                             relative to the world's coordinate system.

    Returns:
        list[tuple[float, float, float]]: A list of 3D points in world-relative
                                          (x, y, z) coordinates.
    """
    heading_rad = math.radians(heading_deg)
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    world_points = []
    
    # The sensor's (x, z) plane is typically the floor plane, and y is height.
    # We rotate around the sensor's y-axis (vertical axis).
    for sensor_x, sensor_y, sensor_z in point_slice:
        # Apply 2D rotation to the horizontal components (x and z).
        world_x = (sensor_x * cos_h) - (sensor_z * sin_h)
        world_z = (sensor_x * sin_h) + (sensor_z * cos_h)
        # The height (y) component remains unchanged by this horizontal rotation.
        world_y = sensor_y
        world_points.append((world_x, world_y, world_z))
    return world_points

def perform_360_capture() -> list[tuple[float, float, float]]:
    """
    Rotates the wheelchair 360 degrees, continuously capturing Kinect depth frames
    and IMU gyroscope readings to build a full 3D point cloud of the environment.

    The function manages the rotation, frame acquisition, 3D conversion, and
    transformation of points into a global coordinate system.

    Returns:
        list[tuple[float, float, float]]: The complete 3D point cloud,
                                          consisting of world-relative (x, y, z) points.
    """
    global listener, ir_params
    print("\n--- Starting 360° 3D Point Cloud Capture ---")
    total_angle_turned = 0.0    # Accumulator for the total angle turned.
    frames = FrameMap()         # FrameMap to hold incoming Kinect frames.
    global_point_cloud = []     # List to store all collected 3D points.

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

            # Convert the current depth frame to sensor-relative 3D points.
            sensor_points = convert_frame_to_3d_points(depth_frame, ir_params)
            # Transform these points to world-relative coordinates using the current heading.
            world_points = transform_points(sensor_points, total_angle_turned)
            global_point_cloud.extend(world_points) # Add to the global point cloud.

            print(f"Scanning... Angle: {total_angle_turned:.1f}° / 360°, Points this frame: {len(world_points): >4}", end='\r')
            
            listener.release(frames) # Release the frames immediately after processing.

    except Exception as e:
        print(f"\nAn error occurred during capture: {e}")
    finally:
        print("\nCapture rotation complete. Stopping motors.")
        wc.stop() # Ensure motors are stopped after capture.
        return global_point_cloud

def save_point_cloud_to_csv(point_cloud_data: list[tuple[float, float, float]]):
    """
    Saves the collected 3D point cloud data to a timestamped CSV file.

    The CSV file will contain columns for world_x_mm, world_y_mm, and world_z_mm.

    Args:
        point_cloud_data (list[tuple[float, float, float]]): The list of 3D points
                                                              to be saved.
    """
    if not point_cloud_data:
        print("No point cloud data to save.")
        return
    
    # Generate a unique filename with a timestamp.
    filename = f"{RAW_DATA_FILENAME_PREFIX}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    try:
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["world_x_mm", "world_y_mm", "world_z_mm"]) # Write header.
            for row in point_cloud_data:
                # Write each point with 0 decimal places for clarity.
                writer.writerow([f"{row[0]:.0f}", f"{row[1]:.0f}", f"{row[2]:.0f}"])
        print(f"\n--- 3D Point cloud map saved successfully to {filename} ---")
    except Exception as e:
        print(f"\n--- Error saving CSV file: {e} ---")

def main():
    """
    Main program flow for the 3D point cloud capture.

    This function initializes all required hardware (DAC, IMU, Kinect),
    prompts the user to start the scan, performs the 360-degree capture,
    saves the resulting point cloud, and ensures proper shutdown of all modules.
    """
    print("--- 3D Point Cloud Capture ---")
    
    # Initialize all critical hardware modules. Exit if any fail.
    if not wc.initialize_dac() or not imu.initialize_imu() or not initialize_kinect():
        print("FATAL: A required hardware module failed to initialize. Exiting.")
        shutdown_kinect() # Ensure Kinect is shut down even on partial failure.
        sys.exit(1)
        
    print("\nAll modules initialized.")
    
    print(f"Output CSV files will be prefixed with '{RAW_DATA_FILENAME_PREFIX}'")
    input("Press Enter to begin the 360-degree data capture...")

    try:
        # Perform the 360-degree scan and retrieve the collected point cloud data.
        point_cloud_data = perform_360_capture()
        
        print("\n--- 3D point cloud capture complete ---")
        
        # Save the captured data to a CSV file.
        save_point_cloud_to_csv(point_cloud_data)

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

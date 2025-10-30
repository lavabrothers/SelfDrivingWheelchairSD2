# File: point_cloud_mapper.py
"""
A new controller script that uses `mapping_sensor.py` to build a
true 2D point-cloud map of the environment.

1.  Points the chair to North (0 degrees).
2.  Rotates 360 degrees, capturing 2D "slices" from the Kinect.
3.  Transforms each slice into "world" coordinates using the IMU heading.
4.  Saves the resulting global point-cloud to a CSV file.
"""

import time
import sys
import math
import csv
import datetime
import numpy as np
import wheelchair_control as wc
import kinectslice as ms  # <-- Using the new mapping_sensor
import mpu as imu

# --- Configuration ---
SCAN_SPEED = 0.2         # Speed to rotate (0.1 = 10%, 0.5 = 50%)
POINT_SPEED = 0.25       # Speed to rotate when pointing at the target
HEADING_TOLERANCE = 3.0  # How close to the target heading to be (in degrees)
SCAN_START_BUFFER = 1.0  # Seconds to wait for rotation to start before scanning
STABILIZE_WAIT = 2.0     # Seconds to wait after pointing North

def get_current_heading():
    """A simple wrapper to get the IMU heading."""
    return imu.get_heading()

def point_at_target(target_heading):
    """
    Rotates the chair to face the target heading.
    (This function is identical to the one in mapping_controller.py)
    """
    print(f"--- Pointing at Target Heading: {target_heading:.1f} ---")
    current_heading = get_current_heading()
    
    while abs(current_heading - target_heading) > HEADING_TOLERANCE:
        angle_diff = target_heading - current_heading
        if angle_diff > 180: angle_diff -= 360
        elif angle_diff < -180: angle_diff += 360
            
        if angle_diff > 0: wc.set_rotation(POINT_SPEED)
        else: wc.set_rotation(-POINT_SPEED)
            
        print(f"Pointing... Current: {current_heading: >6.1f}, Target: {target_heading: >6.1f}", end="\r")
        time.sleep(0.05)
        current_heading = get_current_heading()
        
    wc.stop()
    print("\n--- Pointing Complete: Target Acquired ---")

def transform_points(point_slice, heading_deg):
    """
    Rotates a list of sensor-relative (x,y) points to
    world-relative (x,y) points based on the IMU heading.
    """
    heading_rad = math.radians(heading_deg)
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    
    world_points = []
    
    for sensor_x, sensor_y in point_slice:
        # Standard 2D rotation matrix:
        # world_x = (sensor_x * cos(h)) - (sensor_y * sin(h))
        # world_y = (sensor_x * sin(h)) + (sensor_y * cos(h))
        
        # Note: Our sensor Y is "forward" and sensor X is "right".
        # Our world Y is "North" and world X is "East".
        # This coordinate system mapping is correct.
        
        world_x = (sensor_x * cos_h) - (sensor_y * sin_h)
        world_y = (sensor_x * sin_h) + (sensor_y * cos_h)
        
        world_points.append((world_x, world_y))
        
    return world_points

def scan_and_build_map():
    """
    Rotates the chair 360 degrees and builds a global point cloud.
    """
    print("--- Starting Point Cloud Scan (from North) ---")
    global_point_cloud = [] # This will store all (world_x, world_y) points
    
    start_heading = get_current_heading()
    print(f"Verifying start heading: {start_heading:.1f} degrees.")
    
    wc.set_rotation(SCAN_SPEED)
    time.sleep(SCAN_START_BUFFER)
    
    last_heading = start_heading
    has_left_north = False
    
    print("Scanning... (Will stop when North is reached again)")

    while True:
        current_heading = get_current_heading()
        
        # Get the 2D "slice" from the new sensor module
        scan_slice = ms.get_scan_slice()
        
        # --- Check for stop condition ---
        if not has_left_north:
            angle_diff = current_heading - start_heading
            if angle_diff < -180: angle_diff += 360
            if angle_diff > (HEADING_TOLERANCE * 5):
                has_left_north = True
                print("\n[Scan: Left North, now scanning...]")

        if has_left_north:
            angle_diff_to_north = current_heading - start_heading
            if angle_diff_to_north > 180: angle_diff_to_north -= 360
            if angle_diff_to_north < -180: angle_diff_to_north += 360
            
            if abs(angle_diff_to_north) < HEADING_TOLERANCE:
                print("\n[Scan: Returned to North.]")
                break # Stop the loop
        
        # --- Process Data ---
        if scan_slice and abs(current_heading - last_heading) > 0.1:
            
            # Transform sensor points to world points
            world_points = transform_points(scan_slice, current_heading)
            
            # Add them to our global map
            global_point_cloud.extend(world_points)
            
            last_heading = current_heading
            print(f"Scan... Heading: {current_heading: >6.1f}, Points this frame: {len(scan_slice): >4}", end="\r")
        
        time.sleep(0.02)

    wc.stop()
    print(f"\n--- Scan Complete: {len(global_point_cloud)} total points mapped ---")
    return global_point_cloud

def save_point_cloud_to_csv(point_cloud_data):
    """
    Saves the collected (world_x, world_y) points to a CSV file.
    """
    if not point_cloud_data:
        print("No point cloud data to save.")
        return
        
    filename = f"point_cloud_map_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    try:
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["world_x_mm", "world_y_mm"]) # Header
            
            for row in point_cloud_data:
                writer.writerow([f"{row[0]:.0f}", f"{row[1]:.0f}"])
                
        print(f"\n--- Point cloud map saved successfully to {filename} ---")
        
    except Exception as e:
        print(f"\n--- Error saving CSV file: {e} ---")


def main():
    print("--- Point Cloud Mapper (Scan-to-CSV) ---")
    
    # --- 1. Initialize All Modules ---
    print("Initializing Wheelchair Controller...")
    if not wc.initialize_dac():
        print("FATAL: Could not initialize Wheelchair DAC. Exiting.")
        sys.exit(1)
    
    print("Initializing Mapping Sensor...")
    if not ms.initialize_kinect():
        print("FATAL: Could not initialize Kinect. Exiting.")
        sys.exit(1)
        
    print("Initializing IMU...")
    if not imu.initialize_imu():
        print("FATAL: Could not initialize IMU. Exiting.")
        sys.exit(1)
        
    print("\nAll modules initialized.")
    print("Press Ctrl+C to stop the program at any time.")

    try:
        # --- 2. Point North ---
        print(f"\nStep 1: Pointing chair to North (0.0 degrees)...")
        point_at_target(0.0)
        print(f"Waiting {STABILIZE_WAIT} seconds to stabilize...")
        time.sleep(STABILIZE_WAIT)
        
        # --- 3. Scan Phase ---
        print("\nStep 2: Starting 360-degree scan...")
        point_cloud = scan_and_build_map()
        
        # --- 4. Save Phase ---
        print("\nStep 3: Saving point cloud to CSV...")
        save_point_cloud_to_csv(point_cloud)
        
        print("\n--- Program Complete ---")

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Stopping program.")
    
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")

    finally:
        # --- 5. Safety Shutdown ---
        print("Ensuring wheelchair is stopped and shutting down Kinect.")
        wc.stop()
        ms.shutdown_kinect()
        print("Shutdown complete. Exiting.")

if __name__ == "__main__":
    # --- !! SAFETY WARNING !! ---
    print("WARNING: This script will move the wheelchair!")
    print("Ensure wheels are OFF THE GROUND for initial testing.")
    for i in range(5, 0, -1):
        print(f"Starting in {i}...", end="\r")
        time.sleep(1)
    print("Starting now.                ")
    
    main()

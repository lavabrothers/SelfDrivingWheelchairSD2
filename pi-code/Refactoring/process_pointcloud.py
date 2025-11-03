# File: process_pointcloud.py
"""
This script reads a raw_scan_data_*.csv file, which contains rotational angles and
corresponding vertical depth slices, and converts this data into a 3D point cloud.

This is modeled after the original 'processmap.py'.
"""

import csv
import numpy as np
import glob
import os
import math
import open3d as o3d

# --- Configuration ---
OUTPUT_PCD_FILENAME = "point_cloud_3d.pcd"
SCAN_RANGE_METERS = 8.0
VERTICAL_PIXEL_STEP = 10  # Process every 10th pixel to keep point cloud manageable

# --- Kinect V2 Depth Camera Intrinsics (from processmap.py) ---
# These are typical values, but may not be perfectly calibrated for every device.
FX_DEPTH = 365.481
FY_DEPTH = 365.481
CX_DEPTH = 257.346  # Horizontal center of the sensor (image width is 512)
CY_DEPTH = 210.347  # Vertical center of the sensor (image height is 424)

# --- Path Configuration ---
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
RAW_DATA_DIR = os.path.join(SCRIPT_DIR, "..") # CSV files are in the parent 'pi-code' directory

def find_latest_raw_scan_file(directory):
    """Finds the most recent raw_scan_data_*.csv file."""
    search_pattern = os.path.join(directory, 'raw_scan_data_*.csv')
    files = glob.glob(search_pattern)
    if not files:
        return None
    return max(files, key=os.path.getctime)

def save_points_to_pcd(points, filename):
    """Saves a list of 3D points to a .pcd file."""
    if not points:
        print("No points to save.")
        return
    print(f"\nSaving {len(points)} points to '{filename}'...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
    o3d.io.write_point_cloud(filename, pcd)
    print("✅ Point cloud successfully saved.")

def process_raw_data(filename):
    """
    Reads the raw data file line by line and converts it to a 3D point cloud.
    """
    print(f"Reading raw data from '{filename}'...")
    world_points = []
    
    try:
        with open(filename, 'r') as f:
            csv_reader = csv.reader(f)
            for i, row in enumerate(csv_reader):
                if not row: continue # Skip empty rows
                
                current_angle_deg = float(row[0])
                current_angle_rad = math.radians(current_angle_deg)
                depth_slice_str = row[1:]
                
                # Process the vertical slice of depth data
                for v_raw, depth_str in enumerate(depth_slice_str[::VERTICAL_PIXEL_STEP]):
                    depth_mm = float(depth_str)
                    
                    if depth_mm == 0 or depth_mm > (SCAN_RANGE_METERS * 1000):
                        continue

                    v = v_raw * VERTICAL_PIXEL_STEP
                    # The slice was taken from the horizontal center of the sensor
                    center_u = 256 

                    # Convert depth to camera-space coordinates (in meters)
                    z_cam = depth_mm / 1000.0
                    x_cam = (center_u - CX_DEPTH) * z_cam / FX_DEPTH
                    y_cam = (v - CY_DEPTH) * z_cam / FY_DEPTH

                    # Rotate the camera-space point into world-space
                    world_x = z_cam * math.sin(current_angle_rad) + x_cam * math.cos(current_angle_rad)
                    world_y = y_cam # Height is not affected by rotation around vertical axis
                    world_z = z_cam * math.cos(current_angle_rad) - x_cam * math.sin(current_angle_rad)
                    
                    world_points.append([world_x, world_y, world_z])
                
                print(f"Processing line {i+1}...", end='\r')

    except FileNotFoundError:
        print(f"Error: File not found at '{filename}'")
        return []
    except Exception as e:
        print(f"\nAn error occurred during processing: {e}")
        return []

    return world_points

if __name__ == "__main__":
    print("--- Raw Scan Data Processor ---")
    latest_csv = find_latest_raw_scan_file(RAW_DATA_DIR)
    
    if latest_csv:
        collected_points_3d = process_raw_data(latest_csv)
        
        if collected_points_3d:
            output_path = os.path.join(SCRIPT_DIR, OUTPUT_PCD_FILENAME)
            save_points_to_pcd(collected_points_3d, output_path)
    else:
        print(f"No raw_scan_data_*.csv file found in '{RAW_DATA_DIR}'.")
        print("Please run 'PointcloudMapper.py' first to generate data.")

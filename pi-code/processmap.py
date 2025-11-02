# process_scan.py
# This script reads the raw_scan_data.csv file and converts it into a 3D point cloud.

import math
import csv
import numpy as np
import open3d as o3d

# --- Configuration ---
RAW_DATA_FILENAME = "raw_scan_data_20251101_213006.csv"
OUTPUT_PCD_FILENAME = "point_cloud.pcd"
SCAN_RANGE_FEET = 15.0
VERTICAL_PIXEL_STEP = 10 # Process every 10th pixel to keep point cloud manageable

# --- Kinect V2 Depth Camera Intrinsics ---
FX_DEPTH = 365.481
FY_DEPTH = 365.481
CX_DEPTH = 257.346 # Horizontal center of the sensor
CY_DEPTH = 210.347 # Vertical center of the sensor

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

def process_raw_data():
    """
    Reads the raw data file line by line and converts it to a 3D point cloud.
    """
    print(f"Reading raw data from '{RAW_DATA_FILENAME}'...")
    world_points = []
    
    with open(RAW_DATA_FILENAME, 'r') as f:
        csv_reader = csv.reader(f)
        for i, row in enumerate(csv_reader):
            # Parse the row
            current_angle_deg = float(row[0])
            current_angle_rad = math.radians(current_angle_deg)
            depth_slice_str = row[1:]
            
            # This is the 3D conversion logic, now applied to data from the file
            for v_raw, depth_str in enumerate(depth_slice_str[::VERTICAL_PIXEL_STEP]):
                depth_mm = float(depth_str)
                
                if depth_mm == 0 or depth_mm > (SCAN_RANGE_FEET * 304.8):
                    continue

                v = v_raw * VERTICAL_PIXEL_STEP
                center_u = 256 # Depth image width is 512, so center is ~256

                Z_cam = depth_mm / 1000.0
                X_cam = (center_u - CX_DEPTH) * Z_cam / FX_DEPTH
                Y_cam = (v - CY_DEPTH) * Z_cam / FY_DEPTH

                X_world = Z_cam * math.sin(current_angle_rad) + X_cam * math.cos(current_angle_rad)
                Y_world = Y_cam
                Z_world = Z_cam * math.cos(current_angle_rad) - X_cam * math.sin(current_angle_rad)
                
                world_points.append([X_world, Y_world, Z_world])
            
            print(f"Processing line {i+1}...", end='\r')

    return world_points

if __name__ == "__main__":
    collected_points_3d = process_raw_data()
    save_points_to_pcd(collected_points_3d, OUTPUT_PCD_FILENAME)
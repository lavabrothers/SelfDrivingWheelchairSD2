import csv
import numpy as np
from PIL import Image
import glob
import os
import open3d as o3d

# --- Configuration ---
MAP_RESOLUTION_MM_PER_PIXEL = 50  # Each pixel represents a 5cm x 5cm area
OUTPUT_IMAGE_FILENAME = "map.png"
OUTPUT_PCD_FILENAME = "point_cloud_3d.pcd"
# --- Path Configuration ---
# The script is in 'pi-code/Refactoring', and the CSV files are in 'pi-code'.
# We construct a path that goes one level up from the script's directory.
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
POINT_CLOUD_DIR = os.path.join(SCRIPT_DIR, "..")


def find_latest_point_cloud_file(directory):
    """Finds the most recent point_cloud_map_*.csv file in the specified directory."""
    search_pattern = os.path.join(directory, 'point_cloud_map_*.csv')
    files = glob.glob(search_pattern)
    if not files:
        return None
    latest_file = max(files, key=os.path.getctime)
    return latest_file

def save_points_to_pcd(points, filename):
    """Saves a list of 3D points to a .pcd file."""
    if not points:
        print("No points to save to PCD.")
        return
    
    # Convert points from mm to meters for standard PCD format
    points_in_meters = np.array(points) / 1000.0
    
    print(f"\nSaving {len(points_in_meters)} points to '{filename}'...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_in_meters)
    o3d.io.write_point_cloud(filename, pcd)
    print(f"✅ Point cloud successfully saved to '{filename}'.")


def process_point_cloud(filename):
    """
    Reads a point cloud CSV, converts it to a 2D grid map (PNG), 
    and saves the raw points as a PCD file.
    """
    print(f"Reading point cloud data from '{filename}'...")
    points = []
    try:
        with open(filename, 'r') as f:
            csv_reader = csv.DictReader(f)
            for row in csv_reader:
                try:
                    # Now reading x, y (height), and z coordinates
                    points.append((
                        float(row['world_x_mm']), 
                        float(row['world_y_mm']), 
                        float(row['world_z_mm'])
                    ))
                except (ValueError, KeyError) as e:
                    print(f"Skipping row due to error: {e} - Row: {row}")
                    continue
    except FileNotFoundError:
        print(f"Error: File not found at '{filename}'")
        return

    if not points:
        print("No valid points found in the file.")
        return

    # --- PNG Map Generation (Top-Down View) ---
    # We use x and z for the 2D map projection. y is height.
    min_x = min(p[0] for p in points)
    max_x = max(p[0] for p in points)
    min_z = min(p[2] for p in points)
    max_z = max(p[2] for p in points)

    map_width_mm = max_x - min_x
    map_height_mm = max_z - min_z
    map_width_pixels = int(map_width_mm / MAP_RESOLUTION_MM_PER_PIXEL) + 1
    map_height_pixels = int(map_height_mm / MAP_RESOLUTION_MM_PER_PIXEL) + 1

    print(f"Map dimensions: {map_width_pixels}px x {map_height_pixels}px")

    grid_map = np.zeros((map_height_pixels, map_width_pixels), dtype=np.uint8)

    # Map each point to a pixel on the grid based on its x and z coordinates
    for x_mm, y_mm, z_mm in points:
        translated_x = x_mm - min_x
        translated_z = z_mm - min_z # Use z for the vertical axis of the 2D map

        pixel_x = int(translated_x / MAP_RESOLUTION_MM_PER_PIXEL)
        # Invert z-axis for correct image orientation (0,0 is top-left)
        pixel_y = (map_height_pixels - 1) - int(translated_z / MAP_RESOLUTION_MM_PER_PIXEL)

        if 0 <= pixel_x < map_width_pixels and 0 <= pixel_y < map_height_pixels:
            grid_map[pixel_y, pixel_x] = 255

    img = Image.fromarray(grid_map, 'L')
    
    # Define output paths to be in the same directory as the script.
    script_dir = os.path.dirname(os.path.realpath(__file__))
    png_output_path = os.path.join(script_dir, OUTPUT_IMAGE_FILENAME)
    pcd_output_path = os.path.join(script_dir, OUTPUT_PCD_FILENAME)

    img.save(png_output_path)
    print(f"✅ Map successfully saved to '{png_output_path}'")

    # --- PCD File Generation ---
    save_points_to_pcd(points, pcd_output_path)


if __name__ == "__main__":
    latest_csv = find_latest_point_cloud_file(POINT_CLOUD_DIR)
    if latest_csv:
        process_point_cloud(latest_csv)
    else:
        print(f"No point_cloud_map_*.csv file found in '{POINT_CLOUD_DIR}'.")

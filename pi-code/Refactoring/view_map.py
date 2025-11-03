# File: view_map.py
"""
This script loads and displays a 3D point cloud from a .pcd file.

It uses Matplotlib for visualization to ensure compatibility with systems 
that may lack full graphical desktop environments (like a Raspberry Pi).
"""

import open3d as o3d
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Path Configuration ---
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
PCD_FILENAME = os.path.join(SCRIPT_DIR, "point_cloud_3d.pcd")
MAX_POINTS_TO_DISPLAY = 30000  # Limit points to prevent performance issues

def view_pcd_with_matplotlib(filename):
    """
    Loads a 3D PCD file and displays it using a Matplotlib 3D scatter plot.
    """
    if not os.path.exists(filename):
        print(f"Error: PCD file not found at '{filename}'")
        print("Please run 'process_pointcloud.py' first to generate the 3D PCD file.")
        return

    try:
        print(f"Loading point cloud from '{filename}'...")
        pcd = o3d.io.read_point_cloud(filename)
        if not pcd.has_points():
            print("Error: The PCD file is empty.")
            return

        points = np.asarray(pcd.points)
        print(f"Loaded {len(points)} points.")

        # Sub-sample points if the cloud is too large for Matplotlib
        if len(points) > MAX_POINTS_TO_DISPLAY:
            print(f"Sub-sampling to {MAX_POINTS_TO_DISPLAY} points for display...")
            indices = np.random.choice(points.shape[0], MAX_POINTS_TO_DISPLAY, replace=False)
            points_to_display = points[indices]
        else:
            points_to_display = points

        print("Preparing 3D plot...")
        fig = plt.figure(figsize=(12, 12))
        ax = fig.add_subplot(111, projection='3d')

        x = points_to_display[:, 0]
        y = points_to_display[:, 1]
        z = points_to_display[:, 2]

        # Scatter plot. Color points by height (y-coordinate)
        # We plot (x, z, y) to get a more natural top-down perspective.
        ax.scatter(x, z, y, s=1, c=y, cmap='viridis_r')

        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Z (meters)")
        ax.set_zlabel("Y / Height (meters)")
        ax.set_title("3D Point Cloud Map")
        
        # Set aspect ratio to be equal for a more accurate representation
        max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max()
        mid_x = (x.max()+x.min()) * 0.5
        mid_y = (y.max()+y.min()) * 0.5
        mid_z = (z.max()+z.min()) * 0.5
        ax.set_xlim(mid_x - max_range * 0.5, mid_x + max_range * 0.5)
        ax.set_ylim(mid_z - max_range * 0.5, mid_z + max_range * 0.5)
        ax.set_zlim(mid_y - max_range * 0.5, mid_y + max_range * 0.5)

        print("Displaying plot. Close the plot window to exit.")
        plt.show()

    except Exception as e:
        print(f"An error occurred while trying to display the PCD file: {e}")

if __name__ == "__main__":
    view_pcd_with_matplotlib(PCD_FILENAME)

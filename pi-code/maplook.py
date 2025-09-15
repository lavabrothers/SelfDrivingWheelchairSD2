# view_map.py
# This script loads and displays your 3D point cloud map correctly.

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Configuration ---
PCD_FILENAME = "point_cloud.pcd"
MAX_POINTS_TO_DISPLAY = 20000

def view_pcd_with_matplotlib(filename):
    print(f"Loading point cloud from '{filename}'...")
    try:
        pcd = o3d.io.read_point_cloud(filename)
        if not pcd.has_points():
            print("Error: The PCD file is empty.")
            return
    except Exception as e:
        print(f"Error loading PCD file: {e}")
        return

    points = np.asarray(pcd.points)
    print(f"Loaded {len(points)} points.")

    if len(points) > MAX_POINTS_TO_DISPLAY:
        print(f"Sub-sampling to {MAX_POINTS_TO_DISPLAY} points for display...")
        indices = np.random.choice(points.shape[0], MAX_POINTS_TO_DISPLAY, replace=False)
        points_to_display = points[indices]
    else:
        points_to_display = points

    print("Preparing 3D plot...")
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    x = points_to_display[:, 0]
    y = points_to_display[:, 1]
    z = points_to_display[:, 2]

    # --- THE FIX for the upside-down map ---
    # We plot -y to invert the vertical axis, making it appear upright.
    # We also map z to the y-axis and -y to the z-axis for a more natural top-down initial view.
    ax.scatter(x, z, -y, s=1, c=-y, cmap='terrain')

    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Z (meters)")
    ax.set_zlabel("Y / Height (meters)")
    ax.set_title("3D Point Cloud Map")
    ax.set_aspect('auto')

    print("Displaying plot. Close this window when you have chosen your coordinates.")
    plt.show()

if __name__ == "__main__":
    view_pcd_with_matplotlib(PCD_FILENAME)
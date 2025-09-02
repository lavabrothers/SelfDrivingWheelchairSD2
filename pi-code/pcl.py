import open3d as o3d
import numpy as np

# --- Configuration ---
# The path to your Point Cloud Data file
file_name = 'output.pcd'
# The point from which to measure the distance (x, y, z)
origin_point = np.array([0, 0, 0])


# --- Main Program ---
try:
    # 1. Read the .pcd file
    pcd = o3d.io.read_point_cloud(file_name)
    if not pcd.has_points():
        print(f"Error: The file {file_name} is empty or could not be read.")
    else:
        # 2. Convert the points to a NumPy array
        points = np.asarray(pcd.points)
        print(f"Successfully loaded {len(points)} total points from {file_name}.")

        # 3. Filter out non-finite points (nan, inf)
        finite_mask = np.isfinite(points).all(axis=1)
        points = points[finite_mask]
        
        if len(points) == 0:
            print("Error: After filtering, no valid (finite) points remain.")
        else:
            print(f"Processing {len(points)} valid points after filtering.")

            # 4. Calculate the distance of each valid point from the origin
            distances = np.linalg.norm(points - origin_point, axis=1)

            # 5. Find the index of the point with the minimum distance
            min_index = np.argmin(distances)

            # 6. Get the coordinates and distance of the closest point
            closest_point = points[min_index]
            min_distance = distances[min_index]

            # 7. Print the results
            print("-" * 30)
            print(f"Closest point to {origin_point} is: {closest_point}")
            print(f"Distance: {min_distance:.4f} meters")

except FileNotFoundError:
    print(f"Error: The file '{file_name}' was not found.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
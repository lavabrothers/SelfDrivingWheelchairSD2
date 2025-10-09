# navigate.py
# This script takes user commands to navigate the wheelchair to specific coordinates
# and displays the robot's pose on the 3D map.

import time
import board
import math
import numpy as np
import threading
import open3d as o3d

# --- Adafruit Libraries ---
import adafruit_mpu6050
import adafruit_mcp4728

# --- Configuration ---
METERS_TO_FEET = 3.28084
MOVE_SPEED_OFFSET = 0.15
TURN_SPEED_OFFSET = 0.25
RAMP_STEP = 0.05
RAMP_DELAY = 0.05
POINT_CLOUD_FILENAME = "point_cloud.pcd"

# --- Robot's Current State (Pose) ---
# This dictionary is now thread-safe using a lock
robot_pose = { 'x': 0.0, 'z': 0.0, 'angle_rad': 0.0 }
pose_lock = threading.Lock()

# --- Visualization Globals ---
vis = None
robot_marker = None
shutdown_flag = threading.Event()

# --- Hardware Setup ---
try:
    print("Initializing I2C and sensors...")
    i2c = board.I2C()
    mpu = adafruit_mpu6050.MPU6050(i2c)
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("Hardware OK. ✅")
except Exception as e:
    print(f"Error: Could not find hardware: {e}")
    exit()

# --- NEW: Visualization Functions ---
def create_robot_marker():
    """Creates an Open3D TriangleMesh object to represent the robot's pose."""
    # Creates a coordinate frame: X-Red (forward), Y-Green (up), Z-Blue (left)
    marker = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    return marker

def visualization_thread_func(pcd):
    """
    The target function for the visualization thread.
    This function creates and manages the Open3D window.
    """
    global vis, robot_marker

    # Initialize the visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Robot Navigation View")

    # Add the map (point cloud)
    vis.add_geometry(pcd)

    # Create and add the robot marker
    robot_marker = create_robot_marker()
    vis.add_geometry(robot_marker)

    # Set a nice initial viewpoint
    view_control = vis.get_view_control()
    view_control.set_lookat([0, 0, 0])
    view_control.set_up([0, 1, 0]) # Y-axis is "up"
    view_control.set_zoom(0.5)

    print("Visualization window opened. Close the window to exit the program.")

    while not shutdown_flag.is_set():
        with pose_lock:
            # Get the latest robot pose
            x, z, angle_rad = robot_pose['x'], robot_pose['z'], robot_pose['angle_rad']

        # Create rotation matrix around the Y axis (up)
        # In our coordinate system, Z is forward and X is right. The MPU gyro's z-axis
        # corresponds to rotation around the world's Y (up) axis.
        R = o3d.geometry.get_rotation_matrix_from_xyz((0, -angle_rad, 0))
        
        # Create a combined transformation matrix (rotation + translation)
        transform = np.eye(4)
        transform[:3, :3] = R
        transform[:3, 3] = [x, 0, z] # Translate the marker

        # Apply the transformation
        robot_marker.transform(np.linalg.inv(robot_marker.get_transformation())) # Reset transform
        robot_marker.transform(transform)

        vis.update_geometry(robot_marker)

        # poll_events returns False if the window is closed
        if not vis.poll_events():
            shutdown_flag.set() # Signal main thread to exit
            break
        vis.update_renderer()
        time.sleep(0.03) # ~30 FPS updates

    vis.destroy_window()
    print("Visualization window closed.")


# --- Core Wheelchair Movement Functions (Unchanged) ---
current_fwd_bwd = 0.0
current_left_right = 0.0
def set_movement(fwd_bwd, left_right):
    global current_fwd_bwd, current_left_right
    fwd_bwd=max(-1,min(1,fwd_bwd)); left_right=max(-1,min(1,left_right))
    mcp.channel_a.normalized_value=0.5+(fwd_bwd/2.0); mcp.channel_b.normalized_value=0.5-(fwd_bwd/2.0)
    mcp.channel_c.normalized_value=0.5+(left_right/2.0); mcp.channel_d.normalized_value=0.5-(left_right/2.0)
    current_fwd_bwd=fwd_bwd; current_left_right=left_right

def ramp_to_speed(target_fwd_bwd, target_left_right):
    while abs(target_fwd_bwd - current_fwd_bwd) > 0.01 or abs(target_left_right - current_left_right) > 0.01:
        if shutdown_flag.is_set(): return # Exit early if window is closed
        new_fwd_bwd=current_fwd_bwd; new_left_right=current_left_right
        if target_fwd_bwd > current_fwd_bwd: new_fwd_bwd += RAMP_STEP
        elif target_fwd_bwd < current_fwd_bwd: new_fwd_bwd -= RAMP_STEP
        if target_left_right > current_left_right: new_left_right += RAMP_STEP
        elif target_left_right < current_left_right: new_left_right -= RAMP_STEP
        set_movement(new_fwd_bwd, new_left_right)
        time.sleep(RAMP_DELAY)
    set_movement(target_fwd_bwd, target_left_right)

def stop_all_movement():
    ramp_to_speed(0.0, 0.0)

def execute_turn(direction, target_angle_deg):
    print(f"Executing turn: {direction} {target_angle_deg:.1f}°...")
    turn_value = TURN_SPEED_OFFSET * 2
    if direction == 'left': turn_value *= -1
    ramp_to_speed(0.0, turn_value)
    total_angle_turned = 0.0; last_time = time.monotonic()
    while total_angle_turned < target_angle_deg:
        if shutdown_flag.is_set(): break # Exit early
        current_time = time.monotonic(); time_delta = current_time-last_time; last_time = current_time
        total_angle_turned += abs(math.degrees(mpu.gyro[2] * time_delta))
        time.sleep(0.01)
    stop_all_movement()

def execute_move(target_distance_ft):
    print(f"Executing move: forward {target_distance_ft:.2f} ft...")
    # NOTE: Replace this with your more accurate IMU-based move function if you have it.
    move_duration = target_distance_ft / 1.5  # DUMMY VALUE: Calibrate this! (feet per second)
    ramp_to_speed(MOVE_SPEED_OFFSET * 2, 0.0)
    
    start_time = time.monotonic()
    while time.monotonic() - start_time < move_duration:
        if shutdown_flag.is_set(): break # Exit early
        time.sleep(0.05)
        
    stop_all_movement()

# --- Navigation Logic ---
def navigate_to_target(target_x, target_z):
    global robot_pose
    
    with pose_lock:
        current_x = robot_pose['x']
        current_z = robot_pose['z']
        current_angle = robot_pose['angle_rad']

    delta_x = target_x - current_x; delta_z = target_z - current_z
    distance_m = math.sqrt(delta_x**2 + delta_z**2); distance_ft = distance_m * METERS_TO_FEET
    
    # atan2 uses (y, x), which in our 2D ground plane is (z, x)
    target_angle_rad = math.atan2(delta_z, delta_x)
    
    turn_angle_rad = target_angle_rad - current_angle
    # Normalize the angle to the range [-pi, pi]
    while turn_angle_rad > math.pi: turn_angle_rad -= 2 * math.pi
    while turn_angle_rad < -math.pi: turn_angle_rad += 2 * math.pi
    
    turn_angle_deg = abs(math.degrees(turn_angle_rad))
    turn_direction = 'right' if turn_angle_rad < 0 else 'left'
    
    print("\n--- Path Calculation ---")
    print(f"Action: Turn {turn_direction} by {turn_angle_deg:.1f}°, then move forward {distance_ft:.2f} ft.")
    
    if turn_angle_deg > 5.0: execute_turn(turn_direction, turn_angle_deg)
    if distance_ft > 0.25: execute_move(distance_ft)
        
    with pose_lock:
        robot_pose['x'] = target_x
        robot_pose['z'] = target_z
        robot_pose['angle_rad'] = target_angle_rad
    print("Navigation complete. New pose updated.")


# --- Main Program Loop ---
if __name__ == "__main__":
    # --- Load the Map ---
    try:
        print(f"Loading map from '{POINT_CLOUD_FILENAME}'...")
        pcd = o3d.io.read_point_cloud(POINT_CLOUD_FILENAME)
        if not pcd.has_points():
            print(f"Error: Point cloud '{POINT_CLOUD_FILENAME}' is empty or could not be read.")
            exit()
        print("Map loaded successfully. ✅")
    except Exception as e:
        print(f"Error: Could not load '{POINT_CLOUD_FILENAME}'. Did you run process_scan.py first?")
        print(f"Details: {e}")
        exit()

    set_movement(0.0, 0.0)
    
    # --- Start Visualization in a separate thread ---
    vis_thread = threading.Thread(target=visualization_thread_func, args=(pcd,))
    vis_thread.start()
    
    time.sleep(2) # Give the visualizer a moment to initialize

    print("\n--- Manual Navigation Mode ---")
    print("1. A 3D map view has been opened.")
    print("2. Use commands below to navigate.")
    print("\nCommands:")
    print("  'goto [x] [z]' - e.g., 'goto 1.5 -2.0'")
    print("  'pose'         - Shows the current estimated pose")
    print("  'exit'         - Closes the program")
    print("  (Or just close the 3D window)")
    print("--------------------------------")
    
    try:
        while not shutdown_flag.is_set():
            command_str = input("Enter command > ").lower().strip()
            parts = command_str.split()
            if not parts: continue
            
            command = parts[0]
            if command == 'goto' and len(parts) == 3:
                try:
                    navigate_to_target(float(parts[1]), float(parts[2]))
                except ValueError: print("Error: Invalid coordinates.")
            elif command == 'pose':
                with pose_lock:
                    print(f"Current pose: X={robot_pose['x']:.2f}m, Z={robot_pose['z']:.2f}m, Angle={math.degrees(robot_pose['angle_rad']):.1f}°")
            elif command == 'exit':
                shutdown_flag.set()
                break
            else: print("Invalid command.")
    except (KeyboardInterrupt, EOFError):
        print("\nCtrl+C or EOF detected, shutting down.")
        shutdown_flag.set()
    finally:
        print("Stopping movement and closing threads...")
        stop_all_movement()
        vis_thread.join() # Wait for the visualization thread to finish
        print("Program terminated.")
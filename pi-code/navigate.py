# navigate.py
# This script takes user commands to navigate the wheelchair to specific coordinates.

import time
import board
import math
import numpy as np

# --- Adafruit Libraries ---
import adafruit_mpu6050
import adafruit_mcp4728

# --- Configuration ---
METERS_TO_FEET = 3.28084
MOVE_SPEED_OFFSET = 0.15 # Slightly increased for responsiveness
TURN_SPEED_OFFSET = 0.25
RAMP_STEP = 0.05
RAMP_DELAY = 0.05

# --- Robot's Current State (Pose) ---
robot_pose = { 'x': 0.0, 'z': 0.0, 'angle_rad': 0.0 }

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

# --- Core Wheelchair Movement Functions (Unchanged) ---
current_fwd_bwd = 0.0; current_left_right = 0.0
def set_movement(fwd_bwd, left_right):
    global current_fwd_bwd, current_left_right
    fwd_bwd=max(-1,min(1,fwd_bwd)); left_right=max(-1,min(1,left_right))
    mcp.channel_a.normalized_value=0.5+(fwd_bwd/2.0); mcp.channel_b.normalized_value=0.5-(fwd_bwd/2.0)
    mcp.channel_c.normalized_value=0.5+(left_right/2.0); mcp.channel_d.normalized_value=0.5-(left_right/2.0)
    current_fwd_bwd=fwd_bwd; current_left_right=left_right

def ramp_to_speed(target_fwd_bwd, target_left_right):
    while abs(target_fwd_bwd - current_fwd_bwd) > 0.01 or abs(target_left_right - current_left_right) > 0.01:
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
        current_time = time.monotonic(); time_delta = current_time-last_time; last_time = current_time
        total_angle_turned += abs(math.degrees(mpu.gyro[2] * time_delta))
        time.sleep(0.01)
    stop_all_movement()

def execute_move(target_distance_ft):
    print(f"Executing move: forward {target_distance_ft:.2f} ft...")
    # NOTE: Replace this with your more accurate IMU-based move function if you have it.
    move_duration = target_distance_ft / 1.5  # DUMMY VALUE: Calibrate this! (feet per second)
    ramp_to_speed(MOVE_SPEED_OFFSET * 2, 0.0)
    time.sleep(move_duration)
    stop_all_movement()

# --- Navigation Logic (Unchanged) ---
def navigate_to_target(target_x, target_z):
    global robot_pose
    delta_x = target_x - robot_pose['x']; delta_z = target_z - robot_pose['z']
    distance_m = math.sqrt(delta_x**2 + delta_z**2); distance_ft = distance_m * METERS_TO_FEET
    target_angle_rad = math.atan2(delta_z, delta_x)
    turn_angle_rad = target_angle_rad - robot_pose['angle_rad']
    while turn_angle_rad > math.pi: turn_angle_rad -= 2 * math.pi
    while turn_angle_rad < -math.pi: turn_angle_rad += 2 * math.pi
    turn_angle_deg = abs(math.degrees(turn_angle_rad))
    turn_direction = 'right' if turn_angle_rad < 0 else 'left'
    
    print("\n--- Path Calculation ---")
    print(f"Action: Turn {turn_direction} by {turn_angle_deg:.1f}°, then move forward {distance_ft:.2f} ft.")
    
    if turn_angle_deg > 5.0: execute_turn(turn_direction, turn_angle_deg)
    if distance_ft > 0.25: execute_move(distance_ft)
        
    robot_pose['x'] = target_x; robot_pose['z'] = target_z; robot_pose['angle_rad'] = target_angle_rad
    print("Navigation complete. New pose updated.")

# --- Main Program Loop ---
if __name__ == "__main__":
    set_movement(0.0, 0.0)
    print("\n--- Manual Navigation Mode ---")
    print("1. Run 'view_map.py' to see the map and choose coordinates.")
    print("2. Use commands below to navigate.")
    print("\nCommands:")
    print("  'goto [x] [z]' - e.g., 'goto 1.5 -2.0'")
    print("  'pose'         - Shows the current estimated pose")
    print("  'exit'         - Closes the program")
    print("--------------------------------")
    
    while True:
        command_str = input("Enter command > ").lower().strip()
        parts = command_str.split()
        if not parts: continue
        
        command = parts[0]
        if command == 'goto' and len(parts) == 3:
            try:
                navigate_to_target(float(parts[1]), float(parts[2]))
            except ValueError: print("Error: Invalid coordinates.")
        elif command == 'pose':
            print(f"Current pose: X={robot_pose['x']:.2f}m, Z={robot_pose['z']:.2f}m, Angle={math.degrees(robot_pose['angle_rad']):.1f}°")
        elif command == 'exit':
            stop_all_movement(); break
        else: print("Invalid command.")
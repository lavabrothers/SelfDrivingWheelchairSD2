# Import necessary libraries
import time
import board
import math
import numpy as np
import cv2
import asyncio  # For handling Bluetooth operations
import bleak      # The Bluetooth LE library

# --- Adafruit Libraries for Wheelchair Control ---
import adafruit_mpu6050
import adafruit_mcp4728

# --- Freenect2 Library for Kinect V2 ---
from freenect2 import Device, FrameType

# --- MODIFIED: Bluetooth Constants ---
SERVICE_UUID = "c7a72f3e-3151-4d39-a18e-4a73426b3e2b"
CHAR_RX_UUID = "f3711319-333e-41a4-b04b-32a7b8e1136c" # ESP32 receives on this
CHAR_TX_UUID = "d1aea128-4f7e-4c4f-a7b5-c603a111a00a" # ESP32 transmits on this

# --- NEW: Set your specific ESP32's MAC Address here ---
# To find your ESP32's address, check the Arduino IDE serial monitor when it boots,
# or use a BLE scanner app on your phone (like nRF Connect).
# Replace the placeholder below with your device's address.
ESP32_TARGET_ADDRESS = "68:25:DD:48:10:36" # <-- IMPORTANT: CHANGE THIS!

# --- Constants ---
MOVE_SPEED_OFFSET = 0.1
TURN_SPEED_OFFSET = 0.25
MM_TO_FEET = 0.00328084

# --- Constants for Follow Mode ---
FOLLOW_DEAD_ZONE_FT = 0.5

# --- Constants for Kinect Guidance ---
ROI_WIDTH = 100
ROI_HEIGHT = 50

# --- Ramping Configuration ---
RAMP_STEP = 0.05
RAMP_DELAY = 0.05

# --- Optimization for Raspberry Pi ---
# You MUST keep this number high to prevent crashes.
PROCESS_EVERY_NTH_FRAME = 10

# --- Constants for Green Tracking Mode ---
LOWER_GREEN = np.array([35, 100, 100])
UPPER_GREEN = np.array([85, 255, 255])
TRACKING_CENTER_DEAD_ZONE_PX = 50
MIN_CONTOUR_AREA = 500

# --- Global State Variables ---
current_fwd_bwd = 0.0
current_left_right = 0.0

# --- Hardware Setup ---
try:
    print("Initializing I2C and sensors...")
    i2c = board.I2C()
    mpu = adafruit_mpu6050.MPU6050(i2c)
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MPU650 and MCP4728 found and initialized. ✅")
except ValueError as e:
    print(f"Error: Could not find a required I2C device. Please check connections.")
    print(f"Details: {e}")
    exit()

try:
    print("Initializing Kinect V2 device (for vision modes)...")
    kinect = Device()
    print("Kinect V2 found and initialized. ✅")
except Exception as e:
    print(f"Warning: Could not initialize Kinect V2. Vision modes will be unavailable.")
    kinect = None

# --- DAC Control Functions ---
def set_movement(fwd_bwd, left_right):
    global current_fwd_bwd, current_left_right
    fwd_bwd = max(-1.0, min(1.0, fwd_bwd))
    left_right = max(-1.0, min(1.0, left_right))
    mcp.channel_a.normalized_value = 0.5 + (fwd_bwd / 2.0)
    mcp.channel_b.normalized_value = 0.5 - (fwd_bwd / 2.0)
    mcp.channel_c.normalized_value = 0.5 + (left_right / 2.0)
    mcp.channel_d.normalized_value = 0.5 - (left_right / 2.0)
    current_fwd_bwd = fwd_bwd
    current_left_right = left_right

def stop_all_movement():
    print("\nStopping movement...")
    set_movement(0.0, 0.0)
    print("Stopped.")

def ramp_to_speed(target_fwd_bwd, target_left_right):
    while abs(target_fwd_bwd - current_fwd_bwd) > 0.01 or \
          abs(target_left_right - current_left_right) > 0.01:
        new_fwd_bwd = current_fwd_bwd; new_left_right = current_left_right
        if target_fwd_bwd > current_fwd_bwd: new_fwd_bwd += RAMP_STEP
        elif target_fwd_bwd < current_fwd_bwd: new_fwd_bwd -= RAMP_STEP
        if target_left_right > current_left_right: new_left_right += RAMP_STEP
        elif target_left_right < current_left_right: new_left_right -= RAMP_STEP
        set_movement(new_fwd_bwd, new_left_right)
        time.sleep(RAMP_DELAY)
    set_movement(target_fwd_bwd, target_left_right)

def execute_turn(direction, target_angle_deg):
    print(f"\nExecuting turn: {direction} {target_angle_deg:.1f}°...")
    if target_angle_deg < 1.0:
        print("Turn angle too small, skipping.")
        return

    turn_value = TURN_SPEED_OFFSET
    if direction == 'left':
        turn_value *= -1

    set_movement(0.0, turn_value)
    total_angle_turned = 0.0
    last_time = time.monotonic()
    while total_angle_turned < target_angle_deg:
        current_time = time.monotonic()
        time_delta = current_time - last_time
        last_time = current_time
        total_angle_turned += abs(math.degrees(mpu.gyro[2] * time_delta))
        print(f"  -> Progress: {total_angle_turned:.1f}° / {target_angle_deg:.1f}°", end='\r')
        time.sleep(0.01)

    print(f"\n  -> Target angle reached.")
    stop_all_movement()

# --- Core Logic Functions ---

async def execute_find_bluetooth_mode():
    """
    Performs a 360-degree scan for a specific BLE device.
    """
    is_specific_scan = ESP32_TARGET_ADDRESS and ESP32_TARGET_ADDRESS.upper() != "XX:XX:XX:XX:XX:XX"

    print("\n--- Starting Bluetooth Find Mode ---")
    if is_specific_scan:
        print(f"Scanning for specific ESP32 with address: {ESP32_TARGET_ADDRESS}")
    else:
        print(f"Warning: No specific MAC address set. Scanning for ANY device with UUID...")
        print(f"Service UUID: {SERVICE_UUID}")

    def device_filter(device, advertisement_data):
        """Checks if a discovered device is the one we're looking for."""
        has_service = SERVICE_UUID.lower() in advertisement_data.service_uuids
        if is_specific_scan:
            return device.address.upper() == ESP32_TARGET_ADDRESS.upper() and has_service
        else:
            return has_service

    signal_readings = {}  # Dictionary to store angle: rssi

    # --- Step 1: Perform a 360-degree scan ---
    print("Performing 360-degree environmental scan...")
    set_movement(0.0, TURN_SPEED_OFFSET)  # Start spinning right
    total_angle_turned = 0.0
    last_time = time.monotonic()
    scan_start_time = time.time()

    while total_angle_turned < 360:
        current_time = time.monotonic()
        time_delta = current_time - last_time
        last_time = current_time
        total_angle_turned += abs(math.degrees(mpu.gyro[2] * time_delta))

        # --- MODIFIED: New scanning logic to correctly get RSSI ---
        # We use discover() now, which returns advertisement data containing the RSSI.
        found_device_in_slice = False
        discovered_devices = await bleak.BleakScanner.discover(timeout=0.25, return_adv=True)

        for device, advertisement_data in discovered_devices.values():
            if device_filter(device, advertisement_data):
                rssi = advertisement_data.rssi
                signal_readings[total_angle_turned] = rssi
                print(f"  -> Angle: {total_angle_turned:.1f}° | RSSI: {rssi} dBm | FOUND", end='\r')
                found_device_in_slice = True
                break # Found our device in this slice, move to the next angle

        if not found_device_in_slice:
            print(f"  -> Angle: {total_angle_turned:.1f}° | RSSI: Not found       ", end='\r')
        # --- End of modification ---

        if time.time() - scan_start_time > 45:
            print("\nScan took too long, stopping.")
            break

    stop_all_movement()
    print("\nScan complete.")

    # --- Step 2: Analyze results and find the best angle ---
    if not signal_readings:
        print("Could not find the target ESP32 device during the scan.")
        return

    best_angle = max(signal_readings, key=signal_readings.get)
    best_rssi = signal_readings[best_angle]
    print(f"Strongest signal found at {best_angle:.1f}° (RSSI: {best_rssi} dBm)")

    # --- Step 3: Turn to face the best angle ---
    if best_angle > 180:
        turn_direction = 'left'
        turn_angle = 360 - best_angle
    else:
        turn_direction = 'right'
        turn_angle = best_angle
        
    execute_turn(turn_direction, turn_angle)
    print("Pointing towards the estimated direction of the strongest signal.")

def execute_track_green_mode():
    """
    Spins until a green square is found, then turns to center it.
    """
    print(f"\n--- Starting Green Tracking Mode ---")
    print(f"Looking for a green square...")
    print("Press CTRL+C to stop tracking and return to the main menu.")

    try:
        frame_count = 0
        with kinect.running():
            for frame_type, frame in kinect:
                if frame_type == FrameType.Color:
                    frame_count += 1
                    if frame_count % PROCESS_EVERY_NTH_FRAME != 0:
                        continue

                    color_image = frame.to_array()[:, :, :3]
                    h, w, _ = color_image.shape
                    resized_w = 640
                    resized_h = int(resized_w * (h / w))
                    frame_resized = cv2.resize(color_image, (resized_w, resized_h))
                    hsv = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)
                    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
                    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                    found_target = False
                    if contours:
                        largest_contour = max(contours, key=cv2.contourArea)
                        if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                            peri = cv2.arcLength(largest_contour, True)
                            approx = cv2.approxPolyDP(largest_contour, 0.04 * peri, True)
                            if len(approx) == 4:
                                found_target = True
                                M = cv2.moments(largest_contour)
                                cX = int(M["m10"] / M["m00"])

                    if found_target:
                        frame_center_x = resized_w // 2
                        left_bound = frame_center_x - TRACKING_CENTER_DEAD_ZONE_PX
                        right_bound = frame_center_x + TRACKING_CENTER_DEAD_ZONE_PX

                        if cX < left_bound:
                            set_movement(0.0, -TURN_SPEED_OFFSET)
                            status = "TRACKING (Turn Left) "
                        elif cX > right_bound:
                            set_movement(0.0, TURN_SPEED_OFFSET)
                            status = "TRACKING (Turn Right)"
                        else:
                            set_movement(0.0, 0.0)
                            status = "TARGET CENTERED       "
                        print(f"Status: {status} | Target X: {cX} | Center: {frame_center_x} ", end='\r')
                    else:
                        set_movement(0.0, TURN_SPEED_OFFSET)
                        print(f"Status: SEARCHING... (Spinning Right)                                       ", end='\r')

    except KeyboardInterrupt:
        print("\nTracking mode interrupted by user.")
        stop_all_movement()

def execute_follow_mode(target_distance_ft):
    print(f"\n--- Starting Follow Mode ---")
    print(f"Target distance: {target_distance_ft:.1f} ft (Dead Zone: +/- {FOLLOW_DEAD_ZONE_FT} ft)")
    print("Press CTRL+C to stop following and return to the main menu.")
    target_distance_mm = target_distance_ft / MM_TO_FEET
    upper_bound_mm = target_distance_mm + (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)
    lower_bound_mm = target_distance_mm - (FOLLOW_DEAD_ZONE_FT / MM_TO_FEET)
    try:
        frame_count = 0
        with kinect.running():
            for frame_type, frame in kinect:
                if frame_type == FrameType.Depth:
                    frame_count += 1
                    if frame_count % PROCESS_EVERY_NTH_FRAME != 0:
                        continue
                    depth_data = frame.to_array()
                    height, width = depth_data.shape
                    roi_y1 = (height - ROI_HEIGHT) // 2; roi_y2 = roi_y1 + ROI_HEIGHT
                    roi_x1 = (width - ROI_WIDTH) // 2; roi_x2 = roi_x1 + ROI_WIDTH
                    center_roi = depth_data[roi_y1:roi_y2, roi_x1:roi_x2]
                    valid_depths = center_roi[center_roi > 0]
                    if valid_depths.size > 0:
                        closest_point_mm = np.min(valid_depths)
                        if closest_point_mm > upper_bound_mm:
                            if current_fwd_bwd <= 0: set_movement(MOVE_SPEED_OFFSET, 0.0)
                            status = "MOVING FORWARD"
                        elif closest_point_mm < lower_bound_mm:
                            if current_fwd_bwd > 0: set_movement(0.0, 0.0)
                            status = "STOPPED (Too Close)"
                        else:
                            if current_fwd_bwd > 0: set_movement(0.0, 0.0)
                            status = "HOLDING (In Zone)"
                        current_dist_ft = closest_point_mm * MM_TO_FEET
                        print(f"Target: {target_distance_ft:.1f}ft | Current: {current_dist_ft:.1f}ft | Status: {status}    ", end='\r')
                    else:
                        if current_fwd_bwd > 0: set_movement(0.0, 0.0)
                        print(f"Target: {target_distance_ft:.1f}ft | Current: --- | Status: STOPPED (Path Clear) ", end='\r')
    except KeyboardInterrupt:
        print("\nFollow mode interrupted by user.")
        stop_all_movement()

# --- Main Program Loop ---
if __name__ == "__main__":
    set_movement(0.0, 0.0)
    print("\n--- Autonomous Wheelchair Control ---")
    print("Commands:")
    print("  'find'             - Find ESP32 via Bluetooth signal")
    print("  'track'              - Find and center on a green square.")
    print("  'follow [feet]'      - Maintain a distance from an object.")
    print("  'left [degrees]'     - e.g., 'left 90'")
    print("  'right [degrees]'    - e.g., 'right 45'")
    print("  'stop'               - Halts any current movement")
    print("  'exit'               - Closes the program")
    print("-----------------------------------------")
    
    while True:
        command_str = input("Enter command > ").lower().strip()
        parts = command_str.split()
        if not parts: continue
        command = parts[0]
        
        if len(parts) == 1:
            if command == "find":
                try:
                    asyncio.run(execute_find_bluetooth_mode())
                except bleak.exc.BleakError as e:
                    print(f"\nBluetooth Error: {e}")
                    print("Please ensure Bluetooth is enabled and the script has permissions.")
            elif command == "track":
                if kinect:
                    execute_track_green_mode()
                else:
                    print("Kinect not available for track mode.")
            elif command == "stop":
                stop_all_movement()
            elif command == "exit":
                print("Setting to neutral and exiting program.")
                stop_all_movement()
                break
            else:
                print("Invalid command.")

        elif len(parts) == 2:
            try:
                value = float(parts[1])
                if value <= 0:
                    print("Error: Distance or angle must be a positive number."); continue
                
                if command == 'follow':
                    if kinect:
                        execute_follow_mode(target_distance_ft=value)
                    else:
                        print("Kinect not available for follow mode.")
                elif command in ('left', 'right'):
                    execute_turn(direction=command, target_angle_deg=value)
                else:
                    print("Invalid command.")
            except ValueError:
                print("Error: Invalid distance/angle. Please enter a number.")
        else:
            print("Invalid command format.")
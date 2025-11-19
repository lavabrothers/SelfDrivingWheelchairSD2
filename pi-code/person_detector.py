"""
person_detector.py

This module provides comprehensive vision-based functionalities for the self-driving
wheelchair, primarily utilizing the Kinect V2 sensor. It integrates person detection
using a pre-trained MobileNet-SSD model with depth sensing to identify and track
individuals, as well as detect general obstacles.

Key Features:
- Initializes and manages the Kinect V2 sensor, including shared access via a threading lock.
- Suppresses verbose Kinect V2 library logs for cleaner output.
- Detects persons in the color stream using a deep learning model (MobileNet-SSD).
- Calculates the distance to the detected person and their horizontal position.
- Identifies the nearest non-person obstacle within a defined region.
- Provides a dedicated function for robust center-path obstacle detection,
  ignoring peripheral noise.
- Applies smoothing to distance and depth measurements for stability.
- Includes a visual test mode using OpenCV for real-time feedback.

Dependencies:
- time: For timing operations.
- numpy: For numerical operations and array manipulation of image and depth data.
- cv2 (OpenCV): For image processing, DNN inference, and visualization.
- sys: For system-level operations.
- threading: For managing a lock to ensure safe concurrent access to Kinect frames.
- pylibfreenect2: Python wrapper for libfreenect2, used to interface with the Kinect V2.
"""

import time
import numpy as np
import cv2
import sys
import threading # For lock

# --- Imports for pylibfreenect2 ---
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, FrameMap
from pylibfreenect2 import Logger, setGlobalLogger 

# --- Constants ---
MM_TO_FEET = 0.00328084 # Conversion factor from millimeters to feet.

# --- Constants for find_target_person() ---
PROCESS_EVERY_NTH_FRAME = 3  # Process person detection on every Nth frame to save CPU.
DETECTION_WIDTH_PX = 360     # Width to resize the color frame for DNN input.
CONFIDENCE_THRESHOLD = 0.5   # Minimum confidence to consider a detection valid.
PROTOTXT_FILE = "MobileNetSSD_deploy.prototxt.txt" # Path to the Caffe prototxt file for MobileNet-SSD.
MODEL_FILE = "MobileNetSSD_deploy.caffemodel"      # Path to the Caffe model file for MobileNet-SSD.
CLASS_ID_PERSON = 15         # Class ID for 'person' in the MobileNet-SSD model.
MIN_PERSON_DISTANCE_FT = 0.5 # Minimum safe distance (in feet) for a person.

# --- Constants for general depth processing and get_nearest_object_angle() ---
KINECT_H_FOV = 70.6          # Horizontal Field of View of the Kinect V2 depth sensor in degrees.
KINECT_WIDTH = 512           # Width of the depth map in pixels.
SMOOTHING_FACTOR = 0.9       # Exponential moving average smoothing factor for general depth/angle.
PERSON_SMOOTHING_FACTOR = 0.7 # Smoothing factor specifically for person distance.
CROP_BOTTOM_RATIO = 0.05     # Ratio of the depth frame to crop from the bottom (to ignore floor).
CROP_LEFT_RATIO = 0.30       # Ratio of the depth frame to crop from the left.
CROP_RIGHT_RATIO = 0.30      # Ratio of the depth frame to crop from the right.

# --- Constants for get_center_path_depth() ---
CENTER_PATH_Y_OFFSET_PX = 160   # Vertical offset (downwards) from the center of the depth frame for the path region.
CENTER_PATH_REGION_WIDTH = 200  # Width of the rectangular region to check for obstacles in the center path.
CENTER_PATH_REGION_HEIGHT = 100 # Height of the rectangular region to check for obstacles in the center path.

# --- Module-level Globals ---
freenect2: Freenect2 | None = None          # Freenect2 object for managing Kinect devices.
kinect = None                               # Represents the opened Kinect V2 device.
listener: SyncMultiFrameListener | None = None # Listener for receiving frames from the Kinect.
net = None                                  # OpenCV DNN model for person detection.
frames: FrameMap | None = None              # FrameMap object to hold received frames.
frame_count: int = 0                        # Counter for frames processed.

kinect_lock = threading.Lock()              # A lock to ensure thread-safe access to Kinect frames.

# --- State for find_target_person() ---
last_good_person_state = (None, None, DETECTION_WIDTH_PX) # Stores (distance, center_x, frame_width) of last detected person.
last_known_person_dist_ft: float | None = None # Smoothed distance to the last known person.
consecutive_target_losses: int = 0          # Counter for consecutive frames without person detection.
MAX_CONSECUTIVE_LOSSES: int = 5             # Max losses before declaring target lost.

# --- State for obstacle smoothing (used by find_target_person and get_nearest_object_angle) ---
last_known_obstacle_depth_mm: float | None = None # Smoothed depth of the nearest non-person obstacle.

# --- State for get_nearest_object_angle() ---
last_known_depth: float | None = None       # Smoothed minimum depth from the cropped depth map.
last_known_angle: float | None = None       # Smoothed horizontal angle of the nearest object.
last_known_coords: tuple[float, float] | None = None # Smoothed pixel coordinates of the nearest object.

# --- State for get_center_path_depth() ---
last_known_center_depth: float | None = None # Smoothed minimum depth in the center path region.

class NoLogger(Logger):
    """
    A custom logger class that suppresses all log messages from pylibfreenect2.
    This is used to prevent excessive [Info] messages from cluttering the console.
    """
    def log(self, level, message):
        pass # Do nothing, effectively silencing the logger.

def initialize_detector() -> bool:
    """
    Initializes the Kinect V2 sensor and loads the MobileNet-SSD deep neural network
    model for person detection.

    This function sets up the Freenect2 context, opens the Kinect device, configures
    frame listeners for both color and depth streams, starts the Kinect stream,
    and loads the pre-trained Caffe model. It also enables OpenCL for OpenCV
    for potentially faster processing.

    Returns:
        bool: True if both the Kinect and the DNN model are successfully initialized,
              False otherwise.
    """
    global freenect2, kinect, listener, net, frames
    
    # Set the global logger to our empty 'NoLogger' class to suppress Kinect logs.
    setGlobalLogger(NoLogger())
    
    # 1. Initialize Kinect V2 sensor.
    try:
        print("Initializing Kinect V2 (pylibfreenect2)...")
        freenect2 = Freenect2()
        num_devices = freenect2.enumerateDevices()
        if num_devices == 0:
            print("Error: No Kinect V2 devices found!")
            return False
            
        serial = freenect2.getDeviceSerialNumber(0)
        kinect = freenect2.openDevice(serial)
        
        # Request both Color and Depth frames.
        types = FrameType.Color | FrameType.Depth
        listener = SyncMultiFrameListener(types)
        
        kinect.setColorFrameListener(listener)
        kinect.setIrAndDepthFrameListener(listener)
        
        print(f"Starting Kinect V2 stream (Serial: {serial})...")
        kinect.start()
        frames = FrameMap() # Initialize FrameMap to store incoming frames.
        print("Kinect V2 found and initialized. ✅")
        
    except Exception as e:
        print(f"Error initializing Kinect V2: {e}")
        if "LIBUSB_ERROR_ACCESS" in str(e):
             print("\n--- PERMISSION ERROR ---")
             print("This is likely a USB permission issue. You may need to set up udev rules for the Kinect.")
        return False
        
    # 2. Load the DNN model for person detection.
    try:
        print(f"Loading person detection model ({MODEL_FILE})...")
        net = cv2.dnn.readNetFromCaffe(PROTOTXT_FILE, MODEL_FILE)
        if net is None:
             raise IOError("Could not load model files.")
        print("Person detection model loaded successfully. ✅")
    except Exception as e:
        print(f"Fatal Error: {e}")
        print(f"Please ensure '{PROTOTXT_FILE}' and '{MODEL_FILE}' are in the same directory.")
        shutdown_detector() # Clean up Kinect if model loading fails.
        return False

    # 3. Enable OpenCL for OpenCV for hardware acceleration if available.
    cv2.ocl.setUseOpenCL(True)
    return True

def shutdown_detector():
    """
    Stops and closes the Kinect V2 device.

    This function is critical for releasing hardware resources and should always
    be called when the Kinect is no longer needed to prevent resource leaks or
    issues with subsequent sensor initialization.
    """
    global kinect
    print("\nShutting down Kinect V2...")
    if kinect:
        try:
            kinect.stop()
            kinect.close()
            print("Kinect V2 shut down successfully.")
        except Exception as e:
            print(f"Error during Kinect V2 shutdown: {e}")

def find_target_person(visualize: bool = False) -> tuple[tuple[float | None, int | None, int, float | None], np.ndarray | None]:
    """
    Captures a frame from the Kinect, detects the largest person, and identifies
    the nearest non-person obstacle within a defined region.

    This function processes color frames for person detection using the DNN model
    and depth frames for distance estimation and obstacle avoidance. It applies
    smoothing to the person's distance and the nearest obstacle's depth.

    Args:
        visualize (bool): If True, an OpenCV image with detection bounding boxes
                          and labels will be returned for debugging/display.

    Returns:
        tuple: A tuple containing:
               - (person_dist_ft, person_center_x, frame_width, nearest_obstacle_depth_mm):
                 A tuple of processed data:
                 - person_dist_ft (float | None): Smoothed distance to the person in feet.
                 - person_center_x (int | None): Horizontal center pixel of the person's bounding box.
                 - frame_width (int): Width of the resized frame used for detection.
                 - nearest_obstacle_depth_mm (float | None): Smoothed depth of the nearest
                   non-person obstacle in millimeters.
               - debug_frame (np.ndarray | None): The OpenCV image for visualization if `visualize` is True,
                 otherwise None.
    """
    global listener, frames, frame_count, net, last_good_person_state, last_known_person_dist_ft, consecutive_target_losses
    global last_known_obstacle_depth_mm

    if not listener or not net:
        print("Error: Detector not initialized.")
        return (None, None, DETECTION_WIDTH_PX, None), None

    color_image_bgra_1080p = None
    depth_image_cpu = None

    # Acquire lock to safely access Kinect frames.
    with kinect_lock:
        if not listener.waitForNewFrame(frames, 10 * 1000): # 10-second timeout.
            print("Kinect timeout in find_target_person...", end='\r')
            p_state = last_good_person_state
            # Return last known *smoothed* obstacle depth on timeout.
            return (p_state[0], p_state[1], p_state[2], last_known_obstacle_depth_mm), None
        
        color_frame = frames[FrameType.Color]
        depth_frame = frames[FrameType.Depth]
        
        color_image_bgra_1080p = color_frame.asarray(dtype=np.uint8)
        depth_image_cpu = depth_frame.asarray()
        
        listener.release(frames) # Release frames immediately after copying data.

    frame_count += 1
    # Skip processing on most frames to reduce CPU load.
    if frame_count % PROCESS_EVERY_NTH_FRAME != 0:
        p_state = last_good_person_state
        # Return last known *smoothed* obstacle depth.
        return (p_state[0], p_state[1], p_state[2], last_known_obstacle_depth_mm), None

    # Create a copy of the depth map for obstacle detection, which will be modified.
    obstacle_depth_map = depth_image_cpu.copy()

    # --- Person Detection Logic (using MobileNet-SSD) ---
    color_image_bgr_1080p = cv2.cvtColor(color_image_bgra_1080p, cv2.COLOR_BGRA2BGR)
    frame_umat = cv2.UMat(color_image_bgr_1080p) # Use UMat for potential OpenCL acceleration.

    h_full, w_full, _ = color_image_bgr_1080p.shape
    scale_factor = DETECTION_WIDTH_PX / w_full
    resized_w = DETECTION_WIDTH_PX
    resized_h = int(h_full * scale_factor)
    
    frame_resized_umat = cv2.resize(frame_umat, (resized_w, resized_h))
    
    debug_frame = None
    if visualize:
        debug_frame = frame_resized_umat.get() # Get CPU copy for drawing if visualizing.
    
    frame_to_process = frame_resized_umat if not visualize else debug_frame
    blob = cv2.dnn.blobFromImage(frame_to_process, 0.007843, # Scale factor.
                                 (300, 300), 127.5) # Input size and mean subtraction.
    
    net.setInput(blob)
    detections = net.forward() # Perform inference.

    best_target_confidence = -1
    best_target_box = None

    # Iterate through detections to find the most confident 'person'.
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        class_id = int(detections[0, 0, i, 1])
        
        if class_id == CLASS_ID_PERSON and confidence > CONFIDENCE_THRESHOLD:
            if confidence > best_target_confidence:
                best_target_confidence = confidence
                # Scale bounding box coordinates back to the resized frame.
                box = detections[0, 0, i, 3:7] * np.array([resized_w, resized_h, resized_w, resized_h])
                best_target_box = box.astype("int")

    # --- Process Person Detection Results ---
    current_dist_ft = None
    color_cX = None

    if best_target_box is not None:
        (x_start, y_start, x_end, y_end) = best_target_box
        x, y, w, h = x_start, y_start, x_end - x_start, y_end - y_start

        # Map bounding box coordinates from resized color frame to full-resolution depth frame.
        depth_h, depth_w = depth_image_cpu.shape
        depth_scale_x = depth_w / resized_w
        depth_scale_y = depth_h / resized_h
        color_cX = x + w // 2 # Center X of person in resized color frame.
        
        depth_roi_x = int(x * depth_scale_x)
        depth_roi_y = int(y * depth_scale_y)
        depth_roi_w = int(w * depth_scale_x)
        depth_roi_h = int(h * depth_scale_y)
        
        # Clamp ROI coordinates to depth frame boundaries.
        depth_roi_x = max(0, depth_roi_x)
        depth_roi_y = max(0, depth_roi_y)
        depth_roi_w = min(depth_w - depth_roi_x, depth_roi_w)
        depth_roi_h = min(depth_h - depth_roi_y, depth_roi_h)

        if depth_roi_w > 0 and depth_roi_h > 0:
            # Extract the depth ROI corresponding to the detected person.
            person_roi = depth_image_cpu[depth_roi_y : depth_roi_y + depth_roi_h,
                                         depth_roi_x : depth_roi_x + depth_roi_w]
            
            # Mask out the person from the obstacle map to prevent self-detection.
            obstacle_depth_map[depth_roi_y : depth_roi_y + depth_roi_h,
                               depth_roi_x : depth_roi_x + depth_roi_w] = 0
            
            # Find valid (non-zero) depths within the person's ROI.
            valid_depths = person_roi[person_roi > 0]
            if valid_depths.size > 0:
                closest_point_mm = np.min(valid_depths) # Closest point to the person.
                new_dist_ft = closest_point_mm * MM_TO_FEET

                # Apply minimum distance filter and smoothing for person distance.
                if new_dist_ft < MIN_PERSON_DISTANCE_FT:
                    last_known_person_dist_ft = None # Too close, effectively lost.
                    current_dist_ft = None
                else:
                    if last_known_person_dist_ft is None:
                        current_dist_ft = new_dist_ft
                    else:
                        current_dist_ft = (new_dist_ft * PERSON_SMOOTHING_FACTOR) + \
                                          (last_known_person_dist_ft * (1.0 - PERSON_SMOOTHING_FACTOR))
                    
                    last_known_person_dist_ft = current_dist_ft

        # Add visualization elements if enabled.
        if visualize and debug_frame is not None:
            cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            conf_label = f"Person: {best_target_confidence:.2f}"
            cv2.putText(debug_frame, conf_label, (x, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if current_dist_ft is not None:
                dist_label = f"{current_dist_ft:.1f} ft"
                cv2.putText(debug_frame, dist_label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Update consecutive loss counter.
        if current_dist_ft is not None and current_dist_ft > 1.0: # Consider person "good" if not too close.
            consecutive_target_losses = 0 
            last_good_person_state = (current_dist_ft, color_cX, resized_w)
        else:
            consecutive_target_losses += 1
    else:
        # No person box found in the current frame.
        consecutive_target_losses += 1

    # --- Obstacle Detection Logic with Smoothing (excluding the person) ---
    
    # Apply the same cropping as used in cruise control for consistency.
    height, width = obstacle_depth_map.shape 
    crop_row_bottom = int(height * (1.0 - CROP_BOTTOM_RATIO))
    crop_col_left = int(width * CROP_LEFT_RATIO)
    crop_col_right = int(width * (1.0 - CROP_RIGHT_RATIO))

    # Create the cropped ROI for general obstacle checking.
    obstacle_map_roi = obstacle_depth_map[0:crop_row_bottom, crop_col_left:crop_col_right]
    
    # Find the nearest obstacle *within the cropped ROI*.
    valid_obstacle_depths = obstacle_map_roi[obstacle_map_roi > 0]
    
    nearest_obstacle_depth_mm = None # Final value to be returned.
    
    if valid_obstacle_depths.size > 0:
        new_obstacle_depth = np.min(valid_obstacle_depths)
        
        # Apply smoothing to the obstacle depth.
        if last_known_obstacle_depth_mm is None:
            last_known_obstacle_depth_mm = new_obstacle_depth
        else:
            last_known_obstacle_depth_mm = \
                (new_obstacle_depth * SMOOTHING_FACTOR) + \
                (last_known_obstacle_depth_mm * (1.0 - SMOOTHING_FACTOR))
        
        nearest_obstacle_depth_mm = last_known_obstacle_depth_mm
        
    else:
        # No obstacle found in the cone, reset smoothed value.
        last_known_obstacle_depth_mm = None 
        nearest_obstacle_depth_mm = None
    
    # Add visualization for obstacle depth if enabled.
    if visualize and debug_frame is not None and nearest_obstacle_depth_mm is not None:
        obs_text = f"Obs: {nearest_obstacle_depth_mm / 1000.0:.2f}m"
        cv2.putText(debug_frame, obs_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # --- Final Return Logic ---
    if consecutive_target_losses < MAX_CONSECUTIVE_LOSSES:
        # If the person was recently seen, return its last good state.
        p_state = last_good_person_state
        return (p_state[0], p_state[1], p_state[2], nearest_obstacle_depth_mm), debug_frame
    else:
        # If the target is officially lost, reset person-specific state.
        last_good_person_state = (None, None, resized_w)
        last_known_person_dist_ft = None 
        return (None, None, resized_w, nearest_obstacle_depth_mm), debug_frame


def get_nearest_object_angle(visualize: bool = False) -> tuple[tuple[float | None, float | None, tuple[float, float] | None], np.ndarray | None]:
    """
    Captures a depth frame and finds the closest object within a cropped region
    of interest (ROI), returning its smoothed depth and horizontal angle.

    This function is typically used for general obstacle avoidance in cruise control,
    where a single "nearest object" is sufficient. It applies cropping to ignore
    the floor and side regions, and smoothing to the output values.

    Args:
        visualize (bool): If True, an OpenCV image visualizing the depth map,
                          cropping lines, and the detected object will be returned.

    Returns:
        tuple: A tuple containing:
               - (min_depth_mm, angle_deg, coords_px):
                 A tuple of processed data:
                 - min_depth_mm (float | None): Smoothed minimum depth in millimeters.
                 - angle_deg (float | None): Smoothed horizontal angle in degrees from center.
                 - coords_px (tuple[float, float] | None): Smoothed pixel coordinates (x, y)
                   of the detected point in the full depth frame.
               - debug_frame (np.ndarray | None): The OpenCV image for visualization if `visualize` is True,
                 otherwise None.
    """
    global listener, frames, last_known_depth, last_known_angle, last_known_coords

    if listener is None:
        print("Error: Detector not initialized.")
        return (last_known_depth, last_known_angle, last_known_coords), None
    
    depth_map = None
    debug_frame = None

    try:
        with kinect_lock:
            if not listener.waitForNewFrame(frames, 10 * 1000):
                print("Kinect timeout in get_nearest_object_angle...", end='\r')
                return (last_known_depth, last_known_angle, last_known_coords), None

            depth_frame = frames[FrameType.Depth]
            depth_map = depth_frame.asarray()
            listener.release(frames)
        
        height, width = depth_map.shape 
        # Calculate cropping boundaries.
        crop_row_bottom = int(height * (1.0 - CROP_BOTTOM_RATIO))
        crop_col_left = int(width * CROP_LEFT_RATIO)
        crop_col_right = int(width * (1.0 - CROP_RIGHT_RATIO))

        # Create the Region of Interest (ROI) by slicing the depth map.
        depth_map_roi = depth_map[0:crop_row_bottom, crop_col_left:crop_col_right]
        
        # Filter out zero (invalid) depth readings from the ROI.
        valid_depths = depth_map_roi[depth_map_roi > 0]
        
        if valid_depths.size == 0:
            # No valid depths in ROI, reset smoothed values.
            last_known_depth = None
            last_known_angle = None
            last_known_coords = None 
            
            if visualize:
                # Build visualization even if no object is found.
                debug_frame = _build_depth_visualization(depth_map, None, None, None, (crop_row_bottom, crop_col_left, crop_col_right))
                
            return (None, None, None), debug_frame

        # Use the 1st percentile to find a robust minimum depth, less sensitive to outliers.
        new_depth = np.percentile(valid_depths, 1)

        # Find the pixel coordinates (y, x) of the minimum depth within the ROI.
        search_map = np.where(depth_map_roi == 0, 999999, depth_map_roi) # Replace 0s with large number.
        y_roi, x_roi = np.unravel_index(np.argmin(search_map), search_map.shape)

        # Convert ROI coordinates back to full frame coordinates.
        new_x = x_roi + crop_col_left
        new_y = y_roi 
        
        # Calculate the horizontal angle from the center of the full frame.
        normalized_x = (new_x - (KINECT_WIDTH / 2.0)) / (KINECT_WIDTH / 2.0)
        new_angle = normalized_x * (KINECT_H_FOV / 2.0)

        # Apply exponential smoothing to depth, angle, and coordinates.
        if last_known_depth is None:
            last_known_depth = new_depth
            last_known_angle = new_angle
            last_known_coords = (float(new_x), float(new_y))
        else:
            last_known_depth = (new_depth * SMOOTHING_FACTOR) + \
                               (last_known_depth * (1.0 - SMOOTHING_FACTOR))
            last_known_angle = (new_angle * SMOOTHING_FACTOR) + \
                               (last_known_angle * (1.0 - SMOOTHING_FACTOR))
            
            last_x, last_y = last_known_coords
            smoothed_x = (new_x * SMOOTHING_FACTOR) + (last_x * (1.0 - SMOOTHING_FACTOR))
            smoothed_y = (new_y * SMOOTHING_FACTOR) + (last_y * (1.0 - SMOOTHING_FACTOR))
            last_known_coords = (smoothed_x, smoothed_y)
        
        if visualize:
            # Build visualization with smoothed values.
            debug_frame = _build_depth_visualization(depth_map, last_known_depth, last_known_angle, last_known_coords, (crop_row_bottom, crop_col_left, crop_col_right))

        return (last_known_depth, last_known_angle, last_known_coords), debug_frame

    except Exception as e:
        print(f"Error in get_nearest_object_angle: {e}")
        return (last_known_depth, last_known_angle, last_known_coords), None

def get_center_path_depth(visualize: bool = False) -> tuple[tuple[float | None, tuple[int, int] | None], np.ndarray | None]:
    """
    Captures a depth frame and calculates the minimum depth within a small,
    centrally located rectangular region of interest.

    This function is designed for robust obstacle detection directly in the
    wheelchair's immediate path, minimizing sensitivity to peripheral noise
    or objects outside the direct line of travel. It applies smoothing to the depth.

    Args:
        visualize (bool): If True, an OpenCV image visualizing the depth map
                          and the central path region will be returned.

    Returns:
        tuple: A tuple containing:
               - (min_depth_mm, center_coords_px):
                 A tuple of processed data:
                 - min_depth_mm (float | None): Smoothed minimum depth in millimeters
                   within the central path region.
                 - center_coords_px (tuple[int, int] | None): Pixel coordinates (x, y)
                   of the center of the defined path region.
               - debug_frame (np.ndarray | None): The OpenCV image for visualization if `visualize` is True,
                 otherwise None.
    """
    global listener, frames, last_known_center_depth, kinect_lock
    
    if listener is None:
        print("Error: Detector not initialized.")
        return (last_known_center_depth, None), None
        
    depth_map = None
    debug_frame = None
    
    try:
        with kinect_lock:
            if not listener.waitForNewFrame(frames, 10 * 1000):
                print("Kinect timeout in get_center_path_depth...", end='\r')
                return (last_known_center_depth, None), None

            depth_frame = frames[FrameType.Depth]
            depth_map = depth_frame.asarray()
            listener.release(frames)

        h, w = depth_map.shape # Expected depth map dimensions: 424x512.
        
        # 1. Define the Center Region of Interest.
        center_x_frame = w // 2
        center_y_frame = (h // 2) + CENTER_PATH_Y_OFFSET_PX # Offset downwards.
        
        # Calculate the bounding box coordinates for the central region.
        x1 = center_x_frame - (CENTER_PATH_REGION_WIDTH // 2)
        x2 = center_x_frame + (CENTER_PATH_REGION_WIDTH // 2)
        y1 = center_y_frame - (CENTER_PATH_REGION_HEIGHT // 2)
        y2 = center_y_frame + (CENTER_PATH_REGION_HEIGHT // 2)
        
        # Clamp coordinates to ensure they stay within frame boundaries.
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)

        # 2. Extract the depth data from the defined central region.
        center_region = depth_map[y1:y2, x1:x2]
        
        # 3. Get all valid (non-zero) depth readings from the region.
        # Use percentile for noise rejection and robust minimum finding.
        valid_depths = center_region[center_region > 0] 

        new_depth = None
        if valid_depths.size > 0:
            depth_candidate = np.percentile(valid_depths, 1)
            # Check if the reading is within a reasonable operational range (0.5m to 4.5m).
            if 500 < depth_candidate < 4500:
                new_depth = depth_candidate

        # 4. Apply Smoothing to the center path depth.
        if new_depth is None:
            last_known_center_depth = None # Reset if no valid depth is found.
        else:
            if last_known_center_depth is None:
                last_known_center_depth = new_depth
            else:
                last_known_center_depth = (new_depth * SMOOTHING_FACTOR) + \
                                          (last_known_center_depth * (1.0 - SMOOTHING_FACTOR))
        
        if visualize:
            # Build visualization with the smoothed depth and region coordinates.
            debug_frame = _build_center_path_visualization(depth_map, last_known_center_depth, (x1, y1, x2, y2))
            
        return (last_known_center_depth, (center_x_frame, center_y_frame)), debug_frame

    except Exception as e:
        print(f"Error in get_center_path_depth: {e}")
        return (last_known_center_depth, None), None


def _build_depth_visualization(depth_map: np.ndarray, depth: float | None, angle: float | None, coords: tuple[float, float] | None, crop_lines: tuple[int, int, int]) -> np.ndarray:
    """
    Helper function to generate an OpenCV image for visualizing general depth detection.

    It normalizes the depth map for display, draws cropping lines, and overlays
    information about the detected nearest object (circle, depth, angle).

    Args:
        depth_map (np.ndarray): The raw depth map from the Kinect.
        depth (float | None): The smoothed minimum depth.
        angle (float | None): The smoothed horizontal angle.
        coords (tuple[float, float] | None): The smoothed pixel coordinates of the detected object.
        crop_lines (tuple[int, int, int]): A tuple (crop_row_bottom_viz, crop_col_left_viz, crop_col_right_viz)
                                            defining the visualization of the cropped region.

    Returns:
        np.ndarray: An OpenCV BGR image suitable for display.
    """
    try:
        viz_height, viz_width = depth_map.shape
        crop_row_bottom_viz, crop_col_left_viz, crop_col_right_viz = crop_lines
        
        # Normalize depth map for visual display (clipping and remapping).
        depth_viz = np.clip(depth_map, 500, 4500) # Clip depths between 0.5m and 4.5m.
        depth_viz = (depth_viz - 500) / 4000.0    # Remap to 0-1 range.
        depth_viz = (255 * (1.0 - depth_viz)).astype(np.uint8) # Invert colors (closer = brighter) and scale to 0-255.
        depth_viz[depth_map == 0] = 0 # Make "no-reading" (0) pixels black.
        
        image_color = cv2.cvtColor(depth_viz, cv2.COLOR_GRAY2BGR) # Convert to 3-channel BGR.

        # Draw cropping lines.
        cv2.line(image_color, (0, crop_row_bottom_viz), (viz_width - 1, crop_row_bottom_viz), (255, 255, 0), 1)
        cv2.line(image_color, (crop_col_left_viz, 0), (crop_col_left_viz, viz_height - 1), (255, 255, 0), 1)
        cv2.line(image_color, (crop_col_right_viz, 0), (crop_col_right_viz, viz_height - 1), (255, 255, 0), 1)

        # Overlay detected object information.
        if coords is not None and depth is not None:
            x, y = coords
            cv2.circle(image_color, (int(x), int(y)), 10, (0, 0, 255), 2) # Red circle.
            text = f"{depth/1000.0:.2f} m, {angle:.1f} deg"
            cv2.putText(image_color, text, (int(x) + 15, int(y) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        return image_color
    except Exception as e:
        print(f"Error building depth viz: {e}")
        return np.zeros((424, 512, 3), dtype=np.uint8) # Return a black image on error.

def _build_center_path_visualization(depth_map: np.ndarray, depth: float | None, box_coords: tuple[int, int, int, int]) -> np.ndarray:
    """
    Helper function to generate an OpenCV image for visualizing the central path region.

    It normalizes the depth map for display, draws the bounding box of the central
    path region, and overlays the detected minimum depth within that region.

    Args:
        depth_map (np.ndarray): The raw depth map from the Kinect.
        depth (float | None): The smoothed minimum depth within the central path region.
        box_coords (tuple[int, int, int, int]): A tuple (x1, y1, x2, y2) defining
                                                the bounding box of the central path region.

    Returns:
        np.ndarray: An OpenCV BGR image suitable for display.
    """
    try:
        x1, y1, x2, y2 = box_coords
        
        # Normalize depth map for visual display.
        depth_viz = np.clip(depth_map, 500, 4500)
        depth_viz = (depth_viz - 500) / 4000.0
        depth_viz = (255 * (1.0 - depth_viz)).astype(np.uint8)
        depth_viz[depth_map == 0] = 0
        image_color = cv2.cvtColor(depth_viz, cv2.COLOR_GRAY2BGR)

        # Draw the bounding box for the central path region.
        color = (0, 255, 0) if depth is not None else (0, 0, 255) # Green if depth found, Red otherwise.
        cv2.rectangle(image_color, (x1, y1), (x2, y2), color, 2)

        # Overlay detected depth information.
        if depth is not None:
            text = f"{depth/1000.0:.2f} m"
            cv2.putText(image_color, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return image_color
    except Exception as e:
        print(f"Error building center path viz: {e}")
        return np.zeros((424, 512, 3), dtype=np.uint8) # Return a black image on error.


# --- Test Mode ---
if __name__ == "__main__":
    """
    Main execution block for visual testing of the person detector.

    When run directly, this script initializes the Kinect and MobileNet-SSD model,
    then enters a continuous loop to capture frames, detect persons, and display
    the results in an OpenCV window. It primarily tests `find_target_person()`.
    To test `get_center_path_depth()`, the test loop needs to be modified.
    Press 'q' in the display window or Ctrl+C in the terminal to exit.
    """
    print("Running person_detector.py in VISUAL TEST MODE (with MobileNet-SSD).")
    print("This will test the find_target_person() function.")
    # To test get_center_path_depth, you would need to modify this test loop:
    # (depth_mm, _), debug_frame = get_center_path_depth(visualize=True)
    # obs_depth = None; dist_ft = None # Un-comment these lines to test get_center_path_depth.
    
    if not initialize_detector():
        print("Failed to initialize detector. Exiting.")
        sys.exit(1)
        
    print("\nInitialization complete. Starting test loop...")
    print("Press 'q' in the window to stop.")
    
    window_name = "Person Detector Test"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    
    try:
        while True:
            # Test find_target_person.
            (dist_ft, center_x, frame_w, obs_depth), debug_frame = find_target_person(visualize=True)
            
            if debug_frame is None:
                time.sleep(0.01) # Small delay to prevent busy-waiting.
                continue

            if dist_ft is not None:
                obs_str = f"Obs: {obs_depth/1000.0:.2f}m" if obs_depth else "Obs: N/A"
                print(f"Target: {dist_ft:.1f} ft | {obs_str} | @ pixel {center_x} (Frame: {frame_w}) ", end='\r')
            else:
                print("Target Lost... searching...                                       ", end='\r')
                
            cv2.imshow(window_name, debug_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n'q' pressed. Stopping test.")
                break
                
    except KeyboardInterrupt:
        print("\nStopping test due to KeyboardInterrupt...")
    finally:
        cv2.destroyAllWindows() # Close all OpenCV windows.
        shutdown_detector() # Ensure Kinect is properly shut down.

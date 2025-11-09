#!/usr/bin/env python3

# File: person_detector.py
# Module for initializing the Kinect V2 and providing
# all vision-based functions for the wheelchair.
#
# Provides:
# 1. find_target_person(): Uses DNN to find a person and their distance.
# 2. get_nearest_object_angle(): Uses depth frame to find the closest obstacle.
#
# --- UPDATED: To show distance in visual test mode ---
# --- UPDATED: To make get_nearest_object_angle stateful on object loss ---

import time
import numpy as np
import cv2
import sys

# --- Imports for pylibfreenect2 ---
from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, FrameMap

# --- Constants ---
MM_TO_FEET = 0.00328084

# --- Constants for find_target_person() ---
PROCESS_EVERY_NTH_FRAME = 3 # Skip frames to save CPU
DETECTION_WIDTH_PX = 360  # Resize color image for detection
CONFIDENCE_THRESHOLD = 0.5   # Minimum confidence for DNN
PROTOTXT_FILE = "MobileNetSSD_deploy.prototxt.txt"
MODEL_FILE = "MobileNetSSD_deploy.caffemodel"
CLASS_ID_PERSON = 15         # Class ID for "person" in this model
MIN_PERSON_DISTANCE_FT = 0.5 # Minimum valid distance for a person (filter out noise)

# --- Constants for get_nearest_object_angle() ---
KINECT_H_FOV = 70.6          # Horizontal Field of View (approx. 70.6 degrees)
KINECT_WIDTH = 512           # Depth map width (512 pixels)
# --- MODIFIED: Clarified smoothing comment ---
SMOOTHING_FACTOR = 0.9       # Weight of the NEW value (0.9 = light smoothing, 0.1 = heavy)
PERSON_SMOOTHING_FACTOR = 0.7 # Separate smoothing for person distance
CROP_BOTTOM_RATIO = 0.05     # Crop 5% from bottom (ignore floor)
CROP_LEFT_RATIO = 0.35        # Crop 35% from left
CROP_RIGHT_RATIO = 0.35       # Crop 35% from right

# --- Module-level Globals ---
freenect2 = None
kinect = None
listener = None
net = None                   # For the DNN
frames = None                # Re-usable FrameMap
frame_count = 0              # For frame skipping

# --- State for find_target_person() ---
last_good_target_state = (None, None, DETECTION_WIDTH_PX)
last_known_person_dist_ft = None
consecutive_target_losses = 0 # New state variable
MAX_CONSECUTIVE_LOSSES = 5   # Allow 5 frames of loss before truly losing target

# --- State for get_nearest_object_angle() ---
last_known_depth = None
last_known_angle = None
last_known_coords = None

def initialize_detector():
    """
    Initializes the Kinect V2 sensor and loads the 
    MobileNet-SSD DNN model.
    Returns True on success, False on failure.
    """
    global freenect2, kinect, listener, net, frames
    
    # 1. Initialize Kinect
    try:
        print("Initializing Kinect V2 (pylibfreenect2)...")
        freenect2 = Freenect2()
        num_devices = freenect2.enumerateDevices()
        if num_devices == 0:
            print("Error: No Kinect V2 devices found!")
            return False
            
        serial = freenect2.getDeviceSerialNumber(0)
        kinect = freenect2.openDevice(serial)
        
        # Request both Color and Depth, as both functions need them
        types = FrameType.Color | FrameType.Depth
        listener = SyncMultiFrameListener(types)
        
        kinect.setColorFrameListener(listener)
        kinect.setIrAndDepthFrameListener(listener)
        
        print(f"Starting Kinect V2 stream (Serial: {serial})...")
        kinect.start()
        frames = FrameMap() # Initialize the frame map
        print("Kinect V2 found and initialized. ✅")
        
    except Exception as e:
        print(f"Error initializing Kinect V2: {e}")
        if "LIBUSB_ERROR_ACCESS" in str(e):
             print("\n--- PERMISSION ERROR ---")
             print("This is likely a USB permission issue.")
        return False
        
    # 2. Load the DNN model
    try:
        print(f"Loading person detection model ({MODEL_FILE})...")
        net = cv2.dnn.readNetFromCaffe(PROTOTXT_FILE, MODEL_FILE)
        if net is None:
             raise IOError("Could not load model files.")
        print("Person detection model loaded successfully. ✅")
    except Exception as e:
        print(f"Fatal Error: {e}")
        print(f"Please ensure '{PROTOTXT_FILE}' and '{MODEL_FILE}' are in the same directory.")
        shutdown_detector() # Clean up kinect
        return False

    # 3. Enable OpenCL for OpenCV
    cv2.ocl.setUseOpenCL(True)
    return True

def shutdown_detector():
    """Stops and closes the Kinect V2 device."""
    global kinect
    print("\nShutting down Kinect V2...")
    if kinect:
        try:
            kinect.stop()
            kinect.close()
            print("Kinect V2 shut down successfully.")
        except Exception as e:
            print(f"Error during Kinect V2 shutdown: {e}")

def find_target_person(visualize=False):
    """
    Grabs one frame, processes it, and finds the largest person.
    Uses Color frame (DNN) + Depth frame (Ranging).
    
    Returns:
        tuple: (target_state, debug_frame)
        'target_state' (tuple): (dist_ft, center_x, frame_width)
        'debug_frame' (np.array): OpenCV image or None
    """
    global listener, frames, frame_count, net, last_good_target_state, last_known_person_dist_ft, consecutive_target_losses

    if not listener or not net:
        print("Error: Detector not initialized.")
        return (None, None, DETECTION_WIDTH_PX), None

    # 1. Wait for a new frame pair
    if not listener.waitForNewFrame(frames, 10 * 1000): # 10 sec timeout
        print("Kinect timeout in find_target_person...", end='\r')
        # Return last known state on timeout
        return last_good_target_state, None
    
    color_frame = frames[FrameType.Color]
    depth_frame = frames[FrameType.Depth]

    # 2. Get data
    color_image_bgra_1080p = color_frame.asarray(dtype=np.uint8)
    depth_image_cpu = depth_frame.asarray()
    
    # 3. Release frames *immediately*
    listener.release(frames)

    # 4. Check if we should process this frame
    frame_count += 1
    if frame_count % PROCESS_EVERY_NTH_FRAME != 0:
        # Return last known state on skipped frame
        return last_good_target_state, None 

    # 5. Process the color frame
    color_image_bgr_1080p = cv2.cvtColor(color_image_bgra_1080p, cv2.COLOR_BGRA2BGR)
    frame_umat = cv2.UMat(color_image_bgr_1080p)

    h_full, w_full, _ = color_image_bgr_1080p.shape
    scale_factor = DETECTION_WIDTH_PX / w_full
    resized_w = DETECTION_WIDTH_PX
    resized_h = int(h_full * scale_factor)
    
    frame_resized_umat = cv2.resize(frame_umat, (resized_w, resized_h))
    
    debug_frame = None
    if visualize:
        debug_frame = frame_resized_umat.get()
    
    # 6. DNN DETECTION LOGIC
    frame_to_process = frame_resized_umat if not visualize else debug_frame
    blob = cv2.dnn.blobFromImage(frame_to_process, 0.007843, 
                                 (300, 300), 127.5)
    
    net.setInput(blob)
    detections = net.forward()

    best_target_confidence = -1
    best_target_box = None

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        class_id = int(detections[0, 0, i, 1])
        
        if class_id == CLASS_ID_PERSON and confidence > CONFIDENCE_THRESHOLD:
            if confidence > best_target_confidence:
                best_target_confidence = confidence
                box = detections[0, 0, i, 3:7] * np.array([resized_w, resized_h, resized_w, resized_h])
                best_target_box = box.astype("int")

    # 7. Check if we found a good target
    if best_target_box is not None:
        # --- TARGET FOUND ---
        (x_start, y_start, x_end, y_end) = best_target_box
        x, y, w, h = x_start, y_start, x_end - x_start, y_end - y_start

        # 8. Get Depth Data
        depth_h, depth_w = depth_image_cpu.shape
        depth_scale_x = depth_w / resized_w
        depth_scale_y = depth_h / resized_h
        color_cX = x + w // 2
        
        depth_roi_x = int(x * depth_scale_x)
        depth_roi_y = int(y * depth_scale_y)
        depth_roi_w = int(w * depth_scale_x)
        depth_roi_h = int(h * depth_scale_y)
        
        depth_roi_x = max(0, depth_roi_x)
        depth_roi_y = max(0, depth_roi_y)
        depth_roi_w = min(depth_w - depth_roi_x, depth_roi_w)
        depth_roi_h = min(depth_h - depth_roi_y, depth_roi_h)

        current_dist_ft = None # Default to None

        if depth_roi_w > 0 and depth_roi_h > 0:
            person_roi = depth_image_cpu[depth_roi_y : depth_roi_y + depth_roi_h,
                                         depth_roi_x : depth_roi_x + depth_roi_w]
            
            valid_depths = person_roi[person_roi > 0]
            if valid_depths.size > 0:
                closest_point_mm = np.min(valid_depths)
                new_dist_ft = closest_point_mm * MM_TO_FEET

                # Filter out extremely close, likely erroneous readings
                if new_dist_ft < MIN_PERSON_DISTANCE_FT:
                    # Treat as no valid depth found for this frame
                    last_known_person_dist_ft = None # Reset smoothing if it was active
                    current_dist_ft = None
                else:
                    # Apply smoothing
                    if last_known_person_dist_ft is None:
                        current_dist_ft = new_dist_ft
                    else:
                        current_dist_ft = (new_dist_ft * PERSON_SMOOTHING_FACTOR) + \
                                          (last_known_person_dist_ft * (1.0 - PERSON_SMOOTHING_FACTOR))
                    
                    last_known_person_dist_ft = current_dist_ft
            # If no valid depths, current_dist_ft remains None, and last_known_person_dist_ft retains its last value.
            # This allows smoothing to continue using the last known good value if a frame temporarily loses valid depths.
        # If no ROI, current_dist_ft remains None, and last_known_person_dist_ft retains its last value.
        # This allows smoothing to continue using the last known good value if a frame temporarily loses the ROI.

        if visualize and debug_frame is not None:
            cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            conf_label = f"Person: {best_target_confidence:.2f}"
            cv2.putText(debug_frame, conf_label, (x, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if current_dist_ft is not None:
                dist_label = f"{current_dist_ft:.1f} ft"
                cv2.putText(debug_frame, dist_label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if current_dist_ft is not None and current_dist_ft > 1.0:
            # --- TARGET FOUND ---
            consecutive_target_losses = 0 # Reset loss counter
            last_good_target_state = (current_dist_ft, color_cX, resized_w)
            return last_good_target_state, debug_frame

    # --- TARGET LOST (DNN failed or < 1.0 ft) ---
    consecutive_target_losses += 1

    if consecutive_target_losses < MAX_CONSECUTIVE_LOSSES:
        # Temporarily lost, return the last known good state to maintain stability
        # last_good_target_state already holds the last valid detection
        # last_known_person_dist_ft is also retained for smoothing
        return last_good_target_state, debug_frame
    else:
        # Truly lost the target after exceeding the threshold
        last_good_target_state = (None, None, resized_w)
        last_known_person_dist_ft = None # Reset smoothing only after prolonged loss
        return last_good_target_state, debug_frame

def get_nearest_object_angle():
    """
    Finds the closest point in the depth frame for cruise control.
    Uses Depth frame (Ranging) ONLY.
    
    Returns:
        (float, float, tuple): A tuple of (minimum_depth, angle, (x, y))
                       'minimum_depth' is the depth in **millimeters**.
                       Returns (last_known, last_known, last_known) on *timeout/error*.
                       Returns (None, None, None) if *no object is found*.
    """
    global listener, frames, last_known_depth, last_known_angle, last_known_coords

    if listener is None:
        print("Error: Detector not initialized.")
        return last_known_depth, last_known_angle, last_known_coords
    
    try:
        # 1. Wait for a new frame pair
        if not listener.waitForNewFrame(frames, 10 * 1000): # 10 sec timeout
            print("Kinect timeout in get_nearest_object...", end='\r')
            # On timeout, trust the last known state for safety
            return last_known_depth, last_known_angle, last_known_coords

        # 2. Get depth frame and data
        depth_frame = frames[FrameType.Depth]
        depth_map = depth_frame.asarray()
        
        # 3. Release frames *immediately*
        listener.release(frames)
        
        # 4. Crop logic
        height, width = depth_map.shape # 424, 512
        crop_row_bottom = int(height * (1.0 - CROP_BOTTOM_RATIO))
        crop_col_left = int(width * CROP_LEFT_RATIO)
        crop_col_right = int(width * (1.0 - CROP_RIGHT_RATIO))

        depth_map_roi = depth_map[0:crop_row_bottom, crop_col_left:crop_col_right]
        
        # 5. Find depth
        valid_depths = depth_map_roi[depth_map_roi > 0]
        
        # --- *** MODIFIED FIX *** ---
        # Check if an object was actually found
        if valid_depths.size == 0:
            # NO OBJECT FOUND: This is a valid reading.
            # Reset the state to None so the chair knows it's clear.
            last_known_depth = None
            last_known_angle = None
            last_known_coords = None
            return None, None, None
        # --- *** END OF MODIFICATION *** ---

        # 6. --- OBJECT WAS FOUND ---
        # Calculate new values
        new_depth = np.percentile(valid_depths, 1)

        search_map = np.where(depth_map_roi == 0, 999999, depth_map_roi)
        y_roi, x_roi = np.unravel_index(np.argmin(search_map), search_map.shape)

        x_full_frame = x_roi + crop_col_left
        y_full_frame = y_roi 

        normalized_x = (x_full_frame - (KINECT_WIDTH / 2.0)) / (KINECT_WIDTH / 2.0)
        new_angle = normalized_x * (KINECT_H_FOV / 2.0)

        # 7. Apply smoothing and update the state
        if last_known_depth is None:
            # First detection, just set the values
            last_known_depth = new_depth
            last_known_angle = new_angle
        else:
            # Apply smoothing
            last_known_depth = (new_depth * SMOOTHING_FACTOR) + \
                               (last_known_depth * (1.0 - SMOOTHING_FACTOR))
            last_known_angle = (new_angle * SMOOTHING_FACTOR) + \
                               (last_known_angle * (1.0 - SMOOTHING_FACTOR))

        last_known_coords = (x_full_frame, y_full_frame)
        return last_known_depth, last_known_angle, last_known_coords

    except Exception as e:
        print(f"Error in get_nearest_object_angle: {e}")
        # On a code exception, trust the last state for safety
        return last_known_depth, last_known_angle, last_known_coords

# --- Test Mode ---
if __name__ == "__main__":
    print("Running person_detector.py in VISUAL TEST MODE (with MobileNet-SSD).")
    print("This will test the find_target_person() function.")
    
    if not initialize_detector():
        print("Failed to initialize detector. Exiting.")
        sys.exit(1)
        
    print("\nInitialization complete. Starting test loop...")
    print("Press 'q' in the window to stop.")
    
    window_name = "Person Detector Test"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    
    try:
        while True:
            # Call the function to test
            (dist_ft, center_x, frame_w), debug_frame = find_target_person(visualize=True)
            
            if debug_frame is None:
                time.sleep(0.01) # Frame was skipped
                continue

            # Console print
            if dist_ft is not None:
                print(f"Target Found: {dist_ft:.1f} ft away, at pixel {center_x} (Frame width: {frame_w}) ", end='\r')
            else:
                print("Target Lost... searching...                                       ", end='\r')
                
            # Window display
            cv2.imshow(window_name, debug_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n'q' pressed. Stopping test.")
                break
                
    except KeyboardInterrupt:
        print("\nStopping test...")
    finally:
        cv2.destroyAllWindows()
        shutdown_detector()

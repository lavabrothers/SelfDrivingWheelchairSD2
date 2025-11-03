#!/usr/bin/env python3

# File: kinect_sensor.py
# Version with:
# - Visual OpenCV test mode
# - Bottom-frame cropping (to ignore the floor)
# - Visual crop-line in test mode

from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, FrameMap
import numpy as np
import time
import sys
import cv2 # Used for the visual test mode

# --- Constants ---

# Kinect V2 sensor properties
KINECT_H_FOV = 70.6 # Horizontal Field of View (approx. 70.6 degrees)
KINECT_WIDTH = 512  # Depth map width (512 pixels)

# Smoothing / Damping factor
# 1.0 = no smoothing (raw data)
# 0.1 = very heavy smoothing (slow to react)
SMOOTHING_FACTOR = 0.9

# Ratio of the frame to crop from the bottom (e.g., 0.25 = 25%)
# This is used to ignore the floor.
CROP_BOTTOM_RATIO = 0.05

# Ratios to crop from the left and right (e.g., 0.2 = 20% from each side)
# This is used to narrow the horizontal field of view.
CROP_LEFT_RATIO = 0.2
CROP_RIGHT_RATIO = 0.2

# --- Global variables for the device ---
freenect2 = None
device = None
listener = None
serial = ""

# --- State variables for smoothing ---
last_known_depth = None
last_known_angle = None
last_known_coords = None # Store last coords for visualization

def initialize_kinect():
    """
    Initializes and starts the Kinect V2 sensor.
    This is now a stateful operation.
    """
    global freenect2, device, listener, serial
    try:
        freenect2 = Freenect2()
        num_devices = freenect2.enumerateDevices()
        if num_devices == 0:
            print("Error: No Kinect V2 devices found!")
            return False
            
        serial = freenect2.getDeviceSerialNumber(0)
        print(f"Kinect V2 found with serial: {serial}")
        
        device = freenect2.openDevice(serial)
        
        # Request both Color and Depth
        types = FrameType.Color | FrameType.Depth
        listener = SyncMultiFrameListener(types)
        
        device.setColorFrameListener(listener)
        device.setIrAndDepthFrameListener(listener)
        
        print("Starting Kinect V2 stream...")
        device.start()
        
        print("Kinect V2 sensor initialized and started.")
        return True
        
    except Exception as e:
        print(f"Error initializing Kinect V2: {e}")
        if "LIBUSB_ERROR_ACCESS" in str(e):
             print("\n--- PERMISSION ERROR ---")
             print("This is likely a USB permission issue.")
             print("You may need to set up udev rules for the Kinect.")
        return False

def get_nearest_object_angle(depth_frame_obj=None):
    """
    Gets a depth frame and finds the closest point.
    
    Can optionally be passed a 'depth_frame_obj' to process,
    otherwise it will grab its own.
    
    Returns:
        (float, float, tuple): A tuple of (minimum_depth, angle, (x, y))
                       'minimum_depth' is the depth in **millimeters**.
                       'angle' is the horizontal angle in degrees from center.
                       '(x, y)' are the pixel coordinates of the detected point.
                       Returns (last_known, last_known, last_known) on failure.
    """
    global device, listener, last_known_depth, last_known_angle, last_known_coords
    
    if listener is None and depth_frame_obj is None:
        print("Error: Kinect V2 not initialized.")
        return last_known_depth, last_known_angle, last_known_coords

    try:
        # Frame grabbing logic
        internal_frame_grab = False
        frames = None
        
        if depth_frame_obj is None:
            frames = FrameMap()
            if not listener.waitForNewFrame(frames, 10 * 1000):
                print("Timeout waiting for new frames. Returning last known value.")
                listener.release(frames)
                return last_known_depth, last_known_angle, last_known_coords
            
            depth_frame = frames[FrameType.Depth]
            internal_frame_grab = True
        else:
            depth_frame = depth_frame_obj

        depth_map = depth_frame.asarray()
        
        # If we grabbed our own frame, release it now.
        if internal_frame_grab:
            listener.release(frames)
        
        # --- CROP THE BOTTOM OF THE FRAME TO IGNORE FLOOR ---
        height = depth_map.shape[0] # Should be 424
        # Calculate the row index to crop *at*. (e.g., 424 * (1.0 - 0.25) = 318)
        crop_row_bottom = int(height * (1.0 - CROP_BOTTOM_RATIO))
        
        # Calculate column indices for horizontal cropping
        width = depth_map.shape[1] # Should be 512
        crop_col_left = int(width * CROP_LEFT_RATIO)
        crop_col_right = int(width * (1.0 - CROP_RIGHT_RATIO))

        # Create the Region of Interest (ROI) by slicing
        # Apply both vertical (bottom) and horizontal (left/right) cropping
        depth_map_roi = depth_map[0:crop_row_bottom, crop_col_left:crop_col_right]
        # --- END CROP ---
        
        # --- Use the ROI for all calculations ---
        valid_depths = depth_map_roi[depth_map_roi > 0]
        
        if valid_depths.size == 0:
            # No valid depths in the ROI
            return last_known_depth, last_known_angle, last_known_coords

        new_depth = np.percentile(valid_depths, 1)
        
        # Search for the minimum in the ROI
        search_map = np.where(depth_map_roi == 0, 999999, depth_map_roi)
        # y, x coords will be relative to the ROI
        y_roi, x_roi = np.unravel_index(np.argmin(search_map), search_map.shape)

        # Convert ROI coordinates back to full frame coordinates for angle calculation
        x_full_frame = x_roi + crop_col_left
        y_full_frame = y_roi # y_roi is already relative to the top of the frame, not the cropped bottom

        # Angle calculation now uses the full frame width for normalization,
        # but the detected point is within the cropped horizontal region.
        normalized_x = (x_full_frame - (KINECT_WIDTH / 2.0)) / (KINECT_WIDTH / 2.0)
        new_angle = normalized_x * (KINECT_H_FOV / 2.0)
        
        # Apply smoothing
        if last_known_depth is None:
            last_known_depth = new_depth
            last_known_angle = new_angle
        else:
            last_known_depth = (new_depth * SMOOTHING_FACTOR) + \
                               (last_known_depth * (1.0 - SMOOTHING_FACTOR))
            last_known_angle = (new_angle * SMOOTHING_FACTOR) + \
                               (last_known_angle * (1.0 - SMOOTHING_FACTOR))
        
        # Store coords and return all values
        # The (x, y) coords are correct for visualization, as 'y'
        # will be in the non-cropped section of the frame.
        last_known_coords = (x_full_frame, y_full_frame)
        return last_known_depth, last_known_angle, last_known_coords

    except Exception as e:
        print(f"Error in get_nearest_object_angle: {e}")
        return last_known_depth, last_known_angle, last_known_coords

def shutdown_kinect():
    """
    Stops and closes the Kinect V2 device.
    This is CRITICAL to run on exit.
    """
    global device
    print("Shutting down Kinect V2...")
    if device:
        try:
            device.stop()
            device.close()
            print("Kinect V2 shut down successfully.")
        except Exception as e:
            print(f"Error during Kinect V2 shutdown: {e}")
    

if __name__ == "__main__":
    
    print("Initializing Kinect V2 for VISUAL test...")
    if initialize_kinect():
        print("Testing Kinect V2. Point it at something.")
        print(f"Ignoring bottom {CROP_BOTTOM_RATIO*100:.0f}% of the view (floor).")
        print(f"Ignoring left {CROP_LEFT_RATIO*100:.0f}% and right {CROP_RIGHT_RATIO*100:.0f}% of the view (sides).")
        print("Press 'q' in the window to stop.")
        
        try:
            while True:
                # 1. Grab the frame
                frames = FrameMap()
                if not listener.waitForNewFrame(frames, 10 * 1000):
                    print("Timeout!")
                    continue

                depth_frame = frames[FrameType.Depth]
                
                # 2. Get the raw depth map for drawing
                depth_map = depth_frame.asarray()

                # 3. Call the function to get the *processed* data
                depth, angle, coords = get_nearest_object_angle(depth_frame_obj=depth_frame)

                # 4. Release the frame *now* that it's been processed
                listener.release(frames)

                # 5. Normalize the depth map for display (0-4500mm)
                depth_viz = np.clip(depth_map, 500, 4500) # Clip 0.5m to 4.5m
                depth_viz = (depth_viz - 500) / (4000.0)  # Remap to 0-1
                depth_viz = (255 * (1.0 - depth_viz)).astype(np.uint8) # Invert and scale
                
                # Make "no-reading" (0) pixels black
                depth_viz[depth_map == 0] = 0

                # 6. Convert to color to draw on it
                image_color = cv2.cvtColor(depth_viz, cv2.COLOR_GRAY2BGR)

                # 7. Draw the crop lines for visualization
                viz_height = image_color.shape[0]
                viz_width = image_color.shape[1]

                crop_row_bottom_viz = int(viz_height * (1.0 - CROP_BOTTOM_RATIO))
                crop_col_left_viz = int(viz_width * CROP_LEFT_RATIO)
                crop_col_right_viz = int(viz_width * (1.0 - CROP_RIGHT_RATIO))
                
                # Draw a line to show the bottom cutoff
                cv2.line(image_color, (0, crop_row_bottom_viz), (viz_width - 1, crop_row_bottom_viz), 
                         (255, 255, 0), 1) # Cyan line
                
                # Draw lines to show the left and right cutoffs
                cv2.line(image_color, (crop_col_left_viz, 0), (crop_col_left_viz, viz_height - 1),
                         (255, 255, 0), 1) # Cyan line
                cv2.line(image_color, (crop_col_right_viz, 0), (crop_col_right_viz, viz_height - 1),
                         (255, 255, 0), 1) # Cyan line
                
                # Add text label for the ignored areas
                cv2.putText(image_color, "IGNORING THIS AREA (FLOOR)", 
                            (10, crop_row_bottom_viz + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.putText(image_color, "IGNORING SIDES", 
                            (crop_col_left_viz + 5, 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.putText(image_color, "IGNORING SIDES", 
                            (crop_col_right_viz - 150, 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)


                # 8. Draw the circle and text if an object was found
                if coords is not None and depth is not None:
                    x, y = coords
                    # Check if coords are valid (should be within the new cropped area)
                    if y < crop_row_bottom_viz and x > crop_col_left_viz and x < crop_col_right_viz:
                        cv2.circle(image_color, (x, y), 10, (0, 0, 255), 2) # Red circle
                        
                        text = f"{depth/1000.0:.2f} m, {angle:.1f} deg"
                        cv2.putText(image_color, text, (x + 15, y + 5), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                # 9. Show the image
                cv2.imshow("Kinect Depth Test", image_color)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            print("\nStopping test...")
        finally:
            # Close the OpenCV window
            cv2.destroyAllWindows()
            shutdown_kinect()
    else:
        print("Kinect V2 initialization failed. Exiting.")

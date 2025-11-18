"""
kinectcloseobject.py

This module provides an interface for interacting with the Kinect V2 sensor to detect
the nearest object and its horizontal angle relative to the sensor's center.
It incorporates frame cropping to ignore irrelevant areas (like the floor) and
applies smoothing to the depth and angle measurements for more stable readings.

The module is designed to be used in applications requiring real-time obstacle
detection and spatial awareness, such as autonomous navigation systems.
It includes a visual test mode using OpenCV for live visualization of depth data,
cropping regions, and detected object location.

Dependencies:
- pylibfreenect2: Python wrapper for libfreenect2, used to interface with the Kinect V2.
- numpy: For numerical operations, especially array manipulation of depth data.
- cv2 (OpenCV): Used for visualization in the test mode.
"""

from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType, FrameMap
import numpy as np
import time
import sys
import cv2 # Used for the visual test mode

# --- Constants ---

# Kinect V2 sensor properties
KINECT_H_FOV = 70.6 # Horizontal Field of View of the Kinect V2 depth sensor in degrees.
KINECT_WIDTH = 512  # Width of the depth map in pixels.

# Smoothing / Damping factor for depth and angle measurements.
# A value of 1.0 means no smoothing (raw data).
# A value closer to 0.0 means heavier smoothing (slower to react to changes).
SMOOTHING_FACTOR = 0.9

# Ratio of the frame to crop from the bottom (e.g., 0.05 = 5%).
# This is primarily used to ignore the floor, preventing it from being detected as an obstacle.
CROP_BOTTOM_RATIO = 0.05

# Ratios to crop from the left and right sides of the frame (e.g., 0.2 = 20% from each side).
# This narrows the horizontal field of view, focusing on the central path.
CROP_LEFT_RATIO = 0.2
CROP_RIGHT_RATIO = 0.2

# --- Global variables for the Kinect device and listener ---
freenect2 = None    # Freenect2 object for managing Kinect devices.
device = None       # Represents the opened Kinect V2 device.
listener = None     # Listener for receiving frames from the Kinect.
serial = ""         # Serial number of the connected Kinect device.

# --- State variables for smoothing ---
last_known_depth = None     # Stores the last smoothed minimum depth value.
last_known_angle = None     # Stores the last smoothed horizontal angle.
last_known_coords = None    # Stores the last known pixel coordinates of the detected object for visualization.

def initialize_kinect() -> bool:
    """
    Initializes and starts the Kinect V2 sensor.

    This function sets up the Freenect2 context, enumerates devices, opens the
    first detected Kinect, and configures frame listeners for both color and depth
    streams. It then starts the Kinect V2 stream.

    Returns:
        bool: True if the Kinect V2 sensor was successfully initialized and started,
              False otherwise.
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
        
        # Request both Color and Depth frames for processing.
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

def get_nearest_object_angle(depth_frame_obj=None) -> tuple[float | None, float | None, tuple[int, int] | None]:
    """
    Retrieves a depth frame (or uses a provided one) and calculates the minimum
    depth and its corresponding horizontal angle from the center of the frame.
    It applies cropping to ignore the floor and side regions, and smooths the
    output values.

    Args:
        depth_frame_obj (Frame | None): An optional `Frame` object containing depth data.
                                        If None, the function will wait for a new frame
                                        from the Kinect listener.

    Returns:
        tuple: A tuple containing:
               - minimum_depth (float | None): The smoothed minimum depth in millimeters.
               - angle (float | None): The smoothed horizontal angle in degrees from the center.
               - (x, y) (tuple[int, int] | None): The pixel coordinates of the detected point
                                                  in the full frame.
               Returns (last_known_depth, last_known_angle, last_known_coords) on failure
               or timeout, or (None, None, None) if no previous data exists.
    """
    global device, listener, last_known_depth, last_known_angle, last_known_coords
    
    if listener is None and depth_frame_obj is None:
        print("Error: Kinect V2 not initialized.")
        return last_known_depth, last_known_angle, last_known_coords

    try:
        internal_frame_grab = False
        frames = None
        
        # If no depth frame is provided, grab a new one from the listener.
        if depth_frame_obj is None:
            frames = FrameMap()
            if not listener.waitForNewFrame(frames, 10 * 1000): # Wait up to 10 seconds.
                print("Timeout waiting for new frames. Returning last known value.")
                listener.release(frames)
                return last_known_depth, last_known_angle, last_known_coords
            
            depth_frame = frames[FrameType.Depth]
            internal_frame_grab = True
        else:
            depth_frame = depth_frame_obj

        depth_map = depth_frame.asarray()
        
        # Release the frame if it was grabbed internally to prevent resource leaks.
        if internal_frame_grab:
            listener.release(frames)
        
        # --- CROP THE FRAME TO IGNORE FLOOR AND SIDES ---
        height = depth_map.shape[0] # Expected 424 pixels.
        width = depth_map.shape[1]  # Expected 512 pixels.

        # Calculate the row index to crop from the bottom.
        crop_row_bottom = int(height * (1.0 - CROP_BOTTOM_RATIO))
        
        # Calculate column indices for horizontal cropping.
        crop_col_left = int(width * CROP_LEFT_RATIO)
        crop_col_right = int(width * (1.0 - CROP_RIGHT_RATIO))

        # Create the Region of Interest (ROI) by slicing the depth map.
        # This applies both vertical (bottom) and horizontal (left/right) cropping.
        depth_map_roi = depth_map[0:crop_row_bottom, crop_col_left:crop_col_right]
        # --- END CROP ---
        
        # --- Process the ROI for minimum depth and angle ---
        # Filter out zero (invalid) depth readings from the ROI.
        valid_depths = depth_map_roi[depth_map_roi > 0]
        
        if valid_depths.size == 0:
            # No valid depths found in the ROI, return last known values.
            return last_known_depth, last_known_angle, last_known_coords

        # Use the 1st percentile to find a robust minimum depth, less sensitive to outliers.
        new_depth = np.percentile(valid_depths, 1)
        
        # Find the pixel coordinates (y, x) of the minimum depth within the ROI.
        # Replace 0s with a large number to ensure argmin finds actual objects.
        search_map = np.where(depth_map_roi == 0, 999999, depth_map_roi)
        y_roi, x_roi = np.unravel_index(np.argmin(search_map), search_map.shape)

        # Convert ROI coordinates back to full frame coordinates for accurate angle calculation.
        x_full_frame = x_roi + crop_col_left
        y_full_frame = y_roi # Y-coordinate is relative to the top of the frame, not the cropped bottom.

        # Calculate the horizontal angle from the center of the full frame.
        # Normalized_x ranges from -1 (far left) to 1 (far right).
        normalized_x = (x_full_frame - (KINECT_WIDTH / 2.0)) / (KINECT_WIDTH / 2.0)
        new_angle = normalized_x * (KINECT_H_FOV / 2.0)
        
        # Apply exponential smoothing to depth and angle values.
        if last_known_depth is None:
            last_known_depth = new_depth
            last_known_angle = new_angle
        else:
            last_known_depth = (new_depth * SMOOTHING_FACTOR) + \
                               (last_known_depth * (1.0 - SMOOTHING_FACTOR))
            last_known_angle = (new_angle * SMOOTHING_FACTOR) + \
                               (last_known_angle * (1.0 - SMOOTHING_FACTOR))
        
        # Store and return the smoothed values along with the full frame coordinates.
        last_known_coords = (x_full_frame, y_full_frame)
        return last_known_depth, last_known_angle, last_known_coords

    except Exception as e:
        print(f"Error in get_nearest_object_angle: {e}")
        return last_known_depth, last_known_angle, last_known_coords

def shutdown_kinect():
    """
    Stops and closes the Kinect V2 device.

    This function is critical for releasing hardware resources and should always
    be called when the Kinect is no longer needed to prevent resource leaks or
    issues with subsequent sensor initialization.
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
    """
    Main execution block for visual test mode.

    This block initializes the Kinect, continuously captures depth frames,
    processes them to find the nearest object and its angle, and visualizes
    the depth map with cropping lines and the detected object using OpenCV.
    Press 'q' in the display window to exit.
    """
    print("Initializing Kinect V2 for VISUAL test...")
    if initialize_kinect():
        print("Testing Kinect V2. Point it at something.")
        print(f"Ignoring bottom {CROP_BOTTOM_RATIO*100:.0f}% of the view (floor).")
        print(f"Ignoring left {CROP_LEFT_RATIO*100:.0f}% and right {CROP_RIGHT_RATIO*100:.0f}% of the view (sides).")
        print("Press 'q' in the window to stop.")
        
        try:
            while True:
                # 1. Grab a new frame from the Kinect listener.
                frames = FrameMap()
                if not listener.waitForNewFrame(frames, 10 * 1000):
                    print("Timeout waiting for new frames!")
                    continue

                depth_frame = frames[FrameType.Depth]
                
                # 2. Get the raw depth map for drawing purposes.
                depth_map = depth_frame.asarray()

                # 3. Call the function to get the *processed* data (min depth, angle, coords).
                depth, angle, coords = get_nearest_object_angle(depth_frame_obj=depth_frame)

                # 4. Release the frame immediately after processing to free resources.
                listener.release(frames)

                # 5. Normalize the depth map for display (clipping and remapping for better visualization).
                depth_viz = np.clip(depth_map, 500, 4500) # Clip depths between 0.5m and 4.5m.
                depth_viz = (depth_viz - 500) / (4000.0)  # Remap to 0-1 range.
                depth_viz = (255 * (1.0 - depth_viz)).astype(np.uint8) # Invert colors (closer = brighter) and scale to 0-255.
                
                # Make "no-reading" (0) pixels black for clarity.
                depth_viz[depth_map == 0] = 0

                # 6. Convert the grayscale depth visualization to a 3-channel color image for drawing.
                image_color = cv2.cvtColor(depth_viz, cv2.COLOR_GRAY2BGR)

                # 7. Draw the crop lines for visualization on the color image.
                viz_height = image_color.shape[0]
                viz_width = image_color.shape[1]

                crop_row_bottom_viz = int(viz_height * (1.0 - CROP_BOTTOM_RATIO))
                crop_col_left_viz = int(viz_width * CROP_LEFT_RATIO)
                crop_col_right_viz = int(viz_width * (1.0 - CROP_RIGHT_RATIO))
                
                # Draw a cyan line to indicate the bottom cutoff.
                cv2.line(image_color, (0, crop_row_bottom_viz), (viz_width - 1, crop_row_bottom_viz), 
                         (255, 255, 0), 1) 
                
                # Draw cyan lines to indicate the left and right cutoffs.
                cv2.line(image_color, (crop_col_left_viz, 0), (crop_col_left_viz, viz_height - 1),
                         (255, 255, 0), 1) 
                cv2.line(image_color, (crop_col_right_viz, 0), (crop_col_right_viz, viz_height - 1),
                         (255, 255, 0), 1) 
                
                # Add text labels for the ignored areas.
                cv2.putText(image_color, "IGNORING THIS AREA (FLOOR)", 
                            (10, crop_row_bottom_viz + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.putText(image_color, "IGNORING SIDES", 
                            (crop_col_left_viz + 5, 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                cv2.putText(image_color, "IGNORING SIDES", 
                            (crop_col_right_viz - 150, 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)


                # 8. Draw a circle and text if an object was found and its coordinates are valid.
                if coords is not None and depth is not None:
                    x, y = coords
                    # Ensure the detected point is within the *visible* (non-cropped) area.
                    if y < crop_row_bottom_viz and x > crop_col_left_viz and x < crop_col_right_viz:
                        cv2.circle(image_color, (x, y), 10, (0, 0, 255), 2) # Red circle at detected object.
                        
                        text = f"{depth/1000.0:.2f} m, {angle:.1f} deg"
                        cv2.putText(image_color, text, (x + 15, y + 5), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                # 9. Display the processed image.
                cv2.imshow("Kinect Depth Test", image_color)

                # Wait for 'q' key press to quit.
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            print("\nStopping test due to KeyboardInterrupt...")
        finally:
            # Close all OpenCV windows and shut down the Kinect sensor.
            cv2.destroyAllWindows()
            shutdown_kinect()
    else:
        print("Kinect V2 initialization failed. Exiting.")

#!/usr/bin/env python3

# File: visual_main_flow.py
# Main asynchronous control flow for the autonomous wheelchair with visualization.
# Integrates Bluetooth commands with different autonomous modes and displays video feeds.

import asyncio
import time
from enum import Enum
from bluetooth_module import BluetoothModule
import wheelchair_control as wc
import person_detector as vision
import mapping_module as mapping
import cv2
import numpy as np

# --- Constants ---
ADC_MAX = 4095.0
DEBUG_PRINT = False 

# --- Cruise Mode Constants ---
CRUISE_STOP_DISTANCE_MM = 1500
CRUISE_SPEED = 0.3

# --- Follow Mode Constants ---
FOLLOW_TARGET_DISTANCE_FT = 3.0
FOLLOW_DEAD_ZONE_FT = 0.2
FOLLOW_MOVE_SPEED = 0.3
FOLLOW_TURN_SPEED = 0.4
FOLLOW_TRACKING_DEAD_ZONE_PX = 30
FOLLOW_TARGET_LOST_TIMEOUT_S = 3.0
MM_TO_FEET = 0.00328084

# --- Reconnection Constants ---
RECONNECT_DELAY_S = 2

# --- State Management ---
class ControlState(Enum):
    MANUAL = 1
    CRUISE = 2
    STOPPED = 3
    FOLLOW = 4
    MAPPING = 5

current_state = ControlState.STOPPED
last_command_time = time.time()

# --- State variables for follow mode ---
last_person_detection_time = 0.0
last_person_turn_direction = 0.0

# --- Visualization Global ---
current_visual_frame = np.zeros((480, 640, 3), dtype=np.uint8) 
cv2.putText(current_visual_frame, "Initializing...", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2) 


def handle_incoming_data(data: bytes):
    """
    Parses incoming data and updates the control state.
    """
    global current_state, last_command_time
    
    try:
        message = data.decode().strip()
    except UnicodeDecodeError:
        print("Received non-UTF8 data. Ignoring.")
        return
        
    last_command_time = time.time()
    
    if message == "Cruise":
        if current_state != ControlState.CRUISE:
            print("Switching to CRUISE mode.")
            wc.stop() 
            current_state = ControlState.CRUISE
    elif message == "Follow":
        if current_state != ControlState.FOLLOW:
            print("Switching to FOLLOW mode.")
            wc.stop() 
            current_state = ControlState.FOLLOW
            global last_person_detection_time, last_person_turn_direction
            last_person_detection_time = 0.0
            last_person_turn_direction = 0.0
    elif message == "Map":
        if current_state != ControlState.MAPPING:
            print("Switching to MAPPING mode.")
            wc.stop() 
            current_state = ControlState.MAPPING
    elif message == "STOP":
        if current_state != ControlState.STOPPED:
            print("Switching to STOPPED mode.")
            wc.stop() 
            current_state = ControlState.STOPPED
    else:
        if current_state != ControlState.MANUAL:
            print("Switching to MANUAL mode.")
            current_state = ControlState.MANUAL
        
        try:
            parts = [int(p) for p in message.split(',')]
            if len(parts) == 4:
                fwd = parts[0] / ADC_MAX
                bwd = parts[1] / ADC_MAX
                left = parts[2] / ADC_MAX
                right = parts[3] / ADC_MAX
                wc.set_joystick_values(fwd, bwd, right, left)
        except (ValueError, IndexError):
            print(f"Could not parse joystick data: {message}")

async def cruise_control_loop():
    """The main logic for cruise control mode."""
    global current_visual_frame 
    print("Cruise control loop started.")
    while True:
        if current_state == ControlState.CRUISE:
            
            (depth, angle, _), debug_frame = await asyncio.to_thread(vision.get_nearest_object_angle, visualize=True)
            
            if current_state != ControlState.CRUISE:
                continue

            if debug_frame is not None:
                current_visual_frame = debug_frame 
            
            if depth is not None and depth < CRUISE_STOP_DISTANCE_MM:
                wc.stop()
                if DEBUG_PRINT: print(f"Object detected at {depth/1000.0:.2f}m. Stopping.")
            else:
                wc.set_movement(CRUISE_SPEED, 0.0)
                if DEBUG_PRINT: print(f"Cruising forward. Nearest object > {CRUISE_STOP_DISTANCE_MM/1000.0:.1f}m", end='\r')
        
        await asyncio.sleep(0.1 if current_state == ControlState.CRUISE else 0.5)

async def follow_person_loop():
    """The main logic for person-following mode."""
    global last_person_detection_time, last_person_turn_direction
    global current_visual_frame 
    print("Follow person loop started.")
    
    upper_bound_ft = FOLLOW_TARGET_DISTANCE_FT + FOLLOW_DEAD_ZONE_FT
    lower_bound_ft = FOLLOW_TARGET_DISTANCE_FT - FOLLOW_DEAD_ZONE_FT
    
    while True:
        if current_state == ControlState.FOLLOW:
            
            (dist_ft, center_x, frame_w), debug_frame = await asyncio.to_thread(vision.find_target_person, visualize=True)

            if current_state != ControlState.FOLLOW:
                continue

            if debug_frame is None:
                await asyncio.sleep(0.05 if current_state == ControlState.FOLLOW else 0.5)
                continue

            current_visual_frame = debug_frame
            
            status_text = ""
            status_color = (0, 0, 255) # Red for lost

            if dist_ft is not None:
                last_person_detection_time = time.monotonic()
                
                fwd_bwd_speed = 0.0
                if dist_ft > upper_bound_ft:
                    fwd_bwd_speed = FOLLOW_MOVE_SPEED
                    status_dist = "MOVING FWD"
                elif dist_ft < lower_bound_ft:
                    fwd_bwd_speed = -FOLLOW_MOVE_SPEED
                    status_dist = "TOO CLOSE (BWD)"
                else:
                    fwd_bwd_speed = 0.0
                    status_dist = "IN ZONE"
                
                left_right_speed = 0.0
                frame_center_x = frame_w // 2
                left_bound = frame_center_x - FOLLOW_TRACKING_DEAD_ZONE_PX
                right_bound = frame_center_x + FOLLOW_TRACKING_DEAD_ZONE_PX

                if center_x < left_bound:
                    left_right_speed = FOLLOW_TURN_SPEED
                    status_turn = "TURN LEFT"
                elif center_x > right_bound:
                    left_right_speed = -FOLLOW_TURN_SPEED
                    status_turn = "TURN RIGHT"
                else:
                    left_right_speed = 0.0
                    status_turn = "CENTERED"
                
                last_person_turn_direction = left_right_speed
                wc.set_movement(fwd_bwd_speed, left_right_speed)
                
                if DEBUG_PRINT: print(f"Follow: {dist_ft:.1f}ft ({status_dist}) | Turn: {status_turn} ✅ ", end='\r')

            else:
                time_since_last_seen = time.monotonic() - last_person_detection_time
                
                if last_person_detection_time == 0.0:
                    status_text = "SEARCHING..."
                    if DEBUG_PRINT: print(f"Follow: SEARCHING... ❌                                 ", end='\r')
                
                elif time_since_last_seen < FOLLOW_TARGET_LOST_TIMEOUT_S:
                    wc.set_movement(0.0, last_person_turn_direction)
                    status_text = "RE-ACQUIRING..."
                    status_color = (0, 255, 255) # Yellow
                    if DEBUG_PRINT: print(f"Follow: RE-ACQUIRING... ❓                              ", end='\r')
                
                else:
                    wc.stop()
                    status_text = "TARGET LOST"
                    if DEBUG_PRINT: print(f"Follow: TARGET LOST. STOPPING. ❌                       ", end='\r')

            if status_text:
                cv2.putText(current_visual_frame, status_text, 
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 
                            1, status_color, 2)
        
        await asyncio.sleep(0.05 if current_state == ControlState.FOLLOW else 0.5) 

async def mapping_loop():
    """The main logic for mapping mode."""
    global current_state, current_visual_frame 
    print("Mapping loop started.")
    
    mapping_screen = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(mapping_screen, "Mapping in Progress...", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    while True:
        if current_state == ControlState.MAPPING:
            print("Starting 360-degree mapping scan...")
            current_visual_frame = mapping_screen 
            await asyncio.to_thread(mapping.perform_mapping)
            print("Mapping scan complete. Switching to STOPPED mode.")
            current_state = ControlState.STOPPED
            wc.stop()
        
        await asyncio.sleep(0.5 if current_state == ControlState.MAPPING else 0.5)

async def visual_flow_loop():
    """Displays the appropriate video feed based on the current state."""
    global current_visual_frame 
    window_name = "Visual Flow"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    
    stopped_screen = np.zeros((480, 640, 3), dtype=np.uint8)
    stopped_text = "Current State: STOPPED"
    (text_width, _), _ = cv2.getTextSize(stopped_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
    text_x = (640 - text_width) // 2
    cv2.putText(stopped_screen, stopped_text, (text_x, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    manual_screen = np.zeros((480, 640, 3), dtype=np.uint8)
    manual_text = "Current State: MANUAL"
    (text_width, _), _ = cv2.getTextSize(manual_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
    text_x = (640 - text_width) // 2
    cv2.putText(manual_screen, manual_text, (text_x, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)


    while True:
        if current_state == ControlState.CRUISE or current_state == ControlState.FOLLOW:
            cv2.imshow(window_name, current_visual_frame)
        elif current_state == ControlState.MANUAL:
            cv2.imshow(window_name, manual_screen)
        elif current_state == ControlState.MAPPING:
            cv2.imshow(window_name, current_visual_frame)
        else:
            cv2.imshow(window_name, stopped_screen)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("'q' pressed. Shutting down...")
            # <--- CHANGED: Use get_running_loop() ---
            # This tells the 'while loop.is_running()' in main() to stop
            asyncio.get_running_loop().stop()
            break
        
        await asyncio.sleep(0.03) # ~30 FPS

async def main():
    """Main asynchronous function for the wheelchair control flow."""
    global last_command_time, current_state
    
    dac_initialized = False
    bt_connected = False
    vision_initialized = False
    mapping_initialized = False
    
    cruise_task = None
    follow_task = None
    mapping_task = None
    visual_task = None
    bt_module = None

    try:
        # 1. Initialize DAC
        if not wc.initialize_dac():
            print("Exiting program: DAC initialization failed.")
            return
        dac_initialized = True
    
        # 2. Connect to Bluetooth (Lightweight I/O, before Kinect)
        bt_module = BluetoothModule()
        if not await bt_module.connect():
            print("Exiting program: Bluetooth connection failed.")
            return
        bt_connected = True
        await bt_module.start_listening(handle_incoming_data)

        # 3. Initialize Vision (Heavyweight I/O)
        if not vision.initialize_detector():
            print("Exiting program: Vision module initialization failed.")
            return
        vision_initialized = True
        
        # 4. Initialize Mapping (Depends on Vision module)
        if not mapping.initialize():
            print("Exiting program: Mapping module initialization failed.")
            return
        mapping_initialized = True


        print("All systems initialized. Main loop started. Press 'q' or Ctrl+C to exit.")
        cruise_task = asyncio.create_task(cruise_control_loop())
        follow_task = asyncio.create_task(follow_person_loop())
        mapping_task = asyncio.create_task(mapping_loop())
        visual_task = asyncio.create_task(visual_flow_loop())

        # <--- CHANGED: Get the running loop ---
        loop = asyncio.get_running_loop() 
        while not loop.is_running():
             await asyncio.sleep(0.01)
             
        # <--- This loop now exits cleanly on 'q' or Ctrl+C ---
        while loop.is_running():
            if not bt_module.client.is_connected:
                print("\nBluetooth disconnected. Attempting to reconnect...")
                current_state = ControlState.STOPPED
                wc.stop()
                
                reconnected = False
                while not reconnected and loop.is_running():
                    print(f"Trying to reconnect in {RECONNECT_DELAY_S} seconds...")
                    try:
                        await asyncio.sleep(RECONNECT_DELAY_S)
                    except asyncio.CancelledError:
                        break 
                    
                    if not loop.is_running():
                        break

                    if await bt_module.connect():
                        print("\nBluetooth reconnected successfully!")
                        await bt_module.start_listening(handle_incoming_data)
                        reconnected = True
                    else:
                        print("Reconnection failed. Retrying...")
                
                if not loop.is_running():
                    break
                
                continue

            time_since_cmd = time.time() - last_command_time
            if current_state == ControlState.MANUAL and time_since_cmd > 1.0:
                 print("\nNo MANUAL command received for 1s. Switching to STOPPED for safety.")
                 current_state = ControlState.STOPPED
                 wc.stop()
            
            # This sleep is for the main safety loop.
            # We need to handle the loop being stopped.
            try:
                await asyncio.sleep(0.5)
            except asyncio.CancelledError:
                break # Exit loop if main is cancelled
            
    except asyncio.CancelledError:
        print("Main loop cancelled.")
    finally:
        # --- THIS BLOCK WILL NOW RUN ON CTRL+C ---
        print("\nCleaning up resources...")
        
        if cruise_task: cruise_task.cancel()
        if follow_task: follow_task.cancel()
        if mapping_task: mapping_task.cancel()
        if visual_task: visual_task.cancel()
        
        await asyncio.gather(
            cruise_task if cruise_task else asyncio.sleep(0), 
            follow_task if follow_task else asyncio.sleep(0), 
            mapping_task if mapping_task else asyncio.sleep(0), 
            visual_task if visual_task else asyncio.sleep(0),
            return_exceptions=True
        )
        
        if bt_connected and bt_module:
            await bt_module.disconnect()
        if vision_initialized:
            vision.shutdown_detector()
        if mapping_initialized:
            mapping.shutdown()
        if dac_initialized:
            wc.stop()
            
        cv2.destroyAllWindows()
        print("Program cleaned up and exited.")

# <--- CHANGED: Switched to asyncio.run() for robust Ctrl+C handling ---
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    finally:
        print("Exiting.")
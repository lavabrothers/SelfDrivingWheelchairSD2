#!/usr/bin/env python3

# File: controllerless_main_flow.py
# Main asynchronous control flow for the autonomous wheelchair.
# Integrates Terminal commands with different autonomous modes.

import asyncio
import time
from enum import Enum
import threading
import wheelchair_control as wc
import person_detector as vision
import mapping_module as mapping
import audio_feedback as beeper

# --- Constants ---
ADC_MAX = 4095.0

# --- Cruise Mode Constants ---
CRUISE_STOP_DISTANCE_MM = 1750
CRUISE_SPEED = 0.3

# --- Follow Mode Constants ---
FOLLOW_TARGET_DISTANCE_FT = 3.5
FOLLOW_DEAD_ZONE_FT = 0.3
FOLLOW_MOVE_SPEED = 0.25
FOLLOW_TURN_SPEED = 0.35
FOLLOW_TRACKING_DEAD_ZONE_PX = 30
FOLLOW_TARGET_LOST_TIMEOUT_S = 3.0
FOLLOW_STOP_DISTANCE_MM = 1250  # Stop 1.25m from obstacles
MM_TO_FEET = 0.00328084

# --- State Management ---
class ControlState(Enum):
    MANUAL = 1
    CRUISE = 2
    STOPPED = 3
    FOLLOW = 4
    MAP = 5

current_state = ControlState.STOPPED
last_command_time = time.time()
mapping_stop_event = threading.Event() # For interruptible mapping

# --- State variables for follow mode ---
last_person_detection_time = 0.0
last_person_turn_direction = 0.0


def handle_incoming_data(message: str):
    """
    Parses incoming string commands and updates the control state.
    """
    global current_state, last_command_time, mapping_stop_event
    
    message = message.strip()
    if not message:
        return
        
    last_command_time = time.time()
    wc.stop() 

    if message == "Cruise":
        if current_state != ControlState.CRUISE:
            print("Switching to CRUISE mode.")
            current_state = ControlState.CRUISE
            asyncio.create_task(beeper.play_beep(3))
    elif message == "Follow":
        if current_state != ControlState.FOLLOW:
            print("Switching to FOLLOW mode.")
            current_state = ControlState.FOLLOW
            asyncio.create_task(beeper.play_beep(4))
            global last_person_detection_time, last_person_turn_direction
            last_person_detection_time = 0.0
            last_person_turn_direction = 0.0
    elif message == "MAP":
        if current_state != ControlState.MAP:
            print("Switching to MAP mode.")
            current_state = ControlState.MAP
            asyncio.create_task(beeper.play_beep(5))
            
    elif message == "STOP":
        # Check if we are mapping and tell it to stop
        if current_state == ControlState.MAP:
            print("Requesting mapping task to stop...")
            mapping_stop_event.set()
            
        if current_state != ControlState.STOPPED:
            print("Switching to STOPPED mode.")
            current_state = ControlState.STOPPED
            asyncio.create_task(beeper.play_beep(1))
    else:
        if current_state != ControlState.MANUAL:
            print("Switching to MANUAL mode.")
            current_state = ControlState.MANUAL
            asyncio.create_task(beeper.play_beep(2))
        
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
    print("Cruise control loop started.")
    while True:
        if current_state == ControlState.CRUISE:
            (depth, angle, _), _ = await asyncio.to_thread(vision.get_nearest_object_angle, visualize=False)
            
            if current_state != ControlState.CRUISE:
                continue 
            
            if depth is not None and depth < CRUISE_STOP_DISTANCE_MM:
                wc.stop()
                print(f"Object detected at {depth/1000.0:.2f}m. Stopping.", end='\r')
            else:
                wc.set_movement(CRUISE_SPEED, 0.0)
                print(f"Cruising forward. Nearest object > {CRUISE_STOP_DISTANCE_MM/1000.0:.1f}m", end='\r')
        
        await asyncio.sleep(0.1 if current_state == ControlState.CRUISE else 0.5)

async def follow_person_loop():
    """The main logic for person-following mode."""
    global last_person_detection_time, last_person_turn_direction
    print("Follow person loop started.")
    
    upper_bound_ft = FOLLOW_TARGET_DISTANCE_FT + FOLLOW_DEAD_ZONE_FT
    lower_bound_ft = FOLLOW_TARGET_DISTANCE_FT - FOLLOW_DEAD_ZONE_FT
    
    while True:
        if current_state == ControlState.FOLLOW:
            
            # Expect 4 values from vision module
            (dist_ft, center_x, frame_w, nearest_depth), _ = await asyncio.to_thread(
                vision.find_target_person, 
                visualize=False
            )

            if current_state != ControlState.FOLLOW:
                wc.stop()
                continue

            if dist_ft is not None:
                last_person_detection_time = time.monotonic()
                
                # 1. --- Calculate speed based on person ---
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
                
                # 2. --- Obstacle Override Check ---
                # If we are trying to move forward AND there is an obstacle
                if (fwd_bwd_speed > 0 and 
                    nearest_depth is not None and 
                    nearest_depth < FOLLOW_STOP_DISTANCE_MM):
                    
                    fwd_bwd_speed = 0.0 # Override: Stop forward motion
                    status_dist = "OBSTACLE!" # Update status
                    
                # 3. --- Set final movement ---
                wc.set_movement(fwd_bwd_speed, left_right_speed)
                print(f"Follow: {dist_ft:.1f}ft ({status_dist}) | Turn: {status_turn} ✅ ", end='\r')

            else:
                # ... (rest of the target-lost logic is unchanged) ...
                time_since_last_seen = time.monotonic() - last_person_detection_time
                if last_person_detection_time == 0.0:
                    print(f"Follow: SEARCHING... ❌                                 ", end='\r')
                elif time_since_last_seen < FOLLOW_TARGET_LOST_TIMEOUT_S:
                    wc.set_movement(0.0, last_person_turn_direction)
                    print(f"Follow: RE-ACQUIRING... ❓                              ", end='\r')
                else:
                    wc.stop()
                    print(f"Follow: TARGET LOST. STOPPING. ❌                       ", end='\r')
        
        await asyncio.sleep(0.05 if current_state == ControlState.FOLLOW else 0.5) 


async def mapping_loop():
    """The main logic for mapping mode."""
    global current_state, mapping_stop_event
    print("Mapping loop started.")
    while True:
        if current_state == ControlState.MAP:
            
            # Clear the event flag before starting
            mapping_stop_event.clear()
            
            print("Temporarily shutting down vision module for mapping...")
            vision.shutdown_detector()
            
            print("\n--- Starting Mapping Sequence ---")
            # Pass the shared stop event
            await asyncio.to_thread(mapping.perform_mapping, mapping_stop_event)
            
            if mapping_stop_event.is_set():
                print("\n--- Mapping Sequence INTERRUPTED ---")
            else:
                print("\n--- Mapping Sequence Finished ---")

            # Re-initialize the vision module
            print("Re-initializing vision module...")
            if not vision.initialize_detector():
                print("FATAL: Could not re-initialize vision module. Stopping.")
                current_state = ControlState.STOPPED
            else:
                current_state = ControlState.STOPPED
                print("Returning to STOPPED mode.")
        
        await asyncio.sleep(1.0)


async def terminal_input_loop():
    """Asynchronously listens for terminal input and passes it to the handler."""
    print("\n--- Terminal Control Enabled ---")
    print("Type commands and press Enter.")
    print("Commands: 'Cruise', 'Follow', 'MAP', 'STOP'")
    print("Joystick: 'fwd,bwd,left,right' (e.g., '2000,0,0,0')")
    print("----------------------------------")
    while True:
        try:
            message = await asyncio.to_thread(input, "> ") 
            if message:
                handle_incoming_data(message)
        except (EOFError, KeyboardInterrupt):
            print("\nInput loop stopped.")
            break
        except Exception as e:
            print(f"\nError in input loop: {e}. Stopping.")
            break


async def main():
    """Main asynchronous function for the wheelchair control flow."""
    global last_command_time, current_state
    
    beeper_initialized = False
    vision_initialized = False
    mapping_initialized = False
    dac_initialized = False
    
    cruise_task = None
    follow_task = None
    mapping_task = None
    input_task = None
    
    # --- Initialization ---
    if not wc.initialize_dac():
        print("Exiting program: DAC initialization failed.")
        return
    dac_initialized = True

    if not beeper.initialize_beeper():
        print("Warning: Beeper initialization failed. Continuing without audio.")
    else:
        beeper_initialized = True
        print("Beeper Initialized ✅")
    
    if not vision.initialize_detector():
        print("Exiting program: Vision module initialization failed.")
        wc.stop()
        return
    vision_initialized = True
    
    if not mapping.initialize():
        print("Exiting program: Mapping module initialization failed.")
        vision.shutdown_detector()
        wc.stop()
        return
    mapping_initialized = True

    # --- Start Autonomous Tasks ---
    print("Main loop started. Press Ctrl+C to exit.")
    cruise_task = asyncio.create_task(cruise_control_loop())
    follow_task = asyncio.create_task(follow_person_loop())
    mapping_task = asyncio.create_task(mapping_loop())
    input_task = asyncio.create_task(terminal_input_loop())

    try:
        while True:
            time_since_cmd = time.time() - last_command_time
            if current_state == ControlState.MANUAL and time_since_cmd > 1.0:
                 print("\nNo MANUAL command received for 1s. Switching to STOPPED for safety.")
                 current_state = ControlState.STOPPED
                 wc.stop()

            await asyncio.sleep(0.5)
            
    except asyncio.CancelledError:
        print("Main loop cancelled.")
    finally:
        # --- Cleanup ---
        print("\nCleaning up resources...")
        
        # Ensure mapping stops on exit
        mapping_stop_event.set()
        
        if cruise_task: cruise_task.cancel()
        if follow_task: follow_task.cancel()
        if mapping_task: mapping_task.cancel()
        if input_task: input_task.cancel()
        
        # Wait for tasks to actually cancel
        await asyncio.gather(
            cruise_task, follow_task, mapping_task, input_task,
            return_exceptions=True
        )
        
        if vision_initialized: vision.shutdown_detector()
        if mapping_initialized: mapping.shutdown()
        if beeper_initialized: beeper.cleanup_beeper()
        if dac_initialized: wc.stop()
        print("Program cleaned up and exited.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
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
DEBUG_PRINT = True # Set to True for new cruise/follow printouts

# --- Cruise Mode Constants ---
CRUISE_STOP_DISTANCE_MM = 1750
CRUISE_STOP_DISTANCE_FT = 4.5  # NEW: For person detection
CRUISE_SPEED = 0.3
CRUISE_TURN_TRIM = 0.03  # NEW: For drift correction

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
    
    if message == "Cruise":
        if current_state != ControlState.CRUISE:
            print("Switching to CRUISE mode.")
            wc.stop() 
            current_state = ControlState.CRUISE
            asyncio.create_task(beeper.play_beep(3))
    elif message == "Follow":
        if current_state != ControlState.FOLLOW:
            print("Switching to FOLLOW mode.")
            wc.stop() 
            current_state = ControlState.FOLLOW
            asyncio.create_task(beeper.play_beep(4))
            global last_person_detection_time, last_person_turn_direction
            last_person_detection_time = 0.0
            last_person_turn_direction = 0.0
    elif message == "MAP":
        if current_state != ControlState.MAP:
            print("Switching to MAP mode.")
            wc.stop() 
            current_state = ControlState.MAP
            asyncio.create_task(beeper.play_beep(5))
            
    elif message == "STOP":
        # Check if we are mapping and tell it to stop
        if current_state == ControlState.MAP:
            print("Requesting mapping task to stop...")
            mapping_stop_event.set()
            
        if current_state != ControlState.STOPPED:
            print("Switching to STOPPED mode.")
            wc.stop() 
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
    """
    The main logic for cruise control mode.
    Uses a 2-stage check:
    1. Check for any person (robust to IR).
    2. Check for center-path obstacles (ignores peripheral IR noise).
    """
    print("Cruise control loop started (2-stage check).")
    while True:
        if current_state == ControlState.CRUISE:
            
            # --- CHECK 1: PERSON DETECTION (Robust to IR noise) ---
            (dist_ft, _, _, _), _ = await asyncio.to_thread(vision.find_target_person, visualize=False)
            
            # --- CHECK 2: CENTER PATH OBSTACLE (Ignores peripheral IR noise) ---
            (center_depth, _), _ = await asyncio.to_thread(vision.get_center_path_depth, visualize=False)
            
            if current_state != ControlState.CRUISE:
                continue

            # --- DECISION LOGIC ---
            stop = False
            stop_reason = ""

            # Priority 1: Stop for any person
            if dist_ft is not None and dist_ft < CRUISE_STOP_DISTANCE_FT:
                stop = True
                stop_reason = f"Person detected at {dist_ft:.2f}ft."
            
            # Priority 2: Stop for non-person obstacle in the center path
            elif center_depth is not None and center_depth < CRUISE_STOP_DISTANCE_MM:
                stop = True
                stop_reason = f"Obstacle on path at {center_depth/1000.0:.2f}m."

            # --- ACTION ---
            if stop:
                wc.stop()
                if DEBUG_PRINT: print(f"{stop_reason} Stopping.", end='\r')
            else:
                # No person and no center obstacle, so we can cruise.
                wc.set_movement(CRUISE_SPEED, CRUISE_TURN_TRIM)
                if DEBUG_PRINT: print(f"Cruising (Trim: {CRUISE_TURN_TRIM}). Path clear.", end='\r')
        
        await asyncio.sleep(0.1 if current_state == ControlState.CRUISE else 0.5)

async def follow_person_loop():
    """The main logic for person-following mode."""
    global last_person_detection_time, last_person_turn_direction
    print("Follow person loop started.")
    
    upper_bound_ft = FOLLOW_TARGET_DISTANCE_FT + FOLLOW_DEAD_ZONE_FT
    lower_bound_ft = FOLLOW_TARGET_DISTANCE_FT - FOLLOW_DEAD_ZONE_FT
    
    while True:
        if current_state == ControlState.FOLLOW:
            
            # Expect 4 values, but we will ignore nearest_depth
            (dist_ft, center_x, frame_w, _), _ = await asyncio.to_thread(
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
                
                # 2. --- Obstacle Override Check (REMOVED) ---
                    
                # 3. --- Set final movement ---
                wc.set_movement(fwd_bwd_speed, left_right_speed)
                if DEBUG_PRINT: print(f"Follow: {dist_ft:.1f}ft ({status_dist}) | Turn: {status_turn} ✅ ", end='\r')

            else:
                # ... (rest of the target-lost logic is unchanged) ...
                time_since_last_seen = time.monotonic() - last_person_detection_time
                if last_person_detection_time == 0.0:
                    if DEBUG_PRINT: print(f"Follow: SEARCHING... ❌                                 ", end='\r')
                elif time_since_last_seen < FOLLOW_TARGET_LOST_TIMEOUT_S:
                    wc.set_movement(0.0, last_person_turn_direction)
                    if DEBUG_PRINT: print(f"Follow: RE-ACQUIRING... ❓                              ", end='\r')
                else:
                    wc.stop()
                    if DEBUG_PRINT: print(f"Follow: TARGET LOST. STOPPING. ❌                       ", end='\r')
        
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
        
        await asyncio.sleep(1.0) # Sleep for 1s when not mapping


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
    try:
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
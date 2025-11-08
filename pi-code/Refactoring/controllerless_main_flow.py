#!/usr/bin/env python3

# File: mainflow.py
# Main asynchronous control flow for the autonomous wheelchair.
# Integrates Terminal commands with different autonomous modes.

import asyncio
import time
from enum import Enum
# --- MODIFIED: Removed BluetoothModule ---
# from bluetooth_module import BluetoothModule
import wheelchair_control as wc

# --- MODIFIED: Import our new all-in-one vision module ---
import person_detector as vision 

# --- Constants ---
ADC_MAX = 4095.0  # ESP32's ADC is 12-bit

# --- Cruise Mode Constants ---
CRUISE_STOP_DISTANCE_MM = 1500  # 1.5 meters
CRUISE_SPEED = 0.3              # 30% speed

# --- Follow Mode Constants ---
FOLLOW_TARGET_DISTANCE_FT = 3.0
FOLLOW_DEAD_ZONE_FT = 0.2
FOLLOW_MOVE_SPEED = 0.3
FOLLOW_TURN_SPEED = 0.4
FOLLOW_TRACKING_DEAD_ZONE_PX = 30
FOLLOW_TARGET_LOST_TIMEOUT_S = 3.0 # How long to search before stopping
MM_TO_FEET = 0.00328084 # For distance conversion

# --- State Management ---
class ControlState(Enum):
    MANUAL = 1
    CRUISE = 2
    STOPPED = 3
    FOLLOW = 4  # Added new state

current_state = ControlState.STOPPED # Start in STOPPED mode for safety
last_command_time = time.time()

# --- State variables for follow mode ---
last_person_detection_time = 0.0
last_person_turn_direction = 0.0


# --- MODIFIED: Simplified to accept a string directly ---
def handle_incoming_data(message: str):
    """
    Parses incoming string commands and updates the control state.
    """
    global current_state, last_command_time
    
    message = message.strip()
    if not message:
        return
        
    last_command_time = time.time()
    
    # Always stop movement when changing modes for safety
    wc.stop() 

    if message == "Cruise":
        if current_state != ControlState.CRUISE:
            print("Switching to CRUISE mode.")
            current_state = ControlState.CRUISE
    elif message == "Follow":
        if current_state != ControlState.FOLLOW:
            print("Switching to FOLLOW mode.")
            current_state = ControlState.FOLLOW
            # Reset follow state variables
            global last_person_detection_time, last_person_turn_direction
            last_person_detection_time = 0.0
            last_person_turn_direction = 0.0
    elif message == "STOP":
        if current_state != ControlState.STOPPED:
            print("Switching to STOPPED mode.")
            current_state = ControlState.STOPPED
    else:
        # Assume it's joystick data
        if current_state != ControlState.MANUAL:
            print("Switching to MANUAL mode.")
            current_state = ControlState.MANUAL
        
        try:
            # Parse joystick data: "fwd,bwd,left,right"
            parts = [int(p) for p in message.split(',')]
            if len(parts) == 4:
                # Map analog value (0-4095) to normalized value (0.0-1.0)
                fwd = parts[0] / ADC_MAX
                bwd = parts[1] / ADC_MAX # Assuming separate fwd/bwd pots
                left = parts[2] / ADC_MAX
                right = parts[3] / ADC_MAX # Assuming separate left/right pots
                
                # NOTE: Adjust this logic based on your ESP32's joystick code
                # This example assumes 4 separate values.
                # If it's 2 values (X,Y), you'll need to adapt.
                wc.set_joystick_values(fwd, bwd, right, left)
        except (ValueError, IndexError):
            print(f"Could not parse joystick data: {message}")

async def cruise_control_loop():
    """The main logic for cruise control mode."""
    print("Cruise control loop started.")
    while True:
        if current_state == ControlState.CRUISE:
            # Run the blocking vision.get_...() function in a separate thread
            # so it doesn't block our asyncio event loop.
            depth, angle, _ = await asyncio.to_thread(vision.get_nearest_object_angle)
            
            if depth is not None and depth < CRUISE_STOP_DISTANCE_MM:
                wc.stop()
                print(f"Object detected at {depth/1000.0:.2f}m. Stopping.")
            else:
                wc.set_movement(CRUISE_SPEED, 0.0)
                print(f"Cruising forward. Nearest object > {CRUISE_STOP_DISTANCE_MM/1000.0:.1f}m")
        
        # Run this loop at 10Hz when active, sleep longer when inactive
        await asyncio.sleep(0.1 if current_state == ControlState.CRUISE else 0.5)

async def follow_person_loop():
    """The main logic for person-following mode."""
    global last_person_detection_time, last_person_turn_direction
    print("Follow person loop started.")
    
    # Pre-calculate bounds
    upper_bound_ft = FOLLOW_TARGET_DISTANCE_FT + FOLLOW_DEAD_ZONE_FT
    lower_bound_ft = FOLLOW_TARGET_DISTANCE_FT - FOLLOW_DEAD_ZONE_FT
    
    while True:
        if current_state == ControlState.FOLLOW:
            # Run the blocking vision.find_...() function in a separate thread
            (dist_ft, center_x, frame_w), _ = await asyncio.to_thread(vision.find_target_person, visualize=False)

            if dist_ft is not None:
                # --- TARGET FOUND ---
                last_person_detection_time = time.monotonic()
                
                # 1. Forward/Backward logic
                fwd_bwd_speed = 0.0
                if dist_ft > upper_bound_ft:
                    fwd_bwd_speed = FOLLOW_MOVE_SPEED
                    status_dist = "MOVING FWD"
                elif dist_ft < lower_bound_ft:
                    fwd_bwd_speed = -FOLLOW_MOVE_SPEED # Move backward
                    status_dist = "TOO CLOSE (BWD)"
                else:
                    fwd_bwd_speed = 0.0
                    status_dist = "IN ZONE"
                
                # 2. Turning logic
                left_right_speed = 0.0
                frame_center_x = frame_w // 2
                left_bound = frame_center_x - FOLLOW_TRACKING_DEAD_ZONE_PX
                right_bound = frame_center_x + FOLLOW_TRACKING_DEAD_ZONE_PX

                if center_x < left_bound:
                    left_right_speed = FOLLOW_TURN_SPEED # Turn LEFT
                    status_turn = "TURN LEFT"
                elif center_x > right_bound:
                    left_right_speed = -FOLLOW_TURN_SPEED # Turn RIGHT
                    status_turn = "TURN RIGHT"
                else:
                    left_right_speed = 0.0
                    status_turn = "CENTERED"
                
                last_person_turn_direction = left_right_speed
                wc.set_movement(fwd_bwd_speed, left_right_speed)
                print(f"Follow: {dist_ft:.1f}ft ({status_dist}) | Turn: {status_turn} ✅ ", end='\r')

            else:
                # --- TARGET LOST ---
                time_since_last_seen = time.monotonic() - last_person_detection_time
                if last_person_detection_time == 0.0: # Never seen a target yet
                    print(f"Follow: SEARCHING... ❌                                 ", end='\r')
                elif time_since_last_seen < FOLLOW_TARGET_LOST_TIMEOUT_S:
                    # --- *** MODIFIED FIX *** ---
                    # Re-acquiring: stop forward/back, continue last turn
                    wc.set_movement(0.0, last_person_turn_direction)
                    print(f"Follow: RE-ACQUIRING... ❓                              ", end='\r')
                else:
                    # Truly lost: stop all
                    wc.stop()
                    print(f"Follow: TARGET LOST. STOPPING. ❌                       ", end='\r')
        
        # Run this loop slightly slower as detection is heavy
        # Sleep longer when this mode isn't active
        await asyncio.sleep(0.05 if current_state == ControlState.FOLLOW else 0.5) 


# --- NEW: Function to handle terminal input non-blockingly ---
async def terminal_input_loop():
    """Asynchronously listens for terminal input and passes it to the handler."""
    print("\n--- Terminal Control Enabled ---")
    print("Type commands and press Enter.")
    print("Commands: 'Cruise', 'Follow', 'STOP'")
    print("Joystick: 'fwd,bwd,left,right' (e.g., '2000,0,0,0')")
    print("----------------------------------")
    while True:
        try:
            # Run the blocking input() function in a separate thread
            # so it doesn't block the main asyncio loop.
            message = await asyncio.to_thread(input, "> ") 
            if message:
                handle_incoming_data(message)
        except (EOFError, KeyboardInterrupt):
            print("\nInput loop stopped.")
            break
        except Exception as e:
            print(f"\nError in input loop: {e}. Stopping.")
            break
# --- END NEW ---


async def main():
    """Main asynchronous function for the wheelchair control flow."""
    global last_command_time, current_state
    
    # --- Initialization ---
    if not wc.initialize_dac():
        print("Exiting program: DAC initialization failed.")
        return
    
    if not vision.initialize_detector():
        print("Exiting program: Vision module initialization failed.")
        wc.stop() # Ensure motors are off even if DAC init worked
        return

    # --- MODIFIED: Removed all Bluetooth initialization ---
    # bt_module = BluetoothModule()
    # if not await bt_module.connect():
    #     print("Exiting program: Bluetooth connection failed.")
    #     vision.shutdown_detector()
    #     wc.stop()
    #     return
    # await bt_module.start_listening(handle_incoming_data)
    # --- END MODIFICATION ---

    # --- Start Autonomous Tasks ---
    print("Main loop started. Press Ctrl+C to exit.")
    cruise_task = asyncio.create_task(cruise_control_loop())
    follow_task = asyncio.create_task(follow_person_loop())
    # --- MODIFIED: Added terminal input task ---
    input_task = asyncio.create_task(terminal_input_loop())

    try:
        while True:
            # --- Primary Safety Loop ---
            
            # 1. Check Bluetooth connection status (REMOVED)
            
            # 2. Check for manual mode timeout (STILL A GOOD SAFETY FEATURE)
            time_since_cmd = time.time() - last_command_time
            if current_state == ControlState.MANUAL and time_since_cmd > 1.0:
                 print("\nNo MANUAL command received for 1s. Switching to STOPPED for safety.")
                 current_state = ControlState.STOPPED
                 wc.stop()

            await asyncio.sleep(0.5) # Run safety check 2x per second
            
    except asyncio.CancelledError:
        print("Main loop cancelled.")
    finally:
        # --- Cleanup ---
        print("\nCleaning up resources...")
        cruise_task.cancel()
        follow_task.cancel()
        # --- MODIFIED: Cancel input task ---
        input_task.cancel()
        # await bt_module.disconnect() # Removed
        vision.shutdown_detector()
        wc.stop()
        print("Program cleaned up and exited.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")

#!/usr/bin/env python3

# File: main_flow.py
# Main asynchronous control flow for the autonomous wheelchair.
# Integrates Bluetooth commands with different autonomous modes.

import asyncio
import time
from enum import Enum
import threading  # For the mapping stop event
from bluetooth_module import BluetoothModule
import wheelchair_control as wc
import person_detector as vision
import mapping_module as mapping
import audio_feedback as beeper

# --- Constants ---
ADC_MAX = 4095.0
DEBUG_PRINT = False 

# --- Cruise Mode Constants ---
CRUISE_STOP_DISTANCE_MM = 1350
CRUISE_SPEED = 0.3

# --- Follow Mode Constants ---
FOLLOW_TARGET_DISTANCE_FT = 3.5
FOLLOW_DEAD_ZONE_FT = 0.3
FOLLOW_MOVE_SPEED = 0.25
FOLLOW_TURN_SPEED = 0.35
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
mapping_stop_event = threading.Event()  # Event to signal mapping thread to stop

# --- State variables for follow mode ---
last_person_detection_time = 0.0
last_person_turn_direction = 0.0


def handle_incoming_data(data: bytes):
    """
    Parses incoming data and updates the control state.
    """
    global current_state, last_command_time, mapping_stop_event
    
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
    elif message == "Map":
        if current_state != ControlState.MAPPING:
            print("Switching to MAPPING mode.")
            wc.stop()
            current_state = ControlState.MAPPING
            asyncio.create_task(beeper.play_beep(5))
    elif message == "STOP":
        # If we are mapping, signal the mapping thread to stop
        if current_state == ControlState.MAPPING:
            print("Requesting mapping task to stop...")
            mapping_stop_event.set()  # Signal the thread to stop

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
    """The main logic for cruise control mode."""
    print("Cruise control loop started.")
    while True:
        if current_state == ControlState.CRUISE:
            
            (depth, angle, _), _ = await asyncio.to_thread(vision.get_nearest_object_angle, visualize=False)
            
            if current_state != ControlState.CRUISE:
                continue
            
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
    print("Follow person loop started.")
    
    upper_bound_ft = FOLLOW_TARGET_DISTANCE_FT + FOLLOW_DEAD_ZONE_FT
    lower_bound_ft = FOLLOW_TARGET_DISTANCE_FT - FOLLOW_DEAD_ZONE_FT
    
    while True:
        if current_state == ControlState.FOLLOW:
            
            (dist_ft, center_x, frame_w), _ = await asyncio.to_thread(vision.find_target_person, visualize=False)

            if current_state != ControlState.FOLLOW:
                continue

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
        if current_state == ControlState.MAPPING:
            
            mapping_stop_event.clear()  # Clear the flag before starting a new scan
            
            print("Starting 360-degree mapping scan...")
            
            # Pass the event to the thread function
            await asyncio.to_thread(mapping.perform_mapping, mapping_stop_event)
            
            if mapping_stop_event.is_set():
                print("Mapping scan was INTERRUPTED by user.")
            else:
                print("Mapping scan complete.")
            
            # This now runs after mapping finishes OR is interrupted
            current_state = ControlState.STOPPED 
            wc.stop() 
        
        await asyncio.sleep(0.5 if current_state == ControlState.MAPPING else 0.5)

async def main():
    """Main asynchronous function for the wheelchair control flow."""
    global last_command_time, current_state
    
    # Flags will be set inside the init loop
    dac_initialized = False
    bt_connected = False
    vision_initialized = False
    mapping_initialized = False
    beeper_initialized = False
    
    cruise_task = None
    follow_task = None
    mapping_task = None
    bt_module = None # Define bt_module in outer scope

    try:
        # --- NEW INITIALIZATION LOOP ---
        all_systems_go = False
        while not all_systems_go:
            try:
                # Reset flags for this attempt
                dac_initialized = False
                bt_connected = False
                vision_initialized = False
                mapping_initialized = False
                beeper_initialized = False

                print("\n--- 🛰️ Attempting System Initialization ---")
                
                # 1. Initialize DAC
                if not wc.initialize_dac():
                    print(f"DAC initialization failed. Retrying in {RECONNECT_DELAY_S}s...")
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue # Restart loop
                dac_initialized = True
                print("DAC Initialized ✅")
                
                # 1b. Initialize Beeper
                if not beeper.initialize_beeper():
                    print("Warning: Beeper initialization failed. Continuing without audio.")
                else:
                    beeper_initialized = True
                print("Beeper Initialized ✅")
            
                # 2. Connect to Bluetooth
                # Re-create the module each time to ensure a clean state
                if bt_module and bt_module.client and bt_module.client.is_connected:
                    await bt_module.disconnect() 
                bt_module = BluetoothModule()
                
                if not await bt_module.connect():
                    print(f"Bluetooth connection failed. Retrying in {RECONNECT_DELAY_S}s...")
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue # Restart loop
                bt_connected = True
                print("Bluetooth Connected ✅")
                await bt_module.start_listening(handle_incoming_data)

                # 3. Initialize Vision
                if not vision.initialize_detector():
                    print(f"Vision module initialization failed. Retrying in {RECONNECT_DELAY_S}s...")
                    # This module can fail if Kinect is unplugged, so cleanup
                    if bt_connected:
                        await bt_module.disconnect()
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue # Restart loop
                vision_initialized = True
                print("Vision Module Initialized ✅")
                
                # 4. Initialize Mapping
                if not mapping.initialize():
                    print(f"Mapping module initialization failed. Retrying in {RECONNECT_DELAY_S}s...")
                    # Cleanup previous steps before retry
                    if bt_connected:
                        await bt_module.disconnect()
                    if vision_initialized:
                        vision.shutdown_detector()
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue # Restart loop
                mapping_initialized = True
                print("Mapping Module Initialized ✅")

                # If all steps passed, set flag to exit loop
                all_systems_go = True
                print("\n--- All systems initialized. Main loop started. ---")

            except asyncio.CancelledError:
                print("\nInitialization cancelled.")
                raise # Re-raise to be caught by outer try/except
            except Exception as e:
                print(f"Unexpected error during initialization: {e}. Retrying...")
                # Clean up this attempt's partial inits before retrying
                if bt_connected and bt_module:
                    await bt_module.disconnect()
                if vision_initialized:
                    vision.shutdown_detector()
                if mapping_initialized:
                    mapping.shutdown()
                if dac_initialized:
                    wc.stop()
                
                # Reset flags
                dac_initialized = False
                bt_connected = False
                vision_initialized = False
                mapping_initialized = False
                beeper_initialized = False

                await asyncio.sleep(RECONNECT_DELAY_S)
        # --- END OF INITIALIZATION LOOP ---

        print("Creating autonomous tasks...")
        cruise_task = asyncio.create_task(cruise_control_loop())
        follow_task = asyncio.create_task(follow_person_loop())
        mapping_task = asyncio.create_task(mapping_loop())

        # Main operational loop
        while True:
            if not bt_module.client.is_connected:
                print("\nBluetooth disconnected. Attempting to reconnect...")
                current_state = ControlState.STOPPED
                wc.stop()
                
                reconnected = False
                while not reconnected:
                    print(f"Trying to reconnect in {RECONNECT_DELAY_S} seconds...")
                    try:
                        await asyncio.sleep(RECONNECT_DELAY_S)
                    except asyncio.CancelledError:
                         print("\nShutdown during reconnect.")
                         return 
                    
                    if await bt_module.connect():
                        print("\nBluetooth reconnected successfully!")
                        await bt_module.start_listening(handle_incoming_data) 
                        reconnected = True
                    else:
                        print("Reconnection failed. Retrying...")
                
                continue

            time_since_cmd = time.time() - last_command_time
            if current_state == ControlState.MANUAL and time_since_cmd > 1.0:
                 print("\nNo MANUAL command received for 1s. Switching to STOPPED for safety.")
                 current_state = ControlState.STOPPED
                 wc.stop()
            
            try:
                 await asyncio.sleep(0.5)
            except asyncio.CancelledError:
                break # Exit loop if main is cancelled
            
    except asyncio.CancelledError:
        print("Main loop cancelled.")
    finally:
        # --- THIS BLOCK WILL NOW RUN ON CTRL+C ---
        print("\nCleaning up resources...")
        
        # Set the event on cleanup just in case mapping is running
        mapping_stop_event.set()
        
        if cruise_task: cruise_task.cancel()
        if follow_task: follow_task.cancel()
        if mapping_task: mapping_task.cancel()
        
        await asyncio.gather(
            cruise_task if cruise_task else asyncio.sleep(0), 
            follow_task if follow_task else asyncio.sleep(0), 
            mapping_task if mapping_task else asyncio.sleep(0),
            return_exceptions=True
        )
        
        # Cleanup only if initialization was successful
        if bt_connected and bt_module:
            await bt_module.disconnect()
        if vision_initialized:
            vision.shutdown_detector()
        if mapping_initialized:
            mapping.shutdown()
        if dac_initialized:
            wc.stop()
        if beeper_initialized:
            beeper.cleanup_beeper()
            
        print("Program cleaned up and exited.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    finally:
        print("Exiting.")

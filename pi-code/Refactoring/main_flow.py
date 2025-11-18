#!/usr/bin/env python3

# File: main_flow.py
# Main asynchronous control flow for the autonomous wheelchair.
# Integrates Bluetooth commands with different autonomous modes.

import asyncio
import time
from enum import Enum
import threading
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
CRUISE_STOP_DISTANCE_FT = 4.5  # NEW: (1350mm is ~4.43ft)
CRUISE_SPEED = 0.3
# NEW: Adjust this value to correct for drift (veering).
# Based on follow_mode, negative values turn RIGHT, positive values turn LEFT.
# If "leaning left", you need a small negative value to turn right.
CRUISE_TURN_TRIM = 0.03  # Start with this value and tune as needed

# --- Follow Mode Constants ---
FOLLOW_TARGET_DISTANCE_FT = 3.5
FOLLOW_DEAD_ZONE_FT = 0.3
FOLLOW_MOVE_SPEED = 0.25
FOLLOW_TURN_SPEED = 0.35
FOLLOW_TRACKING_DEAD_ZONE_PX = 30
FOLLOW_TARGET_LOST_TIMEOUT_S = 3.0
FOLLOW_STOP_DISTANCE_MM = 1250  
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
mapping_stop_event = threading.Event()

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
            # We run person detection. dist_ft will be None if no one is found.
            (dist_ft, _, _, _), _ = await asyncio.to_thread(vision.find_target_person, visualize=False)
            
            # --- CHECK 2: CENTER PATH OBSTACLE (Ignores peripheral IR noise) ---
            # This is your new function. It only looks at the depth in the center.
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
                if DEBUG_PRINT: print(f"{stop_reason} Stopping.")
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
            
            # --- Unpack 4 values ---
            # nearest_depth is the closest NON-PERSON obstacle
            (dist_ft, center_x, frame_w, nearest_depth), _ = await asyncio.to_thread(vision.find_target_person, visualize=False)

            if current_state != ControlState.FOLLOW:
                wc.stop()
                continue

            if dist_ft is not None:
                last_person_detection_time = time.monotonic()
                
                # --- OBSTACLE CHECK REMOVED ---
                
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
            
            mapping_stop_event.clear()
            print("Starting 360-degree mapping scan...")
            
            # NOTE: This version does NOT shut down/resume audio
            await asyncio.to_thread(mapping.perform_mapping, mapping_stop_event)
            
            if mapping_stop_event.is_set():
                print("Mapping scan was INTERRUPTED by user.")
            else:
                print("Mapping scan complete.")
            
            current_state = ControlState.STOPPED 
            wc.stop() 
        
        await asyncio.sleep(0.5)

async def main():
    """Main asynchronous function for the wheelchair control flow."""
    global last_command_time, current_state
    
    dac_initialized = False
    bt_connected = False
    vision_initialized = False
    mapping_initialized = False
    beeper_initialized = False
    
    cruise_task = None
    follow_task = None
    mapping_task = None
    bt_module = None 

    try:
        all_systems_go = False
        while not all_systems_go:
            try:
                dac_initialized = False
                bt_connected = False
                vision_initialized = False
                mapping_initialized = False
                beeper_initialized = False

                print("\n--- 🛰️ Attempting System Initialization ---")
                
                if not wc.initialize_dac():
                    print(f"DAC initialization failed. Retrying in {RECONNECT_DELAY_S}s...")
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue
                dac_initialized = True
                print("DAC Initialized ✅")
                
                if not beeper.initialize_beeper():
                    print("Warning: Beeper initialization failed. Continuing without audio.")
                else:
                    beeper_initialized = True
                    print("Beeper Initialized ✅")
            
                if bt_module and bt_module.client and bt_module.client.is_connected:
                    await bt_module.disconnect() 
                bt_module = BluetoothModule()
                
                if not await bt_module.connect():
                    print(f"Bluetooth connection failed. Retrying in {RECONNECT_DELAY_S}s...")
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue
                bt_connected = True
                print("Bluetooth Connected ✅")
                await bt_module.start_listening(handle_incoming_data)

                if not vision.initialize_detector():
                    print(f"Vision module initialization failed. Retrying in {RECONNECT_DELAY_S}s...")
                    if bt_connected: await bt_module.disconnect()
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue
                vision_initialized = True
                print("Vision Module Initialized ✅")
                
                if not mapping.initialize():
                    print(f"Mapping module initialization failed. Retrying in {RECONNECT_DELAY_S}s...")
                    if bt_connected: await bt_module.disconnect()
                    if vision_initialized: vision.shutdown_detector()
                    await asyncio.sleep(RECONNECT_DELAY_S)
                    continue
                mapping_initialized = True
                print("Mapping Module Initialized ✅")

                all_systems_go = True
                print("\n--- All systems initialized. Main loop started. ---")

            except asyncio.CancelledError:
                print("\nInitialization cancelled.")
                raise 
            except Exception as e:
                print(f"Unexpected error during initialization: {e}. Retrying...")
                if bt_connected and bt_module: await bt_module.disconnect()
                if vision_initialized: vision.shutdown_detector()
                if mapping_initialized: mapping.shutdown()
                if dac_initialized: wc.stop()
                await asyncio.sleep(RECONNECT_DELAY_S)

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
                break
            
    except asyncio.CancelledError:
        print("Main loop cancelled.")
    finally:
        print("\nCleaning up resources...")
        
        mapping_stop_event.set()
        
        if cruise_task: cruise_task.cancel()
        if follow_task: follow_task.cancel()
        if mapping_task: mapping_task.cancel()
        
        await asyncio.gather(
            cruise_task, follow_task, mapping_task,
            return_exceptions=True
        )
        
        if bt_connected and bt_module: await bt_module.disconnect()
        if vision_initialized: vision.shutdown_detector()
        if mapping_initialized: mapping.shutdown()
        if dac_initialized: wc.stop()
        if beeper_initialized: beeper.cleanup_beeper()
            
        print("Program cleaned up and exited.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    finally:
        print("Exiting.")

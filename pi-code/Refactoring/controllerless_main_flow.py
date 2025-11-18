"""
controllerless_main_flow.py

This module implements the main asynchronous control flow for the self-driving wheelchair
in a controllerless operation mode. It integrates various autonomous functionalities
such as cruise control, person following, and environmental mapping, responding to
commands received via terminal input.

The system operates in different states (MANUAL, CRUISE, STOPPED, FOLLOW, MAP)
and manages transitions between them. It leverages vision for obstacle detection
and person tracking, and provides audio feedback for state changes.

Dependencies:
- asyncio: For managing asynchronous tasks and event loops.
- threading: For managing concurrent operations, specifically for interrupting mapping.
- wheelchair_control (wc): Module for controlling the wheelchair's motors via a DAC.
- person_detector (vision): Module for detecting persons and obstacles using a Kinect sensor.
- mapping_module (mapping): Module for environmental mapping.
- audio_feedback (beeper): Module for providing audio cues.
"""

import asyncio
import time
from enum import Enum
import threading
import wheelchair_control as wc
import person_detector as vision
import mapping_module as mapping
import audio_feedback as beeper

# --- Constants ---
ADC_MAX = 4095.0        # Maximum value from the ADC for joystick input scaling.
DEBUG_PRINT = True      # Enable/disable debug printouts for cruise/follow modes.

# --- Cruise Mode Constants ---
CRUISE_STOP_DISTANCE_MM = 1750  # Distance in mm at which to stop for general obstacles.
CRUISE_STOP_DISTANCE_FT = 4.5   # Distance in feet at which to stop if a person is detected.
CRUISE_SPEED = 0.3              # Forward movement speed in cruise mode (0.0 to 1.0).
CRUISE_TURN_TRIM = 0.03         # Small turning adjustment for drift correction in cruise mode.

# --- Follow Mode Constants ---
FOLLOW_TARGET_DISTANCE_FT = 3.5     # Desired following distance from a person in feet.
FOLLOW_DEAD_ZONE_FT = 0.3           # Tolerance zone around TARGET_DISTANCE_FT where no forward/backward movement occurs.
FOLLOW_MOVE_SPEED = 0.25            # Forward/backward movement speed in follow mode.
FOLLOW_TURN_SPEED = 0.35            # Turning speed in follow mode.
FOLLOW_TRACKING_DEAD_ZONE_PX = 30   # Pixel tolerance for person's horizontal position in the frame.
FOLLOW_TARGET_LOST_TIMEOUT_S = 3.0  # Time in seconds before declaring target lost and stopping.
FOLLOW_STOP_DISTANCE_MM = 1250      # Distance in mm at which to stop for obstacles during follow mode.
MM_TO_FEET = 0.00328084             # Conversion factor from millimeters to feet.

# --- State Management ---
class ControlState(Enum):
    """
    Defines the possible operational states of the wheelchair.
    """
    MANUAL = 1      # Manual control via terminal joystick commands.
    CRUISE = 2      # Autonomous cruise control with obstacle avoidance.
    STOPPED = 3     # Wheelchair is stationary.
    FOLLOW = 4      # Autonomous person following.
    MAP = 5         # Environmental mapping mode.

current_state = ControlState.STOPPED    # The current operational state of the wheelchair.
last_command_time = time.time()         # Timestamp of the last received command, used for safety timeouts.
mapping_stop_event = threading.Event()  # Event flag to signal the mapping task to stop.

# --- State variables for follow mode ---
last_person_detection_time = 0.0        # Timestamp of the last successful person detection.
last_person_turn_direction = 0.0        # Last turn command issued when a person was tracked.


def handle_incoming_data(message: str):
    """
    Parses incoming string commands from the terminal and updates the wheelchair's
    control state or manual movement.

    Args:
        message (str): The raw string command received from the terminal.
    """
    global current_state, last_command_time, mapping_stop_event
    
    message = message.strip()
    if not message:
        return
        
    last_command_time = time.time() # Update last command time for safety timeout.
    
    if message == "Cruise":
        if current_state != ControlState.CRUISE:
            print("Switching to CRUISE mode.")
            wc.stop() # Stop current movement before changing mode.
            current_state = ControlState.CRUISE
            asyncio.create_task(beeper.play_beep(3)) # Audio feedback for mode change.
    elif message == "Follow":
        if current_state != ControlState.FOLLOW:
            print("Switching to FOLLOW mode.")
            wc.stop() # Stop current movement before changing mode.
            current_state = ControlState.FOLLOW
            asyncio.create_task(beeper.play_beep(4)) # Audio feedback for mode change.
            global last_person_detection_time, last_person_turn_direction
            last_person_detection_time = 0.0 # Reset follow mode specific variables.
            last_person_turn_direction = 0.0
    elif message == "MAP":
        if current_state != ControlState.MAP:
            print("Switching to MAP mode.")
            wc.stop() # Stop current movement before changing mode.
            current_state = ControlState.MAP
            asyncio.create_task(beeper.play_beep(5)) # Audio feedback for mode change.
            
    elif message == "STOP":
        # If currently mapping, signal the mapping task to stop gracefully.
        if current_state == ControlState.MAP:
            print("Requesting mapping task to stop...")
            mapping_stop_event.set()
            
        if current_state != ControlState.STOPPED:
            print("Switching to STOPPED mode.")
            wc.stop() # Ensure wheelchair is stopped.
            current_state = ControlState.STOPPED
            asyncio.create_task(beeper.play_beep(1)) # Audio feedback for mode change.
    else:
        # Default to MANUAL mode if an unrecognized command or joystick data is received.
        if current_state != ControlState.MANUAL:
            print("Switching to MANUAL mode.")
            current_state = ControlState.MANUAL
            asyncio.create_task(beeper.play_beep(2)) # Audio feedback for mode change.
        
        try:
            # Attempt to parse joystick values if the message format matches.
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
    Asynchronous loop for managing the wheelchair in CRUISE mode.

    This loop continuously checks for obstacles and persons using a 2-stage detection:
    1. Prioritizes stopping if a person is detected within CRUISE_STOP_DISTANCE_FT.
    2. If no person, checks for general obstacles in the center path within
       CRUISE_STOP_DISTANCE_MM, ignoring peripheral IR noise.
    The wheelchair moves forward at CRUISE_SPEED with a slight CRUISE_TURN_TRIM
    for drift correction if no obstacles are detected.
    """
    print("Cruise control loop started (2-stage check).")
    while True:
        if current_state == ControlState.CRUISE:
            
            # --- CHECK 1: PERSON DETECTION (Robust to IR noise) ---
            # Attempts to find a person and returns their distance in feet.
            (dist_ft, _, _, _), _ = await asyncio.to_thread(vision.find_target_person, visualize=False)
            
            # --- CHECK 2: CENTER PATH OBSTACLE (Ignores peripheral IR noise) ---
            # Gets the depth of the closest obstacle in the center path.
            (center_depth, _), _ = await asyncio.to_thread(vision.get_center_path_depth, visualize=False)
            
            # Re-check state in case it changed during vision processing.
            if current_state != ControlState.CRUISE:
                continue

            # --- DECISION LOGIC ---
            stop = False
            stop_reason = ""

            # Priority 1: Stop for any person detected within the safety distance.
            if dist_ft is not None and dist_ft < CRUISE_STOP_DISTANCE_FT:
                stop = True
                stop_reason = f"Person detected at {dist_ft:.2f}ft."
            
            # Priority 2: Stop for non-person obstacle in the center path within safety distance.
            elif center_depth is not None and center_depth < CRUISE_STOP_DISTANCE_MM:
                stop = True
                stop_reason = f"Obstacle on path at {center_depth/1000.0:.2f}m."

            # --- ACTION ---
            if stop:
                wc.stop() # Stop the wheelchair if an obstacle or person is too close.
                if DEBUG_PRINT: print(f"{stop_reason} Stopping.", end='\r')
            else:
                # If path is clear, continue cruising.
                wc.set_movement(CRUISE_SPEED, CRUISE_TURN_TRIM)
                if DEBUG_PRINT: print(f"Cruising (Trim: {CRUISE_TURN_TRIM}). Path clear.", end='\r')
        
        # Adjust sleep duration based on current state to save resources when not active.
        await asyncio.sleep(0.1 if current_state == ControlState.CRUISE else 0.5)

async def follow_person_loop():
    """
    Asynchronous loop for managing the wheelchair in FOLLOW mode.

    This loop continuously tracks a target person, adjusting the wheelchair's
    speed and turn to maintain a desired following distance and keep the person
    centered in the frame. It handles cases where the target person is lost
    and attempts to re-acquire them.
    """
    global last_person_detection_time, last_person_turn_direction
    print("Follow person loop started.")
    
    # Define the upper and lower bounds for the desired following distance.
    upper_bound_ft = FOLLOW_TARGET_DISTANCE_FT + FOLLOW_DEAD_ZONE_FT
    lower_bound_ft = FOLLOW_TARGET_DISTANCE_FT - FOLLOW_DEAD_ZONE_FT
    
    while True:
        if current_state == ControlState.FOLLOW:
            
            # Find the target person and get their distance, horizontal center, and frame width.
            # nearest_depth is ignored in this flow.
            (dist_ft, center_x, frame_w, _), _ = await asyncio.to_thread(
                vision.find_target_person, 
                visualize=False
            )

            # Re-check state in case it changed during vision processing.
            if current_state != ControlState.FOLLOW:
                wc.stop()
                continue

            if dist_ft is not None:
                # Person detected, update timestamp.
                last_person_detection_time = time.monotonic()
                
                # 1. --- Calculate speed based on person's distance ---
                fwd_bwd_speed = 0.0
                if dist_ft > upper_bound_ft:
                    fwd_bwd_speed = FOLLOW_MOVE_SPEED
                    status_dist = "MOVING FWD"
                elif dist_ft < lower_bound_ft:
                    fwd_bwd_speed = -FOLLOW_MOVE_SPEED # Move backward if too close.
                    status_dist = "TOO CLOSE (BWD)"
                else:
                    fwd_bwd_speed = 0.0
                    status_dist = "IN ZONE"
                
                # Calculate turning speed to keep the person centered.
                left_right_speed = 0.0
                frame_center_x = frame_w // 2
                left_bound = frame_center_x - FOLLOW_TRACKING_DEAD_ZONE_PX
                right_bound = frame_center_x + FOLLOW_TRACKING_DEAD_ZONE_PX

                if center_x < left_bound:
                    left_right_speed = FOLLOW_TURN_SPEED # Turn left.
                    status_turn = "TURN LEFT"
                elif center_x > right_bound:
                    left_right_speed = -FOLLOW_TURN_SPEED # Turn right.
                    status_turn = "TURN RIGHT"
                else:
                    left_right_speed = 0.0
                    status_turn = "CENTERED"
                
                last_person_turn_direction = left_right_speed # Store last turn for re-acquiring.
                
                # 2. --- Obstacle Override Check (REMOVED from this flow) ---
                    
                # 3. --- Set final movement ---
                wc.set_movement(fwd_bwd_speed, left_right_speed)
                if DEBUG_PRINT: print(f"Follow: {dist_ft:.1f}ft ({status_dist}) | Turn: {status_turn} ✅ ", end='\r')

            else:
                # Target person not detected.
                time_since_last_seen = time.monotonic() - last_person_detection_time
                if last_person_detection_time == 0.0:
                    # Initial search state.
                    if DEBUG_PRINT: print(f"Follow: SEARCHING... ❌                                 ", end='\r')
                elif time_since_last_seen < FOLLOW_TARGET_LOST_TIMEOUT_S:
                    # Attempt to re-acquire by continuing the last turn.
                    wc.set_movement(0.0, last_person_turn_direction)
                    if DEBUG_PRINT: print(f"Follow: RE-ACQUIRING... ❓                              ", end='\r')
                else:
                    # Target lost for too long, stop the wheelchair.
                    wc.stop()
                    if DEBUG_PRINT: print(f"Follow: TARGET LOST. STOPPING. ❌                       ", end='\r')
        
        # Adjust sleep duration based on current state.
        await asyncio.sleep(0.05 if current_state == ControlState.FOLLOW else 0.5) 


async def mapping_loop():
    """
    Asynchronous loop for managing the wheelchair in MAP mode.

    This loop initiates an environmental mapping sequence. It temporarily shuts down
    the vision module to avoid conflicts, performs mapping, and then re-initializes
    the vision module. The mapping process can be interrupted by setting the
    `mapping_stop_event`.
    """
    global current_state, mapping_stop_event
    print("Mapping loop started.")
    while True:
        if current_state == ControlState.MAP:
            
            # Clear the event flag before starting a new mapping sequence.
            mapping_stop_event.clear()
            
            print("Temporarily shutting down vision module for mapping...")
            vision.shutdown_detector() # Disable vision to free up resources for mapping.
            
            print("\n--- Starting Mapping Sequence ---")
            # Execute the mapping process in a separate thread to keep the event loop responsive.
            await asyncio.to_thread(mapping.perform_mapping, mapping_stop_event)
            
            if mapping_stop_event.is_set():
                print("\n--- Mapping Sequence INTERRUPTED ---")
            else:
                print("\n--- Mapping Sequence Finished ---")

            # Re-initialize the vision module after mapping.
            print("Re-initializing vision module...")
            if not vision.initialize_detector():
                print("FATAL: Could not re-initialize vision module. Stopping.")
                current_state = ControlState.STOPPED # Critical error, switch to STOPPED.
            else:
                current_state = ControlState.STOPPED # Mapping finished, return to STOPPED.
                print("Returning to STOPPED mode.")
        
        # Sleep for 1 second when not mapping to reduce CPU usage.
        await asyncio.sleep(1.0)


async def terminal_input_loop():
    """
    Asynchronously listens for user input from the terminal.

    This loop continuously prompts the user for commands and passes them to
    `handle_incoming_data` for processing. It supports various control commands
    and joystick-like inputs.
    """
    print("\n--- Terminal Control Enabled ---")
    print("Type commands and press Enter.")
    print("Commands: 'Cruise', 'Follow', 'MAP', 'STOP'")
    print("Joystick: 'fwd,bwd,left,right' (e.g., '2000,0,0,0')")
    print("----------------------------------")
    while True:
        try:
            # Use asyncio.to_thread for blocking input() call.
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
    """
    Main asynchronous entry point for the controllerless wheelchair control system.

    This function handles the initialization of all necessary modules (DAC, beeper,
    vision, mapping), starts the various asynchronous control loops (cruise, follow,
    mapping, terminal input), and manages the overall state and cleanup process.
    It includes a safety timeout for manual mode.
    """
    global last_command_time, current_state
    
    # Flags to track successful initialization of modules.
    beeper_initialized = False
    vision_initialized = False
    mapping_initialized = False
    dac_initialized = False
    
    # Placeholders for asynchronous task objects.
    cruise_task = None
    follow_task = None
    mapping_task = None
    input_task = None
    
    # --- Initialization ---
    try:
        # Initialize DAC for wheelchair motor control.
        if not wc.initialize_dac():
            print("Exiting program: DAC initialization failed.")
            return
        dac_initialized = True

        # Initialize beeper for audio feedback.
        if not beeper.initialize_beeper():
            print("Warning: Beeper initialization failed. Continuing without audio.")
        else:
            beeper_initialized = True
            print("Beeper Initialized ✅")
        
        # Initialize vision module for person and obstacle detection.
        if not vision.initialize_detector():
            print("Exiting program: Vision module initialization failed.")
            wc.stop() # Ensure wheelchair is stopped on critical failure.
            return
        vision_initialized = True
        
        # Initialize mapping module.
        if not mapping.initialize():
            print("Exiting program: Mapping module initialization failed.")
            vision.shutdown_detector() # Clean up vision if mapping fails.
            wc.stop() # Ensure wheelchair is stopped on critical failure.
            return
        mapping_initialized = True

        # --- Start Autonomous Tasks ---
        print("Main loop started. Press Ctrl+C to exit.")
        # Create and schedule the main asynchronous tasks.
        cruise_task = asyncio.create_task(cruise_control_loop())
        follow_task = asyncio.create_task(follow_person_loop())
        mapping_task = asyncio.create_task(mapping_loop())
        input_task = asyncio.create_task(terminal_input_loop())

        # Main loop for monitoring state and applying safety timeouts.
        while True:
            time_since_cmd = time.time() - last_command_time
            # If in MANUAL mode and no command received for 1 second, switch to STOPPED.
            if current_state == ControlState.MANUAL and time_since_cmd > 1.0:
                 print("\nNo MANUAL command received for 1s. Switching to STOPPED for safety.")
                 current_state = ControlState.STOPPED
                 wc.stop() # Ensure wheelchair stops.

            await asyncio.sleep(0.5) # Short delay to prevent busy-waiting.
            
    except asyncio.CancelledError:
        print("Main loop cancelled.")
    finally:
        # --- Cleanup ---
        print("\nCleaning up resources...")
        
        # Ensure mapping task is signaled to stop before cancellation.
        mapping_stop_event.set()
        
        # Cancel all running asynchronous tasks.
        if cruise_task: cruise_task.cancel()
        if follow_task: follow_task.cancel()
        if mapping_task: mapping_task.cancel()
        if input_task: input_task.cancel()
        
        # Wait for all tasks to complete their cancellation.
        await asyncio.gather(
            cruise_task, follow_task, mapping_task, input_task,
            return_exceptions=True # Allow gathering even if tasks raise CancelledError.
        )
        
        # Perform shutdown/cleanup for all initialized modules.
        if vision_initialized: vision.shutdown_detector()
        if mapping_initialized: mapping.shutdown()
        if beeper_initialized: beeper.cleanup_beeper()
        if dac_initialized: wc.stop() # Ensure DAC outputs are zeroed.
        print("Program cleaned up and exited.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")

import asyncio
import time
from enum import Enum
from bluetooth_module import BluetoothModule
import wheelchair_control as wc
import kinectcloseobject as kinect

# --- Constants ---
ADC_MAX = 4095.0  # ESP32's ADC is 12-bit
CRUISE_STOP_DISTANCE_MM = 1500  # 1 meter
CRUISE_SPEED = 0.3 # 30% speed

# --- State Management ---
class ControlState(Enum):
    MANUAL = 1
    CRUISE = 2
    STOPPED = 3

current_state = ControlState.MANUAL
last_command_time = time.time()

def handle_incoming_data(data: bytes):
    """
    Parses incoming data and updates the control state or DAC values.
    """
    global current_state, last_command_time
    
    message = data.decode().strip()
    last_command_time = time.time()

    if message == "Cruise":
        if current_state != ControlState.CRUISE:
            print("Switching to CRUISE mode.")
            current_state = ControlState.CRUISE
            wc.stop()
    elif message == "Follow":
        print("FOLLOW RECIEVED")
    elif message == "STOP":
        if current_state != ControlState.STOPPED:
            print("Switching to STOPPED mode.")
            current_state = ControlState.STOPPED
            wc.stop()
    else:
        # Assume it's joystick data
        if current_state != ControlState.MANUAL:
            print("Switching to MANUAL mode.")
            current_state = ControlState.MANUAL
        
        try:
            parts = [int(p) for p in message.split(',')]
            if len(parts) == 4:
                # Map analog value (0-4095) to normalized value (0.0-1.0)
                fwd = parts[0] / ADC_MAX
                bwd = 1.0 - fwd
                right = parts[2] / ADC_MAX
                left = 1.0 - right
                wc.set_joystick_values(fwd, bwd, left, right)
        except (ValueError, IndexError):
            print(f"Could not parse joystick data: {message}")

async def cruise_control_loop():
    """The main logic for cruise control mode."""
    print("Cruise control loop started.")
    while True:
        if current_state == ControlState.CRUISE:
            depth, angle, _ = kinect.get_nearest_object_angle()
            
            if depth is not None and depth < CRUISE_STOP_DISTANCE_MM:
                wc.stop()
                print(f"Object detected at {depth/1000.0:.2f}m. Stopping.")
            else:
                wc.set_movement(CRUISE_SPEED, 0.0)
                print(f"Cruising forward. Nearest object > {CRUISE_STOP_DISTANCE_MM/1000.0:.1f}m")
        
        await asyncio.sleep(0.1) # Run the loop at 10Hz

async def main():
    """Main asynchronous function for the wheelchair control flow."""
    global last_command_time, current_state
    
    # --- Initialization ---
    if not wc.initialize_dac():
        print("Exiting program: DAC initialization failed.")
        return
        
    if not kinect.initialize_kinect():
        print("Exiting program: Kinect initialization failed.")
        return

    bt_module = BluetoothModule()
    if not await bt_module.connect():
        print("Exiting program: Bluetooth connection failed.")
        kinect.shutdown_kinect()
        return

    await bt_module.start_listening(handle_incoming_data)

    # --- Main Loops ---
    print("Main loop started. Press Ctrl+C to exit.")
    cruise_task = asyncio.create_task(cruise_control_loop())

    try:
        while True:
            # If no command is received for 1 second in manual mode, stop.
            if current_state == ControlState.MANUAL and (time.time() - last_command_time) > 1.0:
                 print("No command received. Switching to STOPPED mode for safety.")
                 current_state = ControlState.STOPPED
                 wc.stop()

            await asyncio.sleep(0.5)
            
    except asyncio.CancelledError:
        print("Main loop cancelled.")
    finally:
        cruise_task.cancel()
        await bt_module.disconnect()
        kinect.shutdown_kinect()
        wc.stop()
        print("Program cleaned up and exited.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")

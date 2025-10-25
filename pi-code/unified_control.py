# Import necessary libraries
import time
import board
import math
import numpy as np
import asyncio
from bleak import BleakScanner, BleakClient
import smbus2

# --- Adafruit Libraries ---
import adafruit_mcp4728

# --- State Machine ---
class ControlState:
    AUTONOMOUS = 1
    MANUAL = 2

# --- Global State Variables ---
current_state = ControlState.AUTONOMOUS
current_fwd_bwd = 0.0
current_left_right = 0.0

# --- Bluetooth Constants ---
ESP32_DEVICE_NAME = "ESP32_BLE_Server"
SERVICE_UUID = "c7a72f3e-3151-4d39-a18e-4a73426b3e2b"
CHARACTERISTIC_UUID_TX = "d1aea128-4f7e-4c4f-a7b5-c603a111a00a"
ADC_MAX = 4095.0

# --- MPU-9250 Constants ---
MPU9250_ADDRESS = 0x68
AK8963_ADDRESS = 0x0C
PWR_MGMT_1 = 0x6B
INT_PIN_CFG = 0x37
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
AK8963_ST1 = 0x02
AK8963_XOUT_L = 0x03
AK8963_CNTL = 0x0A
AK8963_ASAX = 0x10
ACCEL_FS_2G = 0x00
GYRO_FS_250DPS = 0x00
AK8963_MODE_C100HZ = 0x06
AK8963_BIT_16 = 0x10

# --- General Constants ---
MOVE_SPEED_OFFSET = 0.1
TURN_SPEED_OFFSET = 0.25

# --- Hardware Setup ---
try:
    print("Initializing I2C and sensors...")
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    bus = smbus2.SMBus(1)
    # MPU-9250 Initialization
    bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    bus.write_byte_data(MPU9250_ADDRESS, INT_PIN_CFG, 0x02)
    time.sleep(0.1)
    bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL, (AK8963_BIT_16 | AK8963_MODE_C100HZ))
    print("MCP4728 and MPU-9250 found and initialized. ✅")
except Exception as e:
    print(f"Error: Could not find a required I2C device. Please check connections.")
    print(f"Details: {e}")
    exit()


# --- DAC Control Functions ---
def set_movement(fwd_bwd, left_right):
    global current_fwd_bwd, current_left_right
    fwd_bwd = max(-1.0, min(1.0, fwd_bwd))
    left_right = max(-1.0, min(1.0, left_right))
    mcp.channel_a.normalized_value = 0.5 + (fwd_bwd / 2.0)
    mcp.channel_b.normalized_value = 0.5 - (fwd_bwd / 2.0)
    mcp.channel_c.normalized_value = 0.5 - (left_right / 2.0)
    mcp.channel_d.normalized_value = 0.5 + (left_right / 2.0)
    current_fwd_bwd = fwd_bwd
    current_left_right = left_right

def stop_all_movement():
    print("\nStopping movement...")
    set_movement(0.0, 0.0)
    print("Stopped.")

# --- MPU-9250 Functions ---
def read_word_2c(addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) + low
    return val - 65536 if val >= 0x8000 else val

def read_mag_word(addr, reg):
    low = bus.read_byte_data(addr, reg)
    high = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) | low
    return val - 65536 if val >= 0x8000 else val

def get_heading():
    if bus.read_byte_data(AK8963_ADDRESS, AK8963_ST1) & 0x01:
        mag_x = read_mag_word(AK8963_ADDRESS, AK8963_XOUT_L)
        mag_y = read_mag_word(AK8963_ADDRESS, AK8963_XOUT_L + 2)
        bus.read_byte_data(AK8963_ADDRESS, 0x09) # Must read ST2 to complete
        heading = math.degrees(math.atan2(mag_y, mag_x))
        return (heading + 360) % 360
    return None

# --- Main Application Logic ---

def execute_turn_to_heading(target_heading):
    print(f"\nExecuting turn to heading: {target_heading:.1f}°...")
    
    current_heading = get_heading()
    if current_heading is None:
        print("Could not get heading, skipping turn.")
        return

    heading_diff = (target_heading - current_heading + 180) % 360 - 180
    
    if abs(heading_diff) < 5: # Dead zone of 5 degrees
        print("Already at target heading.")
        return

    turn_direction = -1 if heading_diff > 0 else 1
    set_movement(0.0, turn_direction * TURN_SPEED_OFFSET)

    while abs(heading_diff) > 5:
        current_heading = get_heading()
        if current_heading is None:
            break
        heading_diff = (target_heading - current_heading + 180) % 360 - 180
        print(f"  -> Turning... Current: {current_heading:.1f}°, Target: {target_heading:.1f}°", end='\r')
        time.sleep(0.05)

    stop_all_movement()
    print("\nTurn complete.")

def handle_disconnect(client):
    print(f"Device {client.address} disconnected.")
    stop_all_movement()

def manual_mode_handler(sender, data):
    global current_state
    message = data.decode('utf-8').strip()
    
    if message == "STOP":
        if current_state == ControlState.AUTONOMOUS:
            current_state = ControlState.MANUAL
            print("\nSwitching to MANUAL control.")
            stop_all_movement()
        elif current_state == ControlState.MANUAL:
            current_state = ControlState.AUTONOMOUS
            print("\nSwitching to AUTONOMOUS mode.")
            stop_all_movement()
        return

    if current_state == ControlState.MANUAL:
        try:
            analog_values_str = message.split(',')
            analog_values = [int(v) for v in analog_values_str]
            
            fwd_bwd = (analog_values[0] / ADC_MAX) * 2 - 1
            left_right = (analog_values[2] / ADC_MAX) * 2 - 1
            
            set_movement(fwd_bwd, left_right)
        except (ValueError, IndexError):
            pass # Ignore malformed data

async def main():
    stop_all_movement()
    print("--- Unified Wheelchair Control System ---")

    while True:
        print(f"\nScanning for BLE device: {ESP32_DEVICE_NAME}...")
        device = await BleakScanner.find_device_by_name(ESP32_DEVICE_NAME)

        if device is None:
            print(f"Could not find {ESP32_DEVICE_NAME}. Retrying in 5 seconds...")
            await asyncio.sleep(5)
            continue

        print(f"Found device: {device.name} ({device.address})")

        try:
            async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
                if client.is_connected:
                    print(f"Connected to {device.name}")
                    await asyncio.sleep(1) # Wait for services to stabilize
                    await client.start_notify(CHARACTERISTIC_UUID_TX, manual_mode_handler)
                    
                    while client.is_connected:
                        if current_state == ControlState.AUTONOMOUS:
                            set_movement(MOVE_SPEED_OFFSET, 0.0) # Drive forward slowly
                        
                        # In MANUAL mode, the notification handler takes over.
                        # If we switch back to AUTONOMOUS, this loop will resume control.
                        
                        await asyncio.sleep(0.1) # Main loop update rate
        except Exception as e:
            print(f"Connection lost or failed: {e}")
            stop_all_movement()
            await asyncio.sleep(2) # Wait before retrying


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        stop_all_movement()

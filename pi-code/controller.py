import board
import adafruit_mcp4728
import asyncio
from bleak import BleakScanner, BleakClient

# --- Hardware Setup ---
try:
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("✅ MCP4728 DAC found and initialized.")
except ValueError:
    print("❌ Error: Could not find MCP4728. Please check I2C connections.")
    exit()

# --- BLE Configuration ---
ESP32_DEVICE_NAME = "ESP32_BLE_Server"
SERVICE_UUID = "c7a72f3e-3151-4d39-a18e-4a73426b3e2b"
CHARACTERISTIC_UUID_TX = "d1aea128-4f7e-4c4f-a7b5-c603a111a00a"

# ESP32's ADC is 12-bit, so the max value is 4095
ADC_MAX = 4095.0

def set_neutral():
    """Sets all DAC channels to their neutral (0.5) position for safety."""
    print("\nSetting DAC channels to neutral (0.5V)...")
    mcp.channel_a.normalized_value = 0.5
    mcp.channel_b.normalized_value = 0.5
    mcp.channel_c.normalized_value = 0.5
    mcp.channel_d.normalized_value = 0.5
    print("✅ DACs are in a safe, neutral state.")

def print_status(val_a, val_b, val_c, val_d):
    """Prints the current status of all channels."""
    print(
        f"\rStatus -> Fwd(A): {val_a:.2f}, Bwd(B): {val_b:.2f} | "
        f"Right(C): {val_c:.2f}, Left(D): {val_d:.2f}",
        end=""
    )

def notification_handler(sender, data):
    """
    Called every time the ESP32 sends a notification.
    It parses the data and updates the DAC channels.
    """
    try:
        message = data.decode('utf-8')
        analog_values_str = message.split(',')
        analog_values = [int(v) for v in analog_values_str]

        # Map analog value (0-4095) to normalized value (0.0-1.0)
        val_a = analog_values[0] / ADC_MAX # Forward
        val_b = 1.0 - val_a                # Backward

        val_c = analog_values[2] / ADC_MAX # Right
        val_d = 1.0 - val_c                # Left

        # Clamp values to ensure they are within the 0.0 to 1.0 range
        val_a = max(0.0, min(1.0, val_a))
        val_b = max(0.0, min(1.0, val_b))
        val_c = max(0.0, min(1.0, val_c))
        val_d = max(0.0, min(1.0, val_d))

        # Send the new values to the DAC
        mcp.channel_a.normalized_value = val_a
        mcp.channel_b.normalized_value = val_b
        mcp.channel_c.normalized_value = val_c
        mcp.channel_d.normalized_value = val_d
        
        print_status(val_a, val_b, val_c, val_d)

    except (ValueError, IndexError) as e:
        print(f"\nError processing received data: {data}. Details: {e}")

async def main():
    """
    The main asynchronous function.
    Scans for the device and tries to connect in a persistent loop.
    """
    set_neutral() # Start in a safe state
    
    while True:
        print(f"\n🔄 Scanning for BLE device: {ESP32_DEVICE_NAME}...")
        device = await BleakScanner.find_device_by_name(ESP32_DEVICE_NAME)

        if device is None:
            print(f"❌ Could not find {ESP32_DEVICE_NAME}. Retrying in 5 seconds...")
            await asyncio.sleep(5)
            continue

        print(f"✅ Found device: {device.name} ({device.address})")

        # This function will be called automatically on disconnect
        def handle_disconnect(client: BleakClient):
            print(f"❌ Device {client.address} disconnected.")
            set_neutral()
            # The main loop will now take over and start scanning again.

        async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
            if client.is_connected:
                print(f"✅ Connected to {device.name}")
            else:
                print(f"❌ Failed to connect. Retrying...")
                await asyncio.sleep(2)
                continue

            try:
                print(f"Subscribing to notifications on characteristic {CHARACTERISTIC_UUID_TX}...")
                await client.start_notify(CHARACTERISTIC_UUID_TX, notification_handler)
                
                print("🟢 Listening for joystick data. Move the joystick to control the chair.")
                
                # Keep the program alive while connected
                while client.is_connected:
                    await asyncio.sleep(1)

            except Exception as e:
                print(f"An error occurred during connection: {e}")
                set_neutral() # Ensure safety on unexpected errors
                # The loop will restart and try to reconnect
    
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        # Final safety check to ensure motors are off when the script is fully terminated
        set_neutral()
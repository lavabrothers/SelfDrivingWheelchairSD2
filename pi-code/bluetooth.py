import asyncio
from bleak import BleakScanner, BleakClient

# The same UUIDs defined in the ESP32 code
SERVICE_UUID = "c7a72f3e-3151-4d39-a18e-4a73426b3e2b"
CHAR_RX_UUID = "f3711319-333e-41a4-b04b-32a7b8e1136c" # ESP32 receives on this
CHAR_TX_UUID = "d1aea128-4f7e-4c4f-a7b5-c603a111a00a" # ESP32 transmits on this

def notification_handler(sender, data):
    """Handles incoming data from the ESP32."""
    print(f"Received Notification: {data.decode()}")

async def main():
    print("🔎 Scanning for devices...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: SERVICE_UUID in ad.service_uuids
    )

    if not device:
        print("❌ ESP32 BLE Server not found.")
        return

    print(f"✅ Found ESP32 BLE Server: {device.name} ({device.address})")

    async with BleakClient(device) as client:
        if not client.is_connected:
            print("Failed to connect.")
            return

        print("🔗 Connected successfully!")

        # Subscribe to notifications from the ESP32
        await client.start_notify(CHAR_TX_UUID, notification_handler)
        print("Subscribed to notifications. Waiting for data from ESP32...")
        
        # Interactive loop to send data
        while True:
            try:
                msg = await asyncio.to_thread(input, ">> Enter message to send (or 'quit'): ")
                if msg.lower() == 'quit':
                    break
                
                # Send data to the ESP32
                await client.write_gatt_char(CHAR_RX_UUID, msg.encode())
                print(f"Sent: {msg}")

            except KeyboardInterrupt:
                break

        # Unsubscribe before exiting
        await client.stop_notify(CHAR_TX_UUID)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated.")

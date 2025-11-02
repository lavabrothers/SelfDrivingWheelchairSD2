import asyncio
from bleak import BleakScanner, BleakClient
from typing import Callable, Any

# Service and Characteristic UUIDs for the ESP32 BLE Server
SERVICE_UUID = "c7a72f3e-3151-4d39-a18e-4a73426b3e2b"
CHAR_RX_UUID = "f3711319-333e-41a4-b04b-32a7b8e1136c"  # ESP32 receives on this
CHAR_TX_UUID = "d1aea128-4f7e-4c4f-a7b5-c603a111a00a"  # ESP32 transmits on this

class BluetoothModule:
    """A module to handle BLE communication with an ESP32 device."""

    def __init__(self):
        self.client: BleakClient = None
        self.notification_callback: Callable[[bytes], None] = None

    def _notification_handler(self, sender, data):
        """Internal handler to pass data to the user-defined callback."""
        if self.notification_callback:
            self.notification_callback(data)

    async def connect(self):
        """Scans for the ESP32 device and connects to it."""
        print("🔎 Scanning for devices...")
        device = await BleakScanner.find_device_by_filter(
            lambda d, ad: SERVICE_UUID in ad.service_uuids
        )

        if not device:
            print("❌ ESP32 BLE Server not found.")
            return False

        print(f"✅ Found ESP32 BLE Server: {device.name} ({device.address})")
        self.client = BleakClient(device)
        
        try:
            await self.client.connect()
            print("🔗 Connected successfully!")
            return self.client.is_connected
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False

    async def disconnect(self):
        """Disconnects from the BLE device."""
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            print("🔌 Disconnected.")

    async def start_listening(self, callback: Callable[[bytes], None]):
        """Subscribes to notifications from the ESP32."""
        if self.client and self.client.is_connected:
            self.notification_callback = callback
            await self.client.start_notify(CHAR_TX_UUID, self._notification_handler)
            print("📡 Subscribed to notifications.")
        else:
            print("Not connected. Cannot start listening.")

    async def stop_listening(self):
        """Unsubscribes from notifications."""
        if self.client and self.client.is_connected:
            await self.client.stop_notify(CHAR_TX_UUID)
            print("🚫 Unsubscribed from notifications.")

    async def send(self, data: str):
        """Sends data to the ESP32."""
        if self.client and self.client.is_connected:
            await self.client.write_gatt_char(CHAR_RX_UUID, data.encode())
        else:
            print("Not connected. Cannot send data.")

def default_notification_handler(data: bytes):
    """A default handler for testing purposes."""
    print(f"Received: {data.decode()}")

async def main():
    """Main function to test the BluetoothModule."""
    bt_module = BluetoothModule()
    
    if not await bt_module.connect():
        return

    await bt_module.start_listening(default_notification_handler)

    # Interactive loop to send data
    try:
        while True:
            msg = await asyncio.to_thread(input, ">> Enter message to send (or 'quit'): ")
            if msg.lower() == 'quit':
                break
            await bt_module.send(msg)
    except KeyboardInterrupt:
        pass
    finally:
        await bt_module.disconnect()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated.")

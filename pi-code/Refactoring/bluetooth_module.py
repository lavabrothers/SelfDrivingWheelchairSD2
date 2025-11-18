"""
bluetooth_module.py

This module provides a robust interface for Bluetooth Low Energy (BLE) communication,
specifically designed to interact with an ESP32 device acting as a BLE server.
It utilizes the `bleak` library for asynchronous operations, enabling the Raspberry Pi
to scan for, connect to, send data to, and receive notifications from the ESP32.

Key Features:
- Device scanning and connection based on a specific service UUID.
- Asynchronous data transmission to the ESP32.
- Asynchronous notification subscription and handling for data received from the ESP32.
- Graceful disconnection and error handling.

Dependencies:
- asyncio: For managing asynchronous operations.
- bleak: A powerful, cross-platform Bluetooth Low Energy client library.
"""

import asyncio
from bleak import BleakScanner, BleakClient
from typing import Callable, Any

# Service and Characteristic UUIDs for the ESP32 BLE Server.
# These UUIDs must match those configured on the ESP32 BLE server.
SERVICE_UUID = "c7a72f3e-3151-4d39-a18e-4a73426b3e2b"
CHAR_RX_UUID = "f3711319-333e-41a4-b04b-32a7b8e1136c"  # Characteristic for data sent FROM Raspberry Pi TO ESP32.
CHAR_TX_UUID = "d1aea128-4f7e-4c4f-a7b5-c603a111a00a"  # Characteristic for data sent FROM ESP32 TO Raspberry Pi.

class BluetoothModule:
    """
    Manages Bluetooth Low Energy (BLE) communication with a designated ESP32 device.

    This class encapsulates the functionality for scanning, connecting, sending data,
    and receiving notifications over BLE. It is designed to be used asynchronously.
    """

    def __init__(self):
        """
        Initializes the BluetoothModule.

        Sets up the BleakClient instance and a placeholder for the user-defined
        notification callback.
        """
        self.client: BleakClient = None
        self.notification_callback: Callable[[bytes], None] = None

    def _notification_handler(self, sender: int, data: bytes):
        """
        Internal callback handler for BLE notifications.

        This method is invoked by the BleakClient when a notification is received
        from the ESP32 on the CHAR_TX_UUID. It then forwards the received data
        to the user-defined `notification_callback`.

        Args:
            sender (int): The handle of the characteristic that sent the notification.
            data (bytes): The raw byte data received from the ESP32.
        """
        if self.notification_callback:
            self.notification_callback(data)

    async def connect(self) -> bool:
        """
        Scans for the ESP32 BLE server device and establishes a connection.

        The scan filters devices based on the predefined SERVICE_UUID. If found,
        it attempts to connect to the device.

        Returns:
            bool: True if the connection was successful, False otherwise.
        """
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
        """
        Disconnects from the currently connected BLE device.

        If a client is connected, it performs a graceful disconnection.
        """
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            print("🔌 Disconnected.")

    async def start_listening(self, callback: Callable[[bytes], None]):
        """
        Subscribes to notifications from the ESP32 device.

        Once subscribed, any data transmitted by the ESP32 on CHAR_TX_UUID
        will be received and processed by the provided callback.

        Args:
            callback (Callable[[bytes], None]): An asynchronous function to be called
                                                with the received byte data.
        """
        if self.client and self.client.is_connected:
            self.notification_callback = callback
            await self.client.start_notify(CHAR_TX_UUID, self._notification_handler)
            print("📡 Subscribed to notifications.")
        else:
            print("Not connected. Cannot start listening.")

    async def stop_listening(self):
        """
        Unsubscribes from BLE notifications.

        Stops receiving data from the ESP32 on CHAR_TX_UUID.
        """
        if self.client and self.client.is_connected:
            await self.client.stop_notify(CHAR_TX_UUID)
            print("🚫 Unsubscribed from notifications.")

    async def send(self, data: str):
        """
        Sends a string of data to the connected ESP32 device.

        The string data is encoded into bytes and written to the CHAR_RX_UUID.

        Args:
            data (str): The string message to send to the ESP32.
        """
        if self.client and self.client.is_connected:
            await self.client.write_gatt_char(CHAR_RX_UUID, data.encode())
        else:
            print("Not connected. Cannot send data.")

def default_notification_handler(data: bytes):
    """
    A default notification handler for demonstration and testing purposes.

    Prints the decoded string representation of the received byte data.

    Args:
        data (bytes): The raw byte data received from the BLE device.
    """
    print(f"Received: {data.decode()}")

async def main():
    """
    Main asynchronous function to demonstrate the BluetoothModule functionality.

    This function initializes the module, attempts to connect to the ESP32,
    starts listening for notifications, and provides an interactive loop
    to send messages to the ESP32. It ensures proper disconnection upon exit.
    """
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
        print("\nProgram interrupted by user.")
    finally:
        await bt_module.disconnect()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated.")

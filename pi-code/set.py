import board
import adafruit_mcp4728
import time

i2c = board.I2C()
mcp = adafruit_mcp4728.MCP4728(i2c)

# Use normalized_value which is easier to work with (0.0 to 1.0)
# This sets Channel A to its maximum voltage
mcp.channel_a.normalized_value = 0.6
mcp.channel_b.normalized_value = 0.4
mcp.channel_c.normalized_value = 0.5
mcp.channel_d.normalized_value = 0.5

print("Channel A set to 2.5V voltage.")
print("The script will now do nothing, check the voltage with a multimeter.")

# Keep the script running so the value holds

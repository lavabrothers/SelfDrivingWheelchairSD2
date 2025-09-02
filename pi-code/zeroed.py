import smbus2
import time

# --- Configuration ---
# I2C bus number. For Raspberry Pi 5, it's typically 1.
I2C_BUS = 1

# I2C address of the MCP4728. Default is 0x60.
# Check your hardware for any address modifications.
MCP4728_ADDRESS = 0x60

# --- Target Voltage ---
# The desired static output voltage for all channels. Set to 0.0 to zero out.
TARGET_VOLTAGE = 0.0

# --- MCP4728 Internal Settings ---
# VDD supply voltage for the DAC.
# IMPORTANT: The MCP4728's maximum output voltage is limited by its VDD supply.
VDD = 5.0

# VREF setting: 0 for VDD, 1 for internal 2.048V reference.
VREF_MODE = 0  # 0 = VDD, 1 = Internal 2.048V

# Gain setting: 0 for 1x, 1 for 2x.
GAIN_MODE = 0 # 0 = 1x, 1 = 2x

def main():
    """
    Main function to initialize I2C and set all four channels
    to a static voltage of 0V using individual write commands.
    """
    bus = None
    CHANNEL_MAP = {
        'A': 0b00,
        'B': 0b01,
        'C': 0b10,
        'D': 0b11
    }

    try:
        bus = smbus2.SMBus(I2C_BUS)
        print(f"I2C bus opened. Setting all channels to {TARGET_VOLTAGE}V.")

        # For 0V, the digital value is always 0.
        digital_value = 0

        # Construct the two data bytes once, as they are the same for all channels.
        data_msb = ((VREF_MODE & 0x01) << 7 |
                    (0 & 0x01) << 6 |  # Power Down bits (00 = normal operation)
                    (0 & 0x01) << 5 |
                    (GAIN_MODE & 0x01) << 4 |
                    ((digital_value >> 8) & 0x0F))
        data_lsb = digital_value & 0xFF
        i2c_payload = [data_msb, data_lsb]

        all_successful = True
        for channel in ['A', 'B', 'C', 'D']:
            try:
                # The command byte is specific to each channel for a single write.
                # Format: 01011(C1)(C0)0
                command_byte = 0b01011000 | (CHANNEL_MAP[channel] << 1)

                bus.write_i2c_block_data(MCP4728_ADDRESS, command_byte, i2c_payload)
                print(f"Successfully set channel {channel} to 0V.")
                time.sleep(0.1) # Small delay for stability

            except IOError as e:
                print(f"Error setting channel {channel}: Could not communicate with I2C device at 0x{MCP4728_ADDRESS:02X}. {e}")
                print("Please check connections and I2C configuration.")
                all_successful = False
                break # Stop on first error

        if all_successful:
            print("\nFinished setting all channels to 0V.")

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        if bus:
            bus.close()
            print("I2C bus closed.")


if __name__ == "__main__":
    main()


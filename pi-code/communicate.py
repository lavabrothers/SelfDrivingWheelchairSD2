import smbus2
import time
import math

# --- Configuration ---
# I2C bus number. For Raspberry Pi 5, it's typically 1.
I2C_BUS = 1

# I2C address of the MCP4728. Default is 0x60.
# Check your hardware for any address modifications.
MCP4728_ADDRESS = 0x60

# --- Channel Frequencies ---
# Define the frequency in Hertz (Hz) for the sine wave output on each channel.
CHANNEL_FREQUENCIES = {
    'A': 0.01,  # 0.5 Hz, one cycle every 2 seconds
    'B': 0.01,  # 1.0 Hz, one cycle every 1 second
    'C': 0.01,  # 2.0 Hz, two cycles per second
    'D': 0.01,  # 4.0 Hz, four cycles per second
}

# --- MCP4728 Internal Settings ---
# VDD supply voltage for the DAC.
# This is crucial for calculating the correct digital value and sets the peak voltage.
# IMPORTANT: The MCP4728's maximum output voltage is limited by its VDD supply.
# To get 4.0V out, VDD for the DAC must be > 4.0V (e.g., 5V).
VDD = 5.0

# VREF setting: 0 for VDD, 1 for internal 2.048V reference.
# To achieve > 2.048V output, you MUST use VDD as the reference.
VREF_MODE = 0  # 0 = VDD, 1 = Internal 2.048V

# Gain setting: 0 for 1x, 1 for 2x.
# If using the internal Vref (2.048V), you can use 2x gain to get up to 4.096V.
# If using VDD as Vref, gain must be 1x.
GAIN_MODE = 0 # 0 = 1x, 1 = 2x

# --- Internal Constants ---
CHANNEL_MAP = {
    'A': 0b00,
    'B': 0b01,
    'C': 0b10,
    'D': 0b11
}

def main():
    """
    Main function to initialize I2C and run the voltage oscillation loop.
    Uses a single "Sequential Write" command to update all four channels
    at once for efficiency and reliability.
    """
    bus = None
    try:
        bus = smbus2.SMBus(I2C_BUS)
        print("I2C bus opened. Starting DAC voltage oscillations using sequential writes.")
        print("Press Ctrl+C to stop.")

        # Print a one-time warning if the configuration might be confusing.
        if VREF_MODE == 0 and GAIN_MODE == 1:
            print("Warning: Gain is set to 2x but VDD is used as Vref. Gain will be treated as 1x by the DAC.")

        start_time = time.time()
        first_run = True

        while True:
            elapsed_time = time.time() - start_time

            # This command tells the MCP4728 to expect data for all four channels
            # sequentially, starting from Channel A.
            SEQ_WRITE_COMMAND = 0b01000000

            i2c_payload = []
            
            # The order must be A, B, C, D for a sequential write.
            channel_order = ['A', 'B', 'C', 'D']

            for channel in channel_order:
                frequency = CHANNEL_FREQUENCIES[channel]

                # Calculate the target voltage using a sine wave.
                # The wave oscillates between 0.0V and VDD.
                amplitude = VDD / 2.0
                offset = VDD / 2.0
                voltage = amplitude * math.sin(2 * math.pi * frequency * elapsed_time) + offset

                # --- Calculate Data Bytes for this Channel ---
                v_ref = VDD if VREF_MODE == 0 else 2.048 * (2 if GAIN_MODE == 1 else 1)
                
                # Clamp voltage to the valid range [0, v_ref]
                if voltage > v_ref: voltage = v_ref
                if voltage < 0: voltage = 0
                
                digital_value = int((voltage / v_ref) * 4095)
                
                # Construct the two data bytes for this channel
                data_msb = ((VREF_MODE & 0x01) << 7 |
                            (0 & 0x01) << 6 |  # Power Down bits (00 = normal operation)
                            (0 & 0x01) << 5 |
                            (GAIN_MODE & 0x01) << 4 |
                            ((digital_value >> 8) & 0x0F))
                data_lsb = digital_value & 0xFF
                
                # Add the two bytes for this channel to our payload
                i2c_payload.extend([data_msb, data_lsb])

            try:
                # Write all 8 bytes (2 per channel) in a single I2C transaction
                bus.write_i2c_block_data(MCP4728_ADDRESS, SEQ_WRITE_COMMAND, i2c_payload)
                if first_run:
                    print("First sequential write successful. All channels are now oscillating.")
                    first_run = False
            except IOError as e:
                if first_run:
                    print(f"Error: Could not communicate with I2C device at 0x{MCP4728_ADDRESS:02X}. {e}")
                    print("Please check connections and I2C configuration. The program will exit.")
                    break  # Exit the loop on the first error
                # If not the first run, we can let it try again on the next loop.
            
            # A small delay to control the update rate and prevent 100% CPU usage.
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nProgram stopped by user. Closing I2C bus.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        if bus:
            # As a cleanup, set all channels to 0V before closing.
            print("Setting all DAC channels to 0V.")
            zero_volt_payload = []
            for _ in range(4): # For each of the 4 channels
                # MSB contains VREF and GAIN settings. LSB is all 0 for 0V.
                data_msb = ((VREF_MODE & 0x01) << 7 | (GAIN_MODE & 0x01) << 4)
                data_lsb = 0
                zero_volt_payload.extend([data_msb, data_lsb])
            try:
                SEQ_WRITE_COMMAND = 0b01000000
                bus.write_i2c_block_data(MCP4728_ADDRESS, SEQ_WRITE_COMMAND, zero_volt_payload)
            except IOError:
                print("Could not set DACs to 0V on exit. Device may be disconnected.")
            bus.close()
            print("I2C bus closed.")


if __name__ == "__main__":
    main()


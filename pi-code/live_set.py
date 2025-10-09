import board
import adafruit_mcp4728
import time
import curses
import smbus2
import math

# --- I2C Addresses and Registers ---
MPU9250_ADDRESS = 0x68
AK8963_ADDRESS = 0x0C
# MPU9250 Registers
PWR_MGMT_1 = 0x6B
INT_PIN_CFG = 0x37
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
# AK8963 Registers
AK8963_ST1 = 0x02
AK8963_XOUT_L = 0x03
AK8963_CNTL = 0x0A
AK8963_ASAX = 0x10

# --- Sensor Configuration ---
ACCEL_FS_2G = 0x00
GYRO_FS_250DPS = 0x00
AK8963_MODE_C100HZ = 0x06
AK8963_BIT_16 = 0x10

# --- Setup ---
try:
    bus = smbus2.SMBus(1)
    # Wake up MPU-9250 from sleep
    bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    bus.write_byte_data(MPU9250_ADDRESS, ACCEL_CONFIG, ACCEL_FS_2G)
    bus.write_byte_data(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FS_250DPS)
    # Enable I2C passthrough for magnetometer
    bus.write_byte_data(MPU9250_ADDRESS, INT_PIN_CFG, 0x02)
    time.sleep(0.1)
    # Configure Magnetometer
    bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL, (AK8963_BIT_16 | AK8963_MODE_C100HZ))
    print("MPU-9250 (Accel/Gyro/Mag) fully configured.")
except Exception as e:
    print(f"Error: Could not initialize MPU-9250. Check connections. Details: {e}")
    exit()

try:
    i2c = board.I2C()
    mcp = adafruit_mcp4728.MCP4728(i2c)
    print("MCP4728 DAC found and initialized.")
except ValueError:
    print("Error: Could not find MCP4728. Please check I2C connections.")
    exit()

# --- Helper Functions ---
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

def calculate_heading(mag_x, mag_y):
    heading = math.degrees(math.atan2(mag_y, mag_x))
    return (heading + 360) % 360

def get_sensor_data():
    accel_x = read_word_2c(MPU9250_ADDRESS, ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(MPU9250_ADDRESS, ACCEL_XOUT_H + 2) / 16384.0
    accel_z = read_word_2c(MPU9250_ADDRESS, ACCEL_XOUT_H + 4) / 16384.0
    
    gyro_x = read_word_2c(MPU9250_ADDRESS, GYRO_XOUT_H) / 131.0
    gyro_y = read_word_2c(MPU9250_ADDRESS, GYRO_XOUT_H + 2) / 131.0
    gyro_z = read_word_2c(MPU9250_ADDRESS, GYRO_XOUT_H + 4) / 131.0
    
    mag_x, mag_y = 0, 0
    if bus.read_byte_data(AK8963_ADDRESS, AK8963_ST1) & 0x01:
        mag_x = read_mag_word(AK8963_ADDRESS, AK8963_XOUT_L)
        mag_y = read_mag_word(AK8963_ADDRESS, AK8963_XOUT_L + 2)
        # Must read ST2 register to complete measurement
        bus.read_byte_data(AK8963_ADDRESS, 0x09) 
    
    return (accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), (mag_x, mag_y)

# --- Main Control Loop ---
def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    mcp.channel_a.normalized_value, mcp.channel_b.normalized_value, mcp.channel_c.normalized_value, mcp.channel_d.normalized_value = 0.5, 0.5, 0.5, 0.5

    stdscr.addstr(0, 0, "--- Real-time Wheelchair Control ---")
    stdscr.addstr(1, 0, "Controls: 'w' 'a' 's' 'd' | 'q' to exit")
    stdscr.addstr(2, 0, "------------------------------------")

    mag_cache = (0, 0)

    while True:
        key = stdscr.getch()
        
        val_a, val_b, val_c, val_d = 0.5, 0.5, 0.5, 0.5
        if key == ord('w'): val_a, val_b = 0.75, 0.25
        elif key == ord('s'): val_a, val_b = 0.25, 0.75
        elif key == ord('d'): val_c, val_d = 0.75, 0.25
        elif key == ord('a'): val_c, val_d = 0.25, 0.75
        elif key == ord('q'): break
        
        mcp.channel_a.normalized_value, mcp.channel_b.normalized_value, mcp.channel_c.normalized_value, mcp.channel_d.normalized_value = val_a, val_b, val_c, val_d

        accel, gyro, mag = get_sensor_data()
        if mag != (0, 0):
            mag_cache = mag
        
        heading = calculate_heading(mag_cache[0], mag_cache[1])

        stdscr.addstr(4, 0, f"DAC    : F({val_a:.2f}), B({val_b:.2f}), R({val_c:.2f}), L({val_d:.2f})")
        stdscr.addstr(5, 0, f"Heading: {heading:<7.2f} degrees")
        stdscr.addstr(6, 0, f"Accel g: X={accel[0]:<7.2f} Y={accel[1]:<7.2f} Z={accel[2]:<7.2f}")
        stdscr.addstr(7, 0, f"Gyro dps: X={gyro[0]:<7.2f} Y={gyro[1]:<7.2f} Z={gyro[2]:<7.2f}      ")
        stdscr.refresh()

if __name__ == "__main__":
    try:
        curses.wrapper(main)
    finally:
        mcp.channel_a.normalized_value, mcp.channel_b.normalized_value, mcp.channel_c.normalized_value, mcp.channel_d.normalized_value = 0.5, 0.5, 0.5, 0.5
        print("\nDAC set to neutral. Program terminated.")

import smbus2
import time

# MPU6050 I2C address
MPU6050_ADDRESS = 0x68

# MPU6050 Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
TEMP_OUT_H = 0x41

# Initialize I2C bus
try:
    bus = smbus2.SMBus(1) # Use bus 1 for Raspberry Pi
    print("I2C bus initialized. ✅")
except Exception as e:
    print(f"Error initializing I2C bus: {e}")
    exit()

# Helper function to read 16-bit signed data
def read_word_2c(reg):
    high = bus.read_byte_data(MPU6050_ADDRESS, reg)
    low = bus.read_byte_data(MPU6050_ADDRESS, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

# Wake up MPU6050
try:
    bus.write_byte_data(MPU6050_ADDRESS, PWR_MGMT_1, 0x00)
    print("MPU6050 woken up. ✅")
    time.sleep(0.1)
except Exception as e:
    print(f"Error waking up MPU6050: {e}")
    print("Failed to find MPU6050 - check your wiring and I2C address!")
    exit()

print("Reading MPU6050 data using smbus2...")

# Main loop to read and print data
while True:
    # Read accelerometer data
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0 # LSB sensitivity for +/- 2g
    accel_y = read_word_2c(ACCEL_XOUT_H + 2) / 16384.0
    accel_z = read_word_2c(ACCEL_XOUT_H + 4) / 16384.0
    
    # Read gyroscope data
    gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0 # LSB sensitivity for +/- 250 deg/s
    gyro_y = read_word_2c(GYRO_XOUT_H + 2) / 131.0
    gyro_z = read_word_2c(GYRO_XOUT_H + 4) / 131.0
    
    # Read temperature data
    temp_raw = read_word_2c(TEMP_OUT_H)
    temp_c = (temp_raw / 340.0) + 36.53 # MPU6050 temperature formula
    temp_f = (temp_c * 9/5) + 32
    
    # Print the readings, formatted to two decimal places
    print(f"Acceleration: X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f} g")
    print(f"Gyroscope: X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f} deg/s")
    print(f"Temperature: {temp_f:.2f} °F") 
    
    print("-" * 20) # Separator for readability
    
    # Wait for a short period before the next reading
    time.sleep(0.25)

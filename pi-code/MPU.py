# Import necessary libraries
import time
import board
import adafruit_mpu6050

# Initialize the I2C bus using the board's default I2C pins
i2c = board.I2C()  # uses board.SCL and board.SDA

# Create an MPU6050 sensor object
mpu = adafruit_mpu6050.MPU6050(i2c, address=0x68)

print("Reading MPU6050 data...")

# Main loop to read and print data
while True:
    # Read accelerometer data
    accel_x, accel_y, accel_z = mpu.acceleration
    
    # Read gyroscope data
    gyro_x, gyro_y, gyro_z = mpu.gyro
    
    # --- TEMPERATURE CHANGE START ---
    
    # Read temperature data in Celsius
    temp_c = mpu.temperature
    
    # Convert Celsius to Fahrenheit
    temp_f = (temp_c * 9/5) + 32
    
    # --- TEMPERATURE CHANGE END ---
    
    
    # Print the readings, formatted to two decimal places
    print(f"Acceleration: X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f} m/s^2")
    print(f"Gyroscope: X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f} rad/s")
    
    # --- UPDATED PRINT STATEMENT ---
    print(f"Temperature: {temp_f:.2f} °F") 
    
    print("-" * 20) # Separator for readability
    
    # Wait for a second before the next reading
    time.sleep(0.25)

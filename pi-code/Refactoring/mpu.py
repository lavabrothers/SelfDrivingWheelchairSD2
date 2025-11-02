# File: imu_sensor.py
"""
Module to interface with the MPU-9250 IMU using the 'mpu9250-jmdev' library.

FINAL CORRECTED VERSION:
This version fixes the method naming conventions (uses camelCase) and
the correct register constant (MPU9050_ADDRESS_68) based on the user's
working example script.
"""

import time
import math
import sys
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# --- Configuration ---
# You must run calibration (python3 imu_sensor.py) to get good readings.
# After calibration, find your local magnetic declination and enter it here.
# Find it at: https://www.magnetic-declination.com/
# Example: University Park, FL is -7.11 degrees (Updated for 2025)
MAGNETIC_DECLINATION = -7.11 

# Smoothing factor (0.0 = no smoothing, 0.9 = very heavy smoothing)
SMOOTHING_ALPHA = 0.7 # 0.0 = raw, 0.9 = very smooth

# --- Global Variables ---
mpu = None
last_smoothed_heading = None

def initialize_imu():
    """
    Initializes the MPU-9250 sensor using the mpu9250-jmdev library.
    Returns True on success, False on failure.
    """
    global mpu
    try:
        print("Initializing MPU-9250...")
        mpu = MPU9250(
            address_ak=AK8963_ADDRESS,
            # --- CORRECTED CONSTANT ---
            address_mpu_master=MPU9050_ADDRESS_68, 
            address_mpu_slave=None,
            bus=1,
            gfs=GFS_1000,
            afs=AFS_8G,
            mfs=AK8963_BIT_16,
            mode=AK8963_MODE_C100HZ
        )
        
        # Configure the MPU-9250
        mpu.configure() 
        
        # Load calibration data if it exists
        try:
            # --- CORRECTED METHOD NAME (camelCase) ---
            mpu.loadCalibration()
            print("Loaded existing calibration data.")
        except FileNotFoundError:
            print_calibration_warning()
        except AttributeError:
            print_calibration_warning() # Also catch if method doesn't exist
            
        print("MPU-9250 Initialized Successfully.")
        return True
        
    except Exception as e:
        print(f"Error initializing MPU-9250: {e}")
        print("Please check I2C connection and bus number (bus=1).")
        return False

def print_calibration_warning():
    """Prints a warning message about needing calibration."""
    print("="*50)
    print("WARNING: MPU-9250 CALIBRATION FILE NOT FOUND.")
    print("Magnetometer data will be inaccurate!")
    print("Please run this script directly to calibrate:")
    print("  python3 imu_sensor.py")
    print("="*50)

def get_current_heading():
    """
    Reads the sensors, calculates a tilt-compensated heading, and smooths it.
    Returns the heading in degrees (0-360).
    """
    global mpu, last_smoothed_heading
    
    if mpu is None:
        print("Error: IMU not initialized.")
        return 0.0

    try:
        # 1. Read sensor data
        # --- CORRECTED METHOD NAMES (camelCase) ---
        accel_data = mpu.readAccelerometerMaster()
        mag_data = mpu.readMagnetometerMaster()
        
        ax, ay, az = accel_data[0], accel_data[1], accel_data[2]
        mx, my, mz = mag_data[0], mag_data[1], mag_data[2]

        # 2. Calculate Tilt Compensation
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        
        # Compensate magnetometer readings
        mag_x_comp = mx * math.cos(pitch) + mz * math.sin(pitch)
        mag_y_comp = (mx * math.sin(roll) * math.sin(pitch) + 
                      my * math.cos(roll) - 
                      mz * math.sin(roll) * math.cos(pitch))
        
        # 3. Calculate Heading
        heading_rad = math.atan2(mag_y_comp, mag_x_comp)
        
        # Convert to degrees, apply declination, and normalize
        heading_deg = math.degrees(heading_rad)
        heading_deg += MAGNETIC_DECLINATION
        heading_deg = heading_deg % 360
            
        # 4. Apply Smoothing (Exponential Moving Average)
        if last_smoothed_heading is None:
            last_smoothed_heading = heading_deg
        else:
            diff = heading_deg - last_smoothed_heading
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
                
            new_heading = (last_smoothed_heading * SMOOTHING_ALPHA) + (heading_deg * (1.0 - SMOOTHING_ALPHA))
            last_smoothed_heading = new_heading % 360

        return last_smoothed_heading

    except Exception as e:
        print(f"Error reading IMU: {e}")
        return last_smoothed_heading if last_smoothed_heading is not None else 0.0

# --- CALIBRATION ROUTINE ---
if __name__ == "__main__":
    
    print("Running MPU-9250 Test & Calibration...")
    
    if not initialize_imu():
        print("Initialization failed. Exiting.")
        sys.exit(1)
        
    print("\n--- Sensor Test (Press Ctrl+C to stop test) ---")
    try:
        while True:
            heading = get_current_heading()
            print(f"Current Tilt-Compensated Heading: {heading: >7.2f} degrees", end="\r")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nSensor test finished.")
        pass # Continue to calibration
    
    print("\n--- MPU-9250 CALIBRATION ---")
    print("This will calibrate the magnetometer.")
    print("The library will save calibration data to 'calib.json'.")
    try:
        input("Press Enter to begin magnetometer calibration (move sensor in figure-8)...")
        # --- CORRECTED METHOD NAME (camelCase) ---
        mpu.calibrateMagnetometer() 
        print("\n--- Magnetometer Calibration Complete! ---")
        
        input("Press Enter to test gyroscope calibration (keep sensor still)...")
        # --- CORRECTED METHOD NAME (camelCase) ---
        mpu.calibrateGyroscope()
        print("\n--- Gyroscope Calibration Complete! ---")
        
        print("\nCalibration data has been saved.")
        print("You can now run your main controller scripts.")
        
    except AttributeError:
        print("\n--- CALIBRATION FAILED ---")
        print("Your version of the 'mpu9250-jmdev' library")
        print("does not seem to have the .calibrate...() methods.")
        print("Please find the 'mag_cal.py' script from the library")
        print("and run it manually to generate 'calib.json'.")
    except Exception as e:
        print(f"\nCalibration failed: {e}")
    except KeyboardInterrupt:
        print("\nCalibration cancelled by user.")
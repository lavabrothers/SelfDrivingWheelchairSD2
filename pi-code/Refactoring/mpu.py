"""
mpu.py

This module provides an interface for the MPU-9250 Inertial Measurement Unit (IMU)
using the `mpu9250-jmdev` library. It is designed to initialize the sensor,
read accelerometer and magnetometer data, calculate a tilt-compensated heading,
and apply smoothing to the heading readings.

This version specifically addresses and corrects method naming conventions
(using camelCase) and ensures the correct I2C address constant (MPU9050_ADDRESS_68)
is used, based on a working example. It also includes a calibration routine
for the magnetometer and gyroscope.

Key Features:
- Initializes the MPU-9250 with specified settings (gyroscope, accelerometer, magnetometer ranges).
- Loads existing calibration data or warns if not found.
- Calculates tilt-compensated magnetic heading, applying magnetic declination.
- Applies exponential moving average smoothing to the heading for stability.
- Provides a test and calibration routine when run as a standalone script.

Dependencies:
- time: For delays in the calibration routine.
- math: For trigonometric calculations (roll, pitch, heading).
- sys: For system exit in case of fatal initialization errors.
- mpu9250_jmdev: Library for MPU-9250 communication and data processing.
"""

import time
import math
import sys
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# --- Configuration ---
# Magnetic declination for the operating location.
# This value corrects the magnetic north reading to true north.
# Obtain from: https://www.magnetic-declination.com/ (e.g., University Park, FL is -7.11 degrees for 2025).
MAGNETIC_DECLINATION = -7.11 

# Smoothing factor for the heading calculation (Exponential Moving Average).
# A value of 0.0 means no smoothing (raw data).
# A value closer to 1.0 (e.g., 0.9) means very heavy smoothing (slow to react to changes).
SMOOTHING_ALPHA = 0.7

# --- Global Variables ---
mpu: MPU9250 | None = None  # MPU9250 object instance.
last_smoothed_heading: float | None = None # Stores the last calculated smoothed heading.

def initialize_imu() -> bool:
    """
    Initializes the MPU-9250 sensor.

    This function creates an instance of the MPU9250 class, configures its
    settings (I2C address, gyroscope, accelerometer, magnetometer ranges),
    and attempts to load existing calibration data.

    Returns:
        bool: True if the MPU-9250 sensor was successfully initialized, False otherwise.
    """
    global mpu
    try:
        print("Initializing MPU-9250...")
        mpu = MPU9250(
            address_ak=AK8963_ADDRESS,
            address_mpu_master=MPU9050_ADDRESS_68, # Corrected I2C address for MPU-9250.
            address_mpu_slave=None,
            bus=1,          # I2C bus number (typically 1 for Raspberry Pi).
            gfs=GFS_1000,   # Gyroscope Full Scale Range: +/- 1000 dps.
            afs=AFS_8G,     # Accelerometer Full Scale Range: +/- 8G.
            mfs=AK8963_BIT_16, # Magnetometer Full Scale Range: 16-bit output.
            mode=AK8963_MODE_C100HZ # Magnetometer operating mode: Continuous measurement at 100Hz.
        )
        
        # Apply the configured settings to the MPU-9250.
        mpu.configure() 
        
        # Attempt to load calibration data from 'calib.json'.
        try:
            mpu.loadCalibration() # Corrected method name (camelCase).
            print("Loaded existing calibration data.")
        except FileNotFoundError:
            print_calibration_warning()
        except AttributeError:
            # Catch if the `loadCalibration` method is missing (e.g., older library version).
            print_calibration_warning()
            
        print("MPU-9250 Initialized Successfully.")
        return True
        
    except Exception as e:
        print(f"Error initializing MPU-9250: {e}")
        print("Please check I2C connection and bus number (bus=1).")
        return False

def print_calibration_warning():
    """
    Prints a warning message to the console, advising the user to perform
    magnetometer calibration if the calibration file is not found.
    """
    print("="*50)
    print("WARNING: MPU-9250 CALIBRATION FILE NOT FOUND.")
    print("Magnetometer data will be inaccurate!")
    print("Please run this script directly to calibrate:")
    print("  python3 mpu.py")
    print("="*50)

def get_current_heading() -> float:
    """
    Reads accelerometer and magnetometer data from the MPU-9250,
    calculates a tilt-compensated magnetic heading, applies magnetic declination,
    and then smooths the result using an Exponential Moving Average.

    Returns:
        float: The current smoothed heading in degrees (0-360), where 0 is North.
               Returns 0.0 if the IMU is not initialized or an error occurs.
    """
    global mpu, last_smoothed_heading
    
    if mpu is None:
        print("Error: IMU not initialized.")
        return 0.0

    try:
        # 1. Read raw sensor data.
        accel_data = mpu.readAccelerometerMaster() # Corrected method name (camelCase).
        mag_data = mpu.readMagnetometerMaster()    # Corrected method name (camelCase).
        
        ax, ay, az = accel_data[0], accel_data[1], accel_data[2]
        mx, my, mz = mag_data[0], mag_data[1], mag_data[2]

        # 2. Calculate Roll and Pitch from accelerometer data for tilt compensation.
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        
        # 3. Compensate magnetometer readings for tilt.
        # This rotates the magnetic field vector into the horizontal plane.
        mag_x_comp = mx * math.cos(pitch) + mz * math.sin(pitch)
        mag_y_comp = (mx * math.sin(roll) * math.sin(pitch) + 
                      my * math.cos(roll) - 
                      mz * math.sin(roll) * math.cos(pitch))
        
        # 4. Calculate raw heading from compensated magnetometer data.
        heading_rad = math.atan2(mag_y_comp, mag_x_comp)
        
        # Convert heading to degrees, apply magnetic declination, and normalize to 0-360.
        heading_deg = math.degrees(heading_rad)
        heading_deg += MAGNETIC_DECLINATION
        heading_deg = heading_deg % 360
            
        # 5. Apply Smoothing (Exponential Moving Average) to the heading.
        if last_smoothed_heading is None:
            last_smoothed_heading = heading_deg
        else:
            # Handle wrap-around for heading (e.g., 359 to 1 degree change).
            diff = heading_deg - last_smoothed_heading
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
            
            # Apply smoothing formula.
            new_heading = (last_smoothed_heading * SMOOTHING_ALPHA) + (heading_deg * (1.0 - SMOOTHING_ALPHA))
            last_smoothed_heading = new_heading % 360 # Ensure heading remains within 0-360.

        return last_smoothed_heading

    except Exception as e:
        print(f"Error reading IMU: {e}")
        # Return the last known smoothed heading or 0.0 if none exists.
        return last_smoothed_heading if last_smoothed_heading is not None else 0.0

# --- CALIBRATION ROUTINE ---
if __name__ == "__main__":
    """
    Main execution block for testing and calibrating the MPU-9250 IMU.

    When run directly, this script first performs a continuous sensor test
    displaying the current heading. Upon KeyboardInterrupt, it proceeds to
    guide the user through magnetometer and gyroscope calibration routines.
    Calibration data is saved to 'calib.json'.
    """
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
        pass # Continue to calibration.
    
    print("\n--- MPU-9250 CALIBRATION ---")
    print("This will calibrate the magnetometer.")
    print("The library will save calibration data to 'calib.json'.")
    try:
        input("Press Enter to begin magnetometer calibration (move sensor in figure-8)...")
        mpu.calibrateMagnetometer() # Corrected method name (camelCase).
        print("\n--- Magnetometer Calibration Complete! ---")
        
        input("Press Enter to test gyroscope calibration (keep sensor still)...")
        mpu.calibrateGyroscope() # Corrected method name (camelCase).
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

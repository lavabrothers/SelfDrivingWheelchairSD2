import time
import curses
import smbus2
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# --- I2C Addresses and Registers ---
MPU9250_ADDRESS = 0x68
AK8963_ADDRESS = 0x0C
INT_PIN_CFG = 0x37
USER_CTRL = 0x6A

def enable_passthrough():
    """
    Tries to enable I2C passthrough mode on the MPU-9250.
    This is often necessary to communicate with the AK8963 magnetometer.
    """
    try:
        bus = smbus2.SMBus(1)
        # Disable I2C master mode
        bus.write_byte_data(MPU9250_ADDRESS, USER_CTRL, 0x00)
        time.sleep(0.01)
        # Enable bypass multiplexer
        bus.write_byte_data(MPU9250_ADDRESS, INT_PIN_CFG, 0x02)
        time.sleep(0.01)
        print("I2C passthrough mode enabled.")
        return True
    except Exception as e:
        print(f"Failed to enable I2C passthrough mode: {e}")
        return False

def main(stdscr):
    # --- Curses Setup ---
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    # --- Instructions ---
    stdscr.addstr(0, 0, "--- MPU-9250 Sensor Reading ---")
    stdscr.addstr(1, 0, "Press 'q' to exit.")
    stdscr.addstr(2, 0, "---------------------------------")
    
    # --- Sensor Initialization ---
    if not enable_passthrough():
        stdscr.addstr(4, 0, "Could not enable passthrough. Check MPU-9250 connection.")
        stdscr.refresh()
        time.sleep(5)
        return

    try:
        mpu = MPU9250(
            address_ak=AK8963_ADDRESS,
            address_mpu_master=MPU9250_ADDRESS,
            bus=1,
            gfs=GFS_1000,
            afs=AFS_8G,
            mode=AK8963_MODE_C100HZ
        )
        stdscr.addstr(4, 0, "Calibrating sensor... Please keep it still.")
        stdscr.refresh()
        mpu.calibrate()
        mpu.configure()
        stdscr.addstr(5, 0, "Sensor calibrated and configured.")
        stdscr.refresh()
        time.sleep(1)

    except Exception as e:
        stdscr.addstr(4, 0, f"Failed to initialize MPU-9250 library. Error: {e}")
        stdscr.addstr(5, 0, "This likely confirms a hardware issue with the magnetometer.")
        stdscr.refresh()
        time.sleep(5)
        return

    while True:
        if stdscr.getch() == ord('q'):
            break

        # --- Read and Display Data ---
        accel = mpu.readAccelerometerMaster()
        gyro = mpu.readGyroscopeMaster()
        mag = mpu.readMagnetometerMaster()

        accel_str = f"Accel(g):  X: {accel[0]:>6.2f}, Y: {accel[1]:>6.2f}, Z: {accel[2]:>6.2f}"
        gyro_str = f"Gyro(dps): X: {gyro[0]:>6.2f}, Y: {gyro[1]:>6.2f}, Z: {gyro[2]:>6.2f}"
        mag_str = f"Mag(uT):   X: {mag[0]:>6.2f}, Y: {mag[1]:>6.2f}, Z: {mag[2]:>6.2f}"

        stdscr.addstr(7, 0, accel_str)
        stdscr.addstr(8, 0, gyro_str)
        stdscr.addstr(9, 0, mag_str)
        stdscr.refresh()

        time.sleep(0.1)

if __name__ == "__main__":
    try:
        curses.wrapper(main)
    finally:
        print("\nProgram terminated.")

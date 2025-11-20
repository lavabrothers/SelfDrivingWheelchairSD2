# SmartStride: The Future of Mobility
### Hands-Free, Affordable, and Intelligent

## Overview

SmartStride is an affordable, hands-free mobility system designed to retrofit existing electric wheelchairs with autonomous capabilities. The system integrates a Raspberry Pi 5-based control unit that processes data from a magnetometer and an infrared depth sensor (Kinect V2) to enable real-time obstacle detection and navigation. The goal of this project is to improve the mobility and overall safety of users with limited body control through intelligent motion assistance, all while maintaining an affordable cost and an easy-to-implement modular design.

## Features

The SmartStride system offers several modes of operation, controlled via a custom ESP32-based wireless controller:

- **Mapping:** Utilizes the Kinect sensor's IR Flood Sensor to generate a 3D point cloud map of the surroundings by rotating the chair 360°.
- **Autocruise:** The wheelchair cruises forward at a steady pace and stops promptly if an object obstructs its path. Once the path is clear, it continues cruising.
- **Follow Person:** Using the Kinect's camera, the wheelchair will identify and follow the torso of an individual.
- **Manual Control:** Reverts to standard joystick operation, allowing the wheelchair to be driven manually, just as it was originally designed.

## System Architecture

### Hardware

The system is built around a Raspberry Pi 5 as the central processing unit, which coordinates sensor inputs and motor controls.

- **Central Processing Unit:** Raspberry Pi 5
- **Depth Sensing & Vision:** Microsoft Kinect V2
- **Wireless Control:** ESP32-based controller with joystick
- **Orientation & Motion Tracking:** MPU-9250 (3-axis gyroscope, magnetometer, accelerometer)
- **Digital-to-Analog Conversion:** MCP4728 DAC to interface with the wheelchair's joystick port.
- **Power & Interfacing:** Four custom-designed PCBs for voltage regulation (12V, 5V, 3.3V) with comprehensive connectivity

### Software

The software architecture is designed for dual-mode control, enabling both manual and autonomous operation.

- **Kinect Interfacing:** The open-source `libfreenect2` library is used to interface with the Kinect V2 sensor, providing high-definition color (RGB) streams, an infrared stream, and a Time-of-Flight (ToF) depth map.
- **Control Logic:** The Raspberry Pi runs scripts to handle both autonomous and manual control. In manual mode, it processes Bluetooth commands from the ESP32. In autonomous mode, it uses data from the Kinect to navigate and avoid obstacles.

## Installation

### Raspberry Pi (Wheelchair Side)

Follow these steps to set up the software on the Raspberry Pi.

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/lavabrothers/SelfDrivingWheelchairSD2.git
    cd SelfDrivingWheelchairSD2
    ```

2.  **Install `libfreenect2` (Kinect Driver):**
    The Kinect sensor requires the `libfreenect2` driver to be compiled and installed from source. This is a prerequisite for the Python library.
    
    First, install the build dependencies:
    ```bash
    sudo apt-get update
    sudo apt-get install -y build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev
    ```
    
    Next, clone the `libfreenect2` repository, build, and install it:
    ```bash
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    ```
    
    Finally, set up the udev rules to allow access to the Kinect device.
    ```bash
    sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
    ```
    After this, you should reboot the Raspberry Pi to ensure the new udev rules are loaded.
    ```bash
    cd ../.. # Return to the root of the SelfDrivingWheelchairSD2 directory
    sudo reboot
    ```
    **Important:** After rebooting, you will need to navigate back into the `SelfDrivingWheelchairSD2` directory to continue the setup.

3.  **Set Up the Python Environment:**
    Navigate to the `pi-code` directory to set up the Python environment.
    ```bash
    cd pi-code
    ```
    Create and activate a virtual environment:
    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    ```

4.  **Install Dependencies:**

    First, install the remaining packages from `requirements.txt`.
    ```bash
    pip install -r requirements.txt
    ```

    Next, install `pylibfreenect2` directly from the git repository, as it requires a special installation step.
    ```bash
    pip install git+https://github.com/r9y9/pylibfreenect2.git@40221e815c182ee31e8da33df717ccdef1bc615f
    ```

    *Note: If the above command fails, you may need to check if you installed libfreenect2 correctly*

5.  **Install and Run the Service:**
    Navigate back to the root directory and use the `servicescript.sh` to install and start the main application as a systemd service. This will ensure it runs automatically on boot.
    ```bash
    cd ..
    sudo ./servicescript.sh install
    ```
    You can check the status of the service at any time with:
    ```bash
    sudo ./servicescript.sh status
    ```

## The Team

- **Matthew Itskovich:** Computer Engineering - PCB Design and Hardware Integration.
- **Evan Rees:** Computer Engineering - Software Design and System Architecture.
- **Adam Lilly:** Electrical Engineering - Power Systems and Construction Consulting.
- **Arturo Lara:** Computer Engineering - Embedded Systems and Programming.

## Acknowledgements

We would like to thank Dr. Chung Yong Chan and Dr. Arthur Weeks for their guidance and feedback throughout the SmartStride project. Their support and mentorship were invaluable in shaping the direction, design process, and successful completion of this project.

Please refer to the Documentation folder for some written work on how this project works!

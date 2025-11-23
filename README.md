# SmartStride

[![Watch the video](https://img.youtube.com/vi/vgjbnsrbdY0/0.jpg)](https://youtu.be/vgjbnsrbdY0?si=ctMS2qIhK_sTW9yw)

*A quick, informal video showcasing the project.*

## Overview

SmartStride is an affordable, easy to use mobility upgrade system designed to retrofit existing electric wheelchairs with autonomous capabilities. The system integrates a Raspberry Pi 5-based control unit that processes data from a IMU and an infrared depth sensor (Kinect V2) to enable real-time obstacle detection and navigation. The goal of this project is to improve the mobility and overall safety of users with limited body control through intelligent motion assistance, all while maintaining an affordable cost and an easy-to-implement modular design. We designed the system around the Kinect V2, an inexpensive sensor array that is quite powerful compared to other sensors of its weight class. A good example is hospital settings moving patients around.

## Features

The system offers several modes of operation, controlled via a custom ESP32-based wireless controller:

- **Follow Person:** Using the Kinect's camera, the wheelchair will identify and follow the torso of an individual.
- **Autocruise:** The wheelchair cruises forward at a steady pace and stops promptly if an object obstructs its path. Once the path is clear, it continues cruising.
- **Manual Control:** Reverts to standard joystick operation, allowing the wheelchair to be driven manually, just as it was originally designed.
- **Mapping:** Utilizes the Kinect sensor's IR Flood Sensor to generate a 3D point cloud map of the surroundings by rotating the chair 360°.

## System Architecture

### Hardware

The system is built around a Raspberry Pi 5 as the central processing unit, which coordinates sensor inputs and motor controls.

- **Central Processing Unit:** Raspberry Pi 5
- **Depth Sensing & Vision:** Microsoft Kinect V2
- **Wireless Control:** ESP32-based controller with joystick
- **Orientation & Motion Tracking:** MPU-9250 (3-axis gyroscope, magnetometer, accelerometer)
- **Digital-to-Analog Conversion:** MCP4728 DAC to interface with the wheelchair's joystick port.
- **Power & Interfacing:** Four custom-designed PCBs for voltage regulation (12V, 5V, 3.3V) and comprehensive connectivity between all components via I2C

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
    The Kinect sensor requires the `libfreenect2` driver. Please follow the official installation instructions at [https://github.com/OpenKinect/libfreenect2](https://github.com/OpenKinect/libfreenect2).

3.  **Set Up the Python Environment:**
    Navigate to the `pi-code` directory to set up the Python environment.
    ```bash
    cd pi-code
    ```
    Create and activate a virtual environment (Make sure its Python3.11 or Python3.12):
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

    *Note: If the above command fails, you may need to check if you installed libfreenect2 correctly. Sometimes you need to make sure the proper enviroment varaibles are set for your enviroment*

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

- **Matthew Itskovich:** Computer Engineering: Designed the ESP32 Code and Controller hardware
- **Evan Rees:** Computer Engineering: Designed Python Code and was Team Lead
- **Adam Lilly:** Electrical Engineering: Designed and Manufactured Regulators
- **Arturo Lara:** Computer Engineering: Admin Content and Soldering


## Project Media

**Longer, more detailed video:**
[![Watch the video](https://img.youtube.com/vi/YJEAqcdVi9I/0.jpg)](https://youtu.be/YJEAqcdVi9I?si=8ahl_1vFBt-6s-wN)

## Documentation

For more in-depth information, please refer to the following documents:

- **[Senior Design Conference Paper](./Documentation/Senior%20Design%20Conference%20Paper.pdf)**
- Additional written work on how this project works can be found in the `Documentation` folder.

## Map Example

**Demo Day Map:**
![Demo Day Map](https://github.com/lavabrothers/SelfDrivingWheelchairSD2/blob/88daa3a8553c74b252f4243f2af78e0d579390b1/Map%20Examples/Apartment.png)

## Acknowledgements

We would like to thank Dr. Chung Yong Chan and Dr. Arthur Weeks for their guidance and feedback throughout the SmartStride project. Their support and mentorship were invaluable in shaping the direction, design process, and successful completion of this project.

Thank you Dr.Wei for providing the wheelchair for use in this project.

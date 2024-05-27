# rpi_diff_hw_control

## Overview

`rpi_diff_hw_control` is a Hardware Interface for Raspberry Pi GPIO Control. This ROS2 package provides a ros2_control hardware interface for controlling motors connected to Raspberry Pi GPIO pins on Robot here i'm using Diffbot. It is specifically designed for use with the Differntial Drive robot platform. The package utilizes the pigpio library to handle GPIO control, PWM, and frequency settings.

## Features

- Control GPIO pins on a Raspberry Pi 4.
- Read the state of GPIO pins. 
- Write high or low states to GPIO pins.
- Integrates with the ROS2_Control framework.

## Programming Languages Used

- ![C++](https://img.shields.io/badge/C++-95%25-blue)
- ![Python](https://img.shields.io/badge/Python-5%25-yellow)

## Prerequisites

- Raspberry Pi 4 with a compatible Linux OS.
- ROS 2 Humble installed. Follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).
- pigpio library installed. Follow the [pigpio library installation guide](https://abyz.me.uk/rpi/pigpio/download.html).

## Installation

### Setting Up the Environment
   
1. **Create a ROS 2 workspace if you don't already have one**:
   ```bash
   mkdir -p ~/ros2_control_ws/src
   cd ~/ros2_control_ws/src

2. **Clone the package repository**:
   ```bash
   git clone https://github.com/Dharan-kumar/rpi_diff_hw_control.git
   
3. **Navigate to the workspace directory and build the package also install the pigpio package as mentioned in prerequisites**:
   ```bash
   cd ~/ros2_control_ws
   colcon build --packages-select pigpio
   colcon build --packages-select rpi_diff_hw_control
   
4. **Source the workspace**:
   ```bash
   source ~/ros2_control_ws/install/setup.bash
   
5. **To run the ros2_control hardware interface node**:
   ```bash
   ros2 run rpi_diff_hw_control ros2_control.launch.py
   
![Screenshot from 2024-05-21 18-56-56](https://github.com/Dharan-kumar/rpi_diff_drive/assets/84310855/7c29b3df-66e0-4c8c-a068-248b95302687)


![Screenshot from 2024-05-21 18-57-58](https://github.com/Dharan-kumar/rpi_diff_drive/assets/84310855/1785a453-adb1-4e9d-9bf6-db16e42d564a)



6. **To teleop the robot run the teleop_twist_keyboard node and twist_to_twist_stamped.launch.py launch file [ Converting geometry_msgs::msg::Twist to geometry_msgs::msg::TwistStamped ]**
   ```bash
   ros2 run rpi_diff_hw_control twist_to_twist_stamped.launch.py


[Screencast from 05-21-2024 07:08:27 PM.webm](https://github.com/Dharan-kumar/rpi_diff_drive/assets/84310855/354be246-4298-4f0b-851f-991ae7f55989)


## Note
1. Make Sure to check pigpio daemon is running when checking with rasberrypi as shown in this link [pigpio library installation guide](https://abyz.me.uk/rpi/pigpio/download.html).
2. For the simulation without hardware (Rasberry Pi) no need to run pigpio daemon.

## Contributing
Contributions are welcome!

## Acknowledgements
This package uses the Pigpio library for GPIO control of Rasberry Pi.
Inspired by various open-source GPIO control projects for Raspberry Pi and also diffdrive_arduino package for ros2_control hardware interface implementation.

I hope this README should provide clear and comprehensive instructions for users to understand, install, and use my **rpi_diff_hw_control** package for ros2_control hardware implementation on Raspberry Pi.

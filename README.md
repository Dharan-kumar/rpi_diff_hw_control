# rpi_diff_drive

## Overview
`rpi_diff_drive` is a ROS2 Hardware Interface for Raspberry Pi GPIO Control. This ROS2 package provides a ros2 control hardware interface for controlling motors connected to Raspberry Pi GPIO pins on Diffbot. It is specifically designed for use with the Differntial Drive robot platform. The package utilizes the pigpio library to handle GPIO control, PWM, and frequency settings.

## Features

- Control GPIO pins on a Raspberry Pi 4.
- Read the state of GPIO pins. 
- Write high or low states to GPIO pins.
- Integrates with the ROS 2 control framework.

## Programming Languages Used

- ![C++](https://img.shields.io/badge/C++-95%25-blue)
- ![Python](https://img.shields.io/badge/Python-5%25-yellow)

## Prerequisites

- Raspberry Pi 4 with a compatible Linux OS.
- ROS 2 Humble installed. Follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).
- pigpio library installed. Follow the [pigpio library installation guide](https://abyz.me.uk/rpi/pigpio/download.html).

## Installation

### Setting Up the Raspberry Pi

1. **Clone the package repository**:
   ```bash
   git clone https://github.com/your_username/ros2_gpio_control.git
   
2. **Create a ROS 2 workspace if you don't already have one**:
   ```bash
   mkdir -p ~/ros2_control_ws/src
   cd ~/ros2_control_ws/src
   
3. **Navigate to the workspace directory and build the package**:
   ```bash
   cd ~/ros2_control_ws
   colcon build --packages-select pigpio
   colcon build --packages-select rpi_diff_drive
   
4. **Source the workspace**:
   ```bash
   source ~/ros2_control_ws/install/setup.bash
   
5. **To run the ros2_control hardware interface node**:
   ```bash
   ros2 run rpi_diff_drive ros2_control.launch.py
   
![Screenshot from 2024-05-20 21-02-07](https://github.com/Dharan-kumar/rpi_diff_drive/assets/84310855/85b0e213-0cf5-457d-b292-f9477c1814a6)


## Contributing
Contributions are welcome! Please read our contributing guidelines before submitting a pull request.

## License
This project is licensed under the MIT License.

## Acknowledgements
This package uses the Pigpio library for GPIO control.
Inspired by various open-source GPIO control projects for Raspberry Pi.

I hope this README should provide clear and comprehensive instructions for users to understand, install, and use your ROS 2 GPIO control package for the Raspberry Pi 4.







   

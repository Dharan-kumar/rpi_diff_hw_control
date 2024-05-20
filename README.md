# ros2_hardware_interface_rasberrypi

## Overview

`ros2_gpio_control` is a ROS 2 package designed to interface with the GPIO pins of a Raspberry Pi 4. It provides a ros2_control hardware interface module for controlling GPIO pins, allowing users to read from and write to the GPIO pins using ROS 2.

## Features

- Control GPIO pins on a Raspberry Pi 4.
- Read the state of GPIO pins.
- Write high or low states to GPIO pins.
- Integrates with the ROS 2 control framework.

## Programming Languages Used

- ![C++](https://img.shields.io/badge/C++-90%25-blue)
- ![Python](https://img.shields.io/badge/Python-10%25-yellow)

## Prerequisites

- Raspberry Pi 4 with a compatible Linux OS.
- ROS 2 Humble installed. Follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).
- pigpio library installed. Follow the [pigpio library installation guide](https://abyz.me.uk/rpi/pigpio/download.html).

## Installation

### Setting Up the Raspberry Pi

1. **Install pigpio Library**:
   ```bash
   sudo apt-get install wiringpi
   
2. **Create a ROS 2 workspace if you don't already have one**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   
3. **Clone the package repository**:
   ```bash
   git clone https://github.com/your_username/ros2_gpio_control.git
   
4. **Navigate to the workspace directory and build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build
   
5. **Source the workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   
6. To run the GPIO control node:
   ```bash
   ros2 run ros2_gpio_control gpio_control_node

7. **Configuring GPIO Pins**:
   You can configure which GPIO pins to control using a configuration file. By default, the package uses config/gpio_params.yaml.

## Example gpio_params.yaml:

gpio_pins:
  - pin: 17
    direction: output
  - pin: 18
    direction: input
  - pin: 27
    direction: output
    
## Launch Files
8. To launch the GPIO control node with the default configuration:
   ```bash
   ros2 launch ros2_gpio_control gpio_control.launch.py

## Configuration
The package can be configured using the following parameters in the gpio_params.yaml file:

## Parameter Name	Description	Example Value
pin	The GPIO pin number	17
direction	Direction of the pin (input or output)	output / input
Example Usage
Edit the configuration file to set up your GPIO pins:

gpio_pins:
  - pin: 17
    direction: output
  - pin: 18
    direction: input
    
9. **Launch the node with the configuration**:
   ```bash
   ros2 launch ros2_gpio_control gpio_control.launch.py params_file:=config/gpio_params.yaml


10. **Publish commands to control the GPIO pins (example for setting pin 17 high)**:
   ```bash
   ros2 topic pub /gpio/set_pin std_msgs/msg/Int32 "{data: 17}"
```
## Contributing
Contributions are welcome! Please read our contributing guidelines before submitting a pull request.

## License
This project is licensed under the MIT License.

## Acknowledgements
This package uses the WiringPi library for GPIO control.
Inspired by various open-source GPIO control projects for Raspberry Pi.
markdown


I hope this README should provide clear and comprehensive instructions for users to understand, install, and use your ROS 2 GPIO control package for the Raspberry Pi 4.







   

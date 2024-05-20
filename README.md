# ros2_hardware_interface_rasberrypi

## Overview

`ros2_gpio_control` is a ROS 2 package designed to interface with the GPIO pins of a Raspberry Pi 4. It provides a ros2_control hardware interface module for controlling GPIO pins, allowing users to read from and write to the GPIO pins using ROS 2.

## Features

- Control GPIO pins on a Raspberry Pi 4.
- Read the state of GPIO pins.
- Write high or low states to GPIO pins.
- Integrates with the ROS 2 control framework.

## Prerequisites

- Raspberry Pi 4 with a compatible Linux OS.
- ROS 2 Humble installed. Follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).
- pigpio library installed.

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

Example gpio_params.yaml:

yaml
Copy code
gpio_pins:
  - pin: 17
    direction: output
  - pin: 18
    direction: input
  - pin: 27
    direction: output
Launch Files
To launch the GPIO control node with the default configuration:

bash
Copy code
ros2 launch ros2_gpio_control gpio_control.launch.py
Configuration
The package can be configured using the following parameters in the gpio_params.yaml file:

Parameter Name	Description	Example Value
pin	The GPIO pin number	17
direction	Direction of the pin (input or output)	output / input
Example Usage
Edit the configuration file to set up your GPIO pins:

yaml
Copy code
gpio_pins:
  - pin: 17
    direction: output
  - pin: 18
    direction: input
Launch the node with the configuration:

bash
Copy code
ros2 launch ros2_gpio_control gpio_control.launch.py params_file:=config/gpio_params.yaml
Publish commands to control the GPIO pins (example for setting pin 17 high):

bash
ros2 topic pub /gpio/set_pin std_msgs/msg/Int32 "{data: 17}"
Contributing
Contributions are welcome! Please read our contributing guidelines before submitting a pull request.

License
This project is licensed under the MIT License.

Acknowledgements
This package uses the WiringPi library for GPIO control.
Inspired by various open-source GPIO control projects for Raspberry Pi.
markdown

### Explanation of Sections

1. **Title and Badges**: Displays the name of the package and relevant badges for ROS 2 and Raspberry Pi.
2. **Overview**: Briefly describes the package's purpose and functionality.
3. **Features**: Lists the key features of the package.
4. **Prerequisites**: Specifies the requirements for running the package, including the necessary hardware and software.
5. **Installation**: Provides step-by-step instructions for setting up the package, including workspace setup and package building.
6. **Usage**: Explains how to run the node, configure GPIO pins, and use launch files.
7. **Configuration**: Details how to configure the package using a YAML file, including parameter descriptions.
8. **Example Usage**: Provides a practical example of configuring and running the package.
9. **Contributing**: Encourages contributions and provides a link to the contributing guidelines.
10. **License**: Specifies the license under which the package is distributed.
11. **Acknowledgements**: Credits any libraries or projects that inspired or were used by the package.

This README should provide clear and comprehensive instructions for users to understand, install, and use your ROS 2 GPIO control package for the Raspberry Pi 4.







   

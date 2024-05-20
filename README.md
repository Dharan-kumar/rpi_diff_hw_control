# ros2_hardware_interface_rasberrypi

## Overview

`ros2_gpio_control` is a ROS 2 package designed to interface with the GPIO pins of a Raspberry Pi 4. It provides a hardware interface module for controlling GPIO pins, allowing users to read from and write to the GPIO pins using ROS 2.

## Features

- Control GPIO pins on a Raspberry Pi 4.
- Read the state of GPIO pins.
- Write high or low states to GPIO pins.
- Integrates with the ROS 2 control framework.

## Prerequisites

- Raspberry Pi 4 with a compatible Linux OS (e.g., Raspberry Pi OS).
- ROS 2 Foxy installed. Follow the [ROS 2 Foxy installation guide](https://docs.ros.org/en/foxy/Installation.html).
- pigpio library installed.

## Installation

### Setting Up the Raspberry Pi

1. **Install WiringPi**:
   ```bash
   sudo apt-get install wiringpi
   

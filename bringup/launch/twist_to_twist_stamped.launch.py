import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rpi_diff_drive',
            executable='twist_to_twist_stamped',
            name='twist_to_twist_stamped_node',
            output='screen',
        )
    ])

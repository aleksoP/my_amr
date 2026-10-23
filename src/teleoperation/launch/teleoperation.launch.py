"""Launch the teleoperation manager; run keyboard_input in a terminal."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = os.path.join(
        get_package_share_directory('teleoperation'), 'config', 'teleoperation.yaml')
    return LaunchDescription([
        Node(
            package='teleoperation', executable='teleoperation_manager',
            name='teleoperation_manager', parameters=[parameters], output='screen'),
    ])

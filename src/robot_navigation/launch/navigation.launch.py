"""Launch the complete Nav2 stack for this robot.

This wrapper uses adapted Jazzy bringup launch files to route velocity through
the shared command mux before smoothing and collision monitoring.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration


def generate_launch_description():
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    default_params_file = os.path.join(
        robot_navigation_dir, 'config', 'nav2.yaml'
    )

    launch_arguments = {
        'namespace': LaunchConfiguration('namespace'),
        'use_namespace': LaunchConfiguration('use_namespace'),

        'slam': IfElseSubstitution(
            LaunchConfiguration('slam'), if_value='True', else_value='False'
        ),
        'map': LaunchConfiguration('map'),
        'use_localization': IfElseSubstitution(
            LaunchConfiguration('use_localization'), if_value='True', else_value='False'
        ),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'params_file': LaunchConfiguration('params_file'),
        'autostart': LaunchConfiguration('autostart'),
        'use_composition': IfElseSubstitution(
            LaunchConfiguration('use_composition'), if_value='True', else_value='False'
        ),
        'use_respawn': LaunchConfiguration('use_respawn'),
        'log_level': LaunchConfiguration('log_level'),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='', description='Top-level robot namespace.'
        ),
        DeclareLaunchArgument(
            'use_namespace', default_value='false',
            description='Apply the namespace to the Nav2 stack.'
        ),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='Use SLAM Toolbox instead of AMCL localization.'
        ),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Absolute path to a map YAML file for AMCL localization.'
        ),
        DeclareLaunchArgument(
            'use_localization', default_value='true',
            description='Start Nav2 localization (AMCL or SLAM Toolbox).'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulation clock.'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Absolute path to the Nav2 parameters YAML file.'
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically configure and activate Nav2 lifecycle nodes.'
        ),
        DeclareLaunchArgument(
            'use_composition', default_value='true',
            description='Run Nav2 nodes in a component container.'
        ),
        DeclareLaunchArgument(
            'use_respawn', default_value='false',
            description='Respawn crashed non-composed Nav2 nodes.'
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info', description='ROS log level.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_navigation_dir, 'launch', 'bringup_mux.launch.py')
            ),
            launch_arguments=launch_arguments.items(),
        ),
    ])

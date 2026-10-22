import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def generate_launch_description():

    navigation_pkg = get_package_share_directory("robot_navigation")

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg, "launch", "navigation.launch.py")
        ),
    )

    return LaunchDescription(
        [
            navigation_launch,
        ]
    )

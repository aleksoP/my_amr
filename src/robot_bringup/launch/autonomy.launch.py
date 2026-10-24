"""Start navigation and the prototype motion command mux."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_pkg = get_package_share_directory("robot_navigation")
    bringup_pkg = get_package_share_directory("robot_bringup")
    dock_detector_pkg = get_package_share_directory("lidar_dock_detector")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value="/home/aleks/workspace/maps/square_map/map.yaml",
        description="Absolute path to the occupancy-grid map YAML file.",
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
        }.items(),
    )
    dock_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                dock_detector_pkg, "launch", "lidar_dock_detector.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time", default="false"),
        }.items(),
    )
    motion_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(bringup_pkg, "config", "twist_mux.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time", default="false")},
        ],
        remappings=[("cmd_vel_out", "/cmd_vel_mux")],
        output="screen",
    )
    return LaunchDescription(
        [map_arg, navigation_launch, dock_detector_launch, motion_mux]
    )

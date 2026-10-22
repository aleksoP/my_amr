import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  bringup_pkg = get_package_share_directory('robot_bringup')

  use_sim_time = LaunchConfiguration('use_sim_time')

  # Include the platform - sensors, motors, and other hardware components
  platform_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(bringup_pkg, 'launch', 'platform.launch.py')
    ),
    launch_arguments={
      'use_sim_time': use_sim_time,
    }.items(),
  )

  # Include the autonomy - navigation, perception, and other autonomous capabilities
  autonomy_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(bringup_pkg, 'launch', 'autonomy.launch.py')
    ),
    launch_arguments={
      'use_sim_time': use_sim_time,
    }.items(),
  )

  # Include the application - user interface, monitoring, and other application-level features
  application_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(bringup_pkg, 'launch', 'application.launch.py')
    ),
    launch_arguments={
      'use_sim_time': use_sim_time,
    }.items(),
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='false',
      description='Use simulation time.',
    ),

    platform_launch,
    autonomy_launch,
    application_launch,
  ])

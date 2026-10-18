import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():

  lidar_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(lidar_pkg, 'launch', 'lidar.launch.py')
    ),
  )

  camera_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(camera_pkg, 'launch', 'camera.launch.py')
    ),
  )

  mcu_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(mcu_pkg, 'launch', 'mcu.launch.py')
    ),
  )


  return LaunchDescription([
    description_launch,
    motor_launch,
    imu_launch,
    lidar_launch,
    camera_launch,
    mcu_launch,
  ])

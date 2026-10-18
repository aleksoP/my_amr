import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
  DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation time.',
  )
  
  DeclareLaunchArgument(
    'robot_name',
    default_value='my_amr',
    description='Name of the robot.',
  )
  
  description_pkg = get_package_share_directory('robot_description')
  motor_pkg = get_package_share_directory('motor_base')
  imu_pkg = get_package_share_directory('imu_driver')
  lidar_pkg = get_package_share_directory('lidar_driver')
  camera_pkg = get_package_share_directory('camera_driver')
  mcu_pkg = get_package_share_directory('mcu_driver')

  # Include the description - Static TF
  description_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(description_pkg, 'launch', 'description.launch.py')
    ),
  ) 
  
  # motor package launch file
  motor_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(motor_pkg, 'launch', 'motor.launch.py')
    ),
  )
  
  imu_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(imu_pkg, 'launch', 'imu.launch.py')
    ),
  )
  
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
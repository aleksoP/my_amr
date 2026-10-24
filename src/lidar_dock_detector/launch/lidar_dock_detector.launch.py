from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'lidar_params_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('lidar_dock_detector'), 'config', 'lidar_dock_detector.yaml']
                ),
            ),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            Node(
                package='lidar_dock_detector',
                executable='lidar_dock_detector_node',
                name='lidar_dock_detector',
                output='screen',
                parameters=[lidar_params_file, {'use_sim_time': use_sim_time}],
            ),
        ]
    )

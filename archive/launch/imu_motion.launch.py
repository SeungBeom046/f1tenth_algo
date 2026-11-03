"""Start BEFORE the drive launch, while the vehicle is stationary."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('logs_dir', default_value='logs'),
        Node(package='my_algo', executable='imu_motion_logger', output='screen',
             parameters=[{'imu_topic': LaunchConfiguration('imu_topic'),
                          'logs_dir': LaunchConfiguration('logs_dir')}]),
    ])

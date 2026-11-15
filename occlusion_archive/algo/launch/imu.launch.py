"""/imu 정규화 재발행. 차를 정지시킨 상태에서 시작해야 정지 보정이 끝난다.

예:
    ros2 launch my_algo imu.launch.py
    ros2 launch my_algo imu.launch.py input_topic:=/livox/imu accel_unit:=g
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('my_algo'), 'config', 'imu.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('input_topic', default_value='/livox/imu',
                              description='드라이버 IMU 토픽 (실차 확인 필요)'),
        DeclareLaunchArgument('accel_unit', default_value='g', description='g | mps2'),
        Node(
            package='my_algo',
            executable='imu_republisher',
            name='imu_republisher',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'input_topic': LaunchConfiguration('input_topic'),
                    'accel_unit': LaunchConfiguration('accel_unit'),
                },
            ],
        ),
    ])

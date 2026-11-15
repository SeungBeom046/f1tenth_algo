"""실험용 TF 전체: 정적 TF + (선택) EKF.

odom -> base_link 발행자는 반드시 하나만 둔다.
    use_ekf:=false (기본)  vesc_to_odom이 발행 (VESC 설정에서 publish_tf: true)
    use_ekf:=true          robot_localization EKF가 발행. 이때 vesc_to_odom publish_tf는 false로 꺼야 한다.

    ros2 launch my_algo tf.launch.py
    ros2 launch my_algo tf.launch.py use_ekf:=true odom_topic:=/vesc/odom
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('my_algo')
    return LaunchDescription([
        DeclareLaunchArgument('use_ekf', default_value='false'),
        DeclareLaunchArgument('odom_topic', default_value='/vesc/odom',
                              description='휠 odom 토픽 (실차 확인 필요)'),
        DeclareLaunchArgument('imu_topic', default_value='/imu',
                              description='imu_republisher 출력 (SI 단위)'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'tf_static.launch.py'))),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_ekf')),
            parameters=[
                os.path.join(share, 'config', 'ekf.yaml'),
                {
                    'odom0': LaunchConfiguration('odom_topic'),
                    'imu0': LaunchConfiguration('imu_topic'),
                },
            ],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
    ])

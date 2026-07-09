"""Launch file for real F1TENTH car with Livox Mid-360."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 1. PointCloud2 → LaserScan 변환
        # Livox Mid-360은 3D 포인트 클라우드를 줌
        # 우리 알고리즘은 2D LaserScan을 씀 → 변환 필요
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.01,
                'min_height': -0.1,    # 지면 위 이 높이부터
                'max_height': 0.5,     # 이 높이까지만 사용
                'angle_min': -3.14159, # -180도
                'angle_max': 3.14159,  # +180도
                'angle_increment': 0.00436,  # 0.25도 간격
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 20.0,
                'use_inf': True,
            }],
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan', '/scan'),
            ]
        ),

        # 2. Wall Following 알고리즘
        Node(
            package='my_algo',
            executable='wall_follow_real',
            name='wall_follow_real_node',
            output='screen',
        ),

        # 3. AEB (안전장치, 항상 켜놓기)
        Node(
            package='my_algo',
            executable='aeb_real',
            name='aeb_real_node',
            output='screen',
        ),

        # 4. rosbridge (Foxglove 연결용)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}],
        ),
    ])
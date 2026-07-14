from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


def generate_launch_description():
    return LaunchDescription([

        # 1. PointCloud2 → LaserScan 변환
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'min_height': -0.1,
                'max_height': 0.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00436,
                'scan_time': 0.1,
                'range_min': 0.3,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'target_frame': 'livox_frame',
            }],
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan', '/scan'),
            ]
        ),

        # 2. 조이스틱 드라이버
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
        ),

        # 3. 조이스틱 컨트롤러
        Node(
            package='my_algo',
            executable='joy_controller',
            name='joy_controller_node',
            output='screen',
        ),

        # 4. Wall Following
        Node(
            package='my_algo',
            executable='wall_follow_real',
            name='wall_follow_real_node',
            output='screen',
        ),

        # 5. AEB
        Node(
            package='my_algo',
            executable='aeb_real',
            name='aeb_real_node',
            output='screen',
        ),

        # 6. rosbridge (delay_between_messages를 float으로 수정)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '',
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0.0,  # int → float으로 수정
                'max_message_size': 10000000,
                'unregister_timeout': 10.0,
            }],
        ),
    ])
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
                'range_min': 0.05,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'target_frame': 'livox_frame',
            }],
            remappings=[
                ('cloud_in', '/livox/lidar'),
                ('scan', '/scan'),
            ],
        ),
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
        Node(
            package='my_algo',
            executable='joy_controller',
            name='joy_controller',
            output='screen',
        ),
        Node(
            package='my_algo',
            executable='lidar_reactive_drive',
            name='lidar_reactive_drive',
            output='screen',
        ),
        Node(
            package='my_algo',
            executable='camera_reactive_drive',
            name='camera_reactive_drive',
            output='screen',
        ),
        Node(
            package='my_algo',
            executable='autonomous_drive',
            name='autonomous_drive',
            output='screen',
            parameters=[{
                'drive_source': 'lidar',
                'command_timeout_sec': 0.35,
                'aeb_ttc_threshold': 0.55,
                'aeb_vehicle_half_width': 0.15,
                'aeb_path_margin': 0.10,
                'lidar_to_bumper_dist': 0.30,
            }],
        ),
        Node(
            package='my_algo',
            executable='autonomous_metrics_logger',
            name='autonomous_metrics_logger',
            output='screen',
            parameters=[{
                'logs_dir': 'logs',
                'servo_center': 0.5,
                'servo_gain': 0.60,
                'front_fov_deg': 120.0,
                'lidar_to_bumper_dist': 0.30,
                'max_valid_speed_mps': 4.0,
            }],
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '',
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0.0,
                'max_message_size': 10000000,
                'unregister_timeout': 10.0,
            }],
        ),
    ])

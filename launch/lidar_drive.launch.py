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
        Node(package='joy', executable='joy_node', name='joy_node'),
        Node(package='my_algo', executable='joy_controller', output='screen'),
        Node(
            package='my_algo',
            executable='lidar_reactive_drive',
            output='screen',
            parameters=[{
                'vehicle_width': 0.30,
                'safety_margin': 0.18,
                'target_wall_distance': 0.85,
                'lookahead_distance': 2.2,
                'max_steering': 0.78,
                'lidar_to_bumper_dist': 0.30,
            }],
        ),
        Node(
            package='my_algo',
            executable='autonomous_drive',
            output='screen',
            parameters=[{
                'drive_source': 'lidar',
                'command_timeout_sec': 0.35,
                'aeb_ttc_threshold': 0.55,
                'aeb_vehicle_half_width': 0.15,
                'aeb_path_margin': 0.10,
                'lidar_to_bumper_dist': 0.30,
                'max_steering': 0.78,
            }],
        ),
        Node(
            package='my_algo',
            executable='autonomous_metrics_logger',
            output='screen',
            parameters=[{
                'lidar_to_bumper_dist': 0.30,
            }],
        ),
    ])

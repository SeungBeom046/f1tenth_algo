import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
        ),
        Node(
            package='my_algo',
            executable='camera_reactive_drive',
            output='screen',
            parameters=[{
                'color_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_rect_raw',
                'annotated_topic': '/camera/perception/annotated_image',
                'status_topic': '/camera/perception/status',
                'yolo_model': 'yolov8n.pt',
                'yolo_confidence': 0.25,
                'show_window': True,
                'process_every_n_frames': 2,
                'enable_yolo': True,
                'enable_lane_detection': True,
            }],
        ),
    ])

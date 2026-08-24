from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
            }],
        ),
        Node(
            package='my_algo',
            executable='camera_reactive_drive',
            output='screen',
            parameters=[{
                'color_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'annotated_topic': '/camera/perception/annotated_image',
                'status_topic': '/camera/perception/status',
                'yolo_model': 'yolov8n.pt',
                'yolo_confidence': 0.35,
                'show_window': True,
                'process_every_n_frames': 2,
                'enable_yolo': True,
                'enable_lane_detection': True,
            }],
        ),
    ])

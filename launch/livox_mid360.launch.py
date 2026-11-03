"""Livox Mid-360 드라이버 (RViz 없음). 실험 세션용 — RViz는 노트북에서 띄운다.

livox_ros_driver2의 rviz_MID360_launch.py와 같은 드라이버 설정이고 rviz2 노드만 뺐다.
    xfer_format 0 = PointCloud2 (우리 파이프라인이 구독하는 형식, 점별 timestamp 필드 포함 여부는 세션 1에서 확인)
    노드 이름 livox_lidar_publisher (config/preflight.yaml 허용 이름과 같음)
    출력: /livox/lidar (PointCloud2), /livox/imu (Imu)  — TBD(세션 1) 실제 이름 확인

    ros2 launch my_algo livox_mid360.launch.py
    ros2 launch my_algo livox_mid360.launch.py user_config_path:=/path/MID360_config.json

MID360_config.json(호스트·라이다 IP)은 드라이버 패키지의 config/를 그대로 쓴다 (기존 rviz launch와 같은 파일).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('livox_ros_driver2'),
                                  'config', 'MID360_config.json')
    return LaunchDescription([
        DeclareLaunchArgument('user_config_path', default_value=default_config),
        DeclareLaunchArgument('publish_freq', default_value='10.0'),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[
                {'xfer_format': 0},          # 0 = PointCloud2. 1(CustomMsg)이면 파이프라인이 못 읽는다
                {'multi_topic': 0},
                {'data_src': 0},
                {'publish_freq': LaunchConfiguration('publish_freq')},
                {'output_data_type': 0},
                {'frame_id': 'livox_frame'},  # tf_static.launch.py의 livox_frame과 같아야 한다
                {'lvx_file_path': '/home/livox/livox_test.lvx'},
                {'user_config_path': LaunchConfiguration('user_config_path')},
                {'cmdline_input_bd_code': 'livox0000000001'},
            ],
        ),
    ])

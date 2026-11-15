"""vesc_to_odom (odom 토픽 + odom→base_link TF). exp 세션 전용, 세션 1 결과에 따라 사용.

배경: 차량 B 터미널의 `ros2 launch vesc_driver vesc_driver_node.launch.py`(f1tenth/vesc ros2 브랜치)는
vesc_driver_node만 띄우고 vesc_to_odom은 띄우지 않는다. 세션 1에서 /vesc/odom 발행자가 없으면
exp 세션 T1에서 이 launch를 함께 띄운다. 이미 다른 vesc_to_odom이 돌고 있으면 띄우지 않는다
(odom→base_link TF 발행자는 하나여야 한다 — config/preflight.yaml이 /tf 발행자 수로 확인).
drive 모드 주행·리팩터 검증(세션 1)에는 쓰지 않는다 (drive planner·AEB가 odom 속도를 읽으므로 동작이 달라짐).

계수는 레포의 유일 출처에서 가져온다 (VESC 설정 YAML에 따로 적지 않는다):
    speed_to_erpm_gain = vehicle_utils.ERPM_GAIN (명령과 같은 값 → 명령·odom 환산 일치)
    steering_angle_to_servo_gain/offset = -SERVO_GAIN / SERVO_CENTER (servo = 0.5 - 0.6 * steer)
    wheelbase = vehicle_geometry.yaml

    ros2 launch my_algo vesc_odom.launch.py                   # /vesc/odom, TF odom→base_link
    ros2 launch my_algo vesc_odom.launch.py publish_tf:=false # EKF(tf.launch.py use_ekf:=true)를 쓸 때
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_algo.vehicle_utils import ERPM_GAIN, SERVO_CENTER, SERVO_GAIN


def generate_launch_description():
    geom_path = os.path.join(get_package_share_directory('my_algo'), 'config', 'vehicle_geometry.yaml')
    with open(geom_path, encoding='utf-8') as f:
        wheelbase = float(yaml.safe_load(f)['vehicle_geometry']['ros__parameters']['wheelbase'])
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/vesc/odom', description='TBD(세션 1)'),
        DeclareLaunchArgument('core_topic', default_value='/sensors/core'),
        DeclareLaunchArgument('servo_cmd_topic', default_value='/sensors/servo_position_command'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': LaunchConfiguration('publish_tf'),
                'use_servo_cmd_to_calc_angular_velocity': True,
                'speed_to_erpm_gain': ERPM_GAIN,
                'speed_to_erpm_offset': 0.0,
                'steering_angle_to_servo_gain': -SERVO_GAIN,
                'steering_angle_to_servo_offset': SERVO_CENTER,
                'wheelbase': wheelbase,
            }],
            remappings=[('odom', LaunchConfiguration('odom_topic')),
                        ('sensors/core', LaunchConfiguration('core_topic')),
                        ('sensors/servo_position_command', LaunchConfiguration('servo_cmd_topic'))],
        ),
    ])

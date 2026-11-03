"""정적 TF: base_link -> livox_frame -> livox_imu.

base_link 원점 = 뒤축 중심, z = 0 바닥 (REP-103: x 전방, y 좌측, z 위).
값은 모두 launch 인자로 덮어쓸 수 있다. 측정 후에는 기본값을 교체한다.

    ros2 launch my_algo tf_static.launch.py
    ros2 launch my_algo tf_static.launch.py lidar_z:=0.243 lidar_pitch:=0.012

livox_frame은 livox_ros_driver2 점군의 frame_id와 같아야 한다 (check_vehicle.sh로 확인).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# base_link -> livox_frame (실측)
LIDAR_X = '0.13'          # 뒤축 중심 -> 라이다 원점 [m]
LIDAR_Y = '-0.005'
# [EXP0-MEASURE] z = 0.21(차체 기준 실측) + 원점높이 0.03(임시). measure_lidar_height.py의 height_m으로 교체
LIDAR_Z = '0.24'
# [EXP0-MEASURE] 장착 각도 가정값 0. roll/pitch는 measure_lidar_height.py, yaw는 RViz 벽 평행 검사로 교체 [rad]
LIDAR_ROLL = '0.0'
LIDAR_PITCH = '0.0'
LIDAR_YAW = '0.0'

# livox_frame -> livox_imu (Livox Mid-360 매뉴얼, 회전 없음)
IMU_X = '0.011'
IMU_Y = '0.02329'
IMU_Z = '-0.04412'


def static_tf(name, parent, child, x, y, z, roll, pitch, yaw):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', x, '--y', y, '--z', z,
            '--roll', roll, '--pitch', pitch, '--yaw', yaw,
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    args = [
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('imu_frame', default_value='livox_imu'),
        DeclareLaunchArgument('lidar_x', default_value=LIDAR_X),
        DeclareLaunchArgument('lidar_y', default_value=LIDAR_Y),
        DeclareLaunchArgument('lidar_z', default_value=LIDAR_Z),
        DeclareLaunchArgument('lidar_roll', default_value=LIDAR_ROLL),
        DeclareLaunchArgument('lidar_pitch', default_value=LIDAR_PITCH),
        DeclareLaunchArgument('lidar_yaw', default_value=LIDAR_YAW),
    ]
    c = LaunchConfiguration
    return LaunchDescription(args + [
        static_tf('tf_base_to_lidar', c('base_frame'), c('lidar_frame'),
                  c('lidar_x'), c('lidar_y'), c('lidar_z'),
                  c('lidar_roll'), c('lidar_pitch'), c('lidar_yaw')),
        static_tf('tf_lidar_to_imu', c('lidar_frame'), c('imu_frame'),
                  IMU_X, IMU_Y, IMU_Z, '0', '0', '0'),
    ])

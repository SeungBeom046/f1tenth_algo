"""정적 TF: base_link -> livox_frame -> livox_imu

base_link 원점 = 뒤축 중심, z = 0 바닥.  모든 값 [EXP-FIXED].
실측 2026-11-03. 표시된 [EXP0-MEASURE] 값은 아직 가정값이므로 측정 후 교체.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

# ---- base_link -> livox_frame ---------------------------------------------
LIDAR_X = 0.13     # B1: 뒤축 중심 -> 라이다 중심 (전방 +)
LIDAR_Y = -0.005   # B2: 왼쪽 끝까지 14 cm, 오른쪽 끝까지 13 cm
                   #     -> 중심선에서 오른쪽으로 0.5 cm (ROS 는 왼쪽이 +y)
                   #     측정 정밀도(±5 mm) 수준이라 0 으로 둬도 무방
MOUNT_Z = 0.21     # B3: 바닥 -> 라이다 장착면 (B4 - B3 = 6 cm, 스펙 60 mm 와 일치)
ORIGIN_H = 0.03    # [EXP0-MEASURE] 장착면 -> 점군 원점 O 높이. 임시값(센서 높이 절반).
                   #     C2(평평한 바닥 점군의 z 평균)로 교체할 것.
LIDAR_Z = MOUNT_Z + ORIGIN_H

LIDAR_YAW = 0.0    # [EXP0-MEASURE] B5: 로고가 전방이면 0, 후방이면 3.14159, 좌측 1.5708, 우측 -1.5708
LIDAR_ROLL = 0.0   # [EXP0-MEASURE] C1: 정지 IMU 평균 roll = atan2(ay, az)
LIDAR_PITCH = 0.0  # [EXP0-MEASURE] C1: 정지 IMU 평균 pitch = atan2(-ax, sqrt(ay^2 + az^2))

# ---- livox_frame -> livox_imu (Livox 매뉴얼 값, 회전 없음) ------------------
IMU_X, IMU_Y, IMU_Z = 0.011, 0.02329, -0.04412


def static_tf(name, parent, child, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        static_tf('tf_base_to_livox', 'base_link', 'livox_frame',
                  LIDAR_X, LIDAR_Y, LIDAR_Z, LIDAR_ROLL, LIDAR_PITCH, LIDAR_YAW),
        static_tf('tf_livox_to_imu', 'livox_frame', 'livox_imu',
                  IMU_X, IMU_Y, IMU_Z),
    ])

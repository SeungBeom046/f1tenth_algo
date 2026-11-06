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

LIDAR_YAW = 0.0    # B5 (세션 1, 2026-11-06): 로고 전방 → 0. (후방 3.14159, 좌측 1.5708, 우측 -1.5708)
LIDAR_ROLL = 0.0023   # C1 (세션 1): 정지 IMU roll = atan2(ay, az). 0°/180° 두 방향 평균(바닥 경사 상쇄): 0.00037, 0.00424
LIDAR_PITCH = 0.0557  # C1 (세션 1): 정지 IMU pitch = atan2(-ax, sqrt(ay^2 + az^2)). 0°/180° 평균: 0.05027, 0.06108 (3.2°, 앞이 숙여짐)

# ---- livox_frame -> livox_imu (Livox 매뉴얼 값, 회전 없음) ------------------
IMU_X, IMU_Y, IMU_Z = 0.011, 0.02329, -0.04412


def static_tf(name, parent, child, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    # Foxy static_transform_publisher는 이름 붙은 인자(--x 등)를 지원하지 않는다.
    # 위치 인자 순서: x y z yaw pitch roll parent child (yaw·pitch·roll 순서 주의, 단위 rad)
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[str(x), str(y), str(z), str(yaw), str(pitch), str(roll), parent, child],
    )


def generate_launch_description():
    return LaunchDescription([
        static_tf('tf_base_to_livox', 'base_link', 'livox_frame',
                  LIDAR_X, LIDAR_Y, LIDAR_Z, LIDAR_ROLL, LIDAR_PITCH, LIDAR_YAW),
        static_tf('tf_livox_to_imu', 'livox_frame', 'livox_imu',
                  IMU_X, IMU_Y, IMU_Z),
    ])

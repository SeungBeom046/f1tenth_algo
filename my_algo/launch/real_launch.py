from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 1. PointCloud2 → LaserScan 변환 (Livox Mid-360 전용)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                # 높이 필터 (지면 반사 제거)
                'min_height': -0.1,     # 지면 아래 제거
                'max_height': 0.5,      # 너무 높은 포인트 제거

                # 각도 범위 (Livox Mid-360은 360도)
                'angle_min': -3.14159,  # -180도
                'angle_max': 3.14159,   # +180도
                'angle_increment': 0.00436,  # 0.25도 간격

                # 거리 범위
                'range_min': 0.3,       # 최소 감지 거리 (30cm)
                'range_max': 20.0,      # 최대 감지 거리 (20m)

                # 기타
                'scan_time': 0.1,       # 스캔 주기 (10Hz)
                'use_inf': True,        # 범위 초과 시 inf 사용
                'inf_epsilon': 1.0,

                # 타겟 프레임 (Livox 프레임 이름 확인 필요)
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
                'deadzone': 0.05,        # 스틱 데드존 (미세 떨림 제거)
                'autorepeat_rate': 20.0, # 버튼 반복 주기 (Hz)
                'coalesce_interval': 0.0,
            }],
        ),

        # 3. 조이스틱 컨트롤러
        Node(
            package='my_algo',
            executable='joy_controller',
            name='joy_controller_node',
            output='screen',
        ),

        # 4. Wall Following (자율주행)
        Node(
            package='my_algo',
            executable='wall_follow_real',
            name='wall_follow_real_node',
            output='screen',
        ),

        # 5. AEB (안전장치)
        Node(
            package='my_algo',
            executable='aeb_real',
            name='aeb_real_node',
            output='screen',
        ),

        # 6. rosbridge (Foxglove 원격 시각화)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '',          # 모든 IP 허용
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0,
                'max_message_size': 10000000,  # 10MB (LiDAR 데이터 크기)
                'unregister_timeout': 10.0,
            }],
        ),
    ])
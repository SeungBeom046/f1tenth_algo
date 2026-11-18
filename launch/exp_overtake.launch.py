"""추월 실험 launch. role:=leader(검정이, 앞차 시나리오) / role:=ego(파랑이, 추월하는 차).

    ros2 launch my_algo exp_overtake.launch.py role:=leader intent:=Y run_id:=dY1 W:=1.0     # 검정이
    ros2 launch my_algo exp_overtake.launch.py role:=leader intent:=NY run_id:=dNY1 W:=1.0   # 검정이
    ros2 launch my_algo exp_overtake.launch.py role:=ego method:=d W:=1.0                     # 파랑이

pointcloud_to_laserscan(lidar_drive와 같은 값) + exp_planner + autonomous_drive(geometric AEB).
gap follow(lidar_reactive_drive)와 카메라는 띄우지 않는다. 실행하면 바로 출발한다 (출발 게이트로 대기).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _scan_nodes(context):
    """pointcloud_to_laserscan (lidar_drive와 같은 값). level_scan이면 라이다 장착 기울기를 되돌린 livox_level 프레임에서 자른다.

    정적 TF livox_level → livox_frame = 장착 회전 (roll, pitch). static_transform_publisher 인자 순서는 x y z yaw pitch roll
    (Foxy는 --x 같은 이름 인자를 지원하지 않는다).
    """
    level = LaunchConfiguration('level_scan').perform(context).lower() in ('true', '1', 'yes')
    pitch = LaunchConfiguration('lidar_pitch').perform(context)
    roll = LaunchConfiguration('lidar_roll').perform(context)
    safety = LaunchConfiguration('experiment_safety').perform(context).lower() in ('true', '1', 'yes')
    nodes = []
    if level:
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name='tf_livox_level',
                          arguments=['0', '0', '0', '0', pitch, roll, 'livox_level', 'livox_frame']))
    nodes.append(Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'min_height': -0.20 if safety else -0.1, 'max_height': 0.5,
            'angle_min': -3.14159, 'angle_max': 3.14159, 'angle_increment': 0.00436,
            'scan_time': 0.1, 'range_min': 0.05, 'range_max': 20.0,
            'use_inf': True, 'inf_epsilon': 1.0, 'target_frame': 'livox_level' if level else 'livox_frame',
        }],
        remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
    ))
    return nodes


def generate_launch_description():
    def arg(name, typ):
        return ParameterValue(LaunchConfiguration(name), value_type=typ)

    return LaunchDescription([
        DeclareLaunchArgument('role', default_value='leader', description='leader(앞차) | ego(추월하는 차) | straight(서보 보정 직진)'),
        DeclareLaunchArgument('method', default_value='d', description='(ego) a 고정 규칙 | b 소극적 | c 공격적 | d 제안'),
        # 시뮬 차량과 같은 값(구동 0.15 + 1차 지연 0.10). 기획안 0.15로는 시뮬에서 NY 급정지 때 추돌했다. 보정 실험 값으로 바꾼다
        DeclareLaunchArgument('t_d', default_value='0.25', description='(ego) 제동 지연 [s], scan 주기 0.1 s는 따로 더해진다'),
        DeclareLaunchArgument('intent', default_value='Y', description='Y: 비킨다 / NY: 안 비키고 급정지'),
        DeclareLaunchArgument('run_id', default_value='0', description='회차 이름. d_y·급정지 지연 seed'),
        DeclareLaunchArgument('d_y', default_value='0.0', description='> 0이면 양보 반응 거리 고정 [m]'),
        DeclareLaunchArgument('W', default_value='0.0', description='실측 통로 폭 [m]. 0이면 벽으로 추정'),
        DeclareLaunchArgument('servo_center', default_value='0.5',
                              description='직진 서보 값 (차마다 보정, docs/CAR_TERMINALS.md 서보 중심 보정)'),
        # 실차 앞바퀴 최대 ±20° (0.349 rad). 서보 끝(0.5 ± 0.47)이 20°라고 가정한 게인 0.47/0.349 = 1.35.
        # 원 주행(role:=straight steer:=0.2)으로 확인한다 (docs/CAR_TERMINALS.md 조향 확인). 기본 주행 값은 0.6 / 0.78
        DeclareLaunchArgument('servo_gain', default_value='1.35',
                              description='서보 변환 게인 [서보/rad]. 0.6 = 기본 주행, 바퀴 최대 20°면 1.35 (docs/CAR_TERMINALS.md 조향 확인)'),
        DeclareLaunchArgument('max_steering', default_value='0.349', description='최대 조향 명령 [rad] (앞바퀴 물리 한계 20°)'),
        DeclareLaunchArgument('steer', default_value='0.0', description='(straight) 고정 조향 [rad], 원 주행으로 실제 바퀴 각 확인'),
        # 라이다 장착 기울기 (실측 2026-11-06 세션 1, 정지 IMU: pitch 3.2° 앞이 숙여짐, roll 0.13°)
        DeclareLaunchArgument('level_scan', default_value='true', description='true면 기울기를 되돌린 수평 프레임에서 /scan을 만든다'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0557', description='라이다 pitch [rad], + = 앞이 숙여짐'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0023', description='라이다 roll [rad]'),
        DeclareLaunchArgument('lidar_to_bumper', default_value='0.28', description='라이다 → 앞범퍼 [m] (AEB, 실측 2026-11-03; config lidar_to_front와 같게)'),
        DeclareLaunchArgument('speed_source', default_value='sensors_core', description='sensors_core | command'),
        DeclareLaunchArgument('experiment_safety', default_value='true', description='추적 소실 정지·AEB 정지 유지·헤딩 필터'),
        DeclareLaunchArgument('stationary_pass', default_value='false', description='method a 전용 정지차 우회 시험, 앞차 간격 1.5 m 이상'),
        OpaqueFunction(function=_scan_nodes),
        Node(
            package='my_algo',
            executable='exp_planner',
            output='screen',
            parameters=[{
                'role': arg('role', str), 'intent': arg('intent', str), 'run_id': arg('run_id', str),
                'd_y': arg('d_y', float), 'W': arg('W', float), 'speed_source': arg('speed_source', str),
                'method': arg('method', str), 't_d': arg('t_d', float), 'steer': arg('steer', float),
                'experiment_safety': arg('experiment_safety', bool),
                'stationary_pass': arg('stationary_pass', bool),
            }],
        ),
        Node(
            package='my_algo',
            executable='autonomous_drive',
            output='screen',
            parameters=[{
                'drive_source': 'lidar',
                'start_autonomous': True,
                'command_timeout_sec': 1.00,
                # 시뮬 scenario.yaml aeb와 같은 값: 띠 ±0.16 m(양보 차선 벽 여유 0.235 m보다 좁게), TTC 0.35 s
                'aeb_model': 'geometric',
                'experiment_safety': arg('experiment_safety', bool),
                'aeb_curved_band': True,
                # 휜 띠가 조향을 크게 꺾을 때 옆에 나란한 벽을 앞 장애물로 잡지 않게 (실차 2026-11-17, scenario.yaml aeb와 같은 값)
                'aeb_curve_max_steering': 0.3,
                'aeb_band_intersect': True,
                'aeb_ttc_threshold': 0.35,
                # 띠 = 실측 차 반폭 0.135 (여유 0): 실차는 추월 때 차체-벽 약 5 cm까지 붙는다 (2026-11-17). 정지 거리 0.25 → 0.20 (≥ m 0.15)
                'aeb_vehicle_half_width': 0.135,
                'aeb_path_margin': 0.0,
                'aeb_bumper_emergency_distance': 0.20,
                'aeb_bumper_overlap_tolerance': 0.10,
                'lidar_to_bumper_dist': arg('lidar_to_bumper', float),
                'max_steering': arg('max_steering', float),
                'servo_center': arg('servo_center', float),
                'servo_gain': arg('servo_gain', float),
            }],
        ),
    ])

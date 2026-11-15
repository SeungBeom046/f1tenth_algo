"""돌출 차량 #2 전용 launch (#2 Jetson). 이 launch 하나만 띄운다.

실행하는 것 (전부 namespace /car2):
    VESC 드라이버 (vesc_driver)          → /car2/commands/motor/speed 등 구독, /car2/sensors/core 발행
    vesc_to_odom (publish_tf: false)    → /car2/odom 발행, /tf는 발행하지 않음
    popout_vehicle_node                  → /trigger 구독 (에고와 공유하는 유일한 토픽)
실행하지 않는 것: Livox 드라이버, 정적 TF, joy, 그 밖의 노드 (에고의 /livox/*, /tf_static과 섞이지 않게).

    ros2 launch my_algo car2.launch.py vesc_config:=/home/<user>/car2_vesc.yaml

인자 (TBD(세션 1): 에고의 check_vehicle.sh 로그에서 VESC 패키지·노드 이름 확인 후 기본값 확정):
    vesc_config          #2의 VESC 설정 YAML (#2 실측 speed_to_erpm_gain 등). 필수
    vesc_driver_pkg/exe  기본 vesc_driver / vesc_driver_node
    vesc_odom_pkg/exe    기본 vesc_ackermann / vesc_to_odom_node
    namespace            기본 car2. config/preflight.yaml 규칙과 맞춰야 한다
설정 YAML은 노드 이름 대신 '/**:' 키를 쓰면 namespace와 상관없이 적용된다.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_algo.vehicle_utils import SERVO_GAIN


def nodes(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    ns = arg('namespace')
    if not ns or ns == '/':
        raise RuntimeError('#2는 반드시 namespace를 써야 한다 (에고 토픽과 충돌)')
    vesc_config = arg('vesc_config')
    if not vesc_config or not os.path.exists(os.path.expanduser(vesc_config)):
        raise RuntimeError(f'vesc_config 파일 없음: {vesc_config!r}')
    vesc_config = os.path.expanduser(vesc_config)
    share = get_package_share_directory('my_algo')
    popout = os.path.join(share, 'config', 'popout.yaml')
    with open(popout, encoding='utf-8') as f:
        pp = yaml.safe_load(f)['/**']['ros__parameters']
    with open(os.path.join(share, 'config', 'vehicle_geometry.yaml'), encoding='utf-8') as f:
        wheelbase = float(yaml.safe_load(f)['vehicle_geometry']['ros__parameters']['wheelbase'])
    # vesc_to_odom은 아래 계수가 없으면 시작하지 못한다 (f1tenth/vesc 기본 vesc_config.yaml에 없음).
    # #2 계수는 popout.yaml (#2 실측 [EXP0-MEASURE]), 휠베이스·서보 게인은 에고와 같은 차종으로 가정 (TBD(세션 1))
    odom_params = {'publish_tf': False, 'odom_frame': 'car2_odom', 'base_frame': 'car2_base_link',
                   'use_servo_cmd_to_calc_angular_velocity': True,
                   'speed_to_erpm_gain': float(pp['erpm_gain']), 'speed_to_erpm_offset': 0.0,
                   'steering_angle_to_servo_gain': -SERVO_GAIN,
                   'steering_angle_to_servo_offset': float(pp['servo_center']), 'wheelbase': wheelbase}
    popout_params = [popout]
    if arg('v_popout'):   # Exp0 #2 단독 시험 (0.8 / 0.9 m/s). 확정 후에는 popout.yaml 값을 바꾼다
        popout_params.append({'v_popout': float(arg('v_popout'))})
    return [
        Node(package=arg('vesc_driver_pkg'), executable=arg('vesc_driver_exe'),
             name='vesc_driver_node', namespace=ns, output='screen', parameters=[vesc_config]),
        Node(package=arg('vesc_odom_pkg'), executable=arg('vesc_odom_exe'),
             name='vesc_to_odom_node', namespace=ns, output='screen',
             parameters=[vesc_config, odom_params]),   # publish_tf False: /tf는 namespace가 안 붙는다
        Node(package='my_algo', executable='popout_vehicle_node', name='popout_vehicle_node',
             namespace=ns, output='screen', parameters=popout_params),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car2'),
        DeclareLaunchArgument('vesc_config', default_value='', description='#2 VESC 설정 YAML (필수)'),
        DeclareLaunchArgument('vesc_driver_pkg', default_value='vesc_driver'),
        DeclareLaunchArgument('vesc_driver_exe', default_value='vesc_driver_node'),
        DeclareLaunchArgument('vesc_odom_pkg', default_value='vesc_ackermann'),
        DeclareLaunchArgument('vesc_odom_exe', default_value='vesc_to_odom_node'),
        DeclareLaunchArgument('v_popout', default_value='', description='Exp0 시험용 속도 덮어쓰기'),
        OpaqueFunction(function=nodes),
    ])

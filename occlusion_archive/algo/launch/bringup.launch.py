"""실험 bringup. mode:=drive(기본)는 기존 주행 그대로, mode:=exp는 관측 공백 실험 스택.

    # 기존 주행 (lidar_drive.launch.py를 그대로 include, 동작·화면 불변)
    ros2 launch my_algo bringup.launch.py
    # 실험 회차
    ros2 launch my_algo bringup.launch.py mode:=exp method:=P scenario:=S1 d_e:=1.5 run_id:=3
    # 민감도 분석: [EXP-TUNE] 값만 개별 인자로 덮어쓰기 허용
    ros2 launch my_algo bringup.launch.py mode:=exp method:=P scenario:=S3 run_id:=1 w_min:=0.30

exp 모드 실행 노드: pointcloud_to_laserscan, imu_republisher, (deskew_node), occupancy_grid_node,
    phantom_node, speed_layer_node, exp_aeb_node, exp_steering_node, exp_drive_mux, trigger_node,
    publisher_guard_node (#2 등 예상 외 발행자 점검, config/preflight.yaml),
    joy_node + joy_controller (비상 정지), (realsense 녹화용, depth 끔). YOLO(camera_reactive_drive)는 띄우지 않는다.
정적 TF와 센서 드라이버는 T1 터미널에서 따로 띄운다 (docs/RUNBOOK.md). include_tf:=true면 여기서 같이 띄운다.
파라미터 우선순위: common.yaml < method_*.yaml < scenario_*.yaml < 회차 인자(d_e, run_id) < [EXP-TUNE] 인자.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_algo.run_paths import METHODS, SCENARIOS, run_dir

# 개별 인자로 덮어쓸 수 있는 [EXP-TUNE] 값과 타입. 이 밖의 값은 YAML로만 바꾼다.
TUNE_ARGS = {'v_imp': float, 'w_min': float, 'w_max': float, 'n_frames': int, 'known_on': int,
             'known_off': int, 'b1_k': float, 'b1_roi_length': float, 'b1_roi_half_width': float,
             'row_face_tol': float}


def share(*parts):
    return os.path.join(get_package_share_directory('my_algo'), *parts)


def drive(context):
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        share('launch', 'lidar_drive.launch.py')))]


def exp(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    method, scenario = arg('method'), arg('scenario')
    if method not in METHODS or scenario not in SCENARIOS:
        raise RuntimeError(f'method {METHODS}, scenario {SCENARIOS} 중에서 골라야 한다')
    run_id = int(arg('run_id'))
    d_e = arg('d_e') if scenario == 'S1' else ''
    if scenario == 'S1' and not d_e:
        raise RuntimeError('S1은 d_e:= 가 필요하다 (1.0 / 1.5 / 2.0)')
    rdir = arg('run_dir') or run_dir(scenario, method, d_e, run_id)

    common = share('config', 'common.yaml')
    with open(common, encoding='utf-8') as f:
        common_values = yaml.safe_load(f)['/**']['ros__parameters']
    deskew = arg('deskew') or str(common_values['deskew']).lower()
    deskew = deskew == 'true'
    geometry = share('config', 'vehicle_geometry.yaml')

    run_params = {'run_id': str(run_id), 'run_dir': rdir, 'vehicle_geometry_file': geometry}
    if d_e:
        run_params['d_e'] = float(d_e)
    for name, cast in TUNE_ARGS.items():
        value = arg(name)
        if value:
            run_params[name] = cast(value)
    # Exp0 odom 보정·Exp1 저속 주행 전용. v_ref는 [EXP-FIXED]라 방법 비교(Exp2·Exp3) 회차에는 쓰지 않는다.
    # params.yaml 덤프에 실제 v_ref가 남고, record_run.sh가 meta.json에 test_v_ref를 기록한다.
    # Exp0 i 추종 지연: 주행 거리 test_step_x에서 속도 명령을 test_step_v로 계단처럼 내린다 (방법과 무관, mux에서)
    if arg('test_step_v'):
        run_params['test_step_v'] = float(arg('test_step_v'))
        run_params['test_step_x'] = float(arg('test_step_x') or 2.5)
        print(f'[bringup] 경고: test_step_v={run_params["test_step_v"]} at {run_params["test_step_x"]} m '
              '— Exp0 i 전용')
    if arg('test_v_ref'):
        run_params['v_ref'] = float(arg('test_v_ref'))
        print(f'[bringup] 경고: test_v_ref={run_params["v_ref"]} — 방법 비교 회차에 쓰지 말 것 (Exp0/Exp1 전용)')
    params = [common, share('config', f'method_{method}.yaml'),
              share('config', f'scenario_{scenario}.yaml'), run_params]

    cloud, odom, scan = arg('cloud_topic'), arg('odom_topic'), '/scan'
    grid_cloud = '/exp/cloud_deskewed' if deskew else cloud

    def node(executable, extra=None, remap=None, name=None):
        return Node(package='my_algo', executable=executable, name=name or executable,
                    output='screen', parameters=params + ([extra] if extra else []),
                    remappings=remap or [])

    actions = [
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan', parameters=[common],
             remappings=[('cloud_in', cloud), ('scan', scan)]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share('launch', 'imu.launch.py')),
                                 launch_arguments={'input_topic': arg('imu_topic')}.items()),
        node('occupancy_grid_node', {'cloud_topic': grid_cloud}),
        node('phantom_node'),
        node('speed_layer_node'),
        node('exp_aeb_node', {'scan_topic': scan, 'odom_topic': odom}),
        node('exp_steering_node', remap=[('/reactive/lidar_command', '/exp/steering_command'),
                                         ('/vesc/odom', odom), ('/scan', scan)]),
        node('exp_drive_mux', {'odom_topic': odom}),
        node('trigger_node', {'odom_topic': odom}),
        Node(package='my_algo', executable='publisher_guard_node', name='publisher_guard_node',
             output='screen', parameters=[share('config', 'preflight.yaml')]),
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'autorepeat_rate': 20.0}]),  # 입력이 없어도 20 Hz로 재발행 → 데드맨 감시가 동작
        Node(package='my_algo', executable='joy_controller', name='joy_controller', output='screen'),
    ]
    if deskew:
        actions.append(node('deskew_node', {'cloud_topic': cloud}))
    if arg('use_camera') == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
            launch_arguments={'enable_depth': 'false'}.items()))
    if arg('include_tf') == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('launch', 'tf.launch.py'))))
    print(f'[bringup] exp method={method} scenario={scenario} d_e={d_e or "na"} run_id={run_id} '
          f'deskew={deskew} run_dir={rdir}')
    return actions


def dispatch(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode == 'drive':
        return drive(context)
    if mode == 'exp':
        return exp(context)
    raise RuntimeError("mode는 drive 또는 exp")


def generate_launch_description():
    args = [
        DeclareLaunchArgument('mode', default_value='drive', description='drive | exp'),
        DeclareLaunchArgument('method', default_value='B0', description='[EXP-VAR] B0 | B1 | B2 | P'),
        DeclareLaunchArgument('scenario', default_value='S2', description='[EXP-VAR] S1 | S2 | S3 | S4'),
        DeclareLaunchArgument('d_e', default_value='', description='[EXP-VAR] S1 돌출 시점 1.0 | 1.5 | 2.0'),
        DeclareLaunchArgument('run_id', default_value='0'),
        DeclareLaunchArgument('run_dir', default_value='', description='비우면 run_paths 규칙'),
        DeclareLaunchArgument('deskew', default_value='', description='비우면 common.yaml 값'),
        DeclareLaunchArgument('use_camera', default_value='true', description='전방 카메라 녹화용'),
        DeclareLaunchArgument('include_tf', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar', description='실차 확인 필요'),
        DeclareLaunchArgument('odom_topic', default_value='/vesc/odom', description='실차 확인 필요'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu', description='실차 확인 필요'),
        DeclareLaunchArgument('test_v_ref', default_value='',
                              description='Exp0/Exp1 전용 v_ref 덮어쓰기. 비교 실험 금지'),
        DeclareLaunchArgument('test_step_v', default_value='',
                              description='Exp0 i 전용 속도 계단 목표 [m/s] (예 1.2). 비교 실험 금지'),
        DeclareLaunchArgument('test_step_x', default_value='',
                              description='Exp0 i 속도 계단 위치 (AUTO 후 주행 거리) [m], 기본 2.5'),
    ] + [DeclareLaunchArgument(k, default_value='', description='[EXP-TUNE] 덮어쓰기')
         for k in TUNE_ARGS]
    return LaunchDescription(args + [OpaqueFunction(function=dispatch)])

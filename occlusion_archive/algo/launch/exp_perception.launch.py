"""오프라인 재생 비교용: 인지 → phantom → 속도 계층만 sim time으로 띄운다 (명령 발행 노드 없음).

    ros2 launch my_algo exp_perception.launch.py method:=P scenario:=S3 run_dir:=<회차 폴더>
scripts/offline_replay.sh가 호출한다. 회차 폴더의 config_snapshot/을 쓴다 (기록 당시 설정).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    snap = os.path.join(arg('run_dir'), 'config_snapshot')
    method = arg('method')
    method_file = os.path.join(snap, f'method_{method}.yaml')
    if not os.path.exists(method_file):  # 기록한 회차와 다른 방법으로 재생: share의 방법 파일
        from ament_index_python.packages import get_package_share_directory
        method_file = os.path.join(get_package_share_directory('my_algo'), 'config',
                                   f'method_{method}.yaml')
    params = [os.path.join(snap, 'common.yaml'), method_file,
              os.path.join(snap, f'scenario_{arg("scenario")}.yaml'),
              {'use_sim_time': True,
               'vehicle_geometry_file': os.path.join(snap, 'vehicle_geometry.yaml'),
               'cloud_topic': arg('cloud_topic')}]
    return [Node(package='my_algo', executable=e, name=e, output='screen', parameters=params)
            for e in ('occupancy_grid_node', 'phantom_node', 'speed_layer_node')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method'),
        DeclareLaunchArgument('scenario'),
        DeclareLaunchArgument('run_dir'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar'),
        OpaqueFunction(function=nodes),
    ])

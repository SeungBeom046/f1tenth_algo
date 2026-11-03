"""돌출 차량 #2 (#2 Jetson에서 실행). 에고와 같은 ROS_DOMAIN_ID, 토픽은 namespace로 분리.

    ros2 launch my_algo popout.launch.py namespace:=car2

#2의 VESC 드라이버도 같은 namespace(/car2)로 띄워야 /car2/commands/motor/speed를 받는다.
#2의 vesc_to_odom은 publish_tf:=false (/tf는 namespace가 안 붙어 에고 TF와 섞인다).
/trigger만 절대 이름이라 namespace 밖에서 에고와 공유된다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('my_algo'), 'config', 'popout.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='car2'),
        Node(package='my_algo', executable='popout_vehicle_node', name='popout_vehicle_node',
             namespace=LaunchConfiguration('namespace'), output='screen', parameters=[params]),
    ])

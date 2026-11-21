from glob import glob

from setuptools import setup

package_name = 'my_algo'

setup(
    name=package_name,
    version='0.0.0',
    # my_algo.baseline: 기본 주행 + 공통 유틸, my_algo.aur: 추월 실험 판단 로직 (rclpy 없음). occlusion_archive·archive는 설치하지 않는다
    packages=[package_name, package_name + '.baseline', package_name + '.aur'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            ['config/exp_common.yaml', 'config/perception_record_qos.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Real F1TENTH autonomous driving stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_reactive_drive = my_algo.baseline.lidar_reactive_drive:main',
            'camera_reactive_drive = my_algo.baseline.camera_reactive_drive:main',
            'autonomous_drive = my_algo.baseline.autonomous_drive:main',
            'joy_controller = my_algo.baseline.joy_controller:main',
            'autonomous_metrics_logger = my_algo.baseline.autonomous_metrics_logger:main',
            'exp_planner = my_algo.exp_planner:main',
            'camera_fusion = my_algo.camera_fusion:main',
        ],
    },
)

from glob import glob

from setuptools import setup

package_name = 'my_algo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Real F1TENTH autonomous driving stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lidar_reactive_drive = my_algo.lidar_reactive_drive:main',
            'camera_reactive_drive = my_algo.camera_reactive_drive:main',
            'autonomous_drive = my_algo.autonomous_drive:main',
            'joy_controller = my_algo.joy_controller:main',
            'autonomous_metrics_logger = my_algo.autonomous_metrics_logger:main',
            'imu_republisher = my_algo.imu_republisher:main',
            'deskew_node = my_algo.deskew_node:main',
            'occupancy_grid_node = my_algo.occupancy_grid_node:main',
            'phantom_node = my_algo.phantom_node:main',
            'speed_layer_node = my_algo.speed_layer_node:main',
            'exp_aeb_node = my_algo.exp_aeb_node:main',
            'exp_steering_node = my_algo.exp_steering_node:main',
            'exp_drive_mux = my_algo.exp_drive_mux:main',
            'trigger_node = my_algo.trigger_node:main',
            'popout_vehicle_node = my_algo.popout_vehicle_node:main',
            'publisher_guard_node = my_algo.publisher_guard_node:main',
        ],
    },
)

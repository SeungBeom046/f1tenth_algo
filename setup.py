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
        ],
    },
)

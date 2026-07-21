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
        ('share/' + package_name + '/launch', ['launch/real_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follow = my_algo.wall_follow:main',
            'wall_follow_real = my_algo.wall_follow_real:main',
            'gap_follow_real = my_algo.gap_follow_real:main',
            'aeb = my_algo.aeb:main',
            'aeb_real = my_algo.aeb_real:main',
            'joy_controller = my_algo.joy_controller:main',
        ],
    },
)

from setuptools import setup
import os
from glob import glob
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid

package_name = 'multi_robot_challenge_23'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hvlrobotics',
    maintainer_email='hvlrobotics@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_recognition = multi_robot_challenge_23.marker_pose:main',
            'robot_handler = multi_robot_challenge_23.robot_handler:main',
            'controller = multi_robot_challenge_23.controller:main',
            'wall = multi_robot_challenge_23.wall_follower:main',
            'detection = multi_robot_challenge_23.marker_detection:main',
            'aruco_node = multi_robot_challenge_23.aruco_node:main',
            'map = multi_robot_challenge_23.map_marker:main',
            'live_map = multi_robot_challenge_23.live_map:main',
            'coordinator = multi_robot_challenge_23.coordinator:main',



        ],
    },
)

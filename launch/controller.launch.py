import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
def generate_launch_description():
    tb3_0 = Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_0',
        name='tb3_0'
    )

    tb3_1 = Node(
        package='multi_robot_challenge_23',
        executable='robot_handler',
        namespace='tb3_1',
        name='tb3_1'
    )

    leader = Node(
        package='multi_robot_challenge_23',
        executable='leader',
        name='leader'
    )

    return LaunchDescription([tb3_0, tb3_1, leader])
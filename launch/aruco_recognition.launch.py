import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='tb3_5'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        namespace=namespace,
        parameters=[{"marker_size": 0.5},
                    {"aruco_dictionary_id": "DICT_5X5_250"},
                    {"image_topic": "camera/image_raw"},
                    {"camera_info_topic": "camera/camera_info"},
                    {"use_sim_time": use_sim_time}]
    )

    marker_trans = Node(
        package='multi_robot_challenge_23',
        executable='marker_recognition',
        name='marker_recognition',
        parameters=[{"namespace": namespace},
                    {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        namespace_launch_arg,
        use_sim_time_arg,
        aruco_node,
        marker_trans,
    ])
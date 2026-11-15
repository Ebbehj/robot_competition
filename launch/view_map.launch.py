"""Viser bare kartet, viewpoints og peaks i RViz. Ingen Gazebo og ingen roboter.

Eksempel:
    ros2 launch multi_robot_challenge_23 view_map.launch.py map:=map_dat160_w2.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'multi_robot_challenge_23'


def launch_setup(context):
    share = get_package_share_directory(PACKAGE)

    map_name = LaunchConfiguration('map').perform(context)
    map_path = os.path.join(share, 'maps', map_name)
    if not os.path.isfile(map_path):
        available = sorted(f for f in os.listdir(os.path.join(share, 'maps')) if f.endswith('.yaml'))
        raise RuntimeError(f'Fant ikke kartet {map_name}. Tilgjengelige kart: {available}')

    start_x = float(LaunchConfiguration('start_x').perform(context))
    start_y = float(LaunchConfiguration('start_y').perform(context))
    peak_radius = float(LaunchConfiguration('peak_radius').perform(context))

    # Uten Gazebo finnes ikke /clock, så alt må bruke vanlig klokke.
    use_sim_time = False

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_path,
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': use_sim_time,
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    # Ett startpunkt er nok til å finne hvilket område som kan nås.
    coordinator = Node(
        package=PACKAGE,
        executable='coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'start_xs': [start_x],
            'start_ys': [start_y],
            'robot_names': ['tb3_0'],
            'peak_radius': peak_radius,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share, 'rviz', 'model.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return [map_server, lifecycle_manager, coordinator, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='map_dat160_w1.yaml',
                              description='Filnavn i maps-mappen'),
        DeclareLaunchArgument('start_x', default_value='0.0',
                              description='Et fritt punkt roboter kan starte i'),
        DeclareLaunchArgument('start_y', default_value='-1.0'),
        DeclareLaunchArgument('peak_radius', default_value='2.5',
                              description='Minste avstand mellom peaks i meter'),
        OpaqueFunction(function=launch_setup),
    ])
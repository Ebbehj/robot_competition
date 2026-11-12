"""Start navigasjonen. Gazebo og RViz startes separat."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package = "multi_robot_challenge_23"

    # Alle tre bruker samme simuleringsklokke som Gazebo og TF.
    coordinator = Node(
        package=package,
        executable="coordinator",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    planner = Node(
        package=package,
        executable="path_planner",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    follower = Node(
        package=package,
        executable="path_follower",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Noder kan starte samtidig: planleggeren venter på kart/graf/TF,
    # og rutefølgeren venter på en rute. Ingen veggfølger startes her.
    return LaunchDescription([coordinator, planner, follower])

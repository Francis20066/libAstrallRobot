import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("astrall_sim")
    description_share = get_package_share_directory("astrall_description")

    config = os.path.join(sim_share, "config", "astrall_sim.yaml")
    urdf = os.path.join(description_share, "urdf", "astrall_robot_dog.urdf.xacro")
    rviz_config = os.path.join(description_share, "rviz", "astrall_sim.rviz")

    with open(urdf, "r", encoding="utf-8") as f:
        robot_description = {"robot_description": f.read()}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="astrall_sim",
            executable="astrall_sim_node",
            name="astrall_sim_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])

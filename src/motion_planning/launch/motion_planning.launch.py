from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory("motion_planning")
    config_file = os.path.join(package_share_dir, "config", "motion_planning.yaml")

    return LaunchDescription([
        Node(
            package="motion_planning",
            executable="motion_planning_node",
            name="motion_planning_node",
            output="screen",
            parameters=[config_file],
        )
    ])

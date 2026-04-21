from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory("path_follow_control")
    config_file = os.path.join(package_share_dir, "config", "path_follow_control.yaml")

    return LaunchDescription([
        Node(
            package="path_follow_control",
            executable="path_follow_control_node",
            name="path_follow_control_node",
            output="screen",
            parameters=[config_file],
        )
    ])

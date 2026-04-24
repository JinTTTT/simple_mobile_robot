from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory("localization")
    config_file = os.path.join(package_share_dir, "config", "localization.yaml")

    return LaunchDescription([
        Node(
            package="localization",
            executable="particle_filter_localization_node",
            name="particle_filter_localization_node",
            output="screen",
            parameters=[config_file],
        )
    ])


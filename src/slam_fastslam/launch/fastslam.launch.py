from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory("slam_fastslam")
    config_file = os.path.join(package_share_dir, "config", "fastslam.yaml")

    return LaunchDescription([
        Node(
            package="slam_fastslam",
            executable="fastslam_node",
            name="fastslam_node",
            output="screen",
            parameters=[config_file],
        ),
    ])

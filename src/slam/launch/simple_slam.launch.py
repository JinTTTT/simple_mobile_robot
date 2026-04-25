from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share_dir = get_package_share_directory('slam')
    config_file = os.path.join(package_share_dir, 'config', 'slam.yaml')

    return LaunchDescription([
        Node(
            package='slam',
            executable='simple_slam_node',
            name='simple_slam_node',
            output='screen',
            parameters=[config_file],
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam',
            executable='simple_slam_node',
            name='simple_slam_node',
            output='screen',
        ),
    ])

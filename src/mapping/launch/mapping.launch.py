import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Mapping launch file.

    Starts:
    1. The occupancy mapper node (subscribes to /scan, publishes /map)
    2. A static identity transform: map -> odom
       - During mapping we assume map and odom start at the same origin.
       - This lets RViz2 and other tools visualize the map (frame_id='map')
         alongside robot data (frame_id='odom') without errors.
       - When localization runs later, the localization node will
         REPLACE this static transform with a real correction offset.
    """

    occupancy_mapper = Node(
        package='mapping',
        executable='occupancy_mapper',
        name='occupancy_mapper',
        output='screen',
    )

    # Static identity transform: map -> odom
    # Arguments: x y z yaw pitch roll parent_frame child_frame
    # All zeros = no translation, no rotation → map and odom are identical
    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    return LaunchDescription([
        static_map_odom,    # must come first so TF is ready when mapper starts
        occupancy_mapper,
    ])

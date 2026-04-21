# Motion Planning Package

This package will contain learning implementations for motion planning in ROS 2.

The first goal is to add a simple global planner for a 2D occupancy grid map.

Planned inputs:

- `/map`
- robot start pose
- goal pose

Planned outputs:

- a path as `nav_msgs/msg/Path`

The first learning version will likely start with a grid-based planner such as A*.

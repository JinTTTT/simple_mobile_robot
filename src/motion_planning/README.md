# Motion Planning Package

This package contains learning implementations for motion planning in ROS 2.

The first version is a simple A* global planner for a 2D occupancy grid map.

Inputs:

- `/map`
- `/estimated_pose`
- `/goal_pose`

Output:

- `/planned_path` as `nav_msgs/msg/Path`

Current behavior:

- uses the static occupancy grid from `map_server`
- uses the Kalman localization estimated pose as the start pose
- uses the RViz goal pose as the target
- runs A* on an 8-connected grid
- treats unknown cells as blocked

How to test:

- start simulation
- start the static map server with `src/mapping/maps/maze_map.yaml`
- start `kalman_localization_node`
- start `motion_planning_node`
- open RViz with fixed frame `map`
- add `/map` and `/planned_path`
- click a goal with the `2D Goal Pose` tool

# Motion Planning Package

This package contains learning implementations for motion planning in ROS 2.

The first version is a simple A* global planner for a 2D occupancy grid map.

Inputs:

- `/map`
- `/estimated_pose`
- `/goal_pose`

Output:

- `/planned_path` as `nav_msgs/msg/Path`
- `/inflated_map` as `nav_msgs/msg/OccupancyGrid`

Current behavior:

- uses the static occupancy grid from `map_server`
- uses the Kalman localization estimated pose as the start pose
- uses the RViz goal pose as the target
- inflates obstacles using a conservative circular robot radius of `0.35 m`
- runs A* on an inflated 8-connected grid
- smooths the raw A* path using line-of-sight shortcutting on `inflated_map`
- preserves the RViz goal orientation on the final path pose
- treats unknown cells as blocked

Main tuning parameters:

- `robot_radius_m`
- `occupied_threshold`
- `enable_path_smoothing`

Config file:

- `config/motion_planning.yaml`

Launch file:

- `launch/motion_planning.launch.py`

How to test:

- start simulation
- start the static map server with `src/mapping/maps/maze_map.yaml`
- start `kalman_localization_node`
- start `motion_planning_node` or `ros2 launch motion_planning motion_planning.launch.py`
- open RViz with fixed frame `map`
- add `/map`, `/inflated_map`, and `/planned_path`
- click a goal with the `2D Goal Pose` tool

# Motion Planning Package

This package contains learning implementations for motion planning in ROS 2.

The first version is a simple A* global planner for a 2D occupancy grid map.

Inputs:

- `/map`
- `/estimated_pose`
- `/goal_pose`

Output:

- `/planned_path` as `nav_msgs/msg/Path`
- `/smoothed_planned_path` as `nav_msgs/msg/Path`
- `/inflated_map` as `nav_msgs/msg/OccupancyGrid`

Current behavior:

- uses the static occupancy grid from `map_server`
- uses the Kalman localization estimated pose as the start pose
- uses the RViz goal pose as the target
- inflates obstacles using a conservative circular robot radius of `0.35 m`
- runs A* on an inflated 8-connected grid
- first simplifies the raw A* path using line-of-sight shortcutting on `inflated_map`
- publishes that shortcut result on `/planned_path`
- then tries natural cubic spline smoothing on the shortcut path
- uniformly resamples the final chosen geometry at fixed spacing and publishes it on `/smoothed_planned_path`
- if any spline sample enters occupied or unknown space, the node falls back to the shortcut geometry before the final fixed-spacing resampling step
- preserves the RViz goal orientation on both published path variants
- treats unknown cells as blocked

How spline smoothing is evaluated:

- the spline is sampled from path parameter `t = 0` to the total path length
- `spline_sample_spacing_m` controls the sampling resolution along the curve
- for example, `spline_sample_spacing_m = 0.05` means the spline is evaluated about every `5 cm`
- every sampled spline point is converted back into a map cell with `worldToGrid()`
- every sampled cell is checked with `isCellFreeForPlanning()`
- the spline geometry is accepted only if every sampled point stays inside the map and inside a free cell of the inflated map
- if any sampled point is outside the map, occupied, or unknown, the node logs a warning and falls back to the shortcut geometry
- after choosing either the spline geometry or the fallback shortcut geometry, the final path is uniformly resampled at `spline_sample_spacing_m`

Main tuning parameters:

- `robot_radius_m`
- `occupied_threshold`
- `enable_path_smoothing`
- `enable_cubic_spline_smoothing`
- `spline_sample_spacing_m`

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
- add `/map`, `/inflated_map`, `/planned_path`, and `/smoothed_planned_path`
- click a goal with the `2D Goal Pose` tool

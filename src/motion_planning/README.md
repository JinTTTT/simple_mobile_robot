# Motion Planning Package

This package computes a global path on a known 2D occupancy grid map.

The current implementation is a learning-focused A* planner for a static map.
It takes the estimated robot pose as the start state, takes an RViz goal as the target, and publishes both a shortcut path and a dense final path for downstream use.

## Assumptions

This package assumes:

- a static occupancy grid map is available on `/map`
- localization publishes the robot pose on `/estimated_pose`
- the start and goal poses are already in the `map` frame
- unknown cells should be treated as blocked

## Current Planning Pipeline

The planner currently does the following:

- copies the incoming occupancy grid into an internal planning map
- inflates occupied cells using a conservative circular robot radius
- converts the start and goal world coordinates into grid cells
- runs A* on an 8-connected inflated grid
- applies line-of-sight shortcutting to simplify the raw grid path
- publishes that shortcut path on `/planned_path`
- optionally fits a natural cubic spline through the shortcut path
- rejects the spline if any sampled point enters occupied or unknown space
- uniformly resamples the accepted geometry at fixed spacing
- publishes the final dense path on `/smoothed_planned_path`
- preserves the requested goal orientation on both published path variants

## Interfaces

The planner node subscribes to:

- `/map`: `nav_msgs/msg/OccupancyGrid`
- `/estimated_pose`: `geometry_msgs/msg/PoseStamped`
- `/goal_pose`: `geometry_msgs/msg/PoseStamped`

It publishes:

- `/planned_path`: `nav_msgs/msg/Path`
- `/smoothed_planned_path`: `nav_msgs/msg/Path`
- `/inflated_map`: `nav_msgs/msg/OccupancyGrid`

`/planned_path` is the shortcut path after line-of-sight simplification.
`/smoothed_planned_path` is the dense final path after optional spline smoothing and fixed-spacing resampling.
`/inflated_map` is the occupancy grid used internally for collision checking and planning.

## Package Structure

The package is now split into a ROS wrapper and a reusable planning core:

- `src/motion_planning_node.cpp`: ROS subscriptions, publishers, parameters, and planning triggers
- `include/motion_planning/motion_planner.hpp`: planner interface and result types
- `src/motion_planner.cpp`: A*, map inflation, line-of-sight shortcutting, spline smoothing, and resampling

This keeps the ROS node thin and moves the planning logic into a clearer core module.

## Parameters

Planner tuning lives in:

```text
src/motion_planning/config/motion_planning.yaml
```

Main parameters:

- `robot_radius_m`
- `occupied_threshold`
- `enable_path_smoothing`
- `enable_cubic_spline_smoothing`
- `spline_sample_spacing_m`

Behavior notes:

- `robot_radius_m` controls obstacle inflation before planning
- `enable_path_smoothing` enables line-of-sight shortcutting
- `enable_cubic_spline_smoothing` enables spline smoothing after shortcutting
- `spline_sample_spacing_m` is used for spline sampling and final fixed-spacing resampling

## Run And Visualize

Build the package from the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --packages-select motion_planning
source install/setup.bash
```

Open a new terminal for each command below.
In each terminal, source the workspace first:

```bash
cd ~/workspace/gazebo_ws
source install/setup.bash
```

Start simulation:

```bash
ros2 launch simulation bringup_simulation.launch.py
```

Start the saved map server:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/mapping/maps/maze_map.yaml
```

Activate the map server:

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

Start localization:

```bash
ros2 launch localization kalman_localization.launch.py
```

Start motion planning:

```bash
ros2 launch motion_planning motion_planning.launch.py
```

Start RViz:

```bash
rviz2
```

In RViz:

- set Fixed Frame to `map`
- add `/map`
- add `/inflated_map`
- add `/planned_path`
- add `/smoothed_planned_path`
- use the `2D Goal Pose` tool to request a path

## Limitations

The current package is still a simple global planner:

- it only uses a static occupancy grid
- it does not do local obstacle avoidance
- it does not replan continuously unless a new goal is requested
- it assumes the localization pose is already good enough
- it does not handle dynamic obstacles

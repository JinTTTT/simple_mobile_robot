# Path Follow Control Package

This package is the next learning step after `motion_planning`.

Its role is different from the global planner:

- `motion_planning` decides which path the robot should take
- `path_follow_control` decides what velocity command the robot should send right now

Inputs:

- `/smoothed_planned_path`
- `/estimated_pose`

Output:

- `/cmd_vel`

The first version uses pure pursuit for path following on a static planned path.

Main goal:

- convert a planned path into smooth robot motion

Current behavior:

- rotate toward the path direction
- follow a lookahead point along the path
- compute pure-pursuit curvature from the lookahead point in the robot frame
- set angular speed from `w = v * curvature`
- reduce linear speed on sharper turns with curvature-based slowdown
- slow down near the goal
- stop when the goal position is reached
- rotate in place to align with the final goal orientation
- report continuous path progress and goal distance in the logs
- declare stuck when the robot is commanded to move but does not move enough or reduce goal distance enough over time

Main tuning parameters:

- `lookahead_distance`
- `max_linear_speed`
- `min_linear_speed`
- `max_angular_speed`
- `curvature_slowdown_gain`
- `rotate_in_place_angle_threshold`
- `goal_tolerance_distance`
- `goal_tolerance_angle`
- `slow_down_goal_distance`
- `final_alignment_max_angular_speed`
- `stuck_detection_window_seconds`
- `min_progress_distance_m`
- `min_goal_distance_improvement_m`

Config file:

- `config/path_follow_control.yaml`

Launch file:

- `launch/path_follow_control.launch.py`

How to test:

- start simulation
- start the static map server with `src/mapping/maps/maze_map.yaml`
- start `kalman_localization_node`
- start `motion_planning_node`
- start `path_follow_control_node` or `ros2 launch path_follow_control path_follow_control.launch.py`
- open RViz with fixed frame `map`
- add `/map`, `/inflated_map`, `/planned_path`, and `/smoothed_planned_path`
- click a goal with the `2D Goal Pose` tool, including a desired final heading

This package is not the global planner.
It is also not yet a full local planner with dynamic obstacle avoidance.
Its role is the control layer between path planning and actual robot motion.

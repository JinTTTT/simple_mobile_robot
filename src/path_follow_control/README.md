# Path Follow Control Package

This package is the next learning step after `motion_planning`.

Its role is different from the global planner:

- `motion_planning` decides which path the robot should take
- `path_follow_control` decides what velocity command the robot should send right now

Planned inputs:

- `/planned_path`
- `/estimated_pose`

Planned output:

- `/cmd_vel`

The first version should focus on path following for a static path.

Main goal:

- convert a planned path into smooth robot motion

Likely learning steps:

- rotate toward the path direction
- drive toward the next path point
- slow down near the goal
- stop when the goal is reached

This package is not the global planner.
It is also not yet a full local planner with dynamic obstacle avoidance.
Its role is the control layer between path planning and actual robot motion.

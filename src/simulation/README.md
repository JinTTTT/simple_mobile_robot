# Simulation Package

This package runs the robot simulation.

It uses Gazebo to create:

- a simple differential drive robot
- a 2D lidar
- a world for the robot to drive in
- ROS 2 topics for movement and sensor data

## What It Publishes

The simulation and bridge publish:

- `/odom`
- `/scan`
- `/tf`

## What It Reads

The robot reads:

- `/cmd_vel`

`/cmd_vel` is the velocity command.
For example, teleop publishes to `/cmd_vel` when you press the keyboard.

## Build

From the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Start the simulation:

```bash
ros2 launch simulation bringup_simulation.launch.py
```

In another terminal, start keyboard control:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Useful keys:

- `i`: move forward
- `,`: move backward
- `j`: turn left
- `l`: turn right
- `k`: stop

## View in RViz

Start RViz:

```bash
rviz2
```

For only simulation, use:

- Fixed Frame: `odom`
- Add `TF`
- Add `Odometry`

For mapping or localization, the fixed frame is usually `map`.

## Important Idea

Simulation gives fake sensor data.

This is useful because we can test mapping and localization without a real robot.
The code can subscribe to `/scan` and `/odom` just like it would on a real robot.

# Localization Package

This package is the current learning stage of the repo.

The goal is to estimate where the robot is on a known map.

## Current Situation

The package has a first particle filter.

Right now it can:

- read a map from `/map`
- create 500 particles on free map cells
- read odometry from `/odom`
- move the particles when the robot moves
- publish particles on `/particlecloud`

Right now it does not yet:

- use the laser scan to score particles
- resample particles
- publish the best robot pose
- publish the `map` to `odom` transform

So this is not a full localization system yet.
It is the first step.

## Topics

The localization node subscribes to:

- `/map`
- `/odom`
- `/scan`

It publishes:

- `/particlecloud`

`/particlecloud` is a `PoseArray`.
In RViz, each pose is drawn as one arrow.
Each arrow means one possible robot position and direction.

## Build

From the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --packages-select localization
source install/setup.bash
```

## Run

Terminal 1:

```bash
ros2 launch simulation bringup_simulation.launch.py
```

Terminal 2:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Terminal 3:

```bash
rviz2
```

Terminal 4:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

This is a temporary fix.
It says `map` and `odom` are the same frame.

Terminal 5:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/mapping/maps/maze_map.yaml
```

Terminal 6:

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

Terminal 7:

```bash
ros2 run localization localization_node
```

In RViz:

- set Fixed Frame to `map`
- add `/map`
- add `/particlecloud`

Then drive the robot with teleop.
You should see the particles move.

## How It Works

A particle is one guess about the robot pose.

A pose means:

- x position
- y position
- heading angle

At startup, the node puts particles randomly on free map cells.
This means: "the robot could be anywhere free."

When odometry says the robot moved, every particle moves too.
This means: "if the robot moved forward, every guess should move forward."

The next step is to use `/scan`.
The laser scan should tell which particles match the map well.
Good particles should get high weight.
Bad particles should get low weight.

After that, resampling can keep good particles and remove bad particles.

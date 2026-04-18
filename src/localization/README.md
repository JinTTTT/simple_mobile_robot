# Localization Package

This package is the current learning stage of the repo.

The goal is to estimate where the robot is on a known map.

## Current Situation

The package has a first particle filter.

Right now it can:

- read a map from `/map`
- build and publish a likelihood field on `/likelihood_field`
- create 500 particles on free map cells
- read odometry from `/odom`
- move the particles when the robot moves
- use `/scan` to score particles
- resample particles using stochastic universal resampling
- publish particles on `/particlecloud`
- publish one estimated robot pose on `/estimated_pose`

Right now it does not yet:

- publish the `map` to `odom` transform
- add realistic noise to the odometry motion model
- decide when resampling is needed instead of resampling on every scan

So this is now a working first particle-filter localization version.
It can localize visually in RViz, but it is not a full ROS navigation localizer yet.

## Topics

The localization node subscribes to:

- `/map`
- `/odom`
- `/scan`

It publishes:

- `/particlecloud`
- `/likelihood_field`
- `/estimated_pose`

`/particlecloud` is a `PoseArray`.
In RViz, each pose is drawn as one arrow.
Each arrow means one possible robot position and direction.

`/likelihood_field` is an `OccupancyGrid`.
It shows where laser hits are likely.
Cells near walls have high values.
Cells far from walls have low values.

`/estimated_pose` is a `PoseStamped`.
It is one pose calculated from the particle cloud.
This is the package's current best guess of the robot pose.

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
- add `/likelihood_field`
- add `/particlecloud`
- add `/estimated_pose`

Then drive the robot with teleop.
You should see the particles move and converge near the robot.
You should also see `/estimated_pose` near the center of the particle cloud.

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

When a laser scan arrives, each particle gets a score.
The score asks:

"If the robot were at this particle, would the laser hits land near walls?"

Good particles get higher scores.
Bad particles get lower scores.

After scoring, resampling keeps more good particles and removes more bad particles.
This package uses stochastic universal resampling.
This is also called low-variance resampling.

The node also publishes `/estimated_pose`.
It averages the particle positions.
For the heading angle, it averages `sin(theta)` and `cos(theta)`.
This avoids a common angle problem.

Example:

- `179 degrees`
- `-179 degrees`

These are almost the same direction.
A normal average would give the wrong answer.
The sine/cosine average handles this better.

## Later Improvements

The current version is good for learning and visual testing.
Later we should improve:

- publish the `map -> odom` transform
- add motion noise, because real odometry is not perfect
- resample only when needed, not necessarily on every scan
- improve the laser sensor model
- publish confidence information, so we know how certain the estimate is
- add parameters for particle count, beam step, noise, and resampling behavior

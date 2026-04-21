# Mobile Robot Full Stack Learning Repo

This repo is my place to learn the full software stack of a mobile robot.

The long term goal is to learn:

- simulation: finished
- mapping: finished
- localization : finished
- SLAM : first version finished
- planning : A* planner with collision clearance finished
- navigation : in planning

## Packages

### `simulation`

This package starts the robot in Gazebo.

It gives the robot:

- differential drive movement
- a 2D lidar
- odometry on `/odom`
- laser scans on `/scan`
- TF data for the robot frames

The robot can be controlled with keyboard teleop by publishing velocity commands to `/cmd_vel`.

### `mapping`

This package builds an occupancy grid map.

It reads:

- `/scan` from the lidar
- TF from `odom` to `base_link`

It publishes:

- `/map`

The mapper uses Bresenham ray tracing to detect the cells along the laser beam. Use log-odds to update the map.

It only use odom as a rough guess of the robot pose.

Simple meaning:

- cells along a laser beam become more likely to be free
- the cell hit by the laser becomes more likely to be occupied
- repeated scans make the map more confident

### `localization`

This package contains learning localization nodes.

It assumes a saved global occupancy grid map is available.
It includes two localization approaches:

- particle filter localization
- Kalman filter localization

The particle-filter node reads:

- `/map`
- `/odom`
- `/scan`

It publishes:

- `/particlecloud`
- `/likelihood_field`
- `/estimated_pose`
- TF: `map -> odom`

Simple logic:

- starts particles on free map cells
- builds a likelihood field from the map
- moves particles using odometry with small motion noise
- scores particles using laser scans
- resamples after robot movement
- publishes the estimated pose and the normal ROS `map -> odom` transform

The Kalman-filter node assumes a known initial pose of `0, 0, 0`.
It reads `/map`, `/odom`, and `/scan`.
It publishes `/estimated_pose`, `/estimated_pose_with_covariance`, `/scan_matched_pose`, and `map -> odom`.
The scan-matched pose is also used as a Kalman correction measurement when its score and distance gates pass.
It is a local tracker, so large odometry errors can still make it lose the actual pose.

### `slam`

This package is the first learning version of SLAM.

It reads:

- `/odom`
- `/scan`

It publishes:

- `/map`
- `/corrected_map`
- `/estimated_pose`
- `/scan_matched_pose`
- `/trajectory`
- `/corrected_trajectory`
- `/loop_closure_pose`
- TF: `map -> odom`

Simple logic:

- predict pose from odometry
- use local scan matching for small pose correction
- build a live occupancy-grid map
- store trajectory history and keyframes
- detect loop closure from old keyframes
- apply a simple evenly spread trajectory correction
- rebuild a corrected map from corrected keyframes

This SLAM package is good enough as a first learning version.
It does not yet do full pose graph optimization or robust wall-hit recovery.

### `motion_planning`

This package is the first learning version of global path planning.

It reads:

- `/map`
- `/estimated_pose`
- `/goal_pose`

It publishes:

- `/planned_path`
- `/inflated_map`

Simple logic:

- use the Kalman localization pose as the robot start
- use the RViz goal pose as the planning target
- inflate obstacles using a conservative circular robot radius from the simulation geometry
- convert start and goal from world coordinates into map grid cells
- run A* on an inflated 8-connected occupancy grid
- publish the planned path back in the `map` frame

This first planner uses the static saved map from `nav2_map_server`.
It treats unknown cells as blocked and publishes the inflated map for RViz checking.
It only plans a global path.
It does not yet include path smoothing, local planning, or path following.

## Quick Start

### Prerequisites

Install:

- ROS 2
- Gazebo / Gazebo Sim
- `colcon`
- `teleop_twist_keyboard`
- Nav2 map server tools

Useful ROS packages:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-nav2-map-server ros-humble-nav2-util
```

### Build

From the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --symlink-install
source install/setup.bash
```

Open a new terminal for each command below.
In each new terminal, source the workspace first:

```bash
cd ~/workspace/gazebo_ws
source install/setup.bash
```

## Run Case 1: Mapping

### 1. Start simulation

```bash
ros2 launch simulation bringup_simulation.launch.py
```

### 2. Start teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keyboard to drive the robot around.
The mapper needs robot movement to see the world.

### 3. Start mapper

```bash
ros2 run mapping occupancy_mapper
```

### 4. View in RViz

```bash
rviz2
```

In RViz:

- set Fixed Frame to `map`
- add the `/map` topic

## Run Case 2: Localization

### 1. Start simulation

```bash
ros2 launch simulation bringup_simulation.launch.py
```

### 2. Start teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Start RViz

```bash
rviz2
```

### 4. Publish the saved map

Start the map server:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/mapping/maps/maze_map.yaml
```

Activate the lifecycle node:

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

### 5. Start localization

Option 1: particle-filter localization

```bash
ros2 run localization particle_filter_localization_node
```

Option 2: Kalman-filter localization

```bash
ros2 run localization kalman_localization_node
```

In RViz:

- set Fixed Frame to `map`
- add the `/map` topic
- add the `/likelihood_field` topic
- add the `/particlecloud` topic
- add the `/estimated_pose` topic

The `/particlecloud` topic shows the particles.
Each particle is one possible robot pose.
The `/estimated_pose` topic shows the current best pose estimate.

Do not run a static `map -> odom` transform during localization.
The localization node publishes `map -> odom`.

For the Kalman-filter node, add `/estimated_pose`, `/estimated_pose_with_covariance`, and `/scan_matched_pose` in RViz.

Do not run both localization nodes at the same time.

## Run Case 3: SLAM

### 1. Start simulation

```bash
ros2 launch simulation bringup_simulation.launch.py
```

### 2. Start teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. Start SLAM

```bash
ros2 launch slam simple_slam.launch.py
```

### 4. Start RViz

```bash
rviz2
```

In RViz:

- set Fixed Frame to `map`
- add `/map`
- add `/corrected_map`
- add `/estimated_pose`
- add `/trajectory`
- add `/corrected_trajectory`
- add `/loop_closure_pose`
- add `TF`

During normal driving:

- `/map` is the live online map
- `/corrected_map` is rebuilt from corrected keyframes after loop-closure correction
- `/trajectory` is the raw path
- `/corrected_trajectory` is the loop-corrected keyframe path

## Project Structure

```text
gazebo_ws/
├── README.md
└── src/
    ├── slam/
    ├── simulation/
    ├── mapping/
    └── localization/
```

## Current Issues and Future Improvements

- Simulation works.
- Mapping works as a basic occupancy grid mapper.
- Localization includes a particle-filter node with likelihood-field scan scoring.
- Localization includes a simple Kalman-filter node with odometry prediction, scan-matching correction, and covariance output.
- SLAM v1 includes live mapping, local scan matching, trajectory history, loop-closure detection, simple trajectory correction, and corrected-map rebuilding.
- SLAM v1 still has known limitations in wall-hit / odometry-disagreement cases and does not yet include pose graph optimization.

# Mobile Robot Full Stack Learning Repo

This repo is my place to learn the full software stack of a mobile robot.

The long term goal is to learn:

- simulation
- mapping
- localization
- SLAM
- planning
- navigation

The first localization stage is now complete.
The repo has a working particle-filter localizer on a saved map.

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

The mapper uses Bresenham ray tracing and log-odds updates.

Simple meaning:

- cells along a laser beam become more likely to be free
- the cell hit by the laser becomes more likely to be occupied
- repeated scans make the map more confident

### `localization`

This package is the completed first localization stage.

It runs a first working particle filter localizer.

It reads:

- `/map`
- `/odom`
- `/scan`

It publishes:

- `/particlecloud`
- `/likelihood_field`
- `/estimated_pose`
- TF: `map -> odom`

It uses the saved map, odometry, and laser scans to estimate the robot pose.
The particle filter:

- starts particles on free map cells
- builds a likelihood field from the map
- moves particles using odometry with small motion noise
- scores particles using laser scans
- resamples after robot movement
- publishes the estimated pose and the normal ROS `map -> odom` transform

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

```bash
ros2 run localization localization_node
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

## Project Structure

```text
gazebo_ws/
├── README.md
└── src/
    ├── simulation/
    ├── mapping/
    └── localization/
```

## Current Status

- Simulation works.
- Mapping works as a basic occupancy grid mapper.
- Localization first version is complete.
- Localization uses a particle filter with a likelihood field, laser scan scoring, motion noise, and resampling.
- Localization publishes `/particlecloud`, `/likelihood_field`, `/estimated_pose`, and `map -> odom`.
- Future localization improvements can include better scan scoring, confidence output, and more tuning.

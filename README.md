# Mobile Robot Full Stack Learning Repo

This repo is my place to learn the full software stack of a mobile robot.

The long term goal is to learn:

- simulation
- mapping
- localization
- SLAM
- planning
- navigation

Right now I am working on the localization stage.

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

This package is the current learning stage.

It starts a simple particle filter.

It reads:

- `/map`
- `/odom`
- `/scan`

It publishes:

- `/particlecloud`

For now, the particles are initialized on free map cells and moved with odometry.
The laser scan is subscribed, but it is not used for particle weights yet.

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

For now the TF tree is not complete.
Use this temporary static transform:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

This says: "for now, treat `map` and `odom` as the same frame."

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
- add the `/particlecloud` topic

The `/particlecloud` topic shows the particles.
Each particle is one possible robot pose.

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
- Localization has the first particle filter structure.
- Localization still needs laser scan scoring and resampling.

# Autonomous Driving System

A full-stack autonomous driving codebase using Gazebo for simulation. This project focuses on the fundamental pipeline of autonomous driving with minimal external dependencies to help you understand how each component works.

## Project Overview

This system uses a simple differential drive robot with a 2D lidar sensor in Gazebo simulation. The focus is on core concepts: mapping, localization, simultaneous localization and mapping (SLAM), and navigation.

**Key Features:**
- Simple robot model with differential drive kinematics
- 2D lidar simulation (360 degrees, 10m range)
- Minimal dependencies - understand the fundamentals
- Educational focus on autonomous driving pipeline

## Project Roadmap

### ✅ Phase 1: Simulation System (Complete)
Build a simulation environment with a car and lidar sensor.

**Status:** Done
- Differential drive robot with two wheels and caster
- 2D lidar sensor (360 samples, 10Hz update rate)
- Gazebo world with basic environment
- Odometry and velocity command interfaces

See [`src/my_bot/`](src/my_bot/) for implementation details.

### ✅ Phase 2: Mapping (Complete)
Build 2D occupancy grid maps from lidar scans and odometry data.

**Status:** Done
- Subscribes to `/scan` (lidar data) and `/odom` (odometry)
- Generates 2D occupancy grid map using simple counting method
- Publishes map on `/map` topic at 2 Hz
- Real-time visualization in RViz
- Bresenham ray tracing for efficient grid updates

See [`src/mapping/`](src/mapping/) for implementation details.

### 📋 Phase 3: Localization (Planned)
Estimate robot position using particle filter algorithm.

**Goals:**
- Particle filter implementation
- Use lidar scans to match against known map
- Publish robot pose estimate
- Handle kidnapped robot problem

### 📋 Phase 4: SLAM (Planned)
Build maps and localize simultaneously.

**Goals:**
- Combine mapping and localization
- Loop closure detection
- Map correction and optimization
- Real-time performance

### 📋 Phase 5: Navigation (Planned)
Drive from point A to point B along a collision-free path.

**Goals:**
- Global path planning (A* or Dijkstra)
- Local obstacle avoidance
- Path following control
- Goal reaching behavior

## Quick Start

### Prerequisites
- ROS 2 (Humble or later)
- Gazebo (Ignition/Gazebo)
- Python 3

### Build and Run

1. **Build the workspace:**
```bash
cd ~/workspace/gazebo_ws
colcon build --symlink-install
source install/setup.bash
```

2. **Launch the complete system:**
```bash
ros2 launch my_bot bringup_simulation.launch.py
```

This single command starts:
- Gazebo simulation with robot
- ROS 2 ↔ Gazebo bridge for /cmd_vel, /odom, /tf, /scan

3. **Control the robot** (in a new terminal):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard controls:**
- `i` = forward, `,` = backward
- `j` = turn left, `l` = turn right
- `k` = stop

4. **Run the mapper** (in a new terminal):
```bash
ros2 run mapping occupancy_mapper
```

5. **Visualize the map** (in a new terminal):
```bash
rviz2
```

In RViz: Set Fixed Frame to `odom`, then Add → By topic → `/map` → Map

For detailed instructions, see [`src/my_bot/README.md`](src/my_bot/README.md) and [`src/mapping/README.md`](src/mapping/README.md).

## Project Structure

```
gazebo_ws/
├── src/
│   ├── my_bot/              # Simulation package (Phase 1)
│   │   ├── launch/          # Launch files
│   │   ├── urdf/            # Robot description
│   │   └── worlds/          # Gazebo world files
│   └── mapping/             # Mapping package (Phase 2)
│       ├── src/             # C++ source files
│       └── CMakeLists.txt   # Build configuration
└── README.md                # This file
```

## Learning Path

If you're new to autonomous driving, follow this learning path:

1. **Start with simulation** - Understand how the robot moves and senses
2. **Build maps** - See how lidar data creates 2D representations
3. **Add localization** - Learn how robots know where they are
4. **Combine with SLAM** - Understand the chicken-and-egg problem
5. **Navigate** - Put it all together to drive autonomously

## Contributing

This is an educational project. Feel free to:
- Add comments to explain complex algorithms
- Improve documentation
- Optimize implementations
- Add visualization tools

Keep it simple and educational!

## License

TODO: Add license

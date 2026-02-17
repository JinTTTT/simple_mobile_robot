# Mapping Package

Builds 2D occupancy grid maps from lidar scans and odometry.

## Usage

```bash
# Build
cd ~/workspace/gazebo_ws
colcon build --packages-select mapping
source install/setup.bash

# Run
ros2 run mapping occupancy_mapper

# Visualize in RViz
# Set Fixed Frame to "odom"
# Add → Map → Topic: /map
```

## Algorithm

**Simple Counting Method:**
- Each grid cell has two counters: `hits` (obstacle detected) and `passes` (laser passed through)
- Uses Bresenham ray tracing to find cells along each laser beam
- Occupancy probability = `hits / (hits + passes)`
- Publishes map at 2 Hz

**Map configuration:**
- Resolution: 0.05m (5cm per cell)
- Size: 200×200 cells (10m × 10m)
- Subscribes: `/scan`, `/odom`
- Publishes: `/map`

## Current Problems

1. **Odometry drift** - Trusts odometry 100%, no correction. Long sessions cause duplicate walls when revisiting areas.

2. **Static world assumption** - Counters always increase, cannot forget old obstacles. Moving objects leave permanent marks.

3. **Fuzzy walls** - Grid discretization + sensor noise + odometry drift create inconsistent wall edges.

4. **No loop closure** - Cannot detect when returning to known locations, no map optimization.

**Solutions:** Phase 3 (Localization) and Phase 4 (SLAM) will address these issues.

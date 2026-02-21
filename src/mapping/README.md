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

**Note:** The package includes two implementations:
- `occupancy_mapper.cpp` - Current log-odds Bayesian method
- `occupancy_mapper_simple_count_method.cpp` - Original simple counting method (backup)

## Algorithm used

**Bresenham ray tracing:**
- To return the cells that the laser beam hits.

**Log-Odds Bayesian Update Method:**
- To update the occupancy grid map based on the laser scan.
- Gradually update using bayesian.


## Current Issues to solve

1. Cannot detect removed obstacles. Once an obstacle is mapped, removing it physically won't update the map. 

2. Trusts odometry 100%, no correction. 

3. No loop closure - Cannot detect when returning to known locations, no map optimization.

---


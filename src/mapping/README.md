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

## Algorithm

**Log-Odds Bayesian Update Method:**
- Uses **log-odds representation** for probabilistic occupancy mapping
- Implements proper Bayesian sensor fusion with configurable sensor model
- Real-time map updates using Bresenham ray tracing for each laser beam
- Publishes map at 2 Hz

**Sensor Model (configurable):**
- `P(hit|occupied)` = 0.90 (90% obstacle detection rate)
- `P(hit|free)` = 0.05 (5% false positive rate)
- Log-odds updates: `log_odds_hit` ≈ 2.89, `log_odds_pass` ≈ -2.25
- Saturation limits: ±10.0 (prevents infinite confidence)

**Map Configuration:**
- Resolution: 0.05m (5cm per cell)
- Size: 200×200 cells (10m × 10m)
- Subscribes: `/scan`, `/odom`
- Publishes: `/map`

**Thresholding:**
- Occupied: probability > 65%
- Free: probability < 35%
- Unknown: 35% ≤ probability ≤ 65%

## Current Status and Limitations

**Improvements over simple counting:**
- ✅ Bayesian sensor fusion (mathematically principled)
- ✅ Configurable sensor model (accounts for sensor noise)
- ✅ Saturation limits (prevents over-confidence)
- ✅ Real-time updates with reasonable responsiveness

**Known Issues:**

1. **Static world assumption** - Cannot detect removed obstacles. Once an obstacle is mapped, removing it physically won't update the map. The saturation limits make this worse - a cell at max confidence requires many contradictory observations to flip.

2. **Parameter tuning needed** - Current parameters may not be optimal:
   - `log_odds_max/min` (±10.0) might be too high (slow to adapt)
   - Sensor model probabilities need validation against real performance
   - Thresholds (65%/35%) are somewhat arbitrary

3. **Odometry drift** - Trusts odometry 100%, no correction. Long sessions cause:
   - Duplicate walls when revisiting areas
   - Ghost obstacles from accumulated drift
   - Map inconsistencies

4. **Map accuracy** - Limited by:
   - Grid discretization (5cm cells)
   - Ray-tracing approximation
   - Sensor noise and odometry errors
   - Lack of free space handling for max-range returns

5. **No loop closure** - Cannot detect when returning to known locations, no map optimization.

**Future Improvements:**
- Handle max-range lidar returns (mark free space in open areas)
- Implement map boundary clipping (don't skip out-of-bounds observations)
- Tune saturation limits (try ±5.0 for better responsiveness)
- Phase 3 (Localization) and Phase 4 (SLAM) will address drift and loop closure

## Parameter Tuning Guide

**Saturation Limits (`log_odds_max/min`):**

Current: `±10.0` → 99.995% confidence (very stable, slow to change)

| Value | Max Confidence | Observations to Flip | Use Case |
|-------|----------------|---------------------|----------|
| ±2.0  | 88% | ~2 scans | Dynamic environments, noisy |
| ±5.0  | 99.3% | ~5 scans | **Recommended balance** |
| ±7.0  | 99.9% | ~7 scans | Static, stable mapping |
| ±10.0 | 99.995% | ~9 scans | Very stable, hard to update |

**How to tune:**
- Map too noisy/flickering? → Increase limits (±7.0)
- Ghost obstacles won't disappear? → Decrease limits (±5.0)
- Obstacles slow to appear? → Decrease limits (±3.0)

**Sensor Model Probabilities:**

Current: `P(hit|occupied)=0.90`, `P(hit|free)=0.05`

- Higher `P(hit|occupied)` → Trust obstacle detections more
- Lower `P(hit|free)` → Assume fewer false positives
- For perfect simulation: can increase both (0.95 and 0.02)
- For noisy sensors: decrease confidence (0.7 and 0.1)

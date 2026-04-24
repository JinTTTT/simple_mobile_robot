# Localization Package

This package estimates the robot pose in a known map.

The main goal is to connect the robot's local odometry frame to the global map frame:

```text
map -> odom -> base_link
```

Gazebo publishes `odom -> base_link`.
The localization package estimates where the robot is in the map and publishes `map -> odom`.

## Assumptions

This package assumes:
- a prebuilt global occupancy grid map is available

## Localization Approaches

This package implements two localization algorithms:

- particle filter localization
- Kalman filter localization

The particle filter is a global localization approach. It will be better if the initial pose is unknown or the robot gets lost.
It can start with particles spread across the free space of the map.

The Kalman filter is a local pose tracker. It will be better if the initial pose is close to the actual pose and the robot does not get lost. The localization will be smoother than the particle filter, but it can lose track if the odometry error grows too large or if the scan matcher fails to find a good local match.
It assumes the robot starts near the known initial pose `x=0`, `y=0`, `theta=0`.

## Particle Filter Localization

The particle filter represents the robot pose with many weighted pose guesses.
Each particle is one possible robot position and heading in the map.

At startup, particles are sampled on free map cells.
As the robot moves, odometry moves every particle with noise.
When a scan arrives, each particle is scored by checking whether laser hits land near occupied map cells.
Better particles get higher weights.
The filter then resamples so good pose guesses survive and bad pose guesses disappear.

### Interfaces

The particle-filter node subscribes to:

- `/map`: `nav_msgs/msg/OccupancyGrid`
- `/odom`: `nav_msgs/msg/Odometry`
- `/scan`: `sensor_msgs/msg/LaserScan`

It publishes:

- `/particlecloud`: `geometry_msgs/msg/PoseArray`
- `/likelihood_field`: `nav_msgs/msg/OccupancyGrid`
- `/estimated_pose`: `geometry_msgs/msg/PoseStamped`
- TF: `map -> odom`

`/particlecloud` shows all particles in the map frame.
In RViz, each pose is drawn as one arrow.

`/likelihood_field` shows how close each map cell is to an occupied cell.
Cells near walls have high values.
Cells far from walls have low values.

`/estimated_pose` is the current best pose estimate from the particle cloud.

### Characteristics

Advantages:

- can localize globally because particles can start anywhere in free space
- can represent multiple pose hypotheses before convergence
- works well when the initial robot pose is unknown
- is easy to visualize in RViz through `/particlecloud`

Disadvantages:

- needs enough particles to cover the map
- can be slower than a Kalman filter
- can lose accuracy if the likelihood model is too simple
- may converge to a wrong pose in symmetric environments
- depends on tuning values such as particle count, scan beam step, and resampling noise

### Important Methods

The current implementation uses:

- uniform initialization on free map cells
- likelihood field construction from the occupancy grid map
- odometry motion model with sampled translation and rotation noise
- scan likelihood scoring using every 10th laser beam
- stochastic universal resampling
- pose averaging with sine and cosine for heading

Stochastic universal resampling is also called low-variance resampling.
It walks through the normalized particle weights with evenly spaced pointers.
This keeps more copies of high-weight particles while reducing random sampling noise.

The estimated heading is averaged with `sin(theta)` and `cos(theta)`.
This avoids the angle wraparound problem where `179 degrees` and `-179 degrees` are almost the same direction but a normal average would be wrong.

The node publishes `map -> odom` with:

```text
map_to_odom = map_to_base_link * inverse(odom_to_base_link)
```

`map_to_base_link` comes from the particle filter estimate.
`odom_to_base_link` comes from TF.

### Parameters

Particle-filter tuning lives in:

```text
src/localization/config/localization.yaml
```

Main parameters:

- particle count: `500`
- random seed: `42`
- maximum likelihood field distance: `1.0 m`
- scan beam step: `10`
- movement threshold before motion update: `0.001`
- resampling position noise: `0.02 m`
- resampling heading noise: `0.03 rad`

The parameter names are:

- `num_particles`
- `random_seed`
- `likelihood_max_distance`
- `scan_beam_step`
- `motion_update_min_translation`
- `motion_update_min_rotation`
- `translation_noise_from_translation`
- `translation_noise_base`
- `rotation_noise_from_rotation`
- `rotation_noise_from_translation`
- `rotation_noise_base`
- `resample_xy_noise_std`
- `resample_theta_noise_std`

### Run and Visualize

Build the package from the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --packages-select localization
source install/setup.bash
```

Open a new terminal for each command below.
In each terminal, source the workspace first:

```bash
cd ~/workspace/gazebo_ws
source install/setup.bash
```

Start simulation:

```bash
ros2 launch simulation bringup_simulation.launch.py
```

Start teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Start RViz:

```bash
rviz2
```

Publish the saved map:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=src/mapping/maps/maze_map.yaml
```

Activate the map server:

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

Start particle-filter localization:

```bash
ros2 run localization particle_filter_localization_node
```

or with the package config:

```bash
ros2 launch localization particle_filter_localization.launch.py
```

In RViz:

- set Fixed Frame to `map`
- add `/map`
- add `/likelihood_field`
- add `/particlecloud`
- add `/estimated_pose`

Drive the robot with teleop.
The particles should move with odometry and gradually concentrate near the robot pose.

## Kalman Filter Localization

The Kalman filter estimates the robot pose as one state with uncertainty.
Instead of keeping many particles, it keeps a mean pose and covariance.

This implementation uses odometry prediction and local scan-matching correction.
The scan matcher searches around the current Kalman estimate, scores candidate poses against a likelihood field built from `/map`, and publishes the best local match.
When the scan match score is high enough and the matched pose is close enough to the current prediction, the matched pose is used as a Kalman correction measurement.

This is not global localization.
It is a local tracker from the known initial pose `0, 0, 0`.

### Interfaces

The Kalman-filter node subscribes to:

- `/map`: `nav_msgs/msg/OccupancyGrid`
- `/odom`: `nav_msgs/msg/Odometry`
- `/scan`: `sensor_msgs/msg/LaserScan`

It publishes:

- `/estimated_pose`: `geometry_msgs/msg/PoseStamped`
- `/estimated_pose_with_covariance`: `geometry_msgs/msg/PoseWithCovarianceStamped`
- `/scan_matched_pose`: `geometry_msgs/msg/PoseStamped`
- TF: `map -> odom`

### Characteristics

Advantages:

- computationally cheaper than a particle filter
- directly represents pose uncertainty with covariance
- can be smooth and stable when the initial pose is good
- uses scan matching to correct odometry drift
- publishes `/scan_matched_pose` for debugging the correction source

Disadvantages:

- does not naturally represent multiple possible poses
- depends strongly on a reasonable initial pose
- can fail in ambiguous or highly nonlinear cases
- is only a local tracker, not a global relocalizer
- can lose the actual pose if odometry error grows outside the scan matcher search window
- depends on a reasonable initial pose and scan-matching gate tuning

### Important Methods

The current implementation uses:

- initial pose setup at `x=0`, `y=0`, `theta=0`
- odometry-based prediction for `[x, y, theta]`
- covariance growth from process noise when the robot moves
- likelihood field construction from the occupancy grid map
- local scan matching around the current Kalman estimate
- scan-match gating by score, translation distance, and rotation distance
- Kalman correction with the accepted scan-matched pose
- covariance publishing in `/estimated_pose_with_covariance`

The scan matcher searches around the current estimate with a small grid of candidate poses.
Each candidate pose is scored by projecting selected scan beams into the likelihood field.
The best valid candidate is published on `/scan_matched_pose`.

### Parameters

Kalman-filter and scan-matcher tuning lives in:

```text
src/localization/config/localization.yaml
```

Main parameters:

- initial pose: `x=0`, `y=0`, `theta=0`
- initial standard deviations: `0.02`, `0.02`, `0.02`
- base process standard deviations: `0.005`, `0.005`, `0.002`
- distance noise scale: `0.10`
- rotation noise scale: `0.10`
- minimum odometry update deltas: `0.001 m`, `0.001 rad`
- scan matcher search range: `0.20 m`, `0.20 rad`
- scan matcher search step: `0.05 m`, `0.05 rad`
- scan matcher beam step: `10`
- scan matcher likelihood distance: `1.0 m`
- minimum scan match score for correction: `0.40`
- maximum correction distance: `0.25 m`
- maximum correction rotation: `0.25 rad`
- scan match measurement standard deviations: `0.08`, `0.08`, `0.08`

The parameter groups are:

- initial pose: `initial_x`, `initial_y`, `initial_theta`
- initial covariance: `initial_std_x`, `initial_std_y`, `initial_std_theta`
- process noise: `base_process_std_x`, `base_process_std_y`, `base_process_std_theta`, `distance_noise_scale`, `rotation_noise_scale`
- odometry update thresholds: `min_translation_delta`, `min_rotation_delta`
- scan matching: `search_xy_range`, `search_theta_range`, `search_xy_step`, `search_theta_step`, `scan_match_beam_step`, `likelihood_max_distance`
- scan-match correction gate: `min_scan_match_score`, `max_correction_translation`, `max_correction_rotation`
- scan-match measurement noise: `scan_match_std_x`, `scan_match_std_y`, `scan_match_std_theta`

### Run and Visualize

Use the same simulation, teleop, RViz, and map-server setup from the particle-filter run section.
Then start Kalman-filter localization:

```bash
ros2 run localization kalman_localization_node
```

or with the package config:

```bash
ros2 launch localization kalman_localization.launch.py
```

In RViz:

- set Fixed Frame to `map`
- add `/map`
- add `/estimated_pose`
- add `/estimated_pose_with_covariance`
- add `/scan_matched_pose`

The `/estimated_pose` topic shows the Kalman estimate.
The `/estimated_pose_with_covariance` topic shows the estimate with uncertainty.
The `/scan_matched_pose` topic shows the best local scan-matching pose before gating.

## Existing Issues and Future Improvements

Current issues:

- both localization nodes publish `/estimated_pose` and `map -> odom`, so they cannot run together
- the particle filter uses a simple likelihood field sensor model
- the particle filter estimate is a simple average and does not publish covariance
- the Kalman filter assumes the initial pose is already close to the real pose
- the Kalman filter can lose track if the scan match is outside the local search window
- global localization can still fail in symmetric map areas
- the particle cloud can collapse too much after repeated resampling

Potential improvements:

- publish covariance for the particle-filter estimate
- improve scan scoring with a stronger probabilistic sensor model
- add confidence output for both localization algorithms
- add recovery behavior when localization confidence is low
- add global relocalization or particle-filter reset behavior
- add tests for map indexing, likelihood lookup, resampling, angle averaging, covariance updates, and scan matching

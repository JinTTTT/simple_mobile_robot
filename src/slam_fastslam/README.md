# FastSLAM Package

This package implements particle-based SLAM for the Gazebo mobile robot. Each
particle carries a pose hypothesis, trajectory history, occupancy map, and
likelihood field. The ROS node adapts `/odom` and `/scan` messages into the
algorithm and publishes the selected particle's map, path, and pose.

Run with:

```bash
ros2 run slam_fastslam fastslam_node
```

Or launch it with the package config file:

```bash
ros2 launch slam_fastslam fastslam.launch.py
```

## Package Structure

- `fastslam_node.cpp`: ROS 2 adapter for parameters, subscriptions, publishers,
  TF lookup, and TF broadcast.
- `fastslam.hpp/cpp`: orchestration layer that sequences prediction, scoring,
  selected-particle tracking, resampling, and per-particle map updates.
- `motion_model.hpp/cpp`: odometry-based particle propagation with Gaussian
  motion noise.
- `scan_scorer.hpp/cpp`: scan beam filtering, endpoint likelihood scoring,
  free-space reward, and ray-crossing penalty.
- `likelihood_field.hpp/cpp`: distance-field likelihood map for scan endpoint
  scoring.
- `particle_resampler.hpp/cpp`: effective-particle-count thresholding and
  low-variance/systematic resampling.
- `fastslam_particle.hpp`: particle state shared by the algorithm modules.
- `utils.hpp/cpp`: small shared helpers such as `Pose2D`, angle normalization,
  and ROS pose/quaternion conversion.

## Current Logic

The node currently does this:

1. Start `20` particles at `(0, 0, 0)`.
2. Wait for `/odom` and `/scan`.
3. Accept a scan update only when odometry motion is large enough.
4. Move every particle using odometry plus small Gaussian noise.
5. Score each particle against its own likelihood field and map.
6. Choose the selected output particle. A new strongest particle must win
   repeatedly before replacing the current selected particle.
7. Store one pose for every particle in that particle's trajectory history.
8. Update each particle's own occupancy map using the accepted scan and that particle's pose.
9. Rebuild that particle's likelihood field for the next scoring step.
10. Publish the selected particle's `/map`, `/best_path`, and `/estimated_pose`, plus the full `/particlecloud`.
11. If the particle set becomes too concentrated, resample with low-variance resampling.

This keeps the implementation simple:

- particles represent trajectory hypotheses
- each particle keeps its own map hypothesis
- the published map always comes from the current selected particle

Important detail:

- motion noise is added only during the odometry prediction step
- scan scoring combines endpoint likelihood, free-space reward for max-range
  beams, and a penalty when a ray crosses occupied cells before the endpoint
- endpoint scores are accumulated in log space, then normalized back into particle weights
- resampling only copies/selects particle hypotheses and does not perturb copied trajectories

## Current Topics

Subscriptions:

- `/odom`
- `/scan`

Publications:

- `/map`
- `/particlecloud`
- `/best_path`
- `/estimated_pose`

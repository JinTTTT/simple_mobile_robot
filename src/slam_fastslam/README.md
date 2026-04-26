# FastSLAM Package

This package starts from first principles.

Current goal:

- keep a small particle set for pose hypotheses
- accept scans only after enough odometry motion
- give each particle its own accumulated map state
- score each particle against its own likelihood field map
- publish the best particle map and trajectory
- selectively resample particles with low-variance resampling so good hypotheses survive

Run with:

```bash
ros2 run slam_fastslam fastslam_node
```

## Current Logic

The node currently does this:

1. Start `20` particles at `(0, 0, 0)`.
2. Wait for `/odom` and `/scan`.
3. Accept a scan update only when odometry motion is large enough.
4. Move every particle using odometry plus small Gaussian noise.
5. Score each particle by checking how well scan endpoints fit that particle's own likelihood field.
6. Choose the best particle.
7. Store one pose for every particle in that particle's trajectory history.
8. Update each particle's own occupancy map using the accepted scan and that particle's pose.
9. Rebuild that particle's likelihood field for the next scoring step.
10. Publish the best particle's `/map`, `/best_path`, and `/estimated_pose`, plus the full `/particlecloud`.
11. If the particle set becomes too concentrated, resample with low-variance resampling.

This keeps the implementation simple:

- particles represent trajectory hypotheses
- each particle keeps its own map hypothesis
- the published map always comes from the current best particle

Important detail:

- motion noise is added only during the odometry prediction step
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

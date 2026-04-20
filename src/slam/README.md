# SLAM Package

SLAM means:

```text
Simultaneous Localization And Mapping
```

Simple meaning:

- the robot does not start with a prebuilt map
- the robot builds a map while it drives
- the robot also estimates where it is inside that growing map

This package is for learning and understanding. It starts with a small, readable SLAM version instead of a full professional SLAM system.

## Current Stage

The current node is:

```bash
ros2 run slam simple_slam_node
```

or:

```bash
ros2 launch slam simple_slam.launch.py
```

This currently includes Stage 1 through Stage 5:

- Stage 1: build a live occupancy grid map from lidar scans
- Stage 2: use local scan matching to slightly correct the odometry pose before updating the map
- Stage 3: store and publish the estimated trajectory
- Stage 4: detect simple loop closures
- Stage 5: apply a simple equally spread trajectory correction between loop-closure keyframes

The current algorithm is:

1. Read the robot movement from `/odom`.
2. Apply that movement to the previous SLAM pose.
3. If the robot has not moved enough, skip scan matching and map update.
4. If the map is still too empty, trust the odometry prediction.
5. If the map has enough walls, scan match around the odometry-predicted pose.
6. If scan matching has a good enough score, use the scan-matched pose.
7. Otherwise, use the odometry-predicted pose.
8. Insert the lidar scan into the map using the chosen pose.
9. Add the estimated pose to the trajectory.
10. Save occasional keyframes.
11. Compare the current scan with old nearby keyframes to detect loop closure.
12. If a loop closure is detected, spread the closing error from the old keyframe to the current keyframe.
13. Publish the map, estimated pose, scan-matched pose, raw trajectory, corrected trajectory, loop-closure pose, and `map -> odom`.

In short:

```text
odometry gives the first guess
scan matching can correct the guess
the chosen pose updates the map
```

## Inputs

The node subscribes to:

- `/odom`: odometry pose from the simulation
- `/scan`: 2D lidar scans

## Outputs

The node publishes:

- `/map`: the occupancy grid map being built live
- `/estimated_pose`: the final SLAM pose estimate in the `map` frame
- `/scan_matched_pose`: the pose found by scan matching when scan matching is accepted
- `/trajectory`: the history of estimated robot poses as a `nav_msgs/Path`
- `/corrected_trajectory`: keyframe path after simple loop-closure correction
- `/loop_closure_pose`: the old keyframe pose matched when a loop closure is detected
- TF: `map -> odom`

Gazebo already publishes:

```text
odom -> base_link
```

The SLAM node publishes:

```text
map -> odom
```

Together, RViz can use:

```text
map -> odom -> base_link
```

## Pose Outputs

`/odom` is the raw odometry estimate from the simulator.

`/scan_matched_pose` is the pose where the current laser scan best fits the current map.

`/estimated_pose` is the final pose chosen by this SLAM node:

```text
if scan matching is accepted:
    estimated_pose = scan_matched_pose
else:
    estimated_pose = odometry-predicted pose
```

This first version does not blend odometry and scan matching with a Kalman filter. It uses the simpler rule above because it is easier to inspect and understand.

## Reused Methods From Existing Packages

From `mapping`, this package reuses the ideas of:

- occupancy grid maps
- log-odds map updates
- Bresenham ray tracing
- marking cells along a beam as free
- marking the laser hit cell as occupied
- publishing `/map`

From `localization`, this package reuses the ideas of:

- local scan matching around a predicted pose
- likelihood field scoring
- publishing `/estimated_pose`
- publishing the `map -> odom` transform

The new part is the SLAM loop:

```text
predict pose from odometry
correct pose with scan matching
update the map with the corrected pose
```

This package also adds new learning methods:

- keyframes: saved poses plus a small summary of the laser scan
- trajectory history: all estimated poses published as a path
- loop-closure detection: checking whether the robot has returned near an older keyframe with a similar scan
- simple loop-closure correction: spreading the error evenly between the old matched keyframe and the current keyframe

The correction is intentionally simple. It corrects the published keyframe trajectory, not the live robot pose and not the map.

## Simple Loop-Closure Correction

When loop closure says:

```text
current keyframe should line up with old keyframe
```

the node computes the difference:

```text
error = old corrected pose - current corrected pose
```

Then it spreads that error across the keyframes from the old keyframe to the current keyframe.

Example:

```text
old keyframe = 4
current keyframe = 20
```

The correction is applied like this:

- keyframe 4 gets 0% correction
- keyframe 12 gets about 50% correction
- keyframe 20 gets 100% correction

This bends the corrected keyframe path toward the old place. It is not a full graph optimizer, but it shows the main idea of using loop closure to repair drift.

## Terminal Output

The node prints a small throttled summary while scans are integrated:

```text
SLAM integrated=... stationary_skipped=... keyframes=... loops=... pose=(x, y, theta) match=yes/no score=... occupied=...
```

Meaning:

- `integrated`: scans inserted into the map
- `stationary_skipped`: scans ignored because the robot did not move enough
- `keyframes`: saved keyframes for loop-closure checks
- `loops`: detected loop closures
- `pose`: current estimated pose
- `match`: whether scan matching corrected the pose
- `score`: scan matching score
- `occupied`: occupied cells in the current map

When the robot is not moving, scan matching and map updates are skipped after the first scan. This keeps the node from repeatedly inserting the same scan and making the map over-confident.

## Current Limits

This is still not full SLAM yet.

Current limits:

- no real pose graph solver
- no nonlinear graph optimization
- no global relocalization
- no map rebuilding after old pose corrections
- loop closure corrects only `/corrected_trajectory`, not `/map`
- tuning values are still hard-coded

It can correct small odometry errors, detect some returns to old places, and show a simple corrected trajectory. It cannot fix the map yet.

## Future Stages

Stage 6: pose graph optimization.

- treat old poses as nodes in a graph
- add movement and loop-closure constraints
- adjust old poses so the full path becomes more consistent

Stage 7: rebuild or correct the map.

- after old poses are corrected, rebuild the map using corrected poses
- this is how SLAM can fix a bent map after loop closure

## Run

Build from the workspace root:

```bash
cd ~/workspace/gazebo_ws
colcon build --packages-select slam
source install/setup.bash
```

Start simulation:

```bash
ros2 launch simulation bringup_simulation.launch.py
```

Start SLAM:

```bash
ros2 launch slam simple_slam.launch.py
```

Start teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Start RViz:

```bash
rviz2
```

In RViz:

- set Fixed Frame to `map`
- add `/map`
- add `/estimated_pose`
- add `/scan_matched_pose`
- add `/trajectory`
- add `/corrected_trajectory`
- add `/loop_closure_pose`
- add `TF`

Drive slowly at first. The map should grow around the robot.
To test loop closure, drive a simple loop and return near an older place.
When the detector finds a match, the terminal should print `Loop closure detected`, and `/loop_closure_pose` should point to the older matched keyframe.
Compare `/trajectory` and `/corrected_trajectory` in RViz. After loop closure, the corrected path should bend closer to the old matched place.

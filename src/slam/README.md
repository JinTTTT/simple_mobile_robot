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

## Status

This package is now good enough to treat as SLAM v1.

What SLAM v1 already does:

- build a live occupancy-grid map from `/scan` and `/odom`
- use local scan matching for small pose correction
- publish a normal ROS `map -> odom` transform
- store a trajectory and keyframes
- detect loop closure from keyframe scan similarity
- apply a simple loop-closure trajectory correction
- rebuild a corrected map from corrected keyframes

Known limits for SLAM v1:

- no real pose graph optimizer
- no full global relocalization
- no robust wall-hit / odometry-disagreement recovery yet
- `/corrected_map` is keyframe-based and may be sparser than `/map`

Those limits are acceptable for this first learning version and are better tracked as future issues than blockers for closing the package.

## Current Stage

The current node is:

```bash
ros2 run slam simple_slam_node
```

or:

```bash
ros2 launch slam simple_slam.launch.py
```

This currently includes Stage 1 through Stage 6:

- Stage 1: build a live occupancy grid map from lidar scans
- Stage 2: use local scan matching to slightly correct the odometry pose before updating the map
- Stage 3: store and publish the estimated trajectory
- Stage 4: detect simple loop closures
- Stage 5: apply a simple equally spread trajectory correction between loop-closure keyframes
- Stage 6: rebuild a corrected map from stored keyframe scans and corrected keyframe poses

The current algorithm is:

1. Read the robot movement from `/odom`.
2. Apply that movement to the previous SLAM pose.
3. If the robot has not moved enough, skip scan matching and map update.
4. If the map is still too empty, trust the odometry prediction.
5. If enough movement has happened since the last accepted scan match, scan match around the odometry-predicted pose.
6. Accept scan matching only if it clearly improves the score over the odometry-predicted pose and the correction is not too large.
7. Otherwise, use the odometry-predicted pose.
8. Insert the lidar scan into the map using the chosen pose.
9. Add the estimated pose to the trajectory.
10. Save occasional keyframes.
11. Compare the current scan with old nearby keyframes to detect loop closure.
12. If a loop closure is detected, spread the closing error from the old keyframe to the current keyframe.
13. If correction is applied, rebuild a corrected map from stored keyframe scans.
14. Publish the live map, corrected map, estimated pose, scan-matched pose, raw trajectory, corrected trajectory, loop-closure pose, and `map -> odom`.

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
- `/corrected_map`: a map rebuilt from keyframe scans using corrected keyframe poses
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

## Conservative Scan Matching

The scan matcher is intentionally conservative because wrong scan-matching corrections can draw duplicate or angled walls into the live map.

It now uses these ideas:

- do not start scan matching too early; wait for enough occupied cells in the map
- use a smaller heading search window
- do not try scan matching too often
- compare the best scan-match score against the score at the odometry-predicted pose
- accept scan matching only if the score improvement is meaningful
- reject scan matches that try to move the pose too far from odometry

Simple meaning:

```text
odometry is the default
scan matching is allowed only when it is clearly helpful
```

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
- stored keyframe scans: full lidar ranges saved so the map can be rebuilt later
- trajectory history: all estimated poses published as a path
- loop-closure detection: checking whether the robot has returned near an older keyframe with a similar scan
- simple loop-closure correction: gently spreading part of the error between selected loop-closure keyframes
- corrected map rebuilding: clearing a second map and reinserting stored keyframe scans at corrected poses

The correction is intentionally simple. It corrects the published keyframe trajectory and rebuilds `/corrected_map`. It does not change the live robot pose.

## Simple Loop-Closure Correction

When loop closure says:

```text
current keyframe should line up with old keyframe
```

the node computes the difference:

```text
error = old corrected pose - current corrected pose
```

The node does not correct every detected loop closure. After returning to a known area, many scans may match old places. Correcting every one of those matches can make the corrected path zigzag.

So the node applies correction only when:

- enough scans passed since the last correction
- the old and current keyframes are far enough apart
- the matched old keyframe is not too close to the previously corrected old keyframe

When correction is accepted, the node applies only part of the error:

```text
used_error = 0.35 * full_error
```

Then it spreads that smaller error across the keyframes from the old keyframe to the current keyframe.

Example:

```text
old keyframe = 4
current keyframe = 20
```

The correction is applied like this:

- keyframe 4 gets 0% correction
- keyframe 12 gets about 50% correction
- keyframe 20 gets 100% of the selected correction

This bends the corrected keyframe path toward the old place without yanking it too hard. It is not a full graph optimizer, but it shows the main idea of using loop closure to repair drift.

## Corrected Map

The live `/map` is updated online from every integrated scan using the current estimated pose. If the pose later gets corrected, old scan evidence already inserted into `/map` stays where it was.

The corrected map works differently:

1. Each keyframe stores the full lidar scan ranges.
2. When a loop-closure correction is applied, the node clears `/corrected_map`.
3. It reinserts every stored keyframe scan using that keyframe's `corrected_pose`.
4. It publishes the result on `/corrected_map`.

This means:

- `/map` is the dense live map
- `/corrected_map` is the rebuilt map from corrected keyframes

The corrected map may be sparser than `/map` because it uses keyframe scans only.

## Terminal Output

The node prints a small throttled summary while scans are integrated:

```text
SLAM integrated=... stationary_skipped=... scan_match_used=... keyframes=... loops=... corrections=... pose=(x, y, theta) match=yes/no score=... predicted_score=... occupied=...
```

Meaning:

- `integrated`: scans inserted into the map
- `stationary_skipped`: scans ignored because the robot did not move enough
- `scan_match_used`: scans where scan matching was accepted
- `keyframes`: saved keyframes for loop-closure checks
- `loops`: detected loop closures
- `corrections`: loop closures that actually changed `/corrected_trajectory`
- `pose`: current estimated pose
- `match`: whether scan matching corrected the pose
- `score`: scan matching score
- `predicted_score`: score of the odometry-predicted pose before correction
- `occupied`: occupied cells in the current map

When the robot is not moving, scan matching and map updates are skipped after the first scan. This keeps the node from repeatedly inserting the same scan and making the map over-confident.

## Current Limits

This is still not full SLAM yet.

Current limits:

- no real pose graph solver
- no nonlinear graph optimization
- no global relocalization
- loop closure correction is gated and partial, so some drift may remain
- `/corrected_map` is rebuilt from keyframes only, so it may be less dense than `/map`
- live `/estimated_pose` and live `/map` are not reset to the corrected trajectory
- scan matching is still local and can fail in repeated geometry or large drift
- tuning values are still hard-coded

It can correct small odometry errors, detect some returns to old places, show a simple corrected trajectory, and rebuild a keyframe-based corrected map.

## Future Stages

Stage 7: pose graph optimization.

- treat old poses as nodes in a graph
- add movement and loop-closure constraints
- adjust old poses so the full path becomes more consistent

Stage 8: make the optimized map the main SLAM state.

- feed optimized poses back into the live SLAM state
- decide when `/corrected_map` should replace `/map`

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
- add `/corrected_map`
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
Compare `/map` and `/corrected_map`. After an applied correction, `/corrected_map` should rebuild from the corrected keyframe path.

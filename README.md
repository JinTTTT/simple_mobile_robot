# My Bot - Differential Drive Robot

A ROS 2 differential drive robot with Gazebo simulation support.

## Setup

Build the package:
```bash
cd ~/workspace/gazebo_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the Robot

### 1. Start Gazebo with Robot

Launch the Gazebo simulator with the robot spawned at 0.5m height:

```bash
ros2 launch my_bot spawn_robot.launch.py
```

This starts:
- Gazebo simulator with an empty world
- Robot state publisher (publishes URDF as `/robot_description`)
- Robot spawned in the simulation

### 2. Bridge ROS 2 and Gazebo

In a new terminal, bridge the communication between ROS 2 and Gazebo:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
  /tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V \
  /scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan
```

This bridges:
- `/cmd_vel`: Send velocity commands to Gazebo
- `/odom`: Receive odometry (pose & velocity) from Gazebo
- `/tf`: Receive transform frames from Gazebo

### 3. Teleop with Keyboard

In a new terminal, start keyboard teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i`: Move forward
- `,` (comma): Move backward
- `j`: Spin left (rotate counterclockwise)
- `l`: Spin right (rotate clockwise)
- `k`: Stop (emergency brake)

### 4. Visualize in RViz

In a new terminal, launch RViz for visualization:

```bash
rviz2
```

**Configuration:**
- In RViz, set the **Fixed Frame** to `odom` (currently no map frame)
- Add displays:
  - **TF**: Visualize the robot's coordinate frames (`odom` → `base_link` → wheels)
  - **Odometry**: Visualize the robot's pose and velocity estimates
  - **RobotModel**: Visualize the URDF model (optional, may be redundant with TF/Odometry)

## Robot Specifications

- **Base Link:** 0.6m × 0.4m × 0.2m box (mass: 5 kg)
- **Wheels:** Two driven cylinders (radius: 0.1m, mass: 1 kg each)
- **Caster:** One passive ball at the front (radius: 0.05m, mass: 1 kg)
- **Wheel Separation:** 0.45m
- **Max Linear Velocity:** 0.5 m/s
- **Max Angular Velocity:** 1 rad/s

## Notes

- The robot uses **differential drive** kinematics; left and right wheel speeds are independently controlled
- Odometry is computed from wheel feedback and published on `/odom`
- The transform tree shows `odom` → `base_link`; no map frame is currently configured
- Replace the fixed frame in RViz with `map` once localization/mapping is added

#pragma once

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace slam_fastslam
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

// Keep yaw values in [-pi, pi] so interpolation and motion updates take
// the shortest angular path.
inline double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
Pose2D odometryMsgToPose2D(const nav_msgs::msg::Odometry & msg);

}  // namespace slam_fastslam

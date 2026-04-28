#pragma once

#include <cmath>

namespace slam_fastslam
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

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

}  // namespace slam_fastslam

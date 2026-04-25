#ifndef PATH_FOLLOW_CONTROL__PATH_TYPES_HPP_
#define PATH_FOLLOW_CONTROL__PATH_TYPES_HPP_

#include <vector>

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct ControlCommand
{
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
};

using PosePath = std::vector<Pose2D>;

#endif  // PATH_FOLLOW_CONTROL__PATH_TYPES_HPP_

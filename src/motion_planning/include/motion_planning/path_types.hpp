#ifndef MOTION_PLANNING__PATH_TYPES_HPP_
#define MOTION_PLANNING__PATH_TYPES_HPP_

#include <vector>

struct GridCell
{
  int x = 0;
  int y = 0;
};

struct PathPoint
{
  double x = 0.0;
  double y = 0.0;
};

using GridPath = std::vector<GridCell>;
using PointPath = std::vector<PathPoint>;

#endif  // MOTION_PLANNING__PATH_TYPES_HPP_

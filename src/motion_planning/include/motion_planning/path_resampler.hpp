#ifndef MOTION_PLANNING__PATH_RESAMPLER_HPP_
#define MOTION_PLANNING__PATH_RESAMPLER_HPP_

#include "motion_planning/path_types.hpp"

class PathResampler
{
public:
  PointPath resampleAtFixedSpacing(const PointPath & input_path, double sample_spacing) const;

private:
  double computePathLength(const PointPath & path) const;
};

#endif  // MOTION_PLANNING__PATH_RESAMPLER_HPP_

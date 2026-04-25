#ifndef MOTION_PLANNING__SPLINE_PATH_SMOOTHER_HPP_
#define MOTION_PLANNING__SPLINE_PATH_SMOOTHER_HPP_

#include "motion_planning/path_types.hpp"

#include <functional>
#include <vector>

struct SplineSmoothingResult
{
  PointPath path;
  bool used_collision_fallback = false;
};

class SplinePathSmoother
{
public:
  SplineSmoothingResult smooth(
    const PointPath & base_path,
    double sample_spacing,
    double map_resolution,
    const std::function<bool(const PathPoint &)> & is_point_in_free_space) const;

private:
  std::vector<double> buildArcLengthParameter(const PointPath & points) const;
  std::vector<double> extractXPathValues(const PointPath & points) const;
  std::vector<double> extractYPathValues(const PointPath & points) const;
  std::vector<double> solveNaturalCubicSecondDerivatives(
    const std::vector<double> & parameter_values,
    const std::vector<double> & sample_values) const;
  double evaluateNaturalCubicSpline(
    const std::vector<double> & parameter_values,
    const std::vector<double> & sample_values,
    const std::vector<double> & second_derivatives,
    double query_value) const;
};

#endif  // MOTION_PLANNING__SPLINE_PATH_SMOOTHER_HPP_

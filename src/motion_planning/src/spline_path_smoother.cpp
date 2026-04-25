#include "motion_planning/spline_path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

SplineSmoothingResult SplinePathSmoother::smooth(
  const PointPath & base_path,
  double sample_spacing,
  double map_resolution,
  const std::function<bool(const PathPoint &)> & is_point_in_free_space) const
{
  SplineSmoothingResult result;
  result.path = base_path;

  if (base_path.size() < 3) {
    return result;
  }

  const std::vector<double> path_parameter = buildArcLengthParameter(base_path);
  if (path_parameter.size() < 3 || path_parameter.back() <= 1e-6) {
    return result;
  }

  const std::vector<double> x_values = extractXPathValues(base_path);
  const std::vector<double> y_values = extractYPathValues(base_path);
  const std::vector<double> x_second_derivatives =
    solveNaturalCubicSecondDerivatives(path_parameter, x_values);
  const std::vector<double> y_second_derivatives =
    solveNaturalCubicSecondDerivatives(path_parameter, y_values);

  const double total_length = path_parameter.back();
  const double effective_sample_spacing = std::max(sample_spacing, map_resolution * 0.5);

  PointPath sampled_points;
  sampled_points.reserve(
    std::max<std::size_t>(
      base_path.size(),
      static_cast<std::size_t>(std::ceil(total_length / sample_spacing)) + 1U));
  sampled_points.push_back(base_path.front());

  for (double path_distance = effective_sample_spacing; path_distance < total_length;
    path_distance += effective_sample_spacing)
  {
    sampled_points.push_back(
    {
      evaluateNaturalCubicSpline(path_parameter, x_values, x_second_derivatives, path_distance),
      evaluateNaturalCubicSpline(path_parameter, y_values, y_second_derivatives, path_distance)
    });
  }

  sampled_points.push_back(base_path.back());

  for (const auto & point : sampled_points) {
    if (!is_point_in_free_space(point)) {
      result.used_collision_fallback = true;
      return result;
    }
  }

  result.path = sampled_points;
  return result;
}

std::vector<double> SplinePathSmoother::buildArcLengthParameter(const PointPath & points) const
{
  std::vector<double> parameter_values(points.size(), 0.0);
  for (std::size_t point_index = 1; point_index < points.size(); ++point_index) {
    const double delta_x = points[point_index].x - points[point_index - 1].x;
    const double delta_y = points[point_index].y - points[point_index - 1].y;
    parameter_values[point_index] =
      parameter_values[point_index - 1] + std::sqrt(delta_x * delta_x + delta_y * delta_y);
  }

  return parameter_values;
}

std::vector<double> SplinePathSmoother::extractXPathValues(const PointPath & points) const
{
  std::vector<double> x_values;
  x_values.reserve(points.size());
  for (const auto & point : points) {
    x_values.push_back(point.x);
  }

  return x_values;
}

std::vector<double> SplinePathSmoother::extractYPathValues(const PointPath & points) const
{
  std::vector<double> y_values;
  y_values.reserve(points.size());
  for (const auto & point : points) {
    y_values.push_back(point.y);
  }

  return y_values;
}

std::vector<double> SplinePathSmoother::solveNaturalCubicSecondDerivatives(
  const std::vector<double> & parameter_values,
  const std::vector<double> & sample_values) const
{
  const std::size_t point_count = sample_values.size();
  std::vector<double> second_derivatives(point_count, 0.0);
  if (point_count < 3) {
    return second_derivatives;
  }

  const std::size_t interior_count = point_count - 2;
  std::vector<double> lower_diagonal(interior_count, 0.0);
  std::vector<double> main_diagonal(interior_count, 0.0);
  std::vector<double> upper_diagonal(interior_count, 0.0);
  std::vector<double> right_hand_side(interior_count, 0.0);

  for (std::size_t interior_index = 0; interior_index < interior_count; ++interior_index) {
    const std::size_t knot_index = interior_index + 1;
    const double previous_interval =
      parameter_values[knot_index] - parameter_values[knot_index - 1];
    const double next_interval =
      parameter_values[knot_index + 1] - parameter_values[knot_index];
    if (previous_interval <= 1e-9 || next_interval <= 1e-9) {
      return second_derivatives;
    }

    lower_diagonal[interior_index] = previous_interval;
    main_diagonal[interior_index] = 2.0 * (previous_interval + next_interval);
    upper_diagonal[interior_index] = next_interval;
    right_hand_side[interior_index] = 6.0 * (
      (sample_values[knot_index + 1] - sample_values[knot_index]) / next_interval -
      (sample_values[knot_index] - sample_values[knot_index - 1]) / previous_interval);
  }

  for (std::size_t interior_index = 1; interior_index < interior_count; ++interior_index) {
    const double elimination_factor = lower_diagonal[interior_index] /
      main_diagonal[interior_index - 1];
    main_diagonal[interior_index] -= elimination_factor * upper_diagonal[interior_index - 1];
    right_hand_side[interior_index] -= elimination_factor * right_hand_side[interior_index - 1];
  }

  second_derivatives[point_count - 2] = right_hand_side.back() / main_diagonal.back();
  for (std::size_t interior_index = interior_count - 1; interior_index > 0; --interior_index) {
    second_derivatives[interior_index] =
      (right_hand_side[interior_index - 1] -
      upper_diagonal[interior_index - 1] * second_derivatives[interior_index + 1]) /
      main_diagonal[interior_index - 1];
  }

  return second_derivatives;
}

double SplinePathSmoother::evaluateNaturalCubicSpline(
  const std::vector<double> & parameter_values,
  const std::vector<double> & sample_values,
  const std::vector<double> & second_derivatives,
  double query_value) const
{
  const auto upper_bound = std::upper_bound(
    parameter_values.begin(), parameter_values.end(), query_value);
  const std::size_t segment_index = static_cast<std::size_t>(
    std::max<std::ptrdiff_t>(0, std::distance(parameter_values.begin(), upper_bound) - 1));
  const std::size_t clamped_segment_index =
    std::min(segment_index, parameter_values.size() - 2);

  const double segment_start = parameter_values[clamped_segment_index];
  const double segment_end = parameter_values[clamped_segment_index + 1];
  const double interval = segment_end - segment_start;
  if (interval <= 1e-9) {
    return sample_values[clamped_segment_index];
  }

  const double left_distance = segment_end - query_value;
  const double right_distance = query_value - segment_start;
  return
    second_derivatives[clamped_segment_index] *
    left_distance * left_distance * left_distance / (6.0 * interval) +
    second_derivatives[clamped_segment_index + 1] *
    right_distance * right_distance * right_distance / (6.0 * interval) +
    (sample_values[clamped_segment_index] -
    second_derivatives[clamped_segment_index] * interval * interval / 6.0) *
    (left_distance / interval) +
    (sample_values[clamped_segment_index + 1] -
    second_derivatives[clamped_segment_index + 1] * interval * interval / 6.0) *
    (right_distance / interval);
}

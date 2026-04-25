#include "motion_planning/path_resampler.hpp"

#include <algorithm>
#include <cmath>

PointPath PathResampler::resampleAtFixedSpacing(
  const PointPath & input_path,
  double sample_spacing) const
{
  if (input_path.size() < 2) {
    return input_path;
  }

  const double effective_spacing = std::max(sample_spacing, 1e-3);
  PointPath resampled_points;
  resampled_points.reserve(
    std::max<std::size_t>(
      input_path.size(),
      static_cast<std::size_t>(
        std::ceil(computePathLength(input_path) / effective_spacing)) + 1U));

  resampled_points.push_back(input_path.front());

  double distance_until_next_sample = effective_spacing;

  for (std::size_t segment_index = 0; segment_index + 1 < input_path.size(); ++segment_index) {
    const PathPoint & segment_start = input_path[segment_index];
    const PathPoint & segment_end = input_path[segment_index + 1];
    const double delta_x = segment_end.x - segment_start.x;
    const double delta_y = segment_end.y - segment_start.y;
    const double segment_length = std::sqrt(delta_x * delta_x + delta_y * delta_y);

    if (segment_length <= 1e-9) {
      continue;
    }

    double distance_along_segment = distance_until_next_sample;
    while (distance_along_segment < segment_length) {
      const double interpolation_ratio = distance_along_segment / segment_length;
      resampled_points.push_back(
      {
        segment_start.x + interpolation_ratio * delta_x,
        segment_start.y + interpolation_ratio * delta_y
      });
      distance_along_segment += effective_spacing;
    }

    distance_until_next_sample = distance_along_segment - segment_length;
    if (distance_until_next_sample <= 1e-9) {
      distance_until_next_sample = effective_spacing;
    }
  }

  if (std::hypot(
      resampled_points.back().x - input_path.back().x,
      resampled_points.back().y - input_path.back().y) > 1e-6)
  {
    resampled_points.push_back(input_path.back());
  }

  return resampled_points;
}

double PathResampler::computePathLength(const PointPath & path) const
{
  if (path.size() < 2) {
    return 0.0;
  }

  double total_length = 0.0;
  for (std::size_t point_index = 0; point_index + 1 < path.size(); ++point_index) {
    const double delta_x = path[point_index + 1].x - path[point_index].x;
    const double delta_y = path[point_index + 1].y - path[point_index].y;
    total_length += std::sqrt(delta_x * delta_x + delta_y * delta_y);
  }

  return total_length;
}

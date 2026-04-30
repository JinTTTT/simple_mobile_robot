#include "slam_fastslam/likelihood_field.hpp"

#include <cmath>
#include <cstdint>
#include <utility>

namespace slam_fastslam
{

void LikelihoodField::buildFromOccupancyMap(
  const OccupancyMapper & mapper,
  double max_distance_m,
  double sigma_m)
{
  const auto & cfg = mapper.getConfig();
  const int width = cfg.width;
  const int height = cfg.height;
  const int total_cells = width * height;
  if (total_cells <= 0 || max_distance_m <= 0.0) {
    likelihood_grid_.data.clear();
    distance_to_nearest_obstacle_cells_.clear();
    return;
  }

  const double resolution = cfg.resolution;
  const int max_distance_to_obstacle_cells =
    static_cast<int>(std::ceil(max_distance_m / resolution));

  likelihood_grid_.info.resolution = cfg.resolution;
  likelihood_grid_.info.width = static_cast<std::uint32_t>(width);
  likelihood_grid_.info.height = static_cast<std::uint32_t>(height);
  likelihood_grid_.info.origin.position.x = cfg.origin_x;
  likelihood_grid_.info.origin.position.y = cfg.origin_y;
  likelihood_grid_.info.origin.orientation.w = 1.0;

  std::vector<int> distance_to_nearest_obstacle(
    static_cast<std::size_t>(total_cells), max_distance_to_obstacle_cells);
  std::vector<int> queue(static_cast<std::size_t>(total_cells), 0);
  int queue_head = 0;
  int queue_tail = 0;

  const auto & log_odds = mapper.getLogOdds();
  for (int i = 0; i < total_cells; ++i) {
    if (log_odds[static_cast<std::size_t>(i)] > 0.0) {
      distance_to_nearest_obstacle[static_cast<std::size_t>(i)] = 0;
      queue[static_cast<std::size_t>(queue_tail++)] = i;
    }
  }

  const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  while (queue_head < queue_tail) {
    const int index = queue[static_cast<std::size_t>(queue_head++)];
    const int row = index / width;
    const int col = index % width;
    const int next_distance =
      distance_to_nearest_obstacle[static_cast<std::size_t>(index)] + 1;

    if (next_distance > max_distance_to_obstacle_cells) {
      continue;
    }

    for (const auto & offset : offsets) {
      const int next_col = col + offset[0];
      const int next_row = row + offset[1];

      if (next_col < 0 || next_row < 0 || next_col >= width || next_row >= height) {
        continue;
      }

      const int next_index = next_row * width + next_col;
      if (next_distance >=
        distance_to_nearest_obstacle[static_cast<std::size_t>(next_index)])
      {
        continue;
      }

      distance_to_nearest_obstacle[static_cast<std::size_t>(next_index)] = next_distance;
      queue[static_cast<std::size_t>(queue_tail++)] = next_index;
    }
  }

  const double two_sigma_squared = 2.0 * sigma_m * sigma_m;
  likelihood_grid_.data.assign(static_cast<std::size_t>(total_cells), 0);
  for (int i = 0; i < total_cells; ++i) {
    const double distance_m =
      distance_to_nearest_obstacle[static_cast<std::size_t>(i)] * resolution;
    const double likelihood = std::exp(-(distance_m * distance_m) / two_sigma_squared);
    likelihood_grid_.data[static_cast<std::size_t>(i)] =
      static_cast<int8_t>(std::round(likelihood * 100.0));
  }

  width_ = width;
  height_ = height;
  max_distance_to_obstacle_cells_ = max_distance_to_obstacle_cells;
  resolution_ = resolution;
  two_sigma_squared_ = two_sigma_squared;
  distance_to_nearest_obstacle_cells_ = std::move(distance_to_nearest_obstacle);
}

void LikelihoodField::updateFromMapChanges(
  const OccupancyMapper & mapper,
  double max_distance_m,
  double sigma_m,
  const std::vector<int> & cells_changed_to_occupied,
  const std::vector<int> & cells_changed_to_free,
  std::size_t freed_cells_rebuild_threshold)
{
  const auto & cfg = mapper.getConfig();
  const int new_width = cfg.width;
  const int new_height = cfg.height;
  const int new_max_dist = static_cast<int>(std::ceil(max_distance_m / cfg.resolution));

  const bool dimensions_changed =
    (new_width != width_ || new_height != height_ ||
    new_max_dist != max_distance_to_obstacle_cells_);
  const bool freed_cells_significant =
    cells_changed_to_free.size() > freed_cells_rebuild_threshold;

  if (distance_to_nearest_obstacle_cells_.empty() || dimensions_changed ||
    freed_cells_significant)
  {
    buildFromOccupancyMap(mapper, max_distance_m, sigma_m);
    return;
  }

  if (cells_changed_to_occupied.empty()) {
    return;
  }

  std::vector<int> queue;
  queue.reserve(cells_changed_to_occupied.size() * 64);

  for (const int idx : cells_changed_to_occupied) {
    if (idx < 0 || idx >= static_cast<int>(distance_to_nearest_obstacle_cells_.size())) {
      continue;
    }
    if (distance_to_nearest_obstacle_cells_[static_cast<std::size_t>(idx)] > 0) {
      distance_to_nearest_obstacle_cells_[static_cast<std::size_t>(idx)] = 0;
      likelihood_grid_.data[static_cast<std::size_t>(idx)] = 100;
      queue.push_back(idx);
    }
  }

  const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  for (std::size_t head = 0; head < queue.size(); ++head) {
    const int current = queue[head];
    const int current_dist =
      distance_to_nearest_obstacle_cells_[static_cast<std::size_t>(current)];
    const int next_dist = current_dist + 1;

    if (next_dist > max_distance_to_obstacle_cells_) {
      continue;
    }

    const int row = current / width_;
    const int col = current % width_;

    for (const auto & offset : offsets) {
      const int next_col = col + offset[0];
      const int next_row = row + offset[1];

      if (next_col < 0 || next_row < 0 || next_col >= width_ || next_row >= height_) {
        continue;
      }

      const int next_idx = next_row * width_ + next_col;
      if (next_dist >=
        distance_to_nearest_obstacle_cells_[static_cast<std::size_t>(next_idx)])
      {
        continue;
      }

      distance_to_nearest_obstacle_cells_[static_cast<std::size_t>(next_idx)] = next_dist;
      const double distance_m = next_dist * resolution_;
      const double likelihood = std::exp(-(distance_m * distance_m) / two_sigma_squared_);
      likelihood_grid_.data[static_cast<std::size_t>(next_idx)] =
        static_cast<int8_t>(std::round(likelihood * 100.0));
      queue.push_back(next_idx);
    }
  }
}

bool LikelihoodField::hasMap() const
{
  return !likelihood_grid_.data.empty();
}

double LikelihoodField::likelihoodAtWorld(double x, double y) const
{
  if (!hasMap()) {
    return 0.0;
  }

  const double resolution = likelihood_grid_.info.resolution;
  const double origin_x = likelihood_grid_.info.origin.position.x;
  const double origin_y = likelihood_grid_.info.origin.position.y;
  const int col = static_cast<int>(std::floor((x - origin_x) / resolution));
  const int row = static_cast<int>(std::floor((y - origin_y) / resolution));

  if (col < 0 || row < 0 ||
    col >= static_cast<int>(likelihood_grid_.info.width) ||
    row >= static_cast<int>(likelihood_grid_.info.height))
  {
    return 0.0;
  }

  const int index = row * static_cast<int>(likelihood_grid_.info.width) + col;
  return likelihood_grid_.data[static_cast<std::size_t>(index)] / 100.0;
}

}  // namespace slam_fastslam

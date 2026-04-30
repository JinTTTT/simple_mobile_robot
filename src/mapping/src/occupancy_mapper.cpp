#include "mapping/occupancy_mapper.hpp"

#include <cmath>
#include <string>
#include <utility>

void OccupancyMapper::configure(const Config & config)
{
  config_ = config;
  clear();

  // Use a standard inverse sensor model:
  // - hit_probability is the occupancy probability added at the laser endpoint
  // - free_probability is the occupancy probability added along free space
  //   and should therefore stay below 0.5, producing a negative log-odds increment
  log_odds_hit_ = std::log(config_.hit_probability / (1.0 - config_.hit_probability));
  log_odds_pass_ = std::log(config_.free_probability / (1.0 - config_.free_probability));
}

void OccupancyMapper::clear()
{
  map_log_odds_.assign(config_.width * config_.height, 0.0);
  newly_occupied_.clear();
  newly_freed_.clear();
}

void OccupancyMapper::updateCell(int index, double delta)
{
  const bool was_occupied = map_log_odds_[index] > 0.0;
  map_log_odds_[index] =
    clamp(map_log_odds_[index] + delta, config_.log_odds_min, config_.log_odds_max);
  const bool is_occupied = map_log_odds_[index] > 0.0;
  if (!was_occupied && is_occupied) {
    newly_occupied_.push_back(index);
  } else if (was_occupied && !is_occupied) {
    newly_freed_.push_back(index);
  }
}

OccupancyMapper::MapChanges OccupancyMapper::takeAndClearMapChanges()
{
  MapChanges changes;
  changes.newly_occupied = std::move(newly_occupied_);
  changes.newly_freed = std::move(newly_freed_);
  newly_occupied_.clear();
  newly_freed_.clear();
  return changes;
}

const OccupancyMapper::Config & OccupancyMapper::getConfig() const
{
  return config_;
}

const std::vector<double> & OccupancyMapper::getLogOdds() const
{
  return map_log_odds_;
}

bool OccupancyMapper::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  gx = static_cast<int>(std::floor((wx - config_.origin_x) / config_.resolution));
  gy = static_cast<int>(std::floor((wy - config_.origin_y) / config_.resolution));
  return isValidCell(gx, gy);
}

bool OccupancyMapper::isValidCell(int x, int y) const
{
  return x >= 0 && x < config_.width && y >= 0 && y < config_.height;
}

void OccupancyMapper::updateWithScan(
  const sensor_msgs::msg::LaserScan & scan,
  double robot_x,
  double robot_y,
  double robot_theta)
{
  ScanData scan_data;
  scan_data.ranges = scan.ranges;
  scan_data.angle_min = scan.angle_min;
  scan_data.angle_increment = scan.angle_increment;
  scan_data.range_min = scan.range_min;
  scan_data.range_max = scan.range_max;
  updateWithScanData(scan_data, robot_x, robot_y, robot_theta);
}

void OccupancyMapper::updateWithScanData(
  const ScanData & scan,
  double robot_x,
  double robot_y,
  double robot_theta)
{
  int robot_grid_x = 0;
  int robot_grid_y = 0;
  if (!worldToGrid(robot_x, robot_y, robot_grid_x, robot_grid_y)) {
    return;
  }

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (std::isnan(range) || range < scan.range_min || scan.range_max <= scan.range_min) {
      continue;
    }

    bool endpoint_is_hit = true;
    double usable_range = range;
    if (std::isinf(range)) {
      if (range < 0.0F) {
        continue;
      }
      usable_range = scan.range_max;
      endpoint_is_hit = false;
    } else if (range > scan.range_max) {
      usable_range = scan.range_max;
      endpoint_is_hit = false;
    }

    const double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double world_angle = robot_theta + beam_angle;

    const double end_x = robot_x + usable_range * std::cos(world_angle);
    const double end_y = robot_y + usable_range * std::sin(world_angle);

    int end_grid_x = 0;
    int end_grid_y = 0;
    if (!worldToGrid(end_x, end_y, end_grid_x, end_grid_y)) {
      continue;
    }

    const auto cells = bresenhamLine(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y);
    if (cells.empty()) {
      continue;
    }

    const std::size_t free_cell_count = endpoint_is_hit ? cells.size() - 1U : cells.size();
    for (std::size_t j = 0; j < free_cell_count; ++j) {
      updateCell(gridToIndex(cells[j].first, cells[j].second), log_odds_pass_);
    }

    if (endpoint_is_hit) {
      updateCell(gridToIndex(cells.back().first, cells.back().second), log_odds_hit_);
    }
  }
}

nav_msgs::msg::OccupancyGrid OccupancyMapper::buildOccupancyGridMsg(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp) const
{
  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.stamp = stamp;
  map_msg.header.frame_id = frame_id;
  map_msg.info.resolution = config_.resolution;
  map_msg.info.width = static_cast<std::uint32_t>(config_.width);
  map_msg.info.height = static_cast<std::uint32_t>(config_.height);
  map_msg.info.origin.position.x = config_.origin_x;
  map_msg.info.origin.position.y = config_.origin_y;
  map_msg.info.origin.orientation.w = 1.0;
  map_msg.data.resize(map_log_odds_.size(), -1);

  for (std::size_t i = 0; i < map_log_odds_.size(); ++i) {
    if (config_.publish_unknown_for_unobserved && map_log_odds_[i] == 0.0) {
      map_msg.data[i] = -1;
      continue;
    }

    const double probability = 1.0 / (1.0 + std::exp(-map_log_odds_[i]));
    map_msg.data[i] = static_cast<std::int8_t>(probability * 100.0);
  }

  return map_msg;
}

std::vector<std::pair<int, int>> OccupancyMapper::bresenhamLine(int x0, int y0, int x1, int y1) const
{
  std::vector<std::pair<int, int>> cells;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;

  while (true) {
    cells.emplace_back(x, y);

    if (x == x1 && y == y1) {
      break;
    }

    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }

  return cells;
}

int OccupancyMapper::gridToIndex(int x, int y) const
{
  return y * config_.width + x;
}

double OccupancyMapper::clamp(double value, double low, double high) const
{
  return std::max(low, std::min(value, high));
}

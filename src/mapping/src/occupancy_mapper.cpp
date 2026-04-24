#include "mapping/occupancy_mapper.hpp"

#include <cmath>
#include <string>

void OccupancyMapper::configure(const Config & config)
{
  config_ = config;
  map_log_odds_.assign(config_.width * config_.height, 0.0);

  const double p_hit_occ = config_.hit_probability;
  const double p_pass_occ = 1.0 - p_hit_occ;
  const double p_hit_free = config_.free_probability;
  const double p_pass_free = 1.0 - p_hit_free;

  log_odds_hit_ = std::log(p_hit_occ / p_hit_free);
  log_odds_pass_ = std::log(p_pass_occ / p_pass_free);
}

bool OccupancyMapper::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  gx = static_cast<int>((wx - config_.origin_x) / config_.resolution);
  gy = static_cast<int>((wy - config_.origin_y) / config_.resolution);
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
  int robot_grid_x = 0;
  int robot_grid_y = 0;
  if (!worldToGrid(robot_x, robot_y, robot_grid_x, robot_grid_y)) {
    return;
  }

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (range < scan.range_min || range > scan.range_max || std::isnan(range) ||
      std::isinf(range))
    {
      continue;
    }

    const double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double world_angle = robot_theta + beam_angle;

    const double end_x = robot_x + range * std::cos(world_angle);
    const double end_y = robot_y + range * std::sin(world_angle);

    int end_grid_x = 0;
    int end_grid_y = 0;
    if (!worldToGrid(end_x, end_y, end_grid_x, end_grid_y)) {
      continue;
    }

    const auto cells = bresenhamLine(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y);
    if (cells.empty()) {
      continue;
    }

    for (std::size_t j = 0; j + 1 < cells.size(); ++j) {
      const int index = gridToIndex(cells[j].first, cells[j].second);
      map_log_odds_[index] = clamp(
        map_log_odds_[index] + log_odds_pass_,
        config_.log_odds_min,
        config_.log_odds_max);
    }

    const int hit_index = gridToIndex(cells.back().first, cells.back().second);
    map_log_odds_[hit_index] = clamp(
      map_log_odds_[hit_index] + log_odds_hit_,
      config_.log_odds_min,
      config_.log_odds_max);
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
    const double probability = 1.0 / (1.0 + std::exp(-map_log_odds_[i]));
    map_msg.data[i] = static_cast<std::int8_t>(probability * 100.0);
  }

  return map_msg;
}

std::vector<std::pair<int, int>> OccupancyMapper::bresenhamLine(int x0, int y0, int x1, int y1)
  const
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

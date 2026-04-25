#include "slam/slam_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace slam
{

void SlamMapper::configure(const SimpleSlamConfig & config)
{
  resolution_ = config.resolution;
  width_ = config.width;
  height_ = config.height;
  origin_x_ = config.origin_x;
  origin_y_ = config.origin_y;

  log_odds_hit_ = config.log_odds_hit;
  log_odds_free_ = config.log_odds_free;
  log_odds_min_ = config.log_odds_min;
  log_odds_max_ = config.log_odds_max;

  initializeMap();
}

void SlamMapper::updateWithScan(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & pose,
  const builtin_interfaces::msg::Time & stamp)
{
  insertScanIntoLogOddsMap(makeStoredScan(scan), pose, map_log_odds_);
  updateMapMessage(stamp);
}

void SlamMapper::rebuildCorrectedMap(
  const std::vector<KeyFrame> & keyframes,
  const builtin_interfaces::msg::Time & stamp)
{
  corrected_map_log_odds_.assign(width_ * height_, 0.0);

  for (const auto & keyframe : keyframes) {
    insertScanIntoLogOddsMap(
      keyframe.scan,
      keyframe.corrected_pose,
      corrected_map_log_odds_);
  }

  updateCorrectedMapMessage(stamp);
}

const nav_msgs::msg::OccupancyGrid & SlamMapper::map() const
{
  return map_msg_;
}

const nav_msgs::msg::OccupancyGrid & SlamMapper::correctedMap() const
{
  return corrected_map_msg_;
}

int SlamMapper::occupiedCellCount() const
{
  return occupied_cell_count_;
}

void SlamMapper::initializeMap()
{
  map_log_odds_.assign(width_ * height_, 0.0);
  corrected_map_log_odds_.assign(width_ * height_, 0.0);

  map_msg_.header.frame_id = "map";
  map_msg_.info.resolution = resolution_;
  map_msg_.info.width = width_;
  map_msg_.info.height = height_;
  map_msg_.info.origin.position.x = origin_x_;
  map_msg_.info.origin.position.y = origin_y_;
  map_msg_.info.origin.orientation.w = 1.0;
  map_msg_.data.assign(width_ * height_, -1);
  corrected_map_msg_ = map_msg_;
}

void SlamMapper::insertScanIntoLogOddsMap(
  const StoredScan & scan,
  const Pose2D & pose,
  std::vector<double> & log_odds_map)
{
  int robot_grid_x = 0;
  int robot_grid_y = 0;
  worldToGrid(pose.x, pose.y, robot_grid_x, robot_grid_y);

  if (!isValidCell(robot_grid_x, robot_grid_y)) {
    return;
  }

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    float range = scan.ranges[i];

    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }

    double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    double hit_x = pose.x + range * std::cos(pose.theta + beam_angle);
    double hit_y = pose.y + range * std::sin(pose.theta + beam_angle);

    int hit_grid_x = 0;
    int hit_grid_y = 0;
    worldToGrid(hit_x, hit_y, hit_grid_x, hit_grid_y);

    if (!isValidCell(hit_grid_x, hit_grid_y)) {
      continue;
    }

    auto cells = bresenhamLine(robot_grid_x, robot_grid_y, hit_grid_x, hit_grid_y);
    if (cells.empty()) {
      continue;
    }

    for (std::size_t cell_index = 0; cell_index + 1 < cells.size(); ++cell_index) {
      int x = cells[cell_index].first;
      int y = cells[cell_index].second;
      int index = y * width_ + x;
      log_odds_map[index] =
        clamp(log_odds_map[index] + log_odds_free_, log_odds_min_, log_odds_max_);
    }

    int hit_index = cells.back().second * width_ + cells.back().first;
    log_odds_map[hit_index] =
      clamp(log_odds_map[hit_index] + log_odds_hit_, log_odds_min_, log_odds_max_);
  }
}

void SlamMapper::updateMapMessage(const builtin_interfaces::msg::Time & stamp)
{
  map_msg_.header.stamp = stamp;
  occupied_cell_count_ = 0;

  for (int i = 0; i < width_ * height_; ++i) {
    double probability = 1.0 / (1.0 + std::exp(-map_log_odds_[i]));

    if (map_log_odds_[i] == 0.0) {
      map_msg_.data[i] = -1;
    } else {
      map_msg_.data[i] = static_cast<int8_t>(std::round(probability * 100.0));
    }

    if (map_msg_.data[i] > 50) {
      occupied_cell_count_++;
    }
  }
}

void SlamMapper::updateCorrectedMapMessage(const builtin_interfaces::msg::Time & stamp)
{
  corrected_map_msg_.header.stamp = stamp;

  for (int i = 0; i < width_ * height_; ++i) {
    double probability = 1.0 / (1.0 + std::exp(-corrected_map_log_odds_[i]));

    if (corrected_map_log_odds_[i] == 0.0) {
      corrected_map_msg_.data[i] = -1;
    } else {
      corrected_map_msg_.data[i] = static_cast<int8_t>(std::round(probability * 100.0));
    }
  }
}

StoredScan SlamMapper::makeStoredScan(const sensor_msgs::msg::LaserScan & scan) const
{
  StoredScan stored_scan;
  stored_scan.ranges = scan.ranges;
  stored_scan.angle_min = scan.angle_min;
  stored_scan.angle_increment = scan.angle_increment;
  stored_scan.range_min = scan.range_min;
  stored_scan.range_max = scan.range_max;
  return stored_scan;
}

void SlamMapper::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
}

bool SlamMapper::isValidCell(int x, int y) const
{
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

std::vector<std::pair<int, int>> SlamMapper::bresenhamLine(
  int x0, int y0, int x1, int y1) const
{
  std::vector<std::pair<int, int>> cells;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;

  while (true) {
    cells.push_back({x, y});

    if (x == x1 && y == y1) {
      break;
    }

    int e2 = 2 * err;
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

double SlamMapper::clamp(double value, double low, double high) const
{
  return std::min(std::max(value, low), high);
}

}  // namespace slam

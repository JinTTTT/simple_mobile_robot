#include "slam/slam_localization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace slam
{

void SlamLocalization::configure(const SimpleSlamConfig & config)
{
  min_occupied_cells_for_matching_ = config.min_occupied_cells_for_matching;
  likelihood_max_distance_ = config.likelihood_max_distance;

  scan_match_xy_range_ = config.scan_match_xy_range;
  scan_match_xy_step_ = config.scan_match_xy_step;
  scan_match_theta_range_ = config.scan_match_theta_range;
  scan_match_theta_step_ = config.scan_match_theta_step;
  scan_match_beam_step_ = config.scan_match_beam_step;
  min_scan_match_score_ = config.min_scan_match_score;
  min_scan_match_score_improvement_ = config.min_scan_match_score_improvement;
  max_scan_match_translation_correction_ = config.max_scan_match_translation_correction;
  max_scan_match_rotation_correction_ = config.max_scan_match_rotation_correction;
  min_scan_match_translation_interval_ = config.min_scan_match_translation_interval;
  min_scan_match_rotation_interval_ = config.min_scan_match_rotation_interval;
  min_scan_match_scan_gap_ = config.min_scan_match_scan_gap;

  likelihood_field_dirty_ = true;
  scan_match_pose_initialized_ = false;
  scan_match_used_count_ = 0;
  last_scan_match_scan_index_ = -100000;
}

void SlamLocalization::markMapDirtyForScanMatching()
{
  likelihood_field_dirty_ = true;
}

ScanMatchResult SlamLocalization::matchScan(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & predicted_pose,
  const nav_msgs::msg::OccupancyGrid & map,
  bool scan_received,
  int scans_integrated,
  int occupied_cell_count)
{
  ScanMatchResult result;
  result.pose = predicted_pose;

  if (!scan_received) {
    return result;
  }

  if (scans_integrated - last_scan_match_scan_index_ < min_scan_match_scan_gap_) {
    return result;
  }

  if (occupied_cell_count < min_occupied_cells_for_matching_) {
    return result;
  }

  if (scan_match_pose_initialized_) {
    double distance_since_last_match = poseDistance(predicted_pose, last_scan_match_pose_);
    double rotation_since_last_match =
      std::abs(normalizeAngle(predicted_pose.theta - last_scan_match_pose_.theta));
    if (distance_since_last_match < min_scan_match_translation_interval_ &&
      rotation_since_last_match < min_scan_match_rotation_interval_)
    {
      return result;
    }
  }

  if (likelihood_field_dirty_) {
    rebuildLikelihoodField(map);
    likelihood_field_dirty_ = false;
  }

  result.predicted_score = scoreScanAtPose(scan, predicted_pose);

  double best_score = -std::numeric_limits<double>::infinity();
  Pose2D best_pose = predicted_pose;

  for (double dx = -scan_match_xy_range_; dx <= scan_match_xy_range_ + 1e-9;
    dx += scan_match_xy_step_)
  {
    for (double dy = -scan_match_xy_range_; dy <= scan_match_xy_range_ + 1e-9;
      dy += scan_match_xy_step_)
    {
      for (double dtheta = -scan_match_theta_range_;
        dtheta <= scan_match_theta_range_ + 1e-9;
        dtheta += scan_match_theta_step_)
      {
        Pose2D candidate;
        candidate.x = predicted_pose.x + dx;
        candidate.y = predicted_pose.y + dy;
        candidate.theta = normalizeAngle(predicted_pose.theta + dtheta);

        double score = scoreScanAtPose(scan, candidate);
        if (score > best_score) {
          best_score = score;
          best_pose = candidate;
        }
      }
    }
  }

  result.score = std::max(best_score, 0.0);

  double correction_translation = poseDistance(best_pose, predicted_pose);
  double correction_rotation =
    std::abs(normalizeAngle(best_pose.theta - predicted_pose.theta));
  double score_improvement = result.score - result.predicted_score;

  if (best_score >= min_scan_match_score_ &&
    score_improvement >= min_scan_match_score_improvement_ &&
    correction_translation <= max_scan_match_translation_correction_ &&
    correction_rotation <= max_scan_match_rotation_correction_)
  {
    result.pose = best_pose;
    result.used_scan_matching = true;
    scan_match_used_count_++;
    last_scan_match_scan_index_ = scans_integrated;
    last_scan_match_pose_ = result.pose;
    scan_match_pose_initialized_ = true;
  }

  return result;
}

int SlamLocalization::scanMatchUsedCount() const
{
  return scan_match_used_count_;
}

void SlamLocalization::rebuildLikelihoodField(const nav_msgs::msg::OccupancyGrid & map)
{
  likelihood_field_msg_ = map;
  likelihood_field_msg_.data.assign(map.info.width * map.info.height, 0);

  int width = static_cast<int>(map.info.width);
  int height = static_cast<int>(map.info.height);
  double resolution = map.info.resolution;
  int max_distance_cells = static_cast<int>(std::ceil(likelihood_max_distance_ / resolution));
  std::vector<int> distance_to_wall(width * height, max_distance_cells + 1);
  std::queue<int> cells_to_visit;

  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      int index = row * width + col;
      if (map.data[index] > 50) {
        distance_to_wall[index] = 0;
        cells_to_visit.push(index);
      }
    }
  }

  const int neighbor_offsets[4][2] = {
    {1, 0},
    {-1, 0},
    {0, 1},
    {0, -1}
  };

  while (!cells_to_visit.empty()) {
    int index = cells_to_visit.front();
    cells_to_visit.pop();

    int row = index / width;
    int col = index % width;
    int next_distance = distance_to_wall[index] + 1;

    if (next_distance > max_distance_cells) {
      continue;
    }

    for (const auto & offset : neighbor_offsets) {
      int next_col = col + offset[0];
      int next_row = row + offset[1];

      if (next_col < 0 || next_col >= width || next_row < 0 || next_row >= height) {
        continue;
      }

      int next_index = next_row * width + next_col;
      if (next_distance >= distance_to_wall[next_index]) {
        continue;
      }

      distance_to_wall[next_index] = next_distance;
      cells_to_visit.push(next_index);
    }
  }

  for (int i = 0; i < width * height; ++i) {
    if (distance_to_wall[i] > max_distance_cells) {
      continue;
    }

    double distance_m = distance_to_wall[i] * resolution;
    double likelihood = 1.0 - std::min(distance_m / likelihood_max_distance_, 1.0);
    likelihood_field_msg_.data[i] = static_cast<int8_t>(std::round(likelihood * 100.0));
  }
}

double SlamLocalization::scoreScanAtPose(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & pose) const
{
  double score = 0.0;
  int used_beams = 0;

  for (std::size_t i = 0; i < scan.ranges.size(); i += scan_match_beam_step_) {
    float range = scan.ranges[i];

    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }

    double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    double hit_x = pose.x + range * std::cos(pose.theta + beam_angle);
    double hit_y = pose.y + range * std::sin(pose.theta + beam_angle);

    score += likelihoodAtWorld(hit_x, hit_y);
    used_beams++;
  }

  if (used_beams == 0) {
    return 0.0;
  }

  return score / used_beams;
}

double SlamLocalization::likelihoodAtWorld(double x, double y) const
{
  int col = 0;
  int row = 0;
  if (!worldToGrid(x, y, col, row)) {
    return 0.0;
  }

  int width = static_cast<int>(likelihood_field_msg_.info.width);
  int index = row * width + col;
  return likelihood_field_msg_.data[index] / 100.0;
}

bool SlamLocalization::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  double resolution = likelihood_field_msg_.info.resolution;
  double origin_x = likelihood_field_msg_.info.origin.position.x;
  double origin_y = likelihood_field_msg_.info.origin.position.y;

  gx = static_cast<int>(std::floor((wx - origin_x) / resolution));
  gy = static_cast<int>(std::floor((wy - origin_y) / resolution));
  return isValidCell(gx, gy);
}

bool SlamLocalization::isValidCell(int x, int y) const
{
  int width = static_cast<int>(likelihood_field_msg_.info.width);
  int height = static_cast<int>(likelihood_field_msg_.info.height);
  return x >= 0 && x < width && y >= 0 && y < height;
}

double SlamLocalization::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double SlamLocalization::poseDistance(const Pose2D & first, const Pose2D & second) const
{
  double dx = first.x - second.x;
  double dy = first.y - second.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace slam

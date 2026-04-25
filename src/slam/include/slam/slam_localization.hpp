#ifndef SLAM__SLAM_LOCALIZATION_HPP_
#define SLAM__SLAM_LOCALIZATION_HPP_

#include "slam/simple_slam_types.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace slam
{

class SlamLocalization
{
public:
  void configure(const SimpleSlamConfig & config);
  void markMapDirty();

  ScanMatchResult matchScan(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & predicted_pose,
    const nav_msgs::msg::OccupancyGrid & map,
    bool scan_received,
    int scans_integrated,
    int occupied_cell_count);

  int scanMatchUsedCount() const;

private:
  void rebuildLikelihoodField(const nav_msgs::msg::OccupancyGrid & map);
  double scoreScanAtPose(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & pose) const;
  double likelihoodAtWorld(double x, double y) const;
  bool worldToGrid(double wx, double wy, int & gx, int & gy) const;
  bool isValidCell(int x, int y) const;
  double normalizeAngle(double angle) const;
  double poseDistance(const Pose2D & first, const Pose2D & second) const;

  int min_occupied_cells_for_matching_ = 250;
  bool likelihood_field_dirty_ = true;
  double likelihood_max_distance_ = 1.0;

  double scan_match_xy_range_ = 0.10;
  double scan_match_xy_step_ = 0.05;
  double scan_match_theta_range_ = 0.08;
  double scan_match_theta_step_ = 0.04;
  std::size_t scan_match_beam_step_ = 10;
  double min_scan_match_score_ = 0.20;
  double min_scan_match_score_improvement_ = 0.003;
  double max_scan_match_translation_correction_ = 0.08;
  double max_scan_match_rotation_correction_ = 0.08;
  double min_scan_match_translation_interval_ = 0.02;
  double min_scan_match_rotation_interval_ = 0.02;
  int min_scan_match_scan_gap_ = 2;

  Pose2D last_scan_match_pose_;
  bool scan_match_pose_initialized_ = false;
  int scan_match_used_count_ = 0;
  int last_scan_match_scan_index_ = -100000;

  nav_msgs::msg::OccupancyGrid likelihood_field_msg_;
};

}  // namespace slam

#endif  // SLAM__SLAM_LOCALIZATION_HPP_

#ifndef SLAM__SIMPLE_SLAM_HPP_
#define SLAM__SIMPLE_SLAM_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace slam
{

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct ScanMatchResult
{
  Pose2D pose;
  double score = 0.0;
  double predicted_score = 0.0;
  bool used_scan_matching = false;
};

struct StoredScan
{
  std::vector<float> ranges;
  float angle_min = 0.0F;
  float angle_increment = 0.0F;
  float range_min = 0.0F;
  float range_max = 0.0F;
};

struct KeyFrame
{
  Pose2D pose;
  Pose2D corrected_pose;
  std::vector<float> scan_signature;
  StoredScan scan;
  int scan_index = 0;
};

struct SimpleSlamConfig
{
  double resolution = 0.05;
  int width = 500;
  int height = 500;
  double origin_x = -12.5;
  double origin_y = -12.5;

  double log_odds_hit = 2.89;
  double log_odds_free = -2.25;
  double log_odds_min = -10.0;
  double log_odds_max = 10.0;

  int min_occupied_cells_for_matching = 250;
  double likelihood_max_distance = 1.0;

  double scan_match_xy_range = 0.10;
  double scan_match_xy_step = 0.05;
  double scan_match_theta_range = 0.08;
  double scan_match_theta_step = 0.04;
  std::size_t scan_match_beam_step = 10;
  double min_scan_match_score = 0.20;
  double min_scan_match_score_improvement = 0.003;
  double max_scan_match_translation_correction = 0.08;
  double max_scan_match_rotation_correction = 0.08;
  double min_scan_match_translation_interval = 0.02;
  double min_scan_match_rotation_interval = 0.02;
  int min_scan_match_scan_gap = 2;
  double min_update_translation = 0.01;
  double min_update_rotation = 0.01;

  double keyframe_min_translation = 0.25;
  double keyframe_min_rotation = 0.25;
  std::size_t scan_signature_beam_step = 20;
  int min_loop_closure_keyframe_age = 20;
  double loop_closure_search_radius = 0.45;
  double loop_closure_max_heading_diff = 0.70;
  double loop_closure_max_signature_diff = 0.18;
  int min_loop_closure_scan_gap = 20;
  int min_correction_scan_gap = 80;
  std::size_t min_correction_keyframe_gap = 12;
  std::size_t min_old_keyframe_separation_for_correction = 8;
  double loop_closure_correction_strength = 0.35;
};

struct LoopClosureResult
{
  bool detected = false;
  bool correction_applied = false;
  Pose2D matched_pose;
  std::size_t current_keyframe_index = 0;
  std::size_t matched_keyframe_index = 0;
  int matched_scan_index = 0;
  double signature_difference = 0.0;
};

struct SlamUpdateResult
{
  bool scan_integrated = false;
  bool stationary_scan_skipped = false;
  bool map_updated = false;
  bool corrected_map_updated = false;
  bool trajectory_updated = false;
  bool corrected_trajectory_updated = false;
  bool keyframe_added = false;
  ScanMatchResult scan_match;
  LoopClosureResult loop_closure;
};

class SimpleSlam
{
public:
  SimpleSlam();

  void configure(const SimpleSlamConfig & config);

  bool handleOdometry(const nav_msgs::msg::Odometry & msg);
  SlamUpdateResult handleScan(
    const sensor_msgs::msg::LaserScan & scan,
    const builtin_interfaces::msg::Time & stamp);

  bool hasOdometry() const;
  const Pose2D & slamPose() const;
  const Pose2D & currentOdomPose() const;

  const nav_msgs::msg::OccupancyGrid & map() const;
  const nav_msgs::msg::OccupancyGrid & correctedMap() const;
  const nav_msgs::msg::Path & trajectory() const;
  const nav_msgs::msg::Path & correctedTrajectory() const;

  int scansIntegrated() const;
  int stationaryScansSkipped() const;
  int scanMatchUsedCount() const;
  int loopClosureCount() const;
  int loopClosureCorrectionCount() const;
  int occupiedCellCount() const;
  std::size_t keyframeCount() const;

private:
  void initializeMap();
  void rebuildLikelihoodField();
  ScanMatchResult matchScan(const sensor_msgs::msg::LaserScan & scan, const Pose2D & predicted);
  double scoreScanAtPose(const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose) const;
  double likelihoodAtWorld(double x, double y) const;

  void updateMapWithScan(const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose);
  void insertScanIntoLogOddsMap(
    const StoredScan & scan,
    const Pose2D & pose,
    std::vector<double> & log_odds_map);
  void rebuildCorrectedMap(const builtin_interfaces::msg::Time & stamp);
  void updateMapMessage(const builtin_interfaces::msg::Time & stamp);
  void updateCorrectedMapMessage(const builtin_interfaces::msg::Time & stamp);
  void updateTrajectory(const builtin_interfaces::msg::Time & stamp);
  void updateCorrectedTrajectory(const builtin_interfaces::msg::Time & stamp);

  bool shouldAddKeyFrame() const;
  void addKeyFrame(const sensor_msgs::msg::LaserScan & scan);
  LoopClosureResult detectLoopClosure();
  bool shouldApplyLoopClosureCorrection(
    std::size_t matched_keyframe_index,
    std::size_t current_keyframe_index) const;
  void applyLoopClosureCorrection(
    std::size_t matched_keyframe_index,
    std::size_t current_keyframe_index);
  StoredScan makeStoredScan(const sensor_msgs::msg::LaserScan & scan) const;
  std::vector<float> makeScanSignature(const sensor_msgs::msg::LaserScan & scan) const;
  double scanSignatureDifference(
    const std::vector<float> & first,
    const std::vector<float> & second) const;
  double poseDistance(const Pose2D & first, const Pose2D & second) const;

  Pose2D odomMsgToPose(const nav_msgs::msg::Odometry & msg) const;
  Pose2D applyOdomDeltaToSlamPose(const Pose2D & old_odom, const Pose2D & new_odom) const;
  bool odomMovedEnough(const Pose2D & old_odom, const Pose2D & new_odom) const;
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const;

  void worldToGrid(double wx, double wy, int & gx, int & gy) const;
  bool isValidCell(int x, int y) const;
  std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) const;
  double normalizeAngle(double angle) const;
  double clamp(double value, double low, double high) const;

  nav_msgs::msg::OccupancyGrid map_msg_;
  nav_msgs::msg::OccupancyGrid corrected_map_msg_;
  nav_msgs::msg::OccupancyGrid likelihood_field_msg_;
  nav_msgs::msg::Path trajectory_msg_;
  nav_msgs::msg::Path corrected_trajectory_msg_;
  std::vector<double> map_log_odds_;
  std::vector<double> corrected_map_log_odds_;
  std::vector<KeyFrame> keyframes_;

  Pose2D slam_pose_;
  Pose2D last_odom_pose_;
  Pose2D current_odom_pose_;
  Pose2D last_keyframe_pose_;
  Pose2D last_scan_match_pose_;
  bool odom_initialized_ = false;
  bool scan_received_ = false;
  bool keyframe_initialized_ = false;
  bool scan_match_pose_initialized_ = false;

  double resolution_ = 0.05;
  int width_ = 500;
  int height_ = 500;
  double origin_x_ = -12.5;
  double origin_y_ = -12.5;

  double log_odds_hit_ = 2.89;
  double log_odds_free_ = -2.25;
  double log_odds_min_ = -10.0;
  double log_odds_max_ = 10.0;

  int occupied_cell_count_ = 0;
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
  double min_update_translation_ = 0.01;
  double min_update_rotation_ = 0.01;

  double keyframe_min_translation_ = 0.25;
  double keyframe_min_rotation_ = 0.25;
  std::size_t scan_signature_beam_step_ = 20;
  int min_loop_closure_keyframe_age_ = 20;
  double loop_closure_search_radius_ = 0.45;
  double loop_closure_max_heading_diff_ = 0.70;
  double loop_closure_max_signature_diff_ = 0.18;
  int min_loop_closure_scan_gap_ = 20;
  int min_correction_scan_gap_ = 80;
  std::size_t min_correction_keyframe_gap_ = 12;
  std::size_t min_old_keyframe_separation_for_correction_ = 8;
  double loop_closure_correction_strength_ = 0.35;

  SimpleSlamConfig config_;

  int scans_integrated_ = 0;
  int stationary_scans_skipped_ = 0;
  int scan_match_used_count_ = 0;
  int loop_closure_count_ = 0;
  int loop_closure_correction_count_ = 0;
  int last_scan_match_scan_index_ = -100000;
  int last_loop_closure_scan_index_ = -100000;
  int last_correction_scan_index_ = -100000;
  std::size_t last_corrected_matched_keyframe_index_ = std::numeric_limits<std::size_t>::max();
};

}  // namespace slam

#endif  // SLAM__SIMPLE_SLAM_HPP_

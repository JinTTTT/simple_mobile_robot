#ifndef SLAM__SIMPLE_SLAM_TYPES_HPP_
#define SLAM__SIMPLE_SLAM_TYPES_HPP_

#include <cstddef>
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

}  // namespace slam

#endif  // SLAM__SIMPLE_SLAM_TYPES_HPP_

#include "slam/simple_slam.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace slam
{

SimpleSlam::SimpleSlam()
{
  configure(SimpleSlamConfig());
}

void SimpleSlam::configure(const SimpleSlamConfig & config)
{
  config_ = config;

  resolution_ = config_.resolution;
  width_ = config_.width;
  height_ = config_.height;
  origin_x_ = config_.origin_x;
  origin_y_ = config_.origin_y;

  log_odds_hit_ = config_.log_odds_hit;
  log_odds_free_ = config_.log_odds_free;
  log_odds_min_ = config_.log_odds_min;
  log_odds_max_ = config_.log_odds_max;

  min_occupied_cells_for_matching_ = config_.min_occupied_cells_for_matching;
  likelihood_max_distance_ = config_.likelihood_max_distance;

  scan_match_xy_range_ = config_.scan_match_xy_range;
  scan_match_xy_step_ = config_.scan_match_xy_step;
  scan_match_theta_range_ = config_.scan_match_theta_range;
  scan_match_theta_step_ = config_.scan_match_theta_step;
  scan_match_beam_step_ = config_.scan_match_beam_step;
  min_scan_match_score_ = config_.min_scan_match_score;
  min_scan_match_score_improvement_ = config_.min_scan_match_score_improvement;
  max_scan_match_translation_correction_ = config_.max_scan_match_translation_correction;
  max_scan_match_rotation_correction_ = config_.max_scan_match_rotation_correction;
  min_scan_match_translation_interval_ = config_.min_scan_match_translation_interval;
  min_scan_match_rotation_interval_ = config_.min_scan_match_rotation_interval;
  min_scan_match_scan_gap_ = config_.min_scan_match_scan_gap;
  min_update_translation_ = config_.min_update_translation;
  min_update_rotation_ = config_.min_update_rotation;

  keyframe_min_translation_ = config_.keyframe_min_translation;
  keyframe_min_rotation_ = config_.keyframe_min_rotation;
  scan_signature_beam_step_ = config_.scan_signature_beam_step;
  min_loop_closure_keyframe_age_ = config_.min_loop_closure_keyframe_age;
  loop_closure_search_radius_ = config_.loop_closure_search_radius;
  loop_closure_max_heading_diff_ = config_.loop_closure_max_heading_diff;
  loop_closure_max_signature_diff_ = config_.loop_closure_max_signature_diff;
  min_loop_closure_scan_gap_ = config_.min_loop_closure_scan_gap;
  min_correction_scan_gap_ = config_.min_correction_scan_gap;
  min_correction_keyframe_gap_ = config_.min_correction_keyframe_gap;
  min_old_keyframe_separation_for_correction_ =
    config_.min_old_keyframe_separation_for_correction;
  loop_closure_correction_strength_ = config_.loop_closure_correction_strength;

  initializeMap();
}

bool SimpleSlam::handleOdometry(const nav_msgs::msg::Odometry & msg)
{
  current_odom_pose_ = odomMsgToPose(msg);

  if (!odom_initialized_) {
    last_odom_pose_ = current_odom_pose_;
    odom_initialized_ = true;
    return true;
  }

  return false;
}

SlamUpdateResult SimpleSlam::handleScan(
  const sensor_msgs::msg::LaserScan & scan,
  const builtin_interfaces::msg::Time & stamp)
{
  SlamUpdateResult update;

  if (!odom_initialized_) {
    return update;
  }

  bool moved_enough = odomMovedEnough(last_odom_pose_, current_odom_pose_);
  Pose2D predicted_pose = applyOdomDeltaToSlamPose(last_odom_pose_, current_odom_pose_);
  last_odom_pose_ = current_odom_pose_;

  if (scan_received_ && !moved_enough) {
    stationary_scans_skipped_++;
    update.stationary_scan_skipped = true;
    return update;
  }

  update.scan_match = matchScan(scan, predicted_pose);
  slam_pose_ = update.scan_match.pose;

  updateMapWithScan(scan, slam_pose_);
  updateMapMessage(stamp);

  scan_received_ = true;
  scans_integrated_++;
  likelihood_field_dirty_ = true;
  updateTrajectory(stamp);

  update.scan_integrated = true;
  update.map_updated = true;
  update.trajectory_updated = true;

  if (shouldAddKeyFrame()) {
    addKeyFrame(scan);
    update.keyframe_added = true;
    update.loop_closure = detectLoopClosure();
    if (update.loop_closure.correction_applied) {
      updateCorrectedTrajectory(stamp);
      rebuildCorrectedMap(stamp);
      update.corrected_map_updated = true;
    }
  }

  updateCorrectedTrajectory(stamp);
  update.corrected_trajectory_updated = true;

  return update;
}

void SimpleSlam::initializeMap()
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

  likelihood_field_msg_ = map_msg_;
  likelihood_field_msg_.data.assign(width_ * height_, 0);

  trajectory_msg_.header.frame_id = "map";
  corrected_trajectory_msg_.header.frame_id = "map";
}

Pose2D SimpleSlam::applyOdomDeltaToSlamPose(
  const Pose2D & old_odom, const Pose2D & new_odom) const
{
  double odom_dx = new_odom.x - old_odom.x;
  double odom_dy = new_odom.y - old_odom.y;
  double old_odom_theta = old_odom.theta;

  double local_dx =
    std::cos(old_odom_theta) * odom_dx + std::sin(old_odom_theta) * odom_dy;
  double local_dy =
    -std::sin(old_odom_theta) * odom_dx + std::cos(old_odom_theta) * odom_dy;
  double local_dtheta = normalizeAngle(new_odom.theta - old_odom.theta);

  Pose2D predicted = slam_pose_;
  predicted.x += std::cos(slam_pose_.theta) * local_dx -
    std::sin(slam_pose_.theta) * local_dy;
  predicted.y += std::sin(slam_pose_.theta) * local_dx +
    std::cos(slam_pose_.theta) * local_dy;
  predicted.theta = normalizeAngle(slam_pose_.theta + local_dtheta);
  return predicted;
}

ScanMatchResult SimpleSlam::matchScan(
  const sensor_msgs::msg::LaserScan & scan, const Pose2D & predicted)
{
  ScanMatchResult result;
  result.pose = predicted;

  if (!scan_received_) {
    return result;
  }

  if (scans_integrated_ - last_scan_match_scan_index_ < min_scan_match_scan_gap_) {
    return result;
  }

  if (occupied_cell_count_ < min_occupied_cells_for_matching_) {
    return result;
  }

  if (scan_match_pose_initialized_) {
    double distance_since_last_match = poseDistance(predicted, last_scan_match_pose_);
    double rotation_since_last_match =
      std::abs(normalizeAngle(predicted.theta - last_scan_match_pose_.theta));
    if (distance_since_last_match < min_scan_match_translation_interval_ &&
      rotation_since_last_match < min_scan_match_rotation_interval_)
    {
      return result;
    }
  }

  if (likelihood_field_dirty_) {
    rebuildLikelihoodField();
    likelihood_field_dirty_ = false;
  }

  result.predicted_score = scoreScanAtPose(scan, predicted);

  double best_score = -std::numeric_limits<double>::infinity();
  Pose2D best_pose = predicted;

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
        candidate.x = predicted.x + dx;
        candidate.y = predicted.y + dy;
        candidate.theta = normalizeAngle(predicted.theta + dtheta);

        double score = scoreScanAtPose(scan, candidate);
        if (score > best_score) {
          best_score = score;
          best_pose = candidate;
        }
      }
    }
  }

  result.score = std::max(best_score, 0.0);

  double correction_translation = poseDistance(best_pose, predicted);
  double correction_rotation =
    std::abs(normalizeAngle(best_pose.theta - predicted.theta));
  double score_improvement = result.score - result.predicted_score;

  if (best_score >= min_scan_match_score_ &&
    score_improvement >= min_scan_match_score_improvement_ &&
    correction_translation <= max_scan_match_translation_correction_ &&
    correction_rotation <= max_scan_match_rotation_correction_)
  {
    result.pose = best_pose;
    result.used_scan_matching = true;
    scan_match_used_count_++;
    last_scan_match_scan_index_ = scans_integrated_;
    last_scan_match_pose_ = result.pose;
    scan_match_pose_initialized_ = true;
  }

  return result;
}

void SimpleSlam::rebuildLikelihoodField()
{
  likelihood_field_msg_ = map_msg_;
  likelihood_field_msg_.data.assign(width_ * height_, 0);

  int max_distance_cells = static_cast<int>(std::ceil(likelihood_max_distance_ / resolution_));
  std::vector<int> distance_to_wall(width_ * height_, max_distance_cells + 1);
  std::queue<int> cells_to_visit;

  for (int row = 0; row < height_; ++row) {
    for (int col = 0; col < width_; ++col) {
      int index = row * width_ + col;
      if (map_msg_.data[index] > 50) {
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

    int row = index / width_;
    int col = index % width_;
    int next_distance = distance_to_wall[index] + 1;

    if (next_distance > max_distance_cells) {
      continue;
    }

    for (const auto & offset : neighbor_offsets) {
      int next_col = col + offset[0];
      int next_row = row + offset[1];

      if (!isValidCell(next_col, next_row)) {
        continue;
      }

      int next_index = next_row * width_ + next_col;
      if (next_distance >= distance_to_wall[next_index]) {
        continue;
      }

      distance_to_wall[next_index] = next_distance;
      cells_to_visit.push(next_index);
    }
  }

  for (int i = 0; i < width_ * height_; ++i) {
    if (distance_to_wall[i] > max_distance_cells) {
      continue;
    }

    double distance_m = distance_to_wall[i] * resolution_;
    double likelihood = 1.0 - std::min(distance_m / likelihood_max_distance_, 1.0);
    likelihood_field_msg_.data[i] = static_cast<int8_t>(std::round(likelihood * 100.0));
  }
}

double SimpleSlam::scoreScanAtPose(
  const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose) const
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

double SimpleSlam::likelihoodAtWorld(double x, double y) const
{
  int col = 0;
  int row = 0;
  worldToGrid(x, y, col, row);

  if (!isValidCell(col, row)) {
    return 0.0;
  }

  int index = row * width_ + col;
  return likelihood_field_msg_.data[index] / 100.0;
}

void SimpleSlam::updateMapWithScan(
  const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose)
{
  insertScanIntoLogOddsMap(makeStoredScan(scan), pose, map_log_odds_);
}

void SimpleSlam::insertScanIntoLogOddsMap(
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

void SimpleSlam::rebuildCorrectedMap(const builtin_interfaces::msg::Time & stamp)
{
  corrected_map_log_odds_.assign(width_ * height_, 0.0);

  for (const auto & keyframe : keyframes_) {
    insertScanIntoLogOddsMap(
      keyframe.scan,
      keyframe.corrected_pose,
      corrected_map_log_odds_);
  }

  updateCorrectedMapMessage(stamp);
}

void SimpleSlam::updateMapMessage(const builtin_interfaces::msg::Time & stamp)
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

void SimpleSlam::updateCorrectedMapMessage(const builtin_interfaces::msg::Time & stamp)
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

void SimpleSlam::updateTrajectory(const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = "map";
  pose.pose.position.x = slam_pose_.x;
  pose.pose.position.y = slam_pose_.y;
  pose.pose.orientation = yawToQuaternion(slam_pose_.theta);

  trajectory_msg_.header.stamp = stamp;
  trajectory_msg_.poses.push_back(pose);
}

void SimpleSlam::updateCorrectedTrajectory(const builtin_interfaces::msg::Time & stamp)
{
  corrected_trajectory_msg_.header.stamp = stamp;
  corrected_trajectory_msg_.poses.clear();
  corrected_trajectory_msg_.poses.reserve(keyframes_.size());

  for (const auto & keyframe : keyframes_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = keyframe.corrected_pose.x;
    pose.pose.position.y = keyframe.corrected_pose.y;
    pose.pose.orientation = yawToQuaternion(keyframe.corrected_pose.theta);
    corrected_trajectory_msg_.poses.push_back(pose);
  }
}

bool SimpleSlam::shouldAddKeyFrame() const
{
  if (!keyframe_initialized_) {
    return true;
  }

  double distance = poseDistance(slam_pose_, last_keyframe_pose_);
  double rotation = std::abs(normalizeAngle(slam_pose_.theta - last_keyframe_pose_.theta));

  return distance >= keyframe_min_translation_ || rotation >= keyframe_min_rotation_;
}

void SimpleSlam::addKeyFrame(const sensor_msgs::msg::LaserScan & scan)
{
  KeyFrame keyframe;
  keyframe.pose = slam_pose_;
  keyframe.corrected_pose = slam_pose_;
  keyframe.scan_signature = makeScanSignature(scan);
  keyframe.scan = makeStoredScan(scan);
  keyframe.scan_index = scans_integrated_;

  keyframes_.push_back(keyframe);
  last_keyframe_pose_ = slam_pose_;
  keyframe_initialized_ = true;
}

LoopClosureResult SimpleSlam::detectLoopClosure()
{
  if (keyframes_.size() <= static_cast<std::size_t>(min_loop_closure_keyframe_age_)) {
    return {};
  }

  if (scans_integrated_ - last_loop_closure_scan_index_ < min_loop_closure_scan_gap_) {
    return {};
  }

  std::size_t current_keyframe_index = keyframes_.size() - 1;
  const KeyFrame & current_keyframe = keyframes_[current_keyframe_index];
  double best_signature_difference = std::numeric_limits<double>::max();
  std::size_t best_candidate_index = 0;
  bool found_candidate = false;

  for (std::size_t candidate_index = 0; candidate_index < current_keyframe_index;
    ++candidate_index)
  {
    const KeyFrame & keyframe = keyframes_[candidate_index];
    int keyframe_age = current_keyframe.scan_index - keyframe.scan_index;
    if (keyframe_age < min_loop_closure_keyframe_age_) {
      continue;
    }

    double distance = poseDistance(current_keyframe.pose, keyframe.pose);
    if (distance > loop_closure_search_radius_) {
      continue;
    }

    double heading_difference =
      std::abs(normalizeAngle(current_keyframe.pose.theta - keyframe.pose.theta));
    if (heading_difference > loop_closure_max_heading_diff_) {
      continue;
    }

    double signature_difference =
      scanSignatureDifference(current_keyframe.scan_signature, keyframe.scan_signature);
    if (signature_difference < best_signature_difference) {
      best_signature_difference = signature_difference;
      best_candidate_index = candidate_index;
      found_candidate = true;
    }
  }

  if (!found_candidate || best_signature_difference > loop_closure_max_signature_diff_) {
    return {};
  }

  loop_closure_count_++;
  last_loop_closure_scan_index_ = scans_integrated_;
  const KeyFrame & matched_keyframe = keyframes_[best_candidate_index];

  LoopClosureResult result;
  result.detected = true;
  result.matched_pose = matched_keyframe.pose;
  result.current_keyframe_index = current_keyframe_index;
  result.matched_keyframe_index = best_candidate_index;
  result.matched_scan_index = matched_keyframe.scan_index;
  result.signature_difference = best_signature_difference;

  if (shouldApplyLoopClosureCorrection(best_candidate_index, current_keyframe_index)) {
    applyLoopClosureCorrection(best_candidate_index, current_keyframe_index);
    result.correction_applied = true;
  }

  return result;
}

bool SimpleSlam::shouldApplyLoopClosureCorrection(
  std::size_t matched_keyframe_index, std::size_t current_keyframe_index) const
{
  if (current_keyframe_index <= matched_keyframe_index) {
    return false;
  }

  if (current_keyframe_index - matched_keyframe_index < min_correction_keyframe_gap_) {
    return false;
  }

  if (scans_integrated_ - last_correction_scan_index_ < min_correction_scan_gap_) {
    return false;
  }

  if (last_corrected_matched_keyframe_index_ != std::numeric_limits<std::size_t>::max()) {
    std::size_t separation =
      matched_keyframe_index > last_corrected_matched_keyframe_index_ ?
      matched_keyframe_index - last_corrected_matched_keyframe_index_ :
      last_corrected_matched_keyframe_index_ - matched_keyframe_index;

    if (separation < min_old_keyframe_separation_for_correction_) {
      return false;
    }
  }

  return true;
}

void SimpleSlam::applyLoopClosureCorrection(
  std::size_t matched_keyframe_index, std::size_t current_keyframe_index)
{
  if (current_keyframe_index <= matched_keyframe_index ||
    current_keyframe_index >= keyframes_.size())
  {
    return;
  }

  const Pose2D matched_pose = keyframes_[matched_keyframe_index].corrected_pose;
  const Pose2D current_pose = keyframes_[current_keyframe_index].corrected_pose;

  double error_x = loop_closure_correction_strength_ * (matched_pose.x - current_pose.x);
  double error_y = loop_closure_correction_strength_ * (matched_pose.y - current_pose.y);
  double error_theta =
    loop_closure_correction_strength_ * normalizeAngle(matched_pose.theta - current_pose.theta);
  double span = static_cast<double>(current_keyframe_index - matched_keyframe_index);

  for (std::size_t keyframe_index = matched_keyframe_index + 1;
    keyframe_index <= current_keyframe_index;
    ++keyframe_index)
  {
    double fraction = static_cast<double>(keyframe_index - matched_keyframe_index) / span;
    keyframes_[keyframe_index].corrected_pose.x += fraction * error_x;
    keyframes_[keyframe_index].corrected_pose.y += fraction * error_y;
    keyframes_[keyframe_index].corrected_pose.theta =
      normalizeAngle(keyframes_[keyframe_index].corrected_pose.theta + fraction * error_theta);
  }

  loop_closure_correction_count_++;
  last_correction_scan_index_ = scans_integrated_;
  last_corrected_matched_keyframe_index_ = matched_keyframe_index;
}

StoredScan SimpleSlam::makeStoredScan(const sensor_msgs::msg::LaserScan & scan) const
{
  StoredScan stored_scan;
  stored_scan.ranges = scan.ranges;
  stored_scan.angle_min = scan.angle_min;
  stored_scan.angle_increment = scan.angle_increment;
  stored_scan.range_min = scan.range_min;
  stored_scan.range_max = scan.range_max;
  return stored_scan;
}

std::vector<float> SimpleSlam::makeScanSignature(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<float> signature;
  signature.reserve(scan.ranges.size() / scan_signature_beam_step_ + 1);

  for (std::size_t i = 0; i < scan.ranges.size(); i += scan_signature_beam_step_) {
    float range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min) {
      range = scan.range_max;
    }

    range = std::min(range, scan.range_max);
    signature.push_back(range / scan.range_max);
  }

  return signature;
}

double SimpleSlam::scanSignatureDifference(
  const std::vector<float> & first,
  const std::vector<float> & second) const
{
  std::size_t count = std::min(first.size(), second.size());
  if (count == 0) {
    return std::numeric_limits<double>::max();
  }

  double difference_sum = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    difference_sum += std::abs(first[i] - second[i]);
  }

  return difference_sum / count;
}

double SimpleSlam::poseDistance(const Pose2D & first, const Pose2D & second) const
{
  double dx = first.x - second.x;
  double dy = first.y - second.y;
  return std::sqrt(dx * dx + dy * dy);
}

Pose2D SimpleSlam::odomMsgToPose(const nav_msgs::msg::Odometry & msg) const
{
  Pose2D pose;
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;
  pose.theta = tf2::getYaw(msg.pose.pose.orientation);
  return pose;
}

bool SimpleSlam::odomMovedEnough(const Pose2D & old_odom, const Pose2D & new_odom) const
{
  double dx = new_odom.x - old_odom.x;
  double dy = new_odom.y - old_odom.y;
  double translation = std::sqrt(dx * dx + dy * dy);
  double rotation = std::abs(normalizeAngle(new_odom.theta - old_odom.theta));

  return translation >= min_update_translation_ || rotation >= min_update_rotation_;
}

geometry_msgs::msg::Quaternion SimpleSlam::yawToQuaternion(double yaw) const
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::Quaternion msg;
  msg.x = quaternion.x();
  msg.y = quaternion.y();
  msg.z = quaternion.z();
  msg.w = quaternion.w();
  return msg;
}

void SimpleSlam::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
}

bool SimpleSlam::isValidCell(int x, int y) const
{
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

std::vector<std::pair<int, int>> SimpleSlam::bresenhamLine(
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

double SimpleSlam::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double SimpleSlam::clamp(double value, double low, double high) const
{
  return std::min(std::max(value, low), high);
}


bool SimpleSlam::hasOdometry() const
{
  return odom_initialized_;
}

const Pose2D & SimpleSlam::slamPose() const
{
  return slam_pose_;
}

const Pose2D & SimpleSlam::currentOdomPose() const
{
  return current_odom_pose_;
}

const nav_msgs::msg::OccupancyGrid & SimpleSlam::map() const
{
  return map_msg_;
}

const nav_msgs::msg::OccupancyGrid & SimpleSlam::correctedMap() const
{
  return corrected_map_msg_;
}

const nav_msgs::msg::Path & SimpleSlam::trajectory() const
{
  return trajectory_msg_;
}

const nav_msgs::msg::Path & SimpleSlam::correctedTrajectory() const
{
  return corrected_trajectory_msg_;
}

int SimpleSlam::scansIntegrated() const
{
  return scans_integrated_;
}

int SimpleSlam::stationaryScansSkipped() const
{
  return stationary_scans_skipped_;
}

int SimpleSlam::scanMatchUsedCount() const
{
  return scan_match_used_count_;
}

int SimpleSlam::loopClosureCount() const
{
  return loop_closure_count_;
}

int SimpleSlam::loopClosureCorrectionCount() const
{
  return loop_closure_correction_count_;
}

int SimpleSlam::occupiedCellCount() const
{
  return occupied_cell_count_;
}

std::size_t SimpleSlam::keyframeCount() const
{
  return keyframes_.size();
}

}  // namespace slam

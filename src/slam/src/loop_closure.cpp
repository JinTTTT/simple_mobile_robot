#include "slam/loop_closure.hpp"

#include <algorithm>
#include <cmath>

namespace slam
{

void LoopClosure::configure(const SimpleSlamConfig & config)
{
  keyframe_min_translation_ = config.keyframe_min_translation;
  keyframe_min_rotation_ = config.keyframe_min_rotation;
  scan_signature_beam_step_ = config.scan_signature_beam_step;
  min_loop_closure_keyframe_age_ = config.min_loop_closure_keyframe_age;
  loop_closure_search_radius_ = config.loop_closure_search_radius;
  loop_closure_max_heading_diff_ = config.loop_closure_max_heading_diff;
  loop_closure_max_signature_diff_ = config.loop_closure_max_signature_diff;
  min_loop_closure_scan_gap_ = config.min_loop_closure_scan_gap;
  min_correction_scan_gap_ = config.min_correction_scan_gap;
  min_correction_keyframe_gap_ = config.min_correction_keyframe_gap;
  min_old_keyframe_separation_for_correction_ =
    config.min_old_keyframe_separation_for_correction;
  loop_closure_correction_strength_ = config.loop_closure_correction_strength;

  keyframes_.clear();
  keyframe_initialized_ = false;
  loop_closure_count_ = 0;
  loop_closure_correction_count_ = 0;
  last_loop_closure_scan_index_ = -100000;
  last_correction_scan_index_ = -100000;
  last_corrected_matched_keyframe_index_ = std::numeric_limits<std::size_t>::max();
}

bool LoopClosure::shouldAddKeyFrame(const Pose2D & slam_pose) const
{
  if (!keyframe_initialized_) {
    return true;
  }

  double distance = poseDistance(slam_pose, last_keyframe_pose_);
  double rotation = std::abs(normalizeAngle(slam_pose.theta - last_keyframe_pose_.theta));

  return distance >= keyframe_min_translation_ || rotation >= keyframe_min_rotation_;
}

void LoopClosure::addKeyFrame(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & slam_pose,
  int scan_index)
{
  KeyFrame keyframe;
  keyframe.pose = slam_pose;
  keyframe.corrected_pose = slam_pose;
  keyframe.scan_signature = makeScanSignature(scan);
  keyframe.scan = makeStoredScan(scan);
  keyframe.scan_index = scan_index;

  keyframes_.push_back(keyframe);
  last_keyframe_pose_ = slam_pose;
  keyframe_initialized_ = true;
}

LoopClosureResult LoopClosure::detect(int scans_integrated)
{
  if (keyframes_.size() <= static_cast<std::size_t>(min_loop_closure_keyframe_age_)) {
    return {};
  }

  if (scans_integrated - last_loop_closure_scan_index_ < min_loop_closure_scan_gap_) {
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
  last_loop_closure_scan_index_ = scans_integrated;
  const KeyFrame & matched_keyframe = keyframes_[best_candidate_index];

  LoopClosureResult result;
  result.detected = true;
  result.matched_pose = matched_keyframe.pose;
  result.current_keyframe_index = current_keyframe_index;
  result.matched_keyframe_index = best_candidate_index;
  result.matched_scan_index = matched_keyframe.scan_index;
  result.signature_difference = best_signature_difference;

  if (shouldApplyCorrection(best_candidate_index, current_keyframe_index, scans_integrated)) {
    applyCorrection(best_candidate_index, current_keyframe_index, scans_integrated);
    result.correction_applied = true;
  }

  return result;
}

const std::vector<KeyFrame> & LoopClosure::keyframes() const
{
  return keyframes_;
}

std::size_t LoopClosure::keyframeCount() const
{
  return keyframes_.size();
}

int LoopClosure::loopClosureCount() const
{
  return loop_closure_count_;
}

int LoopClosure::loopClosureCorrectionCount() const
{
  return loop_closure_correction_count_;
}

bool LoopClosure::shouldApplyCorrection(
  std::size_t matched_keyframe_index,
  std::size_t current_keyframe_index,
  int scans_integrated) const
{
  if (current_keyframe_index <= matched_keyframe_index) {
    return false;
  }

  if (current_keyframe_index - matched_keyframe_index < min_correction_keyframe_gap_) {
    return false;
  }

  if (scans_integrated - last_correction_scan_index_ < min_correction_scan_gap_) {
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

void LoopClosure::applyCorrection(
  std::size_t matched_keyframe_index,
  std::size_t current_keyframe_index,
  int scans_integrated)
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
  last_correction_scan_index_ = scans_integrated;
  last_corrected_matched_keyframe_index_ = matched_keyframe_index;
}

StoredScan LoopClosure::makeStoredScan(const sensor_msgs::msg::LaserScan & scan) const
{
  StoredScan stored_scan;
  stored_scan.ranges = scan.ranges;
  stored_scan.angle_min = scan.angle_min;
  stored_scan.angle_increment = scan.angle_increment;
  stored_scan.range_min = scan.range_min;
  stored_scan.range_max = scan.range_max;
  return stored_scan;
}

std::vector<float> LoopClosure::makeScanSignature(
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

double LoopClosure::scanSignatureDifference(
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

double LoopClosure::poseDistance(const Pose2D & first, const Pose2D & second) const
{
  double dx = first.x - second.x;
  double dy = first.y - second.y;
  return std::sqrt(dx * dx + dy * dy);
}

double LoopClosure::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace slam

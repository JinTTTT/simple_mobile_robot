#ifndef SLAM__LOOP_CLOSURE_HPP_
#define SLAM__LOOP_CLOSURE_HPP_

#include "slam/simple_slam_types.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include <limits>
#include <vector>

namespace slam
{

class LoopClosure
{
public:
  void configure(const SimpleSlamConfig & config);

  bool shouldAddKeyFrame(const Pose2D & slam_pose) const;
  void addKeyFrame(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & slam_pose,
    int scan_index);
  LoopClosureResult detect(int scans_integrated);

  const std::vector<KeyFrame> & keyframes() const;
  std::size_t keyframeCount() const;
  int loopClosureCount() const;
  int loopClosureCorrectionCount() const;

private:
  bool shouldApplyCorrection(
    std::size_t matched_keyframe_index,
    std::size_t current_keyframe_index,
    int scans_integrated) const;
  void applyCorrection(
    std::size_t matched_keyframe_index,
    std::size_t current_keyframe_index,
    int scans_integrated);
  StoredScan makeStoredScan(const sensor_msgs::msg::LaserScan & scan) const;
  std::vector<float> makeScanSignature(const sensor_msgs::msg::LaserScan & scan) const;
  double scanSignatureDifference(
    const std::vector<float> & first,
    const std::vector<float> & second) const;
  double poseDistance(const Pose2D & first, const Pose2D & second) const;
  double normalizeAngle(double angle) const;

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

  std::vector<KeyFrame> keyframes_;
  Pose2D last_keyframe_pose_;
  bool keyframe_initialized_ = false;

  int loop_closure_count_ = 0;
  int loop_closure_correction_count_ = 0;
  int last_loop_closure_scan_index_ = -100000;
  int last_correction_scan_index_ = -100000;
  std::size_t last_corrected_matched_keyframe_index_ = std::numeric_limits<std::size_t>::max();
};

}  // namespace slam

#endif  // SLAM__LOOP_CLOSURE_HPP_

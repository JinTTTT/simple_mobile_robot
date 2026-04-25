#ifndef PATH_FOLLOW_CONTROL__PATH_FOLLOW_CONTROLLER_HPP_
#define PATH_FOLLOW_CONTROL__PATH_FOLLOW_CONTROLLER_HPP_

#include "path_follow_control/path_types.hpp"

#include <string>

struct PathFollowControllerConfig
{
  double lookahead_distance = 0.35;
  double max_linear_speed = 0.20;
  double min_linear_speed = 0.05;
  double max_angular_speed = 0.80;
  double curvature_slowdown_gain = 0.60;
  double initial_alignment_angle_threshold = 0.10;
  double rotate_in_place_angle_threshold = 0.50;
  double rotate_in_place_gain = 1.50;
  double goal_tolerance_distance = 0.10;
  double goal_tolerance_angle = 0.15;
  double slow_down_goal_distance = 0.50;
  double final_alignment_gain = 1.50;
  double final_alignment_min_angular_speed = 0.20;
  double final_alignment_max_angular_speed = 0.50;
  double stuck_detection_window_seconds = 4.0;
  double min_progress_distance_m = 0.05;
  double min_goal_distance_improvement_m = 0.05;
  double min_commanded_linear_speed = 0.02;
  double min_commanded_angular_speed = 0.20;
};

struct PathFollowControlResult
{
  ControlCommand command;
  bool has_active_path = false;
  bool should_publish_lookahead_point = false;
  bool used_initial_alignment = false;
  bool initial_alignment_completed = false;
  bool used_final_alignment = false;
  bool reached_goal = false;
  bool is_stuck = false;
  double goal_distance = 0.0;
  double goal_heading_error = 0.0;
  double path_progress_ratio = 0.0;
  Pose2D lookahead_pose;
  std::string status_message;
};

class PathFollowController
{
public:
  void configure(const PathFollowControllerConfig & config);

  void setPath(const PosePath & path);
  void clearPath();

  bool hasPath() const;

  PathFollowControlResult computeControl(
    const Pose2D & current_pose,
    bool has_pose,
    double current_time_seconds);

private:
  std::size_t findClosestPathIndex(const Pose2D & current_pose) const;
  std::size_t findLookaheadIndex(std::size_t closest_index) const;
  double distanceBetween(const Pose2D & first, const Pose2D & second) const;
  double normalizeAngle(double angle) const;
  double clamp(double value, double low, double high) const;
  double computeFinalAlignmentAngularSpeed(double heading_error) const;

  void resetProgressMonitor();
  double computePathProgressDistance(const Pose2D & current_pose) const;
  double computeTotalPathLength() const;
  void updateProgressMonitor(
    const Pose2D & current_pose,
    double current_time_seconds,
    double goal_distance,
    const ControlCommand & command,
    PathFollowControlResult & result);

  PosePath path_;
  PathFollowControllerConfig config_;
  bool has_path_ = false;
  bool needs_initial_alignment_ = false;
  bool progress_monitor_initialized_ = false;
  bool is_stuck_ = false;
  double monitor_start_time_seconds_ = 0.0;
  Pose2D monitor_start_pose_;
  double monitor_start_goal_distance_ = 0.0;
};

#endif  // PATH_FOLLOW_CONTROL__PATH_FOLLOW_CONTROLLER_HPP_

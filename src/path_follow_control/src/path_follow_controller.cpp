#include "path_follow_control/path_follow_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

void PathFollowController::configure(const PathFollowControllerConfig & config)
{
  config_ = config;
}

void PathFollowController::setPath(const PosePath & path)
{
  path_ = path;
  has_path_ = !path_.empty();

  if (!has_path_) {
    needs_initial_alignment_ = false;
    resetProgressMonitor();
    return;
  }

  needs_initial_alignment_ = true;
  resetProgressMonitor();
}

void PathFollowController::clearPath()
{
  path_.clear();
  has_path_ = false;
  needs_initial_alignment_ = false;
  resetProgressMonitor();
}

bool PathFollowController::hasPath() const
{
  return has_path_;
}

PathFollowControlResult PathFollowController::computeControl(
  const Pose2D & current_pose,
  bool has_pose,
  double current_time_seconds)
{
  PathFollowControlResult result;
  result.has_active_path = has_path_;

  if (!has_path_ || !has_pose) {
    result.status_message = "Waiting for path and pose.";
    resetProgressMonitor();
    return result;
  }

  const Pose2D & goal_pose = path_.back();
  const double goal_distance = distanceBetween(current_pose, goal_pose);
  const double goal_heading_error = normalizeAngle(goal_pose.yaw - current_pose.yaw);
  result.goal_distance = goal_distance;
  result.goal_heading_error = goal_heading_error;

  if (goal_distance < config_.goal_tolerance_distance) {
    if (std::abs(goal_heading_error) < config_.goal_tolerance_angle) {
      clearPath();
      result.reached_goal = true;
      result.has_active_path = false;
      result.status_message = "Goal position and final orientation reached.";
      return result;
    }

    result.command.angular_velocity = computeFinalAlignmentAngularSpeed(goal_heading_error);
    result.used_final_alignment = true;
    result.status_message = "Goal position reached. Rotating to final goal orientation.";
    resetProgressMonitor();
    return result;
  }

  const std::size_t closest_index = findClosestPathIndex(current_pose);
  const std::size_t tracking_target_index = findLookaheadIndex(closest_index);
  const std::size_t initial_target_index = findLookaheadIndex(0);
  const Pose2D & target_pose =
    path_[needs_initial_alignment_ ? initial_target_index : tracking_target_index];

  result.lookahead_pose = target_pose;
  result.should_publish_lookahead_point = true;

  const double delta_x = target_pose.x - current_pose.x;
  const double delta_y = target_pose.y - current_pose.y;
  const double target_bearing = std::atan2(delta_y, delta_x);
  const double heading_error = normalizeAngle(target_bearing - current_pose.yaw);
  const double target_x_robot =
    std::cos(current_pose.yaw) * delta_x + std::sin(current_pose.yaw) * delta_y;
  const double target_y_robot =
    -std::sin(current_pose.yaw) * delta_x + std::cos(current_pose.yaw) * delta_y;

  if (needs_initial_alignment_) {
    if (std::abs(heading_error) > config_.initial_alignment_angle_threshold) {
      result.command.angular_velocity = clamp(
        config_.rotate_in_place_gain * heading_error,
        -config_.max_angular_speed,
        config_.max_angular_speed);
      result.used_initial_alignment = true;
      result.status_message = "Initial alignment active.";
      resetProgressMonitor();
      return result;
    }

    needs_initial_alignment_ = false;
    result.initial_alignment_completed = true;
  }

  if (std::abs(heading_error) > config_.rotate_in_place_angle_threshold) {
    result.command.angular_velocity = clamp(
      config_.rotate_in_place_gain * heading_error,
      -config_.max_angular_speed,
      config_.max_angular_speed);
  } else {
    const double lookahead_distance_squared =
      target_x_robot * target_x_robot + target_y_robot * target_y_robot;
    const double curvature = (lookahead_distance_squared > 1e-6) ?
      2.0 * target_y_robot / lookahead_distance_squared : 0.0;
    const double curvature_limited_speed =
      config_.max_linear_speed /
      (1.0 + config_.curvature_slowdown_gain * std::abs(curvature));

    result.command.linear_velocity =
      std::max(config_.min_linear_speed, curvature_limited_speed);

    if (goal_distance < config_.slow_down_goal_distance) {
      const double goal_limited_speed =
        config_.max_linear_speed * (goal_distance / config_.slow_down_goal_distance);
      result.command.linear_velocity =
        std::min(result.command.linear_velocity, goal_limited_speed);
    }

    result.command.linear_velocity = clamp(
      result.command.linear_velocity, 0.0, config_.max_linear_speed);
    result.command.angular_velocity = clamp(
      result.command.linear_velocity * curvature,
      -config_.max_angular_speed,
      config_.max_angular_speed);
  }

  updateProgressMonitor(
    current_pose, current_time_seconds, goal_distance, result.command, result);
  result.has_active_path = has_path_;
  return result;
}

std::size_t PathFollowController::findClosestPathIndex(const Pose2D & current_pose) const
{
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();

  for (std::size_t path_index = 0; path_index < path_.size(); ++path_index) {
    const double distance_to_pose = distanceBetween(current_pose, path_[path_index]);
    if (distance_to_pose < best_distance) {
      best_distance = distance_to_pose;
      best_index = path_index;
    }
  }

  return best_index;
}

std::size_t PathFollowController::findLookaheadIndex(std::size_t closest_index) const
{
  double accumulated_distance = 0.0;

  for (std::size_t path_index = closest_index; path_index + 1 < path_.size(); ++path_index) {
    accumulated_distance += distanceBetween(path_[path_index], path_[path_index + 1]);
    if (accumulated_distance >= config_.lookahead_distance) {
      return path_index + 1;
    }
  }

  return path_.size() - 1;
}

double PathFollowController::distanceBetween(const Pose2D & first, const Pose2D & second) const
{
  const double delta_x = second.x - first.x;
  const double delta_y = second.y - first.y;
  return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

double PathFollowController::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double PathFollowController::clamp(double value, double low, double high) const
{
  return std::max(low, std::min(value, high));
}

double PathFollowController::computeFinalAlignmentAngularSpeed(double heading_error) const
{
  const double commanded_speed = clamp(
    config_.final_alignment_gain * heading_error,
    -config_.final_alignment_max_angular_speed,
    config_.final_alignment_max_angular_speed);

  if (std::abs(commanded_speed) < config_.final_alignment_min_angular_speed) {
    return (heading_error >= 0.0) ?
           config_.final_alignment_min_angular_speed :
           -config_.final_alignment_min_angular_speed;
  }

  return commanded_speed;
}

void PathFollowController::resetProgressMonitor()
{
  progress_monitor_initialized_ = false;
  is_stuck_ = false;
}

double PathFollowController::computePathProgressDistance(const Pose2D & current_pose) const
{
  if (path_.size() < 2) {
    return 0.0;
  }

  double accumulated_distance = 0.0;
  double best_progress_distance = 0.0;
  double best_distance_to_path = std::numeric_limits<double>::infinity();

  for (std::size_t path_index = 0; path_index + 1 < path_.size(); ++path_index) {
    const double x0 = path_[path_index].x;
    const double y0 = path_[path_index].y;
    const double x1 = path_[path_index + 1].x;
    const double y1 = path_[path_index + 1].y;

    const double delta_x = x1 - x0;
    const double delta_y = y1 - y0;
    const double segment_length_squared = delta_x * delta_x + delta_y * delta_y;
    const double segment_length = std::sqrt(segment_length_squared);

    if (segment_length < 1e-6) {
      continue;
    }

    const double projection_numerator =
      (current_pose.x - x0) * delta_x + (current_pose.y - y0) * delta_y;
    const double projection_fraction =
      clamp(projection_numerator / segment_length_squared, 0.0, 1.0);

    const double projected_x = x0 + projection_fraction * delta_x;
    const double projected_y = y0 + projection_fraction * delta_y;
    const double distance_to_segment = std::sqrt(
      (current_pose.x - projected_x) * (current_pose.x - projected_x) +
      (current_pose.y - projected_y) * (current_pose.y - projected_y));

    if (distance_to_segment < best_distance_to_path) {
      best_distance_to_path = distance_to_segment;
      best_progress_distance = accumulated_distance + projection_fraction * segment_length;
    }

    accumulated_distance += segment_length;
  }

  return best_progress_distance;
}

double PathFollowController::computeTotalPathLength() const
{
  if (path_.size() < 2) {
    return 0.0;
  }

  double total_length = 0.0;
  for (std::size_t path_index = 0; path_index + 1 < path_.size(); ++path_index) {
    total_length += distanceBetween(path_[path_index], path_[path_index + 1]);
  }

  return total_length;
}

void PathFollowController::updateProgressMonitor(
  const Pose2D & current_pose,
  double current_time_seconds,
  double goal_distance,
  const ControlCommand & command,
  PathFollowControlResult & result)
{
  const double progress_distance = computePathProgressDistance(current_pose);
  const double total_path_length = computeTotalPathLength();
  result.path_progress_ratio =
    (total_path_length > 1e-6) ? progress_distance / total_path_length : 1.0;

  if (!progress_monitor_initialized_) {
    monitor_start_time_seconds_ = current_time_seconds;
    monitor_start_pose_ = current_pose;
    monitor_start_goal_distance_ = goal_distance;
    progress_monitor_initialized_ = true;
    is_stuck_ = false;
    result.is_stuck = false;
    return;
  }

  const bool commanding_motion =
    std::abs(command.linear_velocity) > config_.min_commanded_linear_speed ||
    std::abs(command.angular_velocity) > config_.min_commanded_angular_speed;

  if (!commanding_motion) {
    monitor_start_time_seconds_ = current_time_seconds;
    monitor_start_pose_ = current_pose;
    monitor_start_goal_distance_ = goal_distance;
    is_stuck_ = false;
    result.is_stuck = false;
    return;
  }

  const double elapsed_time = current_time_seconds - monitor_start_time_seconds_;
  if (elapsed_time < config_.stuck_detection_window_seconds) {
    result.is_stuck = is_stuck_;
    return;
  }

  const double moved_distance = distanceBetween(current_pose, monitor_start_pose_);
  const double goal_distance_improvement = monitor_start_goal_distance_ - goal_distance;

  is_stuck_ =
    moved_distance < config_.min_progress_distance_m &&
    goal_distance_improvement < config_.min_goal_distance_improvement_m;

  monitor_start_time_seconds_ = current_time_seconds;
  monitor_start_pose_ = current_pose;
  monitor_start_goal_distance_ = goal_distance;
  result.is_stuck = is_stuck_;
}

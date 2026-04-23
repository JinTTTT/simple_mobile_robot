#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

class PathFollowControlNode : public rclcpp::Node
{
public:
  PathFollowControlNode()
  : Node("path_follow_control_node")
  {
    lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 0.35);
    max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 0.20);
    min_linear_speed_ = this->declare_parameter<double>("min_linear_speed", 0.05);
    max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 0.80);
    curvature_slowdown_gain_ =
      this->declare_parameter<double>("curvature_slowdown_gain", 0.60);
    initial_alignment_angle_threshold_ =
      this->declare_parameter<double>("initial_alignment_angle_threshold", 0.10);
    rotate_in_place_angle_threshold_ =
      this->declare_parameter<double>("rotate_in_place_angle_threshold", 0.50);
    goal_tolerance_distance_ =
      this->declare_parameter<double>("goal_tolerance_distance", 0.10);
    goal_tolerance_angle_ =
      this->declare_parameter<double>("goal_tolerance_angle", 0.15);
    slow_down_goal_distance_ =
      this->declare_parameter<double>("slow_down_goal_distance", 0.50);
    final_alignment_max_angular_speed_ =
      this->declare_parameter<double>("final_alignment_max_angular_speed", 0.50);
    stuck_detection_window_seconds_ =
      this->declare_parameter<double>("stuck_detection_window_seconds", 4.0);
    min_progress_distance_m_ =
      this->declare_parameter<double>("min_progress_distance_m", 0.05);
    min_goal_distance_improvement_m_ =
      this->declare_parameter<double>("min_goal_distance_improvement_m", 0.05);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/smoothed_planned_path", 10,
      std::bind(&PathFollowControlNode::pathCallback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/estimated_pose", 10,
      std::bind(&PathFollowControlNode::poseCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    lookahead_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("/lookahead_point", 10);

    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PathFollowControlNode::controlLoop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "PathFollowControlNode started. Inputs: /smoothed_planned_path, /estimated_pose. Output: /cmd_vel.");
    RCLCPP_INFO(
      this->get_logger(),
      "Pure pursuit enabled with lookahead %.2f m, initial alignment threshold %.2f rad, and rotate-in-place threshold %.2f rad.",
      lookahead_distance_,
      initial_alignment_angle_threshold_,
      rotate_in_place_angle_threshold_);
    RCLCPP_INFO(
      this->get_logger(),
      "Linear max %.2f m/s, angular max %.2f rad/s, goal tolerances %.2f m / %.2f rad.",
      max_linear_speed_,
      max_angular_speed_,
      goal_tolerance_distance_,
      goal_tolerance_angle_);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_ = *msg;
    has_path_ = !path_.poses.empty();

    if (!has_path_) {
      publishStop();
      needs_initial_alignment_ = false;
      resetProgressMonitor();
      RCLCPP_WARN(this->get_logger(), "Received empty path. Stopping robot.");
      return;
    }

    needs_initial_alignment_ = true;
    resetProgressMonitor();
    RCLCPP_INFO(
      this->get_logger(),
      "Received path with %zu poses. Initial heading alignment enabled.",
      path_.poses.size());
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_ = *msg;
    has_pose_ = true;
  }

  void controlLoop()
  {
    if (!has_path_ || !has_pose_) {
      publishStop();
      resetProgressMonitor();
      return;
    }

    if (path_.header.frame_id != "map" || current_pose_.header.frame_id != "map") {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Controller expects path and pose in map frame.");
      publishStop();
      resetProgressMonitor();
      return;
    }

    const geometry_msgs::msg::PoseStamped & goal_pose = path_.poses.back();
    const double goal_distance = distanceBetween(current_pose_, goal_pose);
    const double goal_heading_error =
      normalizeAngle(getYaw(goal_pose) - getYaw(current_pose_));

    if (goal_distance < goal_tolerance_distance_) {
      if (std::abs(goal_heading_error) < goal_tolerance_angle_) {
        publishStop();
        has_path_ = false;
        needs_initial_alignment_ = false;
        path_.poses.clear();
        resetProgressMonitor();
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Goal position and final orientation reached. Robot stopped.");
        return;
      }

      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = computeFinalAlignmentAngularSpeed(goal_heading_error);
      cmd_vel_pub_->publish(cmd_vel);
      resetProgressMonitor();
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Goal position reached. Rotating to final goal orientation.");
      return;
    }

    const std::size_t closest_index = findClosestPathIndex();
    const std::size_t tracking_target_index = findLookaheadIndex(closest_index);
    const std::size_t initial_target_index = findLookaheadIndex(0);
    const geometry_msgs::msg::PoseStamped & target_pose =
      path_.poses[needs_initial_alignment_ ? initial_target_index : tracking_target_index];
    publishLookaheadPoint(target_pose);

    const double robot_x = current_pose_.pose.position.x;
    const double robot_y = current_pose_.pose.position.y;
    const double robot_yaw = getYaw(current_pose_);
    const double target_x = target_pose.pose.position.x;
    const double target_y = target_pose.pose.position.y;

    const double dx = target_x - robot_x;
    const double dy = target_y - robot_y;
    const double target_bearing = std::atan2(dy, dx);
    const double heading_error = normalizeAngle(target_bearing - robot_yaw);
    const double target_x_robot =
      std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    const double target_y_robot =
      -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

    geometry_msgs::msg::Twist cmd_vel;

    if (needs_initial_alignment_) {
      if (std::abs(heading_error) > initial_alignment_angle_threshold_) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = clamp(
          rotate_in_place_gain_ * heading_error,
          -max_angular_speed_,
          max_angular_speed_);
        cmd_vel_pub_->publish(cmd_vel);
        resetProgressMonitor();
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Initial alignment active. Heading error %.2f rad to first lookahead target.",
          heading_error);
        return;
      }

      needs_initial_alignment_ = false;
      RCLCPP_INFO(this->get_logger(), "Initial alignment finished. Switching to path tracking.");
    }

    if (std::abs(heading_error) > rotate_in_place_angle_threshold_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = clamp(
        rotate_in_place_gain_ * heading_error,
        -max_angular_speed_,
        max_angular_speed_);
    } else {
      const double lookahead_distance_squared =
        target_x_robot * target_x_robot + target_y_robot * target_y_robot;
      const double curvature = (lookahead_distance_squared > 1e-6) ?
        2.0 * target_y_robot / lookahead_distance_squared : 0.0;
      const double curvature_speed =
        max_linear_speed_ / (1.0 + curvature_slowdown_gain_ * std::abs(curvature));

      cmd_vel.linear.x = std::max(min_linear_speed_, curvature_speed);

      if (goal_distance < slow_down_goal_distance_) {
        const double goal_speed =
          max_linear_speed_ * (goal_distance / slow_down_goal_distance_);
        cmd_vel.linear.x = std::min(cmd_vel.linear.x, goal_speed);
      }

      cmd_vel.linear.x = clamp(cmd_vel.linear.x, 0.0, max_linear_speed_);
      cmd_vel.angular.z = clamp(
        cmd_vel.linear.x * curvature,
        -max_angular_speed_,
        max_angular_speed_);
    }

    cmd_vel_pub_->publish(cmd_vel);
    updateProgressMonitor(goal_distance, cmd_vel);
  }

  std::size_t findClosestPathIndex() const
  {
    std::size_t best_index = 0;
    double best_distance = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < path_.poses.size(); ++i) {
      const double distance = distanceBetween(current_pose_, path_.poses[i]);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = i;
      }
    }

    return best_index;
  }

  std::size_t findLookaheadIndex(std::size_t closest_index) const
  {
    double accumulated_distance = 0.0;

    for (std::size_t i = closest_index; i + 1 < path_.poses.size(); ++i) {
      accumulated_distance += distanceBetween(path_.poses[i], path_.poses[i + 1]);
      if (accumulated_distance >= lookahead_distance_) {
        return i + 1;
      }
    }

    return path_.poses.size() - 1;
  }

  double distanceBetween(
    const geometry_msgs::msg::PoseStamped & first,
    const geometry_msgs::msg::PoseStamped & second) const
  {
    const double dx = second.pose.position.x - first.pose.position.x;
    const double dy = second.pose.position.y - first.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double getYaw(const geometry_msgs::msg::PoseStamped & pose) const
  {
    const auto & q = pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double normalizeAngle(double angle) const
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  double clamp(double value, double low, double high) const
  {
    return std::max(low, std::min(value, high));
  }

  double computeFinalAlignmentAngularSpeed(double heading_error) const
  {
    const double commanded_speed = clamp(
      final_alignment_gain_ * heading_error,
      -final_alignment_max_angular_speed_,
      final_alignment_max_angular_speed_);

    if (std::abs(commanded_speed) < final_alignment_min_angular_speed_) {
      return (heading_error >= 0.0) ?
        final_alignment_min_angular_speed_ :
        -final_alignment_min_angular_speed_;
    }

    return commanded_speed;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
  }

  void publishLookaheadPoint(const geometry_msgs::msg::PoseStamped & target_pose)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header = target_pose.header;
    msg.point = target_pose.pose.position;
    lookahead_point_pub_->publish(msg);
  }

  void resetProgressMonitor()
  {
    progress_monitor_initialized_ = false;
    is_stuck_ = false;
  }

  double computePathProgressDistance() const
  {
    if (path_.poses.empty()) {
      return 0.0;
    }

    if (path_.poses.size() == 1) {
      return 0.0;
    }

    const double robot_x = current_pose_.pose.position.x;
    const double robot_y = current_pose_.pose.position.y;

    double accumulated_distance = 0.0;
    double best_progress_distance = 0.0;
    double best_distance_to_path = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i + 1 < path_.poses.size(); ++i) {
      const double x0 = path_.poses[i].pose.position.x;
      const double y0 = path_.poses[i].pose.position.y;
      const double x1 = path_.poses[i + 1].pose.position.x;
      const double y1 = path_.poses[i + 1].pose.position.y;

      const double dx = x1 - x0;
      const double dy = y1 - y0;
      const double segment_length_squared = dx * dx + dy * dy;
      const double segment_length = std::sqrt(segment_length_squared);

      if (segment_length < 1e-6) {
        continue;
      }

      const double projection_numerator =
        (robot_x - x0) * dx + (robot_y - y0) * dy;
      const double t = clamp(projection_numerator / segment_length_squared, 0.0, 1.0);

      const double projected_x = x0 + t * dx;
      const double projected_y = y0 + t * dy;
      const double distance_to_segment =
        std::sqrt(
        (robot_x - projected_x) * (robot_x - projected_x) +
        (robot_y - projected_y) * (robot_y - projected_y));

      if (distance_to_segment < best_distance_to_path) {
        best_distance_to_path = distance_to_segment;
        best_progress_distance = accumulated_distance + t * segment_length;
      }

      accumulated_distance += segment_length;
    }

    return best_progress_distance;
  }

  double computeTotalPathLength() const
  {
    if (path_.poses.size() < 2) {
      return 0.0;
    }

    double total_length = 0.0;
    for (std::size_t i = 0; i + 1 < path_.poses.size(); ++i) {
      total_length += distanceBetween(path_.poses[i], path_.poses[i + 1]);
    }
    return total_length;
  }

  void updateProgressMonitor(
    double goal_distance,
    const geometry_msgs::msg::Twist & cmd_vel)
  {
    const double progress_distance = computePathProgressDistance();
    const double total_path_length = computeTotalPathLength();

    if (!progress_monitor_initialized_) {
      monitor_start_time_ = this->now();
      monitor_start_pose_ = current_pose_;
      monitor_start_goal_distance_ = goal_distance;
      progress_monitor_initialized_ = true;
      is_stuck_ = false;
      return;
    }

    const double path_progress_ratio = (total_path_length > 1e-6) ?
      progress_distance / total_path_length :
      1.0;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Path progress %.0f%%, goal distance %.2f m.",
      path_progress_ratio * 100.0,
      goal_distance);

    const bool commanding_motion =
      std::abs(cmd_vel.linear.x) > min_commanded_linear_speed_ ||
      std::abs(cmd_vel.angular.z) > min_commanded_angular_speed_;

    if (!commanding_motion) {
      monitor_start_time_ = this->now();
      monitor_start_pose_ = current_pose_;
      monitor_start_goal_distance_ = goal_distance;
      is_stuck_ = false;
      return;
    }

    const double elapsed_time = (this->now() - monitor_start_time_).seconds();
    if (elapsed_time < stuck_detection_window_seconds_) {
      return;
    }

    const double moved_distance = distanceBetween(current_pose_, monitor_start_pose_);
    const double goal_distance_improvement = monitor_start_goal_distance_ - goal_distance;

    is_stuck_ =
      moved_distance < min_progress_distance_m_ &&
      goal_distance_improvement < min_goal_distance_improvement_m_;

    if (is_stuck_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Stuck detected: moved %.3f m, goal improved %.3f m over %.1f s.",
        moved_distance,
        goal_distance_improvement,
        elapsed_time);
    }

    monitor_start_time_ = this->now();
    monitor_start_pose_ = current_pose_;
    monitor_start_goal_distance_ = goal_distance;
  }

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped current_pose_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_point_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  bool has_path_ = false;
  bool has_pose_ = false;
  bool needs_initial_alignment_ = false;
  bool progress_monitor_initialized_ = false;
  bool is_stuck_ = false;

  rclcpp::Time monitor_start_time_;
  geometry_msgs::msg::PoseStamped monitor_start_pose_;
  double monitor_start_goal_distance_ = 0.0;

  double lookahead_distance_ = 0.35;
  double max_linear_speed_ = 0.20;
  double min_linear_speed_ = 0.05;
  double max_angular_speed_ = 0.80;
  double curvature_slowdown_gain_ = 0.60;
  double initial_alignment_angle_threshold_ = 0.10;
  double rotate_in_place_angle_threshold_ = 0.50;
  double rotate_in_place_gain_ = 1.50;
  double goal_tolerance_distance_ = 0.10;
  double goal_tolerance_angle_ = 0.15;
  double slow_down_goal_distance_ = 0.50;
  double final_alignment_gain_ = 1.50;
  double final_alignment_min_angular_speed_ = 0.20;
  double final_alignment_max_angular_speed_ = 0.50;
  double stuck_detection_window_seconds_ = 4.0;
  double min_progress_distance_m_ = 0.05;
  double min_goal_distance_improvement_m_ = 0.05;
  double min_commanded_linear_speed_ = 0.02;
  double min_commanded_angular_speed_ = 0.20;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowControlNode>());
  rclcpp::shutdown();
  return 0;
}

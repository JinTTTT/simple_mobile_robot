#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

class PathFollowControlNode : public rclcpp::Node
{
public:
  PathFollowControlNode()
  : Node("path_follow_control_node")
  {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&PathFollowControlNode::pathCallback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/estimated_pose", 10,
      std::bind(&PathFollowControlNode::poseCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PathFollowControlNode::controlLoop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "PathFollowControlNode started. Inputs: /planned_path, /estimated_pose. Output: /cmd_vel.");
    RCLCPP_INFO(
      this->get_logger(),
      "Pure pursuit enabled with lookahead %.2f m and rotate-in-place threshold %.2f rad.",
      lookahead_distance_,
      rotate_in_place_angle_threshold_);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_ = *msg;
    has_path_ = !path_.poses.empty();

    if (!has_path_) {
      publishStop();
      RCLCPP_WARN(this->get_logger(), "Received empty path. Stopping robot.");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received path with %zu poses.",
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
      return;
    }

    if (path_.header.frame_id != "map" || current_pose_.header.frame_id != "map") {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Controller expects path and pose in map frame.");
      publishStop();
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
        path_.poses.clear();
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Goal position and final orientation reached. Robot stopped.");
        return;
      }

      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = computeFinalAlignmentAngularSpeed(goal_heading_error);
      cmd_vel_pub_->publish(cmd_vel);
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Goal position reached. Rotating to final goal orientation.");
      return;
    }

    const std::size_t closest_index = findClosestPathIndex();
    const std::size_t target_index = findLookaheadIndex(closest_index);
    const geometry_msgs::msg::PoseStamped & target_pose = path_.poses[target_index];

    const double robot_x = current_pose_.pose.position.x;
    const double robot_y = current_pose_.pose.position.y;
    const double robot_yaw = getYaw(current_pose_);
    const double target_x = target_pose.pose.position.x;
    const double target_y = target_pose.pose.position.y;

    const double dx = target_x - robot_x;
    const double dy = target_y - robot_y;
    const double target_distance = std::sqrt(dx * dx + dy * dy);
    const double target_bearing = std::atan2(dy, dx);
    const double heading_error = normalizeAngle(target_bearing - robot_yaw);

    geometry_msgs::msg::Twist cmd_vel;

    if (std::abs(heading_error) > rotate_in_place_angle_threshold_) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = clamp(
        rotate_in_place_gain_ * heading_error,
        -max_angular_speed_,
        max_angular_speed_);
    } else {
      const double curvature = (target_distance > 1e-6) ?
        2.0 * std::sin(heading_error) / target_distance : 0.0;

      const double distance_scale = std::min(target_distance / lookahead_distance_, 1.0);
      const double angle_scale = std::max(
        0.0,
        1.0 - std::abs(heading_error) / rotate_in_place_angle_threshold_);

      cmd_vel.linear.x = max_linear_speed_ * distance_scale * angle_scale;
      cmd_vel.angular.z = clamp(
        cmd_vel.linear.x * curvature,
        -max_angular_speed_,
        max_angular_speed_);

      if (goal_distance < slow_down_goal_distance_) {
        cmd_vel.linear.x *= goal_distance / slow_down_goal_distance_;
      }
    }

    cmd_vel_pub_->publish(cmd_vel);
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

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped current_pose_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  bool has_path_ = false;
  bool has_pose_ = false;

  double lookahead_distance_ = 0.35;
  double max_linear_speed_ = 0.20;
  double max_angular_speed_ = 0.80;
  double rotate_in_place_angle_threshold_ = 0.50;
  double rotate_in_place_gain_ = 1.50;
  double goal_tolerance_distance_ = 0.10;
  double goal_tolerance_angle_ = 0.15;
  double slow_down_goal_distance_ = 0.50;
  double final_alignment_gain_ = 1.50;
  double final_alignment_min_angular_speed_ = 0.20;
  double final_alignment_max_angular_speed_ = 0.50;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowControlNode>());
  rclcpp::shutdown();
  return 0;
}

#include "path_follow_control/path_follow_controller.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <functional>

class PathFollowControlNode : public rclcpp::Node
{
public:
  PathFollowControlNode()
  : Node("path_follow_control_node")
  {
    controller_config_.lookahead_distance =
      this->declare_parameter<double>("lookahead_distance", 0.35);
    controller_config_.max_linear_speed =
      this->declare_parameter<double>("max_linear_speed", 0.20);
    controller_config_.min_linear_speed =
      this->declare_parameter<double>("min_linear_speed", 0.05);
    controller_config_.max_angular_speed =
      this->declare_parameter<double>("max_angular_speed", 0.80);
    controller_config_.curvature_slowdown_gain =
      this->declare_parameter<double>("curvature_slowdown_gain", 0.60);
    controller_config_.initial_alignment_angle_threshold =
      this->declare_parameter<double>("initial_alignment_angle_threshold", 0.10);
    controller_config_.rotate_in_place_angle_threshold =
      this->declare_parameter<double>("rotate_in_place_angle_threshold", 0.50);
    controller_config_.goal_tolerance_distance =
      this->declare_parameter<double>("goal_tolerance_distance", 0.10);
    controller_config_.goal_tolerance_angle =
      this->declare_parameter<double>("goal_tolerance_angle", 0.15);
    controller_config_.slow_down_goal_distance =
      this->declare_parameter<double>("slow_down_goal_distance", 0.50);
    controller_config_.final_alignment_max_angular_speed =
      this->declare_parameter<double>("final_alignment_max_angular_speed", 0.50);
    controller_config_.stuck_detection_window_seconds =
      this->declare_parameter<double>("stuck_detection_window_seconds", 4.0);
    controller_config_.min_progress_distance_m =
      this->declare_parameter<double>("min_progress_distance_m", 0.05);
    controller_config_.min_goal_distance_improvement_m =
      this->declare_parameter<double>("min_goal_distance_improvement_m", 0.05);
    controller_.configure(controller_config_);

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
      controller_config_.lookahead_distance,
      controller_config_.initial_alignment_angle_threshold,
      controller_config_.rotate_in_place_angle_threshold);
    RCLCPP_INFO(
      this->get_logger(),
      "Linear max %.2f m/s, angular max %.2f rad/s, goal tolerances %.2f m / %.2f rad.",
      controller_config_.max_linear_speed,
      controller_config_.max_angular_speed,
      controller_config_.goal_tolerance_distance,
      controller_config_.goal_tolerance_angle);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg)
  {
    path_frame_id_ = path_msg->header.frame_id;
    const PosePath path = convertPathMessage(*path_msg);
    controller_.setPath(path);

    if (!controller_.hasPath()) {
      publishStop();
      RCLCPP_WARN(this->get_logger(), "Received empty path. Stopping robot.");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received path with %zu poses. Initial heading alignment enabled.",
      path.size());
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
  {
    current_pose_ = *pose_msg;
    has_pose_ = true;
  }

  void controlLoop()
  {
    if (!controller_.hasPath() || !has_pose_) {
      publishStop();
      return;
    }

    if (path_frame_id_ != "map" || current_pose_.header.frame_id != "map") {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Controller expects path and pose in map frame.");
      publishStop();
      return;
    }

    const PathFollowControlResult control_result = controller_.computeControl(
      convertPoseMessage(current_pose_),
      has_pose_,
      this->now().seconds());

    publishCommand(control_result.command);

    if (control_result.should_publish_lookahead_point) {
      publishLookaheadPoint(control_result.lookahead_pose);
    }

    if (control_result.reached_goal) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Goal position and final orientation reached. Robot stopped.");
      return;
    }

    if (control_result.used_final_alignment) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Goal position reached. Rotating to final goal orientation.");
      return;
    }

    if (control_result.used_initial_alignment) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Initial alignment active. Heading error %.2f rad to first lookahead target.",
        normalizeAngle(control_result.lookahead_pose.yaw - convertPoseMessage(current_pose_).yaw));
      return;
    }

    if (control_result.initial_alignment_completed) {
      RCLCPP_INFO(this->get_logger(), "Initial alignment finished. Switching to path tracking.");
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Path progress %.0f%%, goal distance %.2f m.",
      control_result.path_progress_ratio * 100.0,
      control_result.goal_distance);

    if (control_result.is_stuck) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Stuck detected while following the path.");
    }
  }

  PosePath convertPathMessage(const nav_msgs::msg::Path & path_message) const
  {
    PosePath path;
    path.reserve(path_message.poses.size());
    for (const auto & pose : path_message.poses) {
      path.push_back(convertPoseMessage(pose));
    }
    return path;
  }

  Pose2D convertPoseMessage(const geometry_msgs::msg::PoseStamped & pose_message) const
  {
    const auto & q = pose_message.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    Pose2D pose;
    pose.x = pose_message.pose.position.x;
    pose.y = pose_message.pose.position.y;
    pose.yaw = std::atan2(siny_cosp, cosy_cosp);
    return pose;
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

  void publishCommand(const ControlCommand & command)
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = command.linear_velocity;
    cmd_vel.angular.z = command.angular_velocity;
    cmd_vel_pub_->publish(cmd_vel);
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
  }

  void publishLookaheadPoint(const Pose2D & lookahead_pose)
  {
    geometry_msgs::msg::PointStamped point_message;
    point_message.header.stamp = this->now();
    point_message.header.frame_id = "map";
    point_message.point.x = lookahead_pose.x;
    point_message.point.y = lookahead_pose.y;
    point_message.point.z = 0.0;
    lookahead_point_pub_->publish(point_message);
  }

  PathFollowController controller_;
  PathFollowControllerConfig controller_config_;

  geometry_msgs::msg::PoseStamped current_pose_;
  std::string path_frame_id_ = "map";

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_point_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  bool has_pose_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowControlNode>());
  rclcpp::shutdown();
  return 0;
}

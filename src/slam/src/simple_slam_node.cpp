#include "slam/simple_slam.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <memory>

class SimpleSlamNode : public rclcpp::Node
{
public:
  SimpleSlamNode();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  slam::SimpleSlamConfig loadSlamConfig();
  void publishEstimatedPose(const builtin_interfaces::msg::Time & stamp);
  void publishScanMatchedPose(
    const slam::Pose2D & pose,
    const builtin_interfaces::msg::Time & stamp);
  void publishLoopClosurePose(
    const slam::Pose2D & pose,
    const builtin_interfaces::msg::Time & stamp);
  void publishMapToOdomTf(const builtin_interfaces::msg::Time & stamp);
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const;

  slam::SimpleSlam simple_slam_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr corrected_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr scan_matched_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr corrected_trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr loop_closure_pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

SimpleSlamNode::SimpleSlamNode()
: Node("simple_slam_node")
{
  simple_slam_.configure(loadSlamConfig());

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&SimpleSlamNode::odomCallback, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&SimpleSlamNode::scanCallback, this, std::placeholders::_1));

  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
  corrected_map_pub_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("/corrected_map", map_qos);
  estimated_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
  scan_matched_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("/scan_matched_pose", 10);
  trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
  corrected_trajectory_pub_ =
    this->create_publisher<nav_msgs::msg::Path>("/corrected_trajectory", 10);
  loop_closure_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("/loop_closure_pose", 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(this->get_logger(), "Simple SLAM node started.");
  RCLCPP_INFO(
    this->get_logger(),
    "Inputs: /odom, /scan. Outputs: /map, /corrected_map, /estimated_pose, "
    "/scan_matched_pose, /trajectory, /corrected_trajectory, /loop_closure_pose, "
    "TF map->odom.");
}

slam::SimpleSlamConfig SimpleSlamNode::loadSlamConfig()
{
  slam::SimpleSlamConfig config;

  config.resolution = this->declare_parameter("resolution", config.resolution);
  config.width = this->declare_parameter("width", config.width);
  config.height = this->declare_parameter("height", config.height);
  config.origin_x = this->declare_parameter("origin_x", config.origin_x);
  config.origin_y = this->declare_parameter("origin_y", config.origin_y);

  config.log_odds_hit = this->declare_parameter("log_odds_hit", config.log_odds_hit);
  config.log_odds_free = this->declare_parameter("log_odds_free", config.log_odds_free);
  config.log_odds_min = this->declare_parameter("log_odds_min", config.log_odds_min);
  config.log_odds_max = this->declare_parameter("log_odds_max", config.log_odds_max);

  config.min_occupied_cells_for_matching = this->declare_parameter(
    "min_occupied_cells_for_matching", config.min_occupied_cells_for_matching);
  config.likelihood_max_distance = this->declare_parameter(
    "likelihood_max_distance", config.likelihood_max_distance);

  config.scan_match_xy_range =
    this->declare_parameter("scan_match_xy_range", config.scan_match_xy_range);
  config.scan_match_xy_step =
    this->declare_parameter("scan_match_xy_step", config.scan_match_xy_step);
  config.scan_match_theta_range =
    this->declare_parameter("scan_match_theta_range", config.scan_match_theta_range);
  config.scan_match_theta_step =
    this->declare_parameter("scan_match_theta_step", config.scan_match_theta_step);
  config.scan_match_beam_step = static_cast<std::size_t>(
    this->declare_parameter("scan_match_beam_step", static_cast<int>(config.scan_match_beam_step)));
  config.min_scan_match_score =
    this->declare_parameter("min_scan_match_score", config.min_scan_match_score);
  config.min_scan_match_score_improvement = this->declare_parameter(
    "min_scan_match_score_improvement", config.min_scan_match_score_improvement);
  config.max_scan_match_translation_correction = this->declare_parameter(
    "max_scan_match_translation_correction", config.max_scan_match_translation_correction);
  config.max_scan_match_rotation_correction = this->declare_parameter(
    "max_scan_match_rotation_correction", config.max_scan_match_rotation_correction);
  config.min_scan_match_translation_interval = this->declare_parameter(
    "min_scan_match_translation_interval", config.min_scan_match_translation_interval);
  config.min_scan_match_rotation_interval = this->declare_parameter(
    "min_scan_match_rotation_interval", config.min_scan_match_rotation_interval);
  config.min_scan_match_scan_gap =
    this->declare_parameter("min_scan_match_scan_gap", config.min_scan_match_scan_gap);
  config.min_update_translation =
    this->declare_parameter("min_update_translation", config.min_update_translation);
  config.min_update_rotation =
    this->declare_parameter("min_update_rotation", config.min_update_rotation);

  config.keyframe_min_translation =
    this->declare_parameter("keyframe_min_translation", config.keyframe_min_translation);
  config.keyframe_min_rotation =
    this->declare_parameter("keyframe_min_rotation", config.keyframe_min_rotation);
  config.scan_signature_beam_step = static_cast<std::size_t>(
    this->declare_parameter(
      "scan_signature_beam_step", static_cast<int>(config.scan_signature_beam_step)));
  config.min_loop_closure_keyframe_age = this->declare_parameter(
    "min_loop_closure_keyframe_age", config.min_loop_closure_keyframe_age);
  config.loop_closure_search_radius =
    this->declare_parameter("loop_closure_search_radius", config.loop_closure_search_radius);
  config.loop_closure_max_heading_diff = this->declare_parameter(
    "loop_closure_max_heading_diff", config.loop_closure_max_heading_diff);
  config.loop_closure_max_signature_diff = this->declare_parameter(
    "loop_closure_max_signature_diff", config.loop_closure_max_signature_diff);
  config.min_loop_closure_scan_gap =
    this->declare_parameter("min_loop_closure_scan_gap", config.min_loop_closure_scan_gap);
  config.min_correction_scan_gap =
    this->declare_parameter("min_correction_scan_gap", config.min_correction_scan_gap);
  config.min_correction_keyframe_gap = static_cast<std::size_t>(
    this->declare_parameter(
      "min_correction_keyframe_gap", static_cast<int>(config.min_correction_keyframe_gap)));
  config.min_old_keyframe_separation_for_correction = static_cast<std::size_t>(
    this->declare_parameter(
      "min_old_keyframe_separation_for_correction",
      static_cast<int>(config.min_old_keyframe_separation_for_correction)));
  config.loop_closure_correction_strength = this->declare_parameter(
    "loop_closure_correction_strength", config.loop_closure_correction_strength);

  return config;
}

void SimpleSlamNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  bool first_odom_received = simple_slam_.handleOdometry(*msg);

  if (first_odom_received) {
    publishEstimatedPose(msg->header.stamp);
    publishMapToOdomTf(msg->header.stamp);
    RCLCPP_INFO(this->get_logger(), "First odom received. SLAM pose starts at map origin.");
  }
}

void SimpleSlamNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!simple_slam_.hasOdometry()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Waiting for /odom before using scans.");
    return;
  }

  slam::SlamUpdateResult update = simple_slam_.handleScan(*msg, msg->header.stamp);

  if (update.stationary_scan_skipped) {
    publishEstimatedPose(msg->header.stamp);
    publishMapToOdomTf(msg->header.stamp);
    return;
  }

  if (!update.scan_integrated) {
    return;
  }

  if (update.map_updated) {
    map_pub_->publish(simple_slam_.map());
  }

  publishEstimatedPose(msg->header.stamp);

  if (update.scan_match.used_scan_matching) {
    publishScanMatchedPose(update.scan_match.pose, msg->header.stamp);
  }

  publishMapToOdomTf(msg->header.stamp);

  if (update.trajectory_updated) {
    trajectory_pub_->publish(simple_slam_.trajectory());
  }

  if (update.corrected_trajectory_updated) {
    corrected_trajectory_pub_->publish(simple_slam_.correctedTrajectory());
  }

  if (update.loop_closure.detected) {
    publishLoopClosurePose(update.loop_closure.matched_pose, msg->header.stamp);
    RCLCPP_INFO(
      this->get_logger(),
      "Loop closure detected: current_scan=%d current_keyframe=%zu matched_keyframe=%zu "
      "old_scan=%d signature_diff=%.3f correction=%s",
      simple_slam_.scansIntegrated(),
      update.loop_closure.current_keyframe_index,
      update.loop_closure.matched_keyframe_index,
      update.loop_closure.matched_scan_index,
      update.loop_closure.signature_difference,
      update.loop_closure.correction_applied ? "applied" : "skipped");
  }

  if (update.corrected_map_updated) {
    corrected_map_pub_->publish(simple_slam_.correctedMap());
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5000,
    "SLAM integrated=%d stationary_skipped=%d scan_match_used=%d keyframes=%zu "
    "loops=%d corrections=%d pose=(%.2f, %.2f, %.2f) match=%s score=%.3f "
    "predicted_score=%.3f occupied=%d",
    simple_slam_.scansIntegrated(),
    simple_slam_.stationaryScansSkipped(),
    simple_slam_.scanMatchUsedCount(),
    simple_slam_.keyframeCount(),
    simple_slam_.loopClosureCount(),
    simple_slam_.loopClosureCorrectionCount(),
    simple_slam_.slamPose().x,
    simple_slam_.slamPose().y,
    simple_slam_.slamPose().theta,
    update.scan_match.used_scan_matching ? "yes" : "no",
    update.scan_match.score,
    update.scan_match.predicted_score,
    simple_slam_.occupiedCellCount());
}

void SimpleSlamNode::publishEstimatedPose(const builtin_interfaces::msg::Time & stamp)
{
  const slam::Pose2D & slam_pose = simple_slam_.slamPose();

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  msg.pose.position.x = slam_pose.x;
  msg.pose.position.y = slam_pose.y;
  msg.pose.orientation = yawToQuaternion(slam_pose.theta);
  estimated_pose_pub_->publish(msg);
}

void SimpleSlamNode::publishScanMatchedPose(
  const slam::Pose2D & pose,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  msg.pose.position.x = pose.x;
  msg.pose.position.y = pose.y;
  msg.pose.orientation = yawToQuaternion(pose.theta);
  scan_matched_pose_pub_->publish(msg);
}

void SimpleSlamNode::publishLoopClosurePose(
  const slam::Pose2D & pose,
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  msg.pose.position.x = pose.x;
  msg.pose.position.y = pose.y;
  msg.pose.orientation = yawToQuaternion(pose.theta);
  loop_closure_pose_pub_->publish(msg);
}

void SimpleSlamNode::publishMapToOdomTf(const builtin_interfaces::msg::Time & stamp)
{
  const slam::Pose2D & slam_pose = simple_slam_.slamPose();
  const slam::Pose2D & odom_pose = simple_slam_.currentOdomPose();

  tf2::Quaternion map_to_base_rotation;
  map_to_base_rotation.setRPY(0.0, 0.0, slam_pose.theta);

  tf2::Transform map_to_base;
  map_to_base.setOrigin(tf2::Vector3(slam_pose.x, slam_pose.y, 0.0));
  map_to_base.setRotation(map_to_base_rotation);

  tf2::Quaternion odom_to_base_rotation;
  odom_to_base_rotation.setRPY(0.0, 0.0, odom_pose.theta);

  tf2::Transform odom_to_base;
  odom_to_base.setOrigin(tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));
  odom_to_base.setRotation(odom_to_base_rotation);

  tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  msg.child_frame_id = "odom";
  msg.transform = tf2::toMsg(map_to_odom);
  tf_broadcaster_->sendTransform(msg);
}

geometry_msgs::msg::Quaternion SimpleSlamNode::yawToQuaternion(double yaw) const
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleSlamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

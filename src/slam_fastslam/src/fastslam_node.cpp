#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

class FastSlamNode : public rclcpp::Node
{
public:
  FastSlamNode()
  : Node("fastslam_node")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::SensorDataQoS(),
      std::bind(&FastSlamNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&FastSlamNode::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "fastslam_node started. Subscribing to /odom and /scan.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = msg;
    odom_count_ += 1;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "odom messages=%zu latest_position=(%.3f, %.3f)",
      odom_count_,
      msg->pose.pose.position.x,
      msg->pose.pose.position.y);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = msg;
    scan_count_ += 1;
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "scan messages=%zu beams=%zu range=[%.3f, %.3f]",
      scan_count_,
      msg->ranges.size(),
      msg->range_min,
      msg->range_max);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  std::size_t odom_count_{0};
  std::size_t scan_count_{0};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastSlamNode>());
  rclcpp::shutdown();
  return 0;
}

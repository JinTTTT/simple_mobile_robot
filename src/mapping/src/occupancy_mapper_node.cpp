#include "mapping/occupancy_mapper.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <chrono>
#include <memory>

class OccupancyMapperNode : public rclcpp::Node
{
public:
  OccupancyMapperNode()
  : Node("mapping_node")
  {
    OccupancyMapper::Config config;
    config.resolution = declare_parameter<double>("resolution", 0.05);
    config.width = declare_parameter<int>("width", 500);
    config.height = declare_parameter<int>("height", 500);
    config.origin_x = declare_parameter<double>(
      "origin_x", -static_cast<double>(config.width) * config.resolution / 2.0);
    config.origin_y = declare_parameter<double>(
      "origin_y", -static_cast<double>(config.height) * config.resolution / 2.0);
    config.hit_probability = declare_parameter<double>("hit_probability", 0.90);
    config.free_probability = declare_parameter<double>("free_probability", 0.05);
    config.log_odds_min = declare_parameter<double>("log_odds_min", -10.0);
    config.log_odds_max = declare_parameter<double>("log_odds_max", 10.0);
    const int publish_period_ms = declare_parameter<int>("publish_period_ms", 500);

    mapper_.configure(config);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      10,
      std::bind(&OccupancyMapperNode::scanCallback, this, std::placeholders::_1));

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&OccupancyMapperNode::publishMap, this));

    RCLCPP_INFO(get_logger(), "Occupancy mapper started.");
    RCLCPP_INFO(
      get_logger(),
      "Map size: %d x %d cells, resolution %.2f m/cell, publish period %d ms.",
      config.width,
      config.height,
      config.resolution,
      publish_period_ms);
    RCLCPP_INFO(
      get_logger(),
      "Using TF lookup odom -> base_link and publishing /map in frame map.");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
      transform_stamped = tf_buffer_->lookupTransform(
        "odom",
        "base_link",
        msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Could not get transform: %s",
        ex.what());
      return;
    }

    const double robot_x = transform_stamped.transform.translation.x;
    const double robot_y = transform_stamped.transform.translation.y;

    tf2::Quaternion q(
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z,
      transform_stamped.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll = 0.0;
    double pitch = 0.0;
    double robot_theta = 0.0;
    m.getRPY(roll, pitch, robot_theta);

    int robot_grid_x = 0;
    int robot_grid_y = 0;
    if (!mapper_.worldToGrid(robot_x, robot_y, robot_grid_x, robot_grid_y)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Robot is outside map bounds. Skipping scan.");
      return;
    }

    mapper_.updateWithScan(*msg, robot_x, robot_y, robot_theta);
  }

  void publishMap()
  {
    map_pub_->publish(mapper_.buildOccupancyGridMsg("map", now()));
  }

  OccupancyMapper mapper_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyMapperNode>());
  rclcpp::shutdown();
  return 0;
}

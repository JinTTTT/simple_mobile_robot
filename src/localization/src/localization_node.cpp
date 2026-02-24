#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "localization/particle_filter.hpp"

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode();

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void publish_particles();

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;

    // State
    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_ = false;

    double last_odom_x_     = 0.0;
    double last_odom_y_     = 0.0;
    double last_odom_theta_ = 0.0;
    bool   odom_initialized_ = false;

    ParticleFilter pf_;
};

LocalizationNode::LocalizationNode()
: Node("localization_node"), pf_(500)   // 500 particles
{
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&LocalizationNode::map_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&LocalizationNode::odom_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&LocalizationNode::scan_callback, this, std::placeholders::_1));

    particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);

    RCLCPP_INFO(this->get_logger(), "LocalizationNode started.");
}

void LocalizationNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
    map_received_ = true;
    pf_.initUniform(map_);
    RCLCPP_INFO(this->get_logger(),
        "Map received: %d x %d cells. Particles initialised.", msg->info.width, msg->info.height);
}

void LocalizationNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!map_received_) return;  // wait for map before moving particles

    double x     = msg->pose.pose.position.x;
    double y     = msg->pose.pose.position.y;
    double theta = tf2::getYaw(msg->pose.pose.orientation);

    if (!odom_initialized_) {
        last_odom_x_     = x;
        last_odom_y_     = y;
        last_odom_theta_ = theta;
        odom_initialized_ = true;
        return;
    }

    // Only update if the robot actually moved (avoids jitter when still)
    double dx = x - last_odom_x_;
    double dy = y - last_odom_y_;
    double dtheta = theta - last_odom_theta_;
    if (std::abs(dx) < 0.001 && std::abs(dy) < 0.001 && std::abs(dtheta) < 0.001) return;

    pf_.sampleMotionModel(last_odom_x_, last_odom_y_, last_odom_theta_, x, y, theta);

    last_odom_x_     = x;
    last_odom_y_     = y;
    last_odom_theta_ = theta;

    publish_particles();
}

void LocalizationNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    (void)msg;  // not used yet in v1.0
}

void LocalizationNode::publish_particles()
{
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = "map";

    for (const auto & p : pf_.getParticles()) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        // Convert yaw angle back to quaternion
        pose.orientation.z = std::sin(p.theta / 2.0);
        pose.orientation.w = std::cos(p.theta / 2.0);
        msg.poses.push_back(pose);
    }

    particle_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
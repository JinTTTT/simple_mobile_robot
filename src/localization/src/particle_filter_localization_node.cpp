#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "localization/particle_filter.hpp"

class ParticleFilterLocalizationNode : public rclcpp::Node
{
public:
    ParticleFilterLocalizationNode();

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void publish_particles();
    void publish_estimated_pose();
    void publish_map_to_odom_tf();

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr likelihood_field_pub_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // State
    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_ = false;

    double last_odom_x_     = 0.0;
    double last_odom_y_     = 0.0;
    double last_odom_theta_ = 0.0;
    bool   odom_initialized_ = false;
    bool   particles_moved_since_last_resample_ = false;

    ParticleFilter pf_;
};

ParticleFilterLocalizationNode::ParticleFilterLocalizationNode()
: Node("particle_filter_localization_node"), pf_(500)   // 500 particles
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rclcpp::QoS static_map_qos(1);
    static_map_qos.transient_local();
    static_map_qos.reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", static_map_qos,
        std::bind(&ParticleFilterLocalizationNode::map_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ParticleFilterLocalizationNode::odom_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ParticleFilterLocalizationNode::scan_callback, this, std::placeholders::_1));

    particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);
    estimated_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);

    likelihood_field_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/likelihood_field", static_map_qos);

    RCLCPP_INFO(this->get_logger(), "ParticleFilterLocalizationNode started.");
}

void ParticleFilterLocalizationNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
    map_received_ = true;
    pf_.buildLikelihoodField(map_);
    pf_.initUniform(map_);
    likelihood_field_pub_->publish(pf_.getLikelihoodFieldMap());
    RCLCPP_INFO(this->get_logger(),
        "Map received: %d x %d cells. Likelihood field built. Particles initialised.",
        msg->info.width, msg->info.height);
}

void ParticleFilterLocalizationNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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
    particles_moved_since_last_resample_ = true;

    last_odom_x_     = x;
    last_odom_y_     = y;
    last_odom_theta_ = theta;

    publish_particles();
    publish_estimated_pose();
    publish_map_to_odom_tf();
}

void ParticleFilterLocalizationNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!map_received_) return;

    pf_.scoreParticlesWithScan(*msg);
    if (particles_moved_since_last_resample_) {
        pf_.resample();
        particles_moved_since_last_resample_ = false;
    }
    publish_particles();
    publish_estimated_pose();
    publish_map_to_odom_tf();
}

void ParticleFilterLocalizationNode::publish_particles()
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

void ParticleFilterLocalizationNode::publish_estimated_pose()
{
    EstimatedPose estimate = pf_.estimatePose();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = estimate.x;
    msg.pose.position.y = estimate.y;
    msg.pose.orientation.z = std::sin(estimate.theta / 2.0);
    msg.pose.orientation.w = std::cos(estimate.theta / 2.0);

    estimated_pose_pub_->publish(msg);
}

void ParticleFilterLocalizationNode::publish_map_to_odom_tf()
{
    EstimatedPose estimate = pf_.estimatePose();

    tf2::Quaternion map_to_base_rotation;
    map_to_base_rotation.setRPY(0.0, 0.0, estimate.theta);

    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(estimate.x, estimate.y, 0.0));
    map_to_base.setRotation(map_to_base_rotation);

    geometry_msgs::msg::TransformStamped odom_to_base_msg;
    try {
        odom_to_base_msg = tf_buffer_->lookupTransform(
            "odom",
            "base_link",
            tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Could not publish map->odom TF: %s",
            ex.what());
        return;
    }

    tf2::Transform odom_to_base;
    tf2::fromMsg(odom_to_base_msg.transform, odom_to_base);

    tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = odom_to_base_msg.header.stamp;
    map_to_odom_msg.header.frame_id = "map";
    map_to_odom_msg.child_frame_id = "odom";
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom);

    tf_broadcaster_->sendTransform(map_to_odom_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

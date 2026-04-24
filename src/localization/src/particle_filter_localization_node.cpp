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

#include <algorithm>
#include <cmath>

#include "localization/geometry_utils.hpp"
#include "localization/particle_filter.hpp"

class ParticleFilterLocalizationNode : public rclcpp::Node
{
public:
    ParticleFilterLocalizationNode();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    ParticleFilterParameters loadParticleFilterParameters();
    void publishParticles();
    void publishEstimatedPose();
    void publishMapToOdomTf();

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
    double motion_update_min_translation_ = 0.001;
    double motion_update_min_rotation_ = 0.001;

    ParticleFilter pf_;
};

ParticleFilterLocalizationNode::ParticleFilterLocalizationNode()
: Node("particle_filter_localization_node")
{
    pf_.configure(loadParticleFilterParameters());
    motion_update_min_translation_ =
        this->declare_parameter<double>("motion_update_min_translation", 0.001);
    motion_update_min_rotation_ =
        this->declare_parameter<double>("motion_update_min_rotation", 0.001);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rclcpp::QoS static_map_qos(1);
    static_map_qos.transient_local();
    static_map_qos.reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", static_map_qos,
        std::bind(&ParticleFilterLocalizationNode::mapCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ParticleFilterLocalizationNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ParticleFilterLocalizationNode::scanCallback, this, std::placeholders::_1));

    particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);
    estimated_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);

    likelihood_field_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/likelihood_field", static_map_qos);

    RCLCPP_INFO(this->get_logger(), "ParticleFilterLocalizationNode started.");
}

ParticleFilterParameters ParticleFilterLocalizationNode::loadParticleFilterParameters()
{
    ParticleFilterParameters parameters;
    parameters.num_particles =
        static_cast<int>(std::max<long>(1, this->declare_parameter<int>("num_particles", 500)));
    parameters.random_seed =
        static_cast<unsigned int>(this->declare_parameter<int>("random_seed", 42));
    parameters.likelihood_max_distance =
        std::max(0.001, this->declare_parameter<double>("likelihood_max_distance", 1.0));
    parameters.scan_beam_step =
        static_cast<std::size_t>(
            std::max<long>(1, this->declare_parameter<int>("scan_beam_step", 10)));
    parameters.translation_noise_from_translation =
        this->declare_parameter<double>("translation_noise_from_translation", 0.02);
    parameters.translation_noise_base =
        this->declare_parameter<double>("translation_noise_base", 0.005);
    parameters.rotation_noise_from_rotation =
        this->declare_parameter<double>("rotation_noise_from_rotation", 0.05);
    parameters.rotation_noise_from_translation =
        this->declare_parameter<double>("rotation_noise_from_translation", 0.01);
    parameters.rotation_noise_base =
        this->declare_parameter<double>("rotation_noise_base", 0.002);
    parameters.resample_xy_noise_std =
        this->declare_parameter<double>("resample_xy_noise_std", 0.02);
    parameters.resample_theta_noise_std =
        this->declare_parameter<double>("resample_theta_noise_std", 0.03);
    parameters.recovery_score_high =
        std::clamp(
            this->declare_parameter<double>("recovery_score_high", 0.99),
            0.0,
            1.0);
    parameters.recovery_score_medium =
        std::clamp(
            this->declare_parameter<double>("recovery_score_medium", 0.90),
            0.0,
            1.0);
    parameters.recovery_score_low =
        std::clamp(
            this->declare_parameter<double>("recovery_score_low", 0.80),
            0.0,
            1.0);
    parameters.recovery_score_min =
        std::clamp(
            this->declare_parameter<double>("recovery_score_min", 0.70),
            0.0,
            1.0);
    parameters.recovery_fraction_high =
        std::clamp(
            this->declare_parameter<double>("recovery_fraction_high", 0.0),
            0.0,
            1.0);
    parameters.recovery_fraction_medium =
        std::clamp(
            this->declare_parameter<double>("recovery_fraction_medium", 0.10),
            0.0,
            1.0);
    parameters.recovery_fraction_low =
        std::clamp(
            this->declare_parameter<double>("recovery_fraction_low", 0.30),
            0.0,
            1.0);
    parameters.recovery_fraction_min =
        std::clamp(
            this->declare_parameter<double>("recovery_fraction_min", 0.50),
            0.0,
            1.0);
    return parameters;
}

void ParticleFilterLocalizationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;
    map_received_ = true;
    pf_.buildLikelihoodField(map_);
    pf_.initializeUniform(map_);
    likelihood_field_pub_->publish(pf_.likelihoodFieldMap());
    RCLCPP_INFO(this->get_logger(),
        "Map received: %d x %d cells. Likelihood field built. Particles initialised.",
        msg->info.width, msg->info.height);
}

void ParticleFilterLocalizationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
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
    double dtheta = normalizeAngle(theta - last_odom_theta_);
    double translation = std::sqrt(dx * dx + dy * dy);
    if (translation < motion_update_min_translation_ &&
        std::abs(dtheta) < motion_update_min_rotation_) {
        return;
    }

    pf_.sampleMotionModel(last_odom_x_, last_odom_y_, last_odom_theta_, x, y, theta);
    particles_moved_since_last_resample_ = true;

    last_odom_x_     = x;
    last_odom_y_     = y;
    last_odom_theta_ = theta;

    publishParticles();
    publishEstimatedPose();
    publishMapToOdomTf();
}

void ParticleFilterLocalizationNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!map_received_) return;

    ScanScoreStats stats = pf_.scoreParticlesWithScan(*msg);
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Particle scan score: best=%.3f average=%.3f worst=%.3f",
        stats.best_score,
        stats.average_score,
        stats.worst_score);

    pf_.resample();
    particles_moved_since_last_resample_ = false;

    publishParticles();
    publishEstimatedPose();
    publishMapToOdomTf();
}

void ParticleFilterLocalizationNode::publishParticles()
{
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = "map";

    for (const auto & p : pf_.particles()) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.orientation = yawToQuaternion(p.theta);
        msg.poses.push_back(pose);
    }

    particle_pub_->publish(msg);
}

void ParticleFilterLocalizationNode::publishEstimatedPose()
{
    EstimatedPose estimate = pf_.estimatePose();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = estimate.x;
    msg.pose.position.y = estimate.y;
    msg.pose.orientation = yawToQuaternion(estimate.theta);

    estimated_pose_pub_->publish(msg);
}

void ParticleFilterLocalizationNode::publishMapToOdomTf()
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

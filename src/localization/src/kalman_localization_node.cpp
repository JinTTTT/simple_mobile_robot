#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "localization/geometry_utils.hpp"
#include "localization/kalman_filter.hpp"
#include "localization/scan_matcher.hpp"

class KalmanLocalizationNode : public rclcpp::Node
{
public:
    KalmanLocalizationNode();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    KalmanFilterParameters loadKalmanFilterParameters();
    ScanMatcherParameters loadScanMatcherParameters();
    void publishEstimatedPose();
    void publishEstimatedPoseWithCovariance();
    void publishScanMatchedPose(const ScanMatchResult & match);
    void publishMapToOdomTf(
        double odom_x, double odom_y, double odom_theta,
        const rclcpp::Time & stamp);
    bool shouldUseScanMatchCorrection(
        const ScanMatchResult & match,
        const KalmanPose & prediction) const;
    KalmanFilter::Matrix3x3 scanMatchMeasurementCovariance() const;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr scan_matched_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        estimated_pose_with_covariance_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    KalmanFilter kf_;
    ScanMatcher scan_matcher_;

    double last_odom_x_ = 0.0;
    double last_odom_y_ = 0.0;
    double last_odom_theta_ = 0.0;
    bool odom_initialized_ = false;

    double min_scan_match_score_ = 0.40;
    double max_correction_translation_ = 0.25;
    double max_correction_rotation_ = 0.25;
    double scan_match_std_x_ = 0.08;
    double scan_match_std_y_ = 0.08;
    double scan_match_std_theta_ = 0.08;
    double initial_x_ = 0.0;
    double initial_y_ = 0.0;
    double initial_theta_ = 0.0;
};

KalmanLocalizationNode::KalmanLocalizationNode()
: Node("kalman_localization_node")
{
    initial_x_ = this->declare_parameter<double>("initial_x", 0.0);
    initial_y_ = this->declare_parameter<double>("initial_y", 0.0);
    initial_theta_ = this->declare_parameter<double>("initial_theta", 0.0);
    min_scan_match_score_ = this->declare_parameter<double>("min_scan_match_score", 0.40);
    max_correction_translation_ =
        this->declare_parameter<double>("max_correction_translation", 0.25);
    max_correction_rotation_ =
        this->declare_parameter<double>("max_correction_rotation", 0.25);
    scan_match_std_x_ = this->declare_parameter<double>("scan_match_std_x", 0.08);
    scan_match_std_y_ = this->declare_parameter<double>("scan_match_std_y", 0.08);
    scan_match_std_theta_ = this->declare_parameter<double>("scan_match_std_theta", 0.08);

    kf_ = KalmanFilter(loadKalmanFilterParameters());
    kf_.initialize(initial_x_, initial_y_, initial_theta_);
    scan_matcher_ = ScanMatcher(loadScanMatcherParameters());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rclcpp::QoS static_map_qos(1);
    static_map_qos.transient_local();
    static_map_qos.reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", static_map_qos,
        std::bind(&KalmanLocalizationNode::mapCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&KalmanLocalizationNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&KalmanLocalizationNode::scanCallback, this, std::placeholders::_1));

    estimated_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
    scan_matched_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/scan_matched_pose", 10);
    estimated_pose_with_covariance_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/estimated_pose_with_covariance", 10);

    RCLCPP_INFO(
        this->get_logger(),
        "KalmanLocalizationNode started with initial pose x=%.2f y=%.2f theta=%.2f.",
        initial_x_,
        initial_y_,
        initial_theta_);
}

KalmanFilterParameters KalmanLocalizationNode::loadKalmanFilterParameters()
{
    KalmanFilterParameters parameters;
    parameters.initial_std_x = this->declare_parameter<double>("initial_std_x", 0.02);
    parameters.initial_std_y = this->declare_parameter<double>("initial_std_y", 0.02);
    parameters.initial_std_theta = this->declare_parameter<double>("initial_std_theta", 0.02);
    parameters.base_process_std_x =
        this->declare_parameter<double>("base_process_std_x", 0.005);
    parameters.base_process_std_y =
        this->declare_parameter<double>("base_process_std_y", 0.005);
    parameters.base_process_std_theta =
        this->declare_parameter<double>("base_process_std_theta", 0.002);
    parameters.distance_noise_scale =
        this->declare_parameter<double>("distance_noise_scale", 0.10);
    parameters.rotation_noise_scale =
        this->declare_parameter<double>("rotation_noise_scale", 0.10);
    parameters.min_translation_delta =
        this->declare_parameter<double>("min_translation_delta", 0.0002);
    parameters.min_rotation_delta =
        this->declare_parameter<double>("min_rotation_delta", 0.0002);
    return parameters;
}

ScanMatcherParameters KalmanLocalizationNode::loadScanMatcherParameters()
{
    ScanMatcherParameters parameters;
    parameters.search_xy_range = this->declare_parameter<double>("search_xy_range", 0.20);
    parameters.search_theta_range = this->declare_parameter<double>("search_theta_range", 0.20);
    parameters.search_xy_step =
        std::max(0.001, this->declare_parameter<double>("search_xy_step", 0.05));
    parameters.search_theta_step =
        std::max(0.001, this->declare_parameter<double>("search_theta_step", 0.05));
    parameters.beam_step =
        static_cast<std::size_t>(
            std::max<long>(1, this->declare_parameter<int>("scan_match_beam_step", 10)));
    parameters.likelihood_max_distance =
        std::max(0.001, this->declare_parameter<double>("likelihood_max_distance", 1.0));
    return parameters;
}

void KalmanLocalizationNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    scan_matcher_.setMap(*msg);
    RCLCPP_INFO(
        this->get_logger(),
        "Map received: %d x %d cells. Scan matcher likelihood field built.",
        msg->info.width,
        msg->info.height);
}

void KalmanLocalizationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = tf2::getYaw(msg->pose.pose.orientation);

    if (!odom_initialized_) {
        last_odom_x_ = x;
        last_odom_y_ = y;
        last_odom_theta_ = theta;
        odom_initialized_ = true;

        publishEstimatedPose();
        publishEstimatedPoseWithCovariance();
        publishMapToOdomTf(x, y, theta, msg->header.stamp);
        return;
    }

    kf_.predictFromOdometry(last_odom_x_, last_odom_y_, last_odom_theta_, x, y, theta);

    last_odom_x_ = x;
    last_odom_y_ = y;
    last_odom_theta_ = theta;

    publishEstimatedPose();
    publishEstimatedPoseWithCovariance();
    publishMapToOdomTf(x, y, theta, msg->header.stamp);
}

void KalmanLocalizationNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!scan_matcher_.hasMap()) {
        return;
    }

    KalmanPose estimate = kf_.estimatePose();
    ScanMatchPose initial_guess;
    initial_guess.x = estimate.x;
    initial_guess.y = estimate.y;
    initial_guess.theta = estimate.theta;

    ScanMatchResult match = scan_matcher_.match(*msg, initial_guess);
    if (!match.valid) {
        return;
    }

    publishScanMatchedPose(match);

    if (shouldUseScanMatchCorrection(match, estimate)) {
        kf_.correctWithPoseMeasurement(
            match.pose.x,
            match.pose.y,
            match.pose.theta,
            scanMatchMeasurementCovariance());

        publishEstimatedPose();
        publishEstimatedPoseWithCovariance();

        if (odom_initialized_) {
            publishMapToOdomTf(
                last_odom_x_, last_odom_y_, last_odom_theta_, msg->header.stamp);
        }
    }
}

void KalmanLocalizationNode::publishEstimatedPose()
{
    KalmanPose estimate = kf_.estimatePose();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = estimate.x;
    msg.pose.position.y = estimate.y;
    msg.pose.orientation = yawToQuaternion(estimate.theta);

    estimated_pose_pub_->publish(msg);
}

void KalmanLocalizationNode::publishEstimatedPoseWithCovariance()
{
    KalmanPose estimate = kf_.estimatePose();
    const auto & covariance = kf_.covariance();

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = estimate.x;
    msg.pose.pose.position.y = estimate.y;
    msg.pose.pose.orientation = yawToQuaternion(estimate.theta);

    msg.pose.covariance[0] = covariance[0];
    msg.pose.covariance[1] = covariance[1];
    msg.pose.covariance[5] = covariance[2];
    msg.pose.covariance[6] = covariance[3];
    msg.pose.covariance[7] = covariance[4];
    msg.pose.covariance[11] = covariance[5];
    msg.pose.covariance[30] = covariance[6];
    msg.pose.covariance[31] = covariance[7];
    msg.pose.covariance[35] = covariance[8];

    estimated_pose_with_covariance_pub_->publish(msg);
}

void KalmanLocalizationNode::publishScanMatchedPose(const ScanMatchResult & match)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = match.pose.x;
    msg.pose.position.y = match.pose.y;
    msg.pose.orientation = yawToQuaternion(match.pose.theta);

    scan_matched_pose_pub_->publish(msg);
}

void KalmanLocalizationNode::publishMapToOdomTf(
    double odom_x, double odom_y, double odom_theta,
    const rclcpp::Time & stamp)
{
    KalmanPose estimate = kf_.estimatePose();

    tf2::Quaternion map_to_base_rotation;
    map_to_base_rotation.setRPY(0.0, 0.0, estimate.theta);

    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(estimate.x, estimate.y, 0.0));
    map_to_base.setRotation(map_to_base_rotation);

    tf2::Quaternion odom_to_base_rotation;
    odom_to_base_rotation.setRPY(0.0, 0.0, odom_theta);

    tf2::Transform odom_to_base;
    odom_to_base.setOrigin(tf2::Vector3(odom_x, odom_y, 0.0));
    odom_to_base.setRotation(odom_to_base_rotation);

    tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.child_frame_id = "odom";
    msg.transform = tf2::toMsg(map_to_odom);

    tf_broadcaster_->sendTransform(msg);
}

bool KalmanLocalizationNode::shouldUseScanMatchCorrection(
    const ScanMatchResult & match,
    const KalmanPose & prediction) const
{
    if (match.score < min_scan_match_score_) {
        return false;
    }

    double dx = match.pose.x - prediction.x;
    double dy = match.pose.y - prediction.y;
    double translation_error = std::sqrt(dx * dx + dy * dy);
    double rotation_error = std::abs(normalizeAngle(match.pose.theta - prediction.theta));

    return translation_error <= max_correction_translation_ &&
        rotation_error <= max_correction_rotation_;
}

KalmanFilter::Matrix3x3 KalmanLocalizationNode::scanMatchMeasurementCovariance() const
{
    return {
        square(scan_match_std_x_), 0.0, 0.0,
        0.0, square(scan_match_std_y_), 0.0,
        0.0, 0.0, square(scan_match_std_theta_)
    };
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

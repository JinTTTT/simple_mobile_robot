#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <cmath>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "localization/kalman_filter.hpp"
#include "localization/scan_matcher.hpp"

class KalmanLocalizationNode : public rclcpp::Node
{
public:
    KalmanLocalizationNode();

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void publish_estimated_pose();
    void publish_estimated_pose_with_covariance();
    void publish_scan_matched_pose(const ScanMatchResult & match);
    void publish_map_to_odom_tf(
        double odom_x, double odom_y, double odom_theta,
        const rclcpp::Time & stamp);
    geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) const;

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
};

KalmanLocalizationNode::KalmanLocalizationNode()
: Node("kalman_localization_node")
{
    kf_.initialize(0.0, 0.0, 0.0);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rclcpp::QoS static_map_qos(1);
    static_map_qos.transient_local();
    static_map_qos.reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", static_map_qos,
        std::bind(&KalmanLocalizationNode::map_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&KalmanLocalizationNode::odom_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&KalmanLocalizationNode::scan_callback, this, std::placeholders::_1));

    estimated_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
    scan_matched_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/scan_matched_pose", 10);
    estimated_pose_with_covariance_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/estimated_pose_with_covariance", 10);

    RCLCPP_INFO(
        this->get_logger(),
        "KalmanLocalizationNode started with initial pose x=0.0 y=0.0 theta=0.0.");
}

void KalmanLocalizationNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    scan_matcher_.setMap(*msg);
    RCLCPP_INFO(
        this->get_logger(),
        "Map received: %d x %d cells. Scan matcher likelihood field built.",
        msg->info.width,
        msg->info.height);
}

void KalmanLocalizationNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = tf2::getYaw(msg->pose.pose.orientation);

    if (!odom_initialized_) {
        last_odom_x_ = x;
        last_odom_y_ = y;
        last_odom_theta_ = theta;
        odom_initialized_ = true;

        publish_estimated_pose();
        publish_estimated_pose_with_covariance();
        publish_map_to_odom_tf(x, y, theta, msg->header.stamp);
        return;
    }

    kf_.predictFromOdometry(last_odom_x_, last_odom_y_, last_odom_theta_, x, y, theta);

    last_odom_x_ = x;
    last_odom_y_ = y;
    last_odom_theta_ = theta;

    publish_estimated_pose();
    publish_estimated_pose_with_covariance();
    publish_map_to_odom_tf(x, y, theta, msg->header.stamp);
}

void KalmanLocalizationNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
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

    publish_scan_matched_pose(match);
}

void KalmanLocalizationNode::publish_estimated_pose()
{
    KalmanPose estimate = kf_.estimatePose();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = estimate.x;
    msg.pose.position.y = estimate.y;
    msg.pose.orientation = yaw_to_quaternion(estimate.theta);

    estimated_pose_pub_->publish(msg);
}

void KalmanLocalizationNode::publish_estimated_pose_with_covariance()
{
    KalmanPose estimate = kf_.estimatePose();
    const auto & covariance = kf_.covariance();

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = estimate.x;
    msg.pose.pose.position.y = estimate.y;
    msg.pose.pose.orientation = yaw_to_quaternion(estimate.theta);

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

void KalmanLocalizationNode::publish_scan_matched_pose(const ScanMatchResult & match)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = match.pose.x;
    msg.pose.position.y = match.pose.y;
    msg.pose.orientation = yaw_to_quaternion(match.pose.theta);

    scan_matched_pose_pub_->publish(msg);
}

void KalmanLocalizationNode::publish_map_to_odom_tf(
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

geometry_msgs::msg::Quaternion KalmanLocalizationNode::yaw_to_quaternion(double yaw) const
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
    auto node = std::make_shared<KalmanLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

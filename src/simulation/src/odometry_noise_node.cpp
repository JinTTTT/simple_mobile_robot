#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <random>

// Adds realistic drift to Gazebo's perfect odometry using Thrun's velocity motion model
// (Probabilistic Robotics, Ch. 5):
//
//   σ²_v     = α1·v² + α2·ω²   (linear velocity noise)
//   σ²_ω     = α3·v² + α4·ω²   (angular velocity noise)
//
// Noisy velocities are integrated to produce a drifting pose that mimics real encoders.

class OdometryNoiseNode : public rclcpp::Node
{
public:
  OdometryNoiseNode()
  : Node("odometry_noise_node"), rng_(std::random_device{}())
  {
    alpha1_ = declare_parameter<double>("alpha1", 0.1);
    alpha2_ = declare_parameter<double>("alpha2", 0.01);
    alpha3_ = declare_parameter<double>("alpha3", 0.01);
    alpha4_ = declare_parameter<double>("alpha4", 0.1);

    const auto in  = declare_parameter<std::string>("input_topic",  "/odom_raw");
    const auto out = declare_parameter<std::string>("output_topic", "/odom");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      in, 10,
      std::bind(&OdometryNoiseNode::odomCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(out, 10);

    RCLCPP_INFO(
      get_logger(),
      "Odometry noise node: %s -> %s  |  α1=%.3f α2=%.3f α3=%.3f α4=%.3f",
      in.c_str(), out.c_str(), alpha1_, alpha2_, alpha3_, alpha4_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);

    if (!initialized_) {
      noisy_x_     = msg->pose.pose.position.x;
      noisy_y_     = msg->pose.pose.position.y;
      noisy_theta_ = yawFromQuaternion(msg->pose.pose.orientation);
      last_stamp_  = stamp;
      initialized_ = true;
      return;
    }

    const double dt = (stamp - last_stamp_).seconds();
    last_stamp_ = stamp;

    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    const double v     = msg->twist.twist.linear.x;
    const double omega = msg->twist.twist.angular.z;

    // Sample noise standard deviations from the motion model
    const double sigma_v = std::sqrt(alpha1_ * v * v + alpha2_ * omega * omega);
    const double sigma_w = std::sqrt(alpha3_ * v * v + alpha4_ * omega * omega);

    const double v_noisy     = v     + std::normal_distribution<double>(0.0, sigma_v)(rng_);
    const double omega_noisy = omega + std::normal_distribution<double>(0.0, sigma_w)(rng_);

    // Euler-integrate noisy velocities into the noisy pose
    noisy_x_     += v_noisy * std::cos(noisy_theta_) * dt;
    noisy_y_     += v_noisy * std::sin(noisy_theta_) * dt;
    noisy_theta_ += omega_noisy * dt;

    // Wrap theta to [-π, π]
    noisy_theta_ = std::atan2(std::sin(noisy_theta_), std::cos(noisy_theta_));

    // Build the noisy odometry message, inheriting header / twist from the source
    auto out_msg = *msg;
    out_msg.pose.pose.position.x  = noisy_x_;
    out_msg.pose.pose.position.y  = noisy_y_;
    out_msg.pose.pose.orientation = quaternionFromYaw(noisy_theta_);

    // Per-step pose covariance (diagonal, positional noise from linear, yaw from angular)
    const double cov_pos = sigma_v * dt * sigma_v * dt;
    const double cov_yaw = sigma_w * dt * sigma_w * dt;
    out_msg.pose.covariance.fill(0.0);
    out_msg.pose.covariance[0]  = cov_pos;  // x
    out_msg.pose.covariance[7]  = cov_pos;  // y
    out_msg.pose.covariance[35] = cov_yaw;  // yaw

    odom_pub_->publish(out_msg);
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw / 2.0);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw / 2.0);
    return q;
  }

  double alpha1_, alpha2_, alpha3_, alpha4_;
  bool initialized_ = false;
  double noisy_x_ = 0.0, noisy_y_ = 0.0, noisy_theta_ = 0.0;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::mt19937 rng_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNoiseNode>());
  rclcpp::shutdown();
  return 0;
}

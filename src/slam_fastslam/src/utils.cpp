#include "slam_fastslam/utils.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace slam_fastslam
{

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
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

Pose2D odometryMsgToPose2D(const nav_msgs::msg::Odometry & msg)
{
  Pose2D pose;
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;

  const auto & orientation = msg.pose.pose.orientation;
  tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, pose.theta);
  return pose;
}

}  // namespace slam_fastslam

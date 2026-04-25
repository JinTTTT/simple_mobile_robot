#include "slam/simple_slam.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>

namespace slam
{

SimpleSlam::SimpleSlam()
{
  configure(SimpleSlamConfig());
}

void SimpleSlam::configure(const SimpleSlamConfig & config)
{
  config_ = config;
  min_update_translation_ = config_.min_update_translation;
  min_update_rotation_ = config_.min_update_rotation;

  mapper_.configure(config_);
  localization_.configure(config_);
  loop_closure_.configure(config_);

  trajectory_msg_.header.frame_id = "map";
  corrected_trajectory_msg_.header.frame_id = "map";
}

bool SimpleSlam::handleOdometry(const nav_msgs::msg::Odometry & msg)
{
  current_odom_pose_ = odomMsgToPose(msg);

  if (!odom_initialized_) {
    last_odom_pose_ = current_odom_pose_;
    odom_initialized_ = true;
    return true;
  }

  return false;
}

SlamUpdateResult SimpleSlam::handleScan(
  const sensor_msgs::msg::LaserScan & scan,
  const builtin_interfaces::msg::Time & stamp)
{
  SlamUpdateResult update;

  if (!odom_initialized_) {
    return update;
  }

  bool moved_enough = odomMovedEnough(last_odom_pose_, current_odom_pose_);
  Pose2D predicted_pose = applyOdomDeltaToSlamPose(last_odom_pose_, current_odom_pose_);
  last_odom_pose_ = current_odom_pose_;

  if (scan_received_ && !moved_enough) {
    stationary_scans_skipped_++;
    update.stationary_scan_skipped = true;
    return update;
  }

  update.scan_match = localization_.matchScan(
    scan,
    predicted_pose,
    mapper_.map(),
    scan_received_,
    scans_integrated_,
    mapper_.occupiedCellCount());
  slam_pose_ = update.scan_match.pose;

  mapper_.updateWithScan(scan, slam_pose_, stamp);
  localization_.markMapDirty();

  scan_received_ = true;
  scans_integrated_++;
  updateTrajectory(stamp);

  update.scan_integrated = true;
  update.map_updated = true;
  update.trajectory_updated = true;

  if (loop_closure_.shouldAddKeyFrame(slam_pose_)) {
    loop_closure_.addKeyFrame(scan, slam_pose_, scans_integrated_);
    update.keyframe_added = true;
    update.loop_closure = loop_closure_.detect(scans_integrated_);
    if (update.loop_closure.correction_applied) {
      updateCorrectedTrajectory(stamp);
      mapper_.rebuildCorrectedMap(loop_closure_.keyframes(), stamp);
      update.corrected_map_updated = true;
    }
  }

  updateCorrectedTrajectory(stamp);
  update.corrected_trajectory_updated = true;

  return update;
}

bool SimpleSlam::hasOdometry() const
{
  return odom_initialized_;
}

const Pose2D & SimpleSlam::slamPose() const
{
  return slam_pose_;
}

const Pose2D & SimpleSlam::currentOdomPose() const
{
  return current_odom_pose_;
}

const nav_msgs::msg::OccupancyGrid & SimpleSlam::map() const
{
  return mapper_.map();
}

const nav_msgs::msg::OccupancyGrid & SimpleSlam::correctedMap() const
{
  return mapper_.correctedMap();
}

const nav_msgs::msg::Path & SimpleSlam::trajectory() const
{
  return trajectory_msg_;
}

const nav_msgs::msg::Path & SimpleSlam::correctedTrajectory() const
{
  return corrected_trajectory_msg_;
}

int SimpleSlam::scansIntegrated() const
{
  return scans_integrated_;
}

int SimpleSlam::stationaryScansSkipped() const
{
  return stationary_scans_skipped_;
}

int SimpleSlam::scanMatchUsedCount() const
{
  return localization_.scanMatchUsedCount();
}

int SimpleSlam::loopClosureCount() const
{
  return loop_closure_.loopClosureCount();
}

int SimpleSlam::loopClosureCorrectionCount() const
{
  return loop_closure_.loopClosureCorrectionCount();
}

int SimpleSlam::occupiedCellCount() const
{
  return mapper_.occupiedCellCount();
}

std::size_t SimpleSlam::keyframeCount() const
{
  return loop_closure_.keyframeCount();
}

void SimpleSlam::updateTrajectory(const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = "map";
  pose.pose.position.x = slam_pose_.x;
  pose.pose.position.y = slam_pose_.y;
  pose.pose.orientation = yawToQuaternion(slam_pose_.theta);

  trajectory_msg_.header.stamp = stamp;
  trajectory_msg_.poses.push_back(pose);
}

void SimpleSlam::updateCorrectedTrajectory(const builtin_interfaces::msg::Time & stamp)
{
  corrected_trajectory_msg_.header.stamp = stamp;
  corrected_trajectory_msg_.poses.clear();
  corrected_trajectory_msg_.poses.reserve(loop_closure_.keyframes().size());

  for (const auto & keyframe : loop_closure_.keyframes()) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = keyframe.corrected_pose.x;
    pose.pose.position.y = keyframe.corrected_pose.y;
    pose.pose.orientation = yawToQuaternion(keyframe.corrected_pose.theta);
    corrected_trajectory_msg_.poses.push_back(pose);
  }
}

Pose2D SimpleSlam::odomMsgToPose(const nav_msgs::msg::Odometry & msg) const
{
  Pose2D pose;
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;
  pose.theta = tf2::getYaw(msg.pose.pose.orientation);
  return pose;
}

Pose2D SimpleSlam::applyOdomDeltaToSlamPose(
  const Pose2D & old_odom,
  const Pose2D & new_odom) const
{
  double odom_dx = new_odom.x - old_odom.x;
  double odom_dy = new_odom.y - old_odom.y;
  double old_odom_theta = old_odom.theta;

  double local_dx =
    std::cos(old_odom_theta) * odom_dx + std::sin(old_odom_theta) * odom_dy;
  double local_dy =
    -std::sin(old_odom_theta) * odom_dx + std::cos(old_odom_theta) * odom_dy;
  double local_dtheta = normalizeAngle(new_odom.theta - old_odom.theta);

  Pose2D predicted = slam_pose_;
  predicted.x += std::cos(slam_pose_.theta) * local_dx -
    std::sin(slam_pose_.theta) * local_dy;
  predicted.y += std::sin(slam_pose_.theta) * local_dx +
    std::cos(slam_pose_.theta) * local_dy;
  predicted.theta = normalizeAngle(slam_pose_.theta + local_dtheta);
  return predicted;
}

bool SimpleSlam::odomMovedEnough(const Pose2D & old_odom, const Pose2D & new_odom) const
{
  double dx = new_odom.x - old_odom.x;
  double dy = new_odom.y - old_odom.y;
  double translation = std::sqrt(dx * dx + dy * dy);
  double rotation = std::abs(normalizeAngle(new_odom.theta - old_odom.theta));

  return translation >= min_update_translation_ || rotation >= min_update_rotation_;
}

geometry_msgs::msg::Quaternion SimpleSlam::yawToQuaternion(double yaw) const
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

double SimpleSlam::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace slam

#ifndef SLAM__SIMPLE_SLAM_HPP_
#define SLAM__SIMPLE_SLAM_HPP_

#include "slam/loop_closure.hpp"
#include "slam/simple_slam_types.hpp"
#include "slam/slam_localization.hpp"
#include "slam/slam_mapper.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cstddef>

namespace slam
{

class SimpleSlam
{
public:
  SimpleSlam();

  void configure(const SimpleSlamConfig & config);

  bool handleOdometry(const nav_msgs::msg::Odometry & msg);
  SlamUpdateResult handleScan(
    const sensor_msgs::msg::LaserScan & scan,
    const builtin_interfaces::msg::Time & stamp);

  bool hasOdometry() const;
  const Pose2D & slamPose() const;
  const Pose2D & currentOdomPose() const;

  const nav_msgs::msg::OccupancyGrid & map() const;
  const nav_msgs::msg::OccupancyGrid & correctedMap() const;
  const nav_msgs::msg::Path & trajectory() const;
  const nav_msgs::msg::Path & correctedTrajectory() const;

  int scansIntegrated() const;
  int stationaryScansSkipped() const;
  int scanMatchUsedCount() const;
  int loopClosureCount() const;
  int loopClosureCorrectionCount() const;
  int occupiedCellCount() const;
  std::size_t keyframeCount() const;

private:
  void updateTrajectory(const builtin_interfaces::msg::Time & stamp);
  void updateCorrectedTrajectory(const builtin_interfaces::msg::Time & stamp);

  Pose2D odomMsgToPose(const nav_msgs::msg::Odometry & msg) const;
  Pose2D applyOdomDeltaToSlamPose(const Pose2D & old_odom, const Pose2D & new_odom) const;
  bool odomMovedEnough(const Pose2D & old_odom, const Pose2D & new_odom) const;
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const;
  double normalizeAngle(double angle) const;

  SimpleSlamConfig config_;
  SlamMapper mapper_;
  SlamLocalization localization_;
  LoopClosure loop_closure_;

  nav_msgs::msg::Path trajectory_msg_;
  nav_msgs::msg::Path corrected_trajectory_msg_;

  Pose2D slam_pose_;
  Pose2D last_odom_pose_;
  Pose2D current_odom_pose_;
  bool odom_initialized_ = false;
  bool scan_received_ = false;

  double min_update_translation_ = 0.01;
  double min_update_rotation_ = 0.01;

  int scans_integrated_ = 0;
  int stationary_scans_skipped_ = 0;
};

}  // namespace slam

#endif  // SLAM__SIMPLE_SLAM_HPP_

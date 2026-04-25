#include "slam/slam_mapper.hpp"

#include <cmath>

namespace slam
{

void SlamMapper::configure(const SimpleSlamConfig & config)
{
  const OccupancyMapper::Config mapper_config = makeMapperConfig(config);
  live_mapper_.configure(mapper_config);
  corrected_mapper_.configure(mapper_config);

  map_msg_ = live_mapper_.buildOccupancyGridMsg("map", builtin_interfaces::msg::Time());
  corrected_map_msg_ = corrected_mapper_.buildOccupancyGridMsg("map", builtin_interfaces::msg::Time());
  occupied_cell_count_ = 0;
}

void SlamMapper::updateWithScan(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & pose,
  const builtin_interfaces::msg::Time & stamp)
{
  live_mapper_.updateWithScan(scan, pose.x, pose.y, pose.theta);
  refreshLiveMap(stamp);
}

void SlamMapper::rebuildCorrectedMap(
  const std::vector<KeyFrame> & keyframes,
  const builtin_interfaces::msg::Time & stamp)
{
  corrected_mapper_.clear();

  for (const auto & keyframe : keyframes) {
    corrected_mapper_.updateWithScanData(
      toScanData(keyframe.scan),
      keyframe.corrected_pose.x,
      keyframe.corrected_pose.y,
      keyframe.corrected_pose.theta);
  }

  corrected_map_msg_ = corrected_mapper_.buildOccupancyGridMsg("map", stamp);
}

const nav_msgs::msg::OccupancyGrid & SlamMapper::map() const
{
  return map_msg_;
}

const nav_msgs::msg::OccupancyGrid & SlamMapper::correctedMap() const
{
  return corrected_map_msg_;
}

int SlamMapper::occupiedCellCount() const
{
  return occupied_cell_count_;
}

OccupancyMapper::Config SlamMapper::makeMapperConfig(const SimpleSlamConfig & config) const
{
  OccupancyMapper::Config mapper_config;
  mapper_config.resolution = config.resolution;
  mapper_config.width = config.width;
  mapper_config.height = config.height;
  mapper_config.origin_x = config.origin_x;
  mapper_config.origin_y = config.origin_y;
  mapper_config.hit_probability = 1.0 / (1.0 + std::exp(-config.log_odds_hit));
  mapper_config.free_probability = 1.0 / (1.0 + std::exp(-config.log_odds_free));
  mapper_config.log_odds_min = config.log_odds_min;
  mapper_config.log_odds_max = config.log_odds_max;
  mapper_config.publish_unknown_for_unobserved = true;
  return mapper_config;
}

OccupancyMapper::ScanData SlamMapper::toScanData(const StoredScan & scan) const
{
  OccupancyMapper::ScanData scan_data;
  scan_data.ranges = scan.ranges;
  scan_data.angle_min = scan.angle_min;
  scan_data.angle_increment = scan.angle_increment;
  scan_data.range_min = scan.range_min;
  scan_data.range_max = scan.range_max;
  return scan_data;
}

void SlamMapper::refreshLiveMap(const builtin_interfaces::msg::Time & stamp)
{
  map_msg_ = live_mapper_.buildOccupancyGridMsg("map", stamp);
  occupied_cell_count_ = 0;

  for (const auto cell_value : map_msg_.data) {
    if (cell_value > 50) {
      occupied_cell_count_++;
    }
  }
}

}  // namespace slam

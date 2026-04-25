#ifndef SLAM__SLAM_MAPPER_HPP_
#define SLAM__SLAM_MAPPER_HPP_

#include "mapping/occupancy_mapper.hpp"
#include "slam/simple_slam_types.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>

namespace slam
{

class SlamMapper
{
public:
  void configure(const SimpleSlamConfig & config);

  void updateWithScan(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & pose,
    const builtin_interfaces::msg::Time & stamp);
  void rebuildCorrectedMap(
    const std::vector<KeyFrame> & keyframes,
    const builtin_interfaces::msg::Time & stamp);

  const nav_msgs::msg::OccupancyGrid & map() const;
  const nav_msgs::msg::OccupancyGrid & correctedMap() const;
  int occupiedCellCount() const;

private:
  OccupancyMapper::Config makeMapperConfig(const SimpleSlamConfig & config) const;
  OccupancyMapper::ScanData toScanData(const StoredScan & scan) const;
  void refreshLiveMap(const builtin_interfaces::msg::Time & stamp);

  OccupancyMapper live_mapper_;
  OccupancyMapper corrected_mapper_;
  nav_msgs::msg::OccupancyGrid map_msg_;
  nav_msgs::msg::OccupancyGrid corrected_map_msg_;
  int occupied_cell_count_ = 0;
};

}  // namespace slam

#endif  // SLAM__SLAM_MAPPER_HPP_

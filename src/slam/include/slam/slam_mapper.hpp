#ifndef SLAM__SLAM_MAPPER_HPP_
#define SLAM__SLAM_MAPPER_HPP_

#include "slam/simple_slam_types.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <utility>
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
  void initializeMap();
  void insertScanIntoLogOddsMap(
    const StoredScan & scan,
    const Pose2D & pose,
    std::vector<double> & log_odds_map);
  void updateMapMessage(const builtin_interfaces::msg::Time & stamp);
  void updateCorrectedMapMessage(const builtin_interfaces::msg::Time & stamp);
  StoredScan makeStoredScan(const sensor_msgs::msg::LaserScan & scan) const;

  void worldToGrid(double wx, double wy, int & gx, int & gy) const;
  bool isValidCell(int x, int y) const;
  std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) const;
  double clamp(double value, double low, double high) const;

  double resolution_ = 0.05;
  int width_ = 500;
  int height_ = 500;
  double origin_x_ = -12.5;
  double origin_y_ = -12.5;

  double log_odds_hit_ = 2.89;
  double log_odds_free_ = -2.25;
  double log_odds_min_ = -10.0;
  double log_odds_max_ = 10.0;

  nav_msgs::msg::OccupancyGrid map_msg_;
  nav_msgs::msg::OccupancyGrid corrected_map_msg_;
  std::vector<double> map_log_odds_;
  std::vector<double> corrected_map_log_odds_;
  int occupied_cell_count_ = 0;
};

}  // namespace slam

#endif  // SLAM__SLAM_MAPPER_HPP_

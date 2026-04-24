#ifndef MAPPING__OCCUPANCY_MAPPER_HPP_
#define MAPPING__OCCUPANCY_MAPPER_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <string>

#include <cstdint>
#include <utility>
#include <vector>

class OccupancyMapper
{
public:
  struct Config
  {
    double resolution = 0.05;
    int width = 500;
    int height = 500;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double hit_probability = 0.90;
    double free_probability = 0.05;
    double log_odds_min = -10.0;
    double log_odds_max = 10.0;
  };

  void configure(const Config & config);

  bool worldToGrid(double wx, double wy, int & gx, int & gy) const;
  bool isValidCell(int x, int y) const;
  void updateWithScan(
    const sensor_msgs::msg::LaserScan & scan,
    double robot_x,
    double robot_y,
    double robot_theta);
  nav_msgs::msg::OccupancyGrid buildOccupancyGridMsg(
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const;

private:
  std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) const;
  int gridToIndex(int x, int y) const;
  double clamp(double value, double low, double high) const;

  Config config_;
  double log_odds_hit_ = 0.0;
  double log_odds_pass_ = 0.0;
  std::vector<double> map_log_odds_;
};

#endif  // MAPPING__OCCUPANCY_MAPPER_HPP_

#pragma once

#include <cstddef>
#include <vector>

#include "mapping/occupancy_mapper.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace slam_fastslam
{

class LikelihoodField
{
public:
  void build(const OccupancyMapper & mapper, double max_distance_m, double sigma_m);

  void incrementalUpdate(
    const OccupancyMapper & mapper,
    double max_distance_m,
    double sigma_m,
    const std::vector<int> & newly_occupied,
    const std::vector<int> & newly_freed,
    std::size_t freed_cells_rebuild_threshold);

  bool hasMap() const;
  double valueAtWorld(double x, double y) const;

private:
  nav_msgs::msg::OccupancyGrid field_map_{};
  std::vector<int> distance_cells_{};
  int width_{0};
  int height_{0};
  int max_distance_cells_{0};
  double resolution_{0.0};
  double two_sigma_sq_{0.0};
};

}  // namespace slam_fastslam

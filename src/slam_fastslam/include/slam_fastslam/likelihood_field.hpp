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
  void buildFromOccupancyMap(
    const OccupancyMapper & mapper,
    double max_distance_m, // distance range that a obstacle can influence
    double sigma_m); // how quickly the likelihood decreases with distancce increase

  void updateFromMapChanges(
    const OccupancyMapper & mapper,
    double max_distance_m,
    double sigma_m,
    const std::vector<int> & cells_changed_to_occupied,
    const std::vector<int> & cells_changed_to_free,
    std::size_t freed_cells_rebuild_threshold);

  bool hasMap() const;
  double likelihoodAtWorld(double x, double y) const;

private:
  nav_msgs::msg::OccupancyGrid likelihood_grid_{};
  std::vector<int> distance_to_nearest_obstacle_cells_{};
  int width_{0};
  int height_{0};
  int max_distance_to_obstacle_cells_{0};
  double resolution_{0.0};
  double two_sigma_squared_{0.0}; // sigma^2 * 2 precomputed for efficiency
};

}  // namespace slam_fastslam

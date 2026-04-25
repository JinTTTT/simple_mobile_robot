#ifndef MOTION_PLANNING__A_STAR_PLANNER_HPP_
#define MOTION_PLANNING__A_STAR_PLANNER_HPP_

#include "motion_planning/path_types.hpp"

#include <functional>

class AStarPlanner
{
public:
  GridPath plan(
    const GridCell & start_cell,
    const GridCell & goal_cell,
    int grid_width,
    int grid_height,
    const std::function<bool(const GridCell &)> & is_cell_free) const;

private:
  struct OpenSetEntry
  {
    int index = -1;
    double estimated_total_cost = 0.0;
  };

  struct CompareOpenSetEntry
  {
    bool operator()(const OpenSetEntry & left, const OpenSetEntry & right) const;
  };

  GridPath reconstructPath(
    const std::vector<int> & predecessor_by_index,
    int goal_index,
    int grid_width) const;
  int gridToIndex(const GridCell & cell, int grid_width) const;
  GridCell indexToGridCell(int index, int grid_width) const;
  double computeHeuristicCost(const GridCell & from_cell, const GridCell & to_cell) const;
};

#endif  // MOTION_PLANNING__A_STAR_PLANNER_HPP_

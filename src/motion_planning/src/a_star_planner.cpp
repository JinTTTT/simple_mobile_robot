#include "motion_planning/a_star_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

bool AStarPlanner::CompareOpenSetEntry::operator()(
  const OpenSetEntry & left,
  const OpenSetEntry & right) const
{
  return left.estimated_total_cost > right.estimated_total_cost;
}

GridPath AStarPlanner::plan(
  const GridCell & start_cell,
  const GridCell & goal_cell,
  int grid_width,
  int grid_height,
  const std::function<bool(const GridCell &)> & is_cell_free) const
{
  const int total_cells = grid_width * grid_height;
  const int start_index = gridToIndex(start_cell, grid_width);
  const int goal_index = gridToIndex(goal_cell, grid_width);

  std::vector<double> cost_from_start(total_cells, std::numeric_limits<double>::infinity());
  std::vector<int> predecessor_by_index(total_cells, -1);
  std::vector<bool> closed_set(total_cells, false);

  std::priority_queue<OpenSetEntry, std::vector<OpenSetEntry>, CompareOpenSetEntry> open_set;

  cost_from_start[start_index] = 0.0;
  open_set.push({start_index, computeHeuristicCost(start_cell, goal_cell)});

  const std::vector<GridCell> neighbor_offsets = {
    {-1, -1}, {0, -1}, {1, -1},
    {-1, 0}, {1, 0},
    {-1, 1}, {0, 1}, {1, 1}
  };

  while (!open_set.empty()) {
    const OpenSetEntry current_entry = open_set.top();
    open_set.pop();

    const int current_index = current_entry.index;
    if (closed_set[current_index]) {
      continue;
    }
    closed_set[current_index] = true;

    if (current_index == goal_index) {
      return reconstructPath(predecessor_by_index, goal_index, grid_width);
    }

    const GridCell current_cell = indexToGridCell(current_index, grid_width);

    for (const auto & offset : neighbor_offsets) {
      const GridCell next_cell{current_cell.x + offset.x, current_cell.y + offset.y};

      if (next_cell.x < 0 || next_cell.y < 0 || next_cell.x >= grid_width ||
        next_cell.y >= grid_height)
      {
        continue;
      }

      if (!is_cell_free(next_cell)) {
        continue;
      }

      const int next_index = gridToIndex(next_cell, grid_width);
      if (closed_set[next_index]) {
        continue;
      }

      const bool is_diagonal_move = offset.x != 0 && offset.y != 0;
      const double step_cost = is_diagonal_move ? std::sqrt(2.0) : 1.0;
      const double tentative_cost_from_start = cost_from_start[current_index] + step_cost;

      if (tentative_cost_from_start >= cost_from_start[next_index]) {
        continue;
      }

      predecessor_by_index[next_index] = current_index;
      cost_from_start[next_index] = tentative_cost_from_start;
      const double estimated_total_cost =
        tentative_cost_from_start + computeHeuristicCost(next_cell, goal_cell);
      open_set.push({next_index, estimated_total_cost});
    }
  }

  return {};
}

GridPath AStarPlanner::reconstructPath(
  const std::vector<int> & predecessor_by_index,
  int goal_index,
  int grid_width) const
{
  GridPath path;
  int current_index = goal_index;

  while (current_index != -1) {
    path.push_back(indexToGridCell(current_index, grid_width));
    current_index = predecessor_by_index[current_index];
  }

  std::reverse(path.begin(), path.end());
  return path;
}

int AStarPlanner::gridToIndex(const GridCell & cell, int grid_width) const
{
  return cell.y * grid_width + cell.x;
}

GridCell AStarPlanner::indexToGridCell(int index, int grid_width) const
{
  return GridCell{index % grid_width, index / grid_width};
}

double AStarPlanner::computeHeuristicCost(
  const GridCell & from_cell,
  const GridCell & to_cell) const
{
  const double delta_x = static_cast<double>(to_cell.x - from_cell.x);
  const double delta_y = static_cast<double>(to_cell.y - from_cell.y);
  return std::sqrt(delta_x * delta_x + delta_y * delta_y);
}

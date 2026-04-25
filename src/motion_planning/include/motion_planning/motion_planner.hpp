#ifndef MOTION_PLANNING__MOTION_PLANNER_HPP_
#define MOTION_PLANNING__MOTION_PLANNER_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include <string>
#include <utility>
#include <vector>

struct MotionPlannerConfig
{
  int occupied_threshold = 50;
  double robot_radius_m = 0.35;
  bool enable_line_of_sight_path_smoothing = true;
  bool enable_cubic_spline_smoothing = true;
  double path_sample_spacing_m = 0.05;
};

struct MotionPlanningResult
{
  bool success = false;
  std::string status_message;
  nav_msgs::msg::Path shortcut_path;
  nav_msgs::msg::Path final_path;
  bool used_spline_fallback = false;
  std::size_t geometry_pose_count = 0;
  std::size_t raw_grid_pose_count = 0;
  int start_grid_x = 0;
  int start_grid_y = 0;
  int goal_grid_x = 0;
  int goal_grid_y = 0;
};

class MotionPlanner
{
public:
  void configure(const MotionPlannerConfig & config);

  void setMap(
    const nav_msgs::msg::OccupancyGrid & map,
    const builtin_interfaces::msg::Time & stamp);

  bool hasMap() const;

  const nav_msgs::msg::OccupancyGrid & inflatedMap() const;

  int inflationRadiusCells() const;

  MotionPlanningResult plan(
    const geometry_msgs::msg::PoseStamped & start_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const builtin_interfaces::msg::Time & stamp) const;

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

  struct PathPoint
  {
    double x = 0.0;
    double y = 0.0;
  };

  std::vector<int> runAStarSearch(int start_x, int start_y, int goal_x, int goal_y) const;
  std::vector<int> reconstructCellPath(
    const std::vector<int> & predecessor_by_index,
    int goal_index) const;
  std::vector<int> simplifyPathByLineOfSight(const std::vector<int> & raw_path_indices) const;
  bool hasLineOfSightBetweenCells(int start_index, int end_index) const;
  std::vector<std::pair<int, int>> traceLineCells(int x0, int y0, int x1, int y1) const;

  nav_msgs::msg::Path createPathMessageFromGridPath(
    const std::vector<int> & path_indices,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const builtin_interfaces::msg::Time & stamp) const;
  nav_msgs::msg::Path createSplineSmoothedPath(
    const nav_msgs::msg::Path & base_path,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    bool & used_collision_fallback) const;
  nav_msgs::msg::Path resamplePathAtFixedSpacing(
    const nav_msgs::msg::Path & input_path,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    double sample_spacing) const;

  double computeGridPathYaw(
    const std::vector<int> & path_indices,
    std::size_t path_index) const;
  double computePathLength(const nav_msgs::msg::Path & path) const;
  std::vector<double> buildArcLengthParameter(const std::vector<PathPoint> & points) const;
  std::vector<double> extractXPathValues(const std::vector<PathPoint> & points) const;
  std::vector<double> extractYPathValues(const std::vector<PathPoint> & points) const;
  std::vector<double> solveNaturalCubicSecondDerivatives(
    const std::vector<double> & parameter_values,
    const std::vector<double> & sample_values) const;
  double evaluateNaturalCubicSpline(
    const std::vector<double> & parameter_values,
    const std::vector<double> & sample_values,
    const std::vector<double> & second_derivatives,
    double query_value) const;

  bool worldToGrid(double world_x, double world_y, int & grid_x, int & grid_y) const;
  void gridToWorld(int grid_x, int grid_y, double & world_x, double & world_y) const;
  bool isValidCell(int grid_x, int grid_y) const;
  bool isCellFreeForPlanning(int grid_x, int grid_y) const;
  int gridToIndex(int grid_x, int grid_y) const;
  double computeHeuristicCost(int x0, int y0, int x1, int y1) const;

  void rebuildInflatedMap(const builtin_interfaces::msg::Time & stamp);
  bool isValidInflationTarget(int grid_x, int grid_y, int width, int height) const;

  MotionPlannerConfig config_;
  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::OccupancyGrid inflated_map_;
  bool has_map_ = false;
  int inflation_radius_cells_ = 0;
};

#endif  // MOTION_PLANNING__MOTION_PLANNER_HPP_

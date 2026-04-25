#ifndef MOTION_PLANNING__MOTION_PLANNER_HPP_
#define MOTION_PLANNING__MOTION_PLANNER_HPP_

#include "motion_planning/a_star_planner.hpp"
#include "motion_planning/path_resampler.hpp"
#include "motion_planning/path_shortcutter.hpp"
#include "motion_planning/path_types.hpp"
#include "motion_planning/spline_path_smoother.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include <string>

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
  PointPath convertGridPathToWorldPath(const GridPath & grid_path) const;
  nav_msgs::msg::Path createPathMessageFromWorldPath(
    const PointPath & world_path,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    const builtin_interfaces::msg::Time & stamp) const;
  double computePathYaw(const PointPath & world_path, std::size_t path_index) const;

  bool worldToGrid(double world_x, double world_y, int & grid_x, int & grid_y) const;
  bool worldToGridCell(const PathPoint & world_point, GridCell & grid_cell) const;
  void gridToWorld(int grid_x, int grid_y, double & world_x, double & world_y) const;
  PathPoint gridToWorldPoint(const GridCell & grid_cell) const;
  bool isValidCell(int grid_x, int grid_y) const;
  bool isValidCell(const GridCell & grid_cell) const;
  bool isCellFreeForPlanning(int grid_x, int grid_y) const;
  bool isCellFreeForPlanning(const GridCell & grid_cell) const;
  int gridToIndex(int grid_x, int grid_y) const;
  bool isPointInFreeSpace(const PathPoint & world_point) const;
  bool hasLineOfSightBetweenCells(const GridCell & start_cell, const GridCell & end_cell) const;
  GridPath traceLineCells(const GridCell & start_cell, const GridCell & end_cell) const;

  void rebuildInflatedMap(const builtin_interfaces::msg::Time & stamp);
  bool isValidInflationTarget(int grid_x, int grid_y, int width, int height) const;

  AStarPlanner a_star_planner_;
  PathShortcutter path_shortcutter_;
  SplinePathSmoother spline_path_smoother_;
  PathResampler path_resampler_;
  MotionPlannerConfig config_;
  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::OccupancyGrid inflated_map_;
  bool has_map_ = false;
  int inflation_radius_cells_ = 0;
};

#endif  // MOTION_PLANNING__MOTION_PLANNER_HPP_

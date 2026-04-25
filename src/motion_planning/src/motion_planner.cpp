#include "motion_planning/motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

void MotionPlanner::configure(const MotionPlannerConfig & config)
{
  config_ = config;
}

void MotionPlanner::setMap(
  const nav_msgs::msg::OccupancyGrid & map,
  const builtin_interfaces::msg::Time & stamp)
{
  map_ = map;
  has_map_ = true;
  rebuildInflatedMap(stamp);
}

bool MotionPlanner::hasMap() const
{
  return has_map_;
}

const nav_msgs::msg::OccupancyGrid & MotionPlanner::inflatedMap() const
{
  return inflated_map_;
}

int MotionPlanner::inflationRadiusCells() const
{
  return inflation_radius_cells_;
}

MotionPlanningResult MotionPlanner::plan(
  const geometry_msgs::msg::PoseStamped & start_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const builtin_interfaces::msg::Time & stamp) const
{
  MotionPlanningResult result;

  if (!has_map_) {
    result.status_message = "Planner has no map yet.";
    return result;
  }

  GridCell start_cell;
  if (!worldToGrid(
      start_pose.pose.position.x, start_pose.pose.position.y,
      start_cell.x, start_cell.y))
  {
    result.status_message = "Start pose is outside the map bounds.";
    return result;
  }

  GridCell goal_cell;
  if (!worldToGrid(
      goal_pose.pose.position.x, goal_pose.pose.position.y,
      goal_cell.x, goal_cell.y))
  {
    result.status_message = "Goal pose is outside the map bounds.";
    return result;
  }

  result.start_grid_x = start_cell.x;
  result.start_grid_y = start_cell.y;
  result.goal_grid_x = goal_cell.x;
  result.goal_grid_y = goal_cell.y;

  if (!isCellFreeForPlanning(start_cell)) {
    result.status_message = "Start cell is occupied, unknown, or inside inflated clearance.";
    return result;
  }

  if (!isCellFreeForPlanning(goal_cell)) {
    result.status_message = "Goal cell is occupied, unknown, or inside inflated clearance.";
    return result;
  }

  const GridPath raw_grid_path = a_star_planner_.plan(
    start_cell,
    goal_cell,
    static_cast<int>(inflated_map_.info.width),
    static_cast<int>(inflated_map_.info.height),
    [this](const GridCell & cell) {return isCellFreeForPlanning(cell);});
  if (raw_grid_path.empty()) {
    result.status_message = "A* could not find a path.";
    return result;
  }

  const GridPath shortcut_grid_path =
    config_.enable_line_of_sight_path_smoothing ?
    path_shortcutter_.shortcut(
    raw_grid_path,
    [this](const GridCell & from_cell, const GridCell & to_cell) {
      return hasLineOfSightBetweenCells(from_cell, to_cell);
    }) :
    raw_grid_path;

  const PointPath shortcut_world_path = convertGridPathToWorldPath(shortcut_grid_path);
  result.shortcut_path = createPathMessageFromWorldPath(shortcut_world_path, goal_pose, stamp);

  const SplineSmoothingResult smoothing_result =
    config_.enable_cubic_spline_smoothing ?
    spline_path_smoother_.smooth(
    shortcut_world_path,
    config_.path_sample_spacing_m,
    map_.info.resolution,
    [this](const PathPoint & point) {return isPointInFreeSpace(point);}) :
    SplineSmoothingResult{shortcut_world_path, false};

  const PointPath resampled_world_path = path_resampler_.resampleAtFixedSpacing(
    smoothing_result.path, config_.path_sample_spacing_m);
  result.final_path = createPathMessageFromWorldPath(resampled_world_path, goal_pose, stamp);
  result.used_spline_fallback = smoothing_result.used_collision_fallback;
  result.geometry_pose_count = smoothing_result.path.size();
  result.raw_grid_pose_count = raw_grid_path.size();
  result.success = true;
  return result;
}

PointPath MotionPlanner::convertGridPathToWorldPath(const GridPath & grid_path) const
{
  PointPath world_path;
  world_path.reserve(grid_path.size());
  for (const auto & grid_cell : grid_path) {
    world_path.push_back(gridToWorldPoint(grid_cell));
  }

  return world_path;
}

nav_msgs::msg::Path MotionPlanner::createPathMessageFromWorldPath(
  const PointPath & world_path,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const builtin_interfaces::msg::Time & stamp) const
{
  nav_msgs::msg::Path path_message;
  path_message.header.stamp = stamp;
  path_message.header.frame_id = "map";
  path_message.poses.reserve(world_path.size());

  for (std::size_t path_index = 0; path_index < world_path.size(); ++path_index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_message.header;
    pose.pose.position.x = world_path[path_index].x;
    pose.pose.position.y = world_path[path_index].y;
    pose.pose.position.z = 0.0;

    if (path_index == world_path.size() - 1) {
      pose.pose.orientation = goal_pose.pose.orientation;
    } else {
      const double yaw = computePathYaw(world_path, path_index);
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
    }

    path_message.poses.push_back(pose);
  }

  return path_message;
}

double MotionPlanner::computePathYaw(const PointPath & world_path, std::size_t path_index) const
{
  if (world_path.size() < 2) {
    return 0.0;
  }

  const std::size_t next_path_index = std::min(path_index + 1, world_path.size() - 1);
  const std::size_t prev_path_index = (path_index == 0) ? 0 : path_index - 1;

  return std::atan2(
    world_path[next_path_index].y - world_path[prev_path_index].y,
    world_path[next_path_index].x - world_path[prev_path_index].x);
}

bool MotionPlanner::worldToGrid(double world_x, double world_y, int & grid_x, int & grid_y) const
{
  const double resolution = map_.info.resolution;
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;

  grid_x = static_cast<int>(std::floor((world_x - origin_x) / resolution));
  grid_y = static_cast<int>(std::floor((world_y - origin_y) / resolution));
  return isValidCell(grid_x, grid_y);
}

bool MotionPlanner::worldToGridCell(const PathPoint & world_point, GridCell & grid_cell) const
{
  return worldToGrid(world_point.x, world_point.y, grid_cell.x, grid_cell.y);
}

void MotionPlanner::gridToWorld(int grid_x, int grid_y, double & world_x, double & world_y) const
{
  const double resolution = map_.info.resolution;
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;

  world_x = origin_x + (static_cast<double>(grid_x) + 0.5) * resolution;
  world_y = origin_y + (static_cast<double>(grid_y) + 0.5) * resolution;
}

PathPoint MotionPlanner::gridToWorldPoint(const GridCell & grid_cell) const
{
  PathPoint world_point;
  gridToWorld(grid_cell.x, grid_cell.y, world_point.x, world_point.y);
  return world_point;
}

bool MotionPlanner::isValidCell(int grid_x, int grid_y) const
{
  return grid_x >= 0 &&
         grid_y >= 0 &&
         grid_x < static_cast<int>(inflated_map_.info.width) &&
         grid_y < static_cast<int>(inflated_map_.info.height);
}

bool MotionPlanner::isValidCell(const GridCell & grid_cell) const
{
  return isValidCell(grid_cell.x, grid_cell.y);
}

bool MotionPlanner::isCellFreeForPlanning(int grid_x, int grid_y) const
{
  const int8_t cell_value = inflated_map_.data[gridToIndex(grid_x, grid_y)];
  return cell_value >= 0 && cell_value < config_.occupied_threshold;
}

bool MotionPlanner::isCellFreeForPlanning(const GridCell & grid_cell) const
{
  return isValidCell(grid_cell) && isCellFreeForPlanning(grid_cell.x, grid_cell.y);
}

int MotionPlanner::gridToIndex(int grid_x, int grid_y) const
{
  return grid_y * static_cast<int>(map_.info.width) + grid_x;
}

bool MotionPlanner::isPointInFreeSpace(const PathPoint & world_point) const
{
  GridCell grid_cell;
  return worldToGridCell(world_point, grid_cell) && isCellFreeForPlanning(grid_cell);
}

bool MotionPlanner::hasLineOfSightBetweenCells(
  const GridCell & start_cell,
  const GridCell & end_cell) const
{
  const GridPath cells_on_line = traceLineCells(start_cell, end_cell);
  for (const auto & cell : cells_on_line) {
    if (!isCellFreeForPlanning(cell)) {
      return false;
    }
  }

  return true;
}

GridPath MotionPlanner::traceLineCells(const GridCell & start_cell, const GridCell & end_cell) const
{
  GridPath cells_on_line;

  int delta_x = std::abs(end_cell.x - start_cell.x);
  int delta_y = std::abs(end_cell.y - start_cell.y);
  int step_x = (start_cell.x < end_cell.x) ? 1 : -1;
  int step_y = (start_cell.y < end_cell.y) ? 1 : -1;
  int error = delta_x - delta_y;

  GridCell current_cell = start_cell;

  while (true) {
    cells_on_line.push_back(current_cell);
    if (current_cell.x == end_cell.x && current_cell.y == end_cell.y) {
      break;
    }

    const int doubled_error = 2 * error;
    if (doubled_error > -delta_y) {
      error -= delta_y;
      current_cell.x += step_x;
    }
    if (doubled_error < delta_x) {
      error += delta_x;
      current_cell.y += step_y;
    }
  }

  return cells_on_line;
}

void MotionPlanner::rebuildInflatedMap(const builtin_interfaces::msg::Time & stamp)
{
  inflated_map_ = map_;

  const double resolution = map_.info.resolution;
  inflation_radius_cells_ = static_cast<int>(std::ceil(config_.robot_radius_m / resolution));

  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const int total_cells = width * height;

  for (int cell_index = 0; cell_index < total_cells; ++cell_index) {
    if (map_.data[cell_index] >= config_.occupied_threshold) {
      inflated_map_.data[cell_index] = 100;
      continue;
    }

    if (map_.data[cell_index] < 0) {
      inflated_map_.data[cell_index] = -1;
    } else {
      inflated_map_.data[cell_index] = 0;
    }
  }

  for (int obstacle_y = 0; obstacle_y < height; ++obstacle_y) {
    for (int obstacle_x = 0; obstacle_x < width; ++obstacle_x) {
      const int obstacle_index = gridToIndex(obstacle_x, obstacle_y);
      if (map_.data[obstacle_index] < config_.occupied_threshold) {
        continue;
      }

      for (int delta_y = -inflation_radius_cells_; delta_y <= inflation_radius_cells_; ++delta_y) {
        for (int delta_x = -inflation_radius_cells_; delta_x <= inflation_radius_cells_;
          ++delta_x)
        {
          const int target_x = obstacle_x + delta_x;
          const int target_y = obstacle_y + delta_y;

          if (!isValidInflationTarget(target_x, target_y, width, height)) {
            continue;
          }

          const double distance_in_cells = std::sqrt(
            static_cast<double>(delta_x * delta_x + delta_y * delta_y));
          if (distance_in_cells > static_cast<double>(inflation_radius_cells_)) {
            continue;
          }

          const int target_index = gridToIndex(target_x, target_y);
          if (inflated_map_.data[target_index] >= 0) {
            inflated_map_.data[target_index] = 100;
          }
        }
      }
    }
  }

  inflated_map_.header.stamp = stamp;
  inflated_map_.header.frame_id = "map";
}

bool MotionPlanner::isValidInflationTarget(int grid_x, int grid_y, int width, int height) const
{
  return grid_x >= 0 && grid_y >= 0 && grid_x < width && grid_y < height;
}

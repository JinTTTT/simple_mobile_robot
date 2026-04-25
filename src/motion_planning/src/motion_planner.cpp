#include "motion_planning/motion_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

bool MotionPlanner::CompareOpenSetEntry::operator()(
  const OpenSetEntry & left,
  const OpenSetEntry & right) const
{
  return left.estimated_total_cost > right.estimated_total_cost;
}

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

  if (!worldToGrid(
      start_pose.pose.position.x, start_pose.pose.position.y,
      result.start_grid_x, result.start_grid_y))
  {
    result.status_message = "Start pose is outside the map bounds.";
    return result;
  }

  if (!worldToGrid(
      goal_pose.pose.position.x, goal_pose.pose.position.y,
      result.goal_grid_x, result.goal_grid_y))
  {
    result.status_message = "Goal pose is outside the map bounds.";
    return result;
  }

  if (!isCellFreeForPlanning(result.start_grid_x, result.start_grid_y)) {
    result.status_message = "Start cell is occupied, unknown, or inside inflated clearance.";
    return result;
  }

  if (!isCellFreeForPlanning(result.goal_grid_x, result.goal_grid_y)) {
    result.status_message = "Goal cell is occupied, unknown, or inside inflated clearance.";
    return result;
  }

  const std::vector<int> raw_path_indices = runAStarSearch(
    result.start_grid_x, result.start_grid_y, result.goal_grid_x, result.goal_grid_y);
  if (raw_path_indices.empty()) {
    result.status_message = "A* could not find a path.";
    return result;
  }

  const std::vector<int> simplified_path_indices =
    config_.enable_line_of_sight_path_smoothing ?
    simplifyPathByLineOfSight(raw_path_indices) :
    raw_path_indices;

  result.shortcut_path = createPathMessageFromGridPath(
    simplified_path_indices, goal_pose, stamp);
  bool used_spline_collision_fallback = false;
  const nav_msgs::msg::Path geometry_path =
    config_.enable_cubic_spline_smoothing ?
    createSplineSmoothedPath(
    result.shortcut_path, goal_pose, used_spline_collision_fallback) :
    result.shortcut_path;
  result.used_spline_fallback = used_spline_collision_fallback;
  result.final_path = resamplePathAtFixedSpacing(
    geometry_path, goal_pose, config_.path_sample_spacing_m);
  result.geometry_pose_count = geometry_path.poses.size();
  result.raw_grid_pose_count = raw_path_indices.size();
  result.success = true;
  return result;
}

std::vector<int> MotionPlanner::runAStarSearch(
  int start_x, int start_y, int goal_x,
  int goal_y) const
{
  const int width = static_cast<int>(inflated_map_.info.width);
  const int height = static_cast<int>(inflated_map_.info.height);
  const int total_cells = width * height;
  const int start_index = gridToIndex(start_x, start_y);
  const int goal_index = gridToIndex(goal_x, goal_y);

  std::vector<double> cost_from_start(total_cells, std::numeric_limits<double>::infinity());
  std::vector<int> predecessor_by_index(total_cells, -1);
  std::vector<bool> closed_set(total_cells, false);

  std::priority_queue<OpenSetEntry, std::vector<OpenSetEntry>, CompareOpenSetEntry> open_set;

  cost_from_start[start_index] = 0.0;
  open_set.push({start_index, computeHeuristicCost(start_x, start_y, goal_x, goal_y)});

  const std::vector<std::pair<int, int>> neighbor_offsets = {
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
      return reconstructCellPath(predecessor_by_index, goal_index);
    }

    const int current_x = current_index % width;
    const int current_y = current_index / width;

    for (const auto & offset : neighbor_offsets) {
      const int next_x = current_x + offset.first;
      const int next_y = current_y + offset.second;

      if (!isValidCell(next_x, next_y) || !isCellFreeForPlanning(next_x, next_y)) {
        continue;
      }

      const int next_index = gridToIndex(next_x, next_y);
      if (closed_set[next_index]) {
        continue;
      }

      const bool is_diagonal_move = offset.first != 0 && offset.second != 0;
      const double step_cost = is_diagonal_move ? std::sqrt(2.0) : 1.0;
      const double tentative_cost_from_start = cost_from_start[current_index] + step_cost;

      if (tentative_cost_from_start >= cost_from_start[next_index]) {
        continue;
      }

      predecessor_by_index[next_index] = current_index;
      cost_from_start[next_index] = tentative_cost_from_start;
      const double estimated_total_cost =
        tentative_cost_from_start + computeHeuristicCost(next_x, next_y, goal_x, goal_y);
      open_set.push({next_index, estimated_total_cost});
    }
  }

  return {};
}

std::vector<int> MotionPlanner::reconstructCellPath(
  const std::vector<int> & predecessor_by_index,
  int goal_index) const
{
  std::vector<int> path_indices;
  int current_index = goal_index;

  while (current_index != -1) {
    path_indices.push_back(current_index);
    current_index = predecessor_by_index[current_index];
  }

  std::reverse(path_indices.begin(), path_indices.end());
  return path_indices;
}

std::vector<int> MotionPlanner::simplifyPathByLineOfSight(
  const std::vector<int> & raw_path_indices) const
{
  if (raw_path_indices.size() <= 2) {
    return raw_path_indices;
  }

  std::vector<int> simplified_path_indices;
  simplified_path_indices.reserve(raw_path_indices.size());
  simplified_path_indices.push_back(raw_path_indices.front());

  std::size_t anchor_path_index = 0;
  while (anchor_path_index < raw_path_indices.size() - 1) {
    std::size_t furthest_visible_path_index = anchor_path_index + 1;

    for (std::size_t candidate_path_index = anchor_path_index + 1;
      candidate_path_index < raw_path_indices.size();
      ++candidate_path_index)
    {
      if (!hasLineOfSightBetweenCells(
          raw_path_indices[anchor_path_index],
          raw_path_indices[candidate_path_index]))
      {
        break;
      }
      furthest_visible_path_index = candidate_path_index;
    }

    simplified_path_indices.push_back(raw_path_indices[furthest_visible_path_index]);
    anchor_path_index = furthest_visible_path_index;
  }

  return simplified_path_indices;
}

bool MotionPlanner::hasLineOfSightBetweenCells(int start_index, int end_index) const
{
  const int width = static_cast<int>(inflated_map_.info.width);

  const int start_x = start_index % width;
  const int start_y = start_index / width;
  const int end_x = end_index % width;
  const int end_y = end_index / width;

  const auto cells_on_line = traceLineCells(start_x, start_y, end_x, end_y);
  for (const auto & cell : cells_on_line) {
    if (!isValidCell(cell.first, cell.second) ||
      !isCellFreeForPlanning(cell.first, cell.second))
    {
      return false;
    }
  }

  return true;
}

std::vector<std::pair<int, int>> MotionPlanner::traceLineCells(int x0, int y0, int x1, int y1) const
{
  std::vector<std::pair<int, int>> cells_on_line;

  int delta_x = std::abs(x1 - x0);
  int delta_y = std::abs(y1 - y0);
  int step_x = (x0 < x1) ? 1 : -1;
  int step_y = (y0 < y1) ? 1 : -1;
  int error = delta_x - delta_y;

  int current_x = x0;
  int current_y = y0;

  while (true) {
    cells_on_line.emplace_back(current_x, current_y);
    if (current_x == x1 && current_y == y1) {
      break;
    }

    const int doubled_error = 2 * error;
    if (doubled_error > -delta_y) {
      error -= delta_y;
      current_x += step_x;
    }
    if (doubled_error < delta_x) {
      error += delta_x;
      current_y += step_y;
    }
  }

  return cells_on_line;
}

nav_msgs::msg::Path MotionPlanner::createPathMessageFromGridPath(
  const std::vector<int> & path_indices,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const builtin_interfaces::msg::Time & stamp) const
{
  nav_msgs::msg::Path path_message;
  path_message.header.stamp = stamp;
  path_message.header.frame_id = "map";
  path_message.poses.reserve(path_indices.size());

  for (std::size_t path_index = 0; path_index < path_indices.size(); ++path_index) {
    const int grid_index = path_indices[path_index];
    const int grid_x = grid_index % static_cast<int>(map_.info.width);
    const int grid_y = grid_index / static_cast<int>(map_.info.width);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_message.header;
    gridToWorld(grid_x, grid_y, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;

    if (path_index == path_indices.size() - 1) {
      pose.pose.orientation = goal_pose.pose.orientation;
    } else {
      const double yaw = computeGridPathYaw(path_indices, path_index);
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
    }

    path_message.poses.push_back(pose);
  }

  return path_message;
}

double MotionPlanner::computePathLength(const nav_msgs::msg::Path & path) const
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double total_length = 0.0;
  for (std::size_t path_index = 0; path_index + 1 < path.poses.size(); ++path_index) {
    const double delta_x =
      path.poses[path_index + 1].pose.position.x - path.poses[path_index].pose.position.x;
    const double delta_y =
      path.poses[path_index + 1].pose.position.y - path.poses[path_index].pose.position.y;
    total_length += std::sqrt(delta_x * delta_x + delta_y * delta_y);
  }

  return total_length;
}

nav_msgs::msg::Path MotionPlanner::resamplePathAtFixedSpacing(
  const nav_msgs::msg::Path & input_path,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  double sample_spacing) const
{
  if (input_path.poses.size() < 2) {
    return input_path;
  }

  const double effective_spacing = std::max(sample_spacing, 1e-3);
  std::vector<PathPoint> resampled_points;
  resampled_points.reserve(
    std::max<std::size_t>(
      input_path.poses.size(),
      static_cast<std::size_t>(
        std::ceil(computePathLength(input_path) / effective_spacing)) + 1U));

  const auto & first_position = input_path.poses.front().pose.position;
  resampled_points.push_back({first_position.x, first_position.y});

  double distance_until_next_sample = effective_spacing;

  for (std::size_t segment_index = 0; segment_index + 1 < input_path.poses.size();
    ++segment_index)
  {
    const auto & segment_start = input_path.poses[segment_index].pose.position;
    const auto & segment_end = input_path.poses[segment_index + 1].pose.position;
    const double delta_x = segment_end.x - segment_start.x;
    const double delta_y = segment_end.y - segment_start.y;
    const double segment_length = std::sqrt(delta_x * delta_x + delta_y * delta_y);

    if (segment_length <= 1e-9) {
      continue;
    }

    double distance_along_segment = distance_until_next_sample;
    while (distance_along_segment < segment_length) {
      const double interpolation_ratio = distance_along_segment / segment_length;
      resampled_points.push_back(
      {
        segment_start.x + interpolation_ratio * delta_x,
        segment_start.y + interpolation_ratio * delta_y
      });
      distance_along_segment += effective_spacing;
    }

    distance_until_next_sample = distance_along_segment - segment_length;
    if (distance_until_next_sample <= 1e-9) {
      distance_until_next_sample = effective_spacing;
    }
  }

  const auto & last_position = input_path.poses.back().pose.position;
  if (std::hypot(
      resampled_points.back().x - last_position.x,
      resampled_points.back().y - last_position.y) > 1e-6)
  {
    resampled_points.push_back({last_position.x, last_position.y});
  }

  nav_msgs::msg::Path resampled_path;
  resampled_path.header = input_path.header;
  resampled_path.poses.reserve(resampled_points.size());

  for (std::size_t point_index = 0; point_index < resampled_points.size(); ++point_index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = resampled_path.header;
    pose.pose.position.x = resampled_points[point_index].x;
    pose.pose.position.y = resampled_points[point_index].y;
    pose.pose.position.z = 0.0;

    if (point_index == resampled_points.size() - 1) {
      pose.pose.orientation = goal_pose.pose.orientation;
    } else {
      const std::size_t next_point_index = std::min(point_index + 1, resampled_points.size() - 1);
      const std::size_t prev_point_index = (point_index == 0) ? 0 : point_index - 1;
      const double yaw = std::atan2(
        resampled_points[next_point_index].y - resampled_points[prev_point_index].y,
        resampled_points[next_point_index].x - resampled_points[prev_point_index].x);
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
    }

    resampled_path.poses.push_back(pose);
  }

  return resampled_path;
}

double MotionPlanner::computeGridPathYaw(
  const std::vector<int> & path_indices,
  std::size_t path_index) const
{
  if (path_indices.size() < 2) {
    return 0.0;
  }

  const std::size_t next_path_index = std::min(path_index + 1, path_indices.size() - 1);
  const std::size_t prev_path_index = (path_index == 0) ? 0 : path_index - 1;

  double from_x = 0.0;
  double from_y = 0.0;
  double to_x = 0.0;
  double to_y = 0.0;

  const int from_grid_index = path_indices[prev_path_index];
  const int to_grid_index = path_indices[next_path_index];

  gridToWorld(
    from_grid_index % static_cast<int>(map_.info.width),
    from_grid_index / static_cast<int>(map_.info.width),
    from_x, from_y);
  gridToWorld(
    to_grid_index % static_cast<int>(map_.info.width),
    to_grid_index / static_cast<int>(map_.info.width),
    to_x, to_y);

  return std::atan2(to_y - from_y, to_x - from_x);
}

nav_msgs::msg::Path MotionPlanner::createSplineSmoothedPath(
  const nav_msgs::msg::Path & base_path,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  bool & used_collision_fallback) const
{
  used_collision_fallback = false;

  if (base_path.poses.size() < 3) {
    return base_path;
  }

  std::vector<PathPoint> control_points;
  control_points.reserve(base_path.poses.size());
  for (const auto & pose : base_path.poses) {
    control_points.push_back({pose.pose.position.x, pose.pose.position.y});
  }

  const std::vector<double> path_parameter = buildArcLengthParameter(control_points);
  if (path_parameter.size() < 3 || path_parameter.back() <= 1e-6) {
    return base_path;
  }

  const std::vector<double> x_values = extractXPathValues(control_points);
  const std::vector<double> y_values = extractYPathValues(control_points);
  const std::vector<double> x_second_derivatives =
    solveNaturalCubicSecondDerivatives(path_parameter, x_values);
  const std::vector<double> y_second_derivatives =
    solveNaturalCubicSecondDerivatives(path_parameter, y_values);

  nav_msgs::msg::Path spline_path;
  spline_path.header = base_path.header;
  spline_path.poses.reserve(
    std::max<std::size_t>(
      base_path.poses.size(),
      static_cast<std::size_t>(
        std::ceil(path_parameter.back() / config_.path_sample_spacing_m)) + 1U));

  const double total_length = path_parameter.back();
  const double sample_step = std::max(config_.path_sample_spacing_m, map_.info.resolution * 0.5);

  std::vector<PathPoint> sampled_points;
  sampled_points.reserve(spline_path.poses.capacity());
  sampled_points.push_back(control_points.front());

  for (double path_distance = sample_step; path_distance < total_length;
    path_distance += sample_step)
  {
    sampled_points.push_back(
    {
      evaluateNaturalCubicSpline(path_parameter, x_values, x_second_derivatives, path_distance),
      evaluateNaturalCubicSpline(path_parameter, y_values, y_second_derivatives, path_distance)
    });
  }

  sampled_points.push_back(control_points.back());

  for (const auto & point : sampled_points) {
    int grid_x = 0;
    int grid_y = 0;
    if (!worldToGrid(
        point.x, point.y, grid_x,
        grid_y) || !isCellFreeForPlanning(grid_x, grid_y))
    {
      used_collision_fallback = true;
      return base_path;
    }
  }

  for (std::size_t point_index = 0; point_index < sampled_points.size(); ++point_index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = spline_path.header;
    pose.pose.position.x = sampled_points[point_index].x;
    pose.pose.position.y = sampled_points[point_index].y;
    pose.pose.position.z = 0.0;

    if (point_index == sampled_points.size() - 1) {
      pose.pose.orientation = goal_pose.pose.orientation;
    } else {
      const std::size_t next_point_index = std::min(point_index + 1, sampled_points.size() - 1);
      const std::size_t prev_point_index = (point_index == 0) ? 0 : point_index - 1;
      const double yaw = std::atan2(
        sampled_points[next_point_index].y - sampled_points[prev_point_index].y,
        sampled_points[next_point_index].x - sampled_points[prev_point_index].x);
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
    }

    spline_path.poses.push_back(pose);
  }

  return spline_path;
}

std::vector<double> MotionPlanner::buildArcLengthParameter(const std::vector<PathPoint> & points)
const
{
  std::vector<double> parameter_values(points.size(), 0.0);
  for (std::size_t point_index = 1; point_index < points.size(); ++point_index) {
    const double delta_x = points[point_index].x - points[point_index - 1].x;
    const double delta_y = points[point_index].y - points[point_index - 1].y;
    parameter_values[point_index] =
      parameter_values[point_index - 1] + std::sqrt(delta_x * delta_x + delta_y * delta_y);
  }

  return parameter_values;
}

std::vector<double> MotionPlanner::extractXPathValues(const std::vector<PathPoint> & points) const
{
  std::vector<double> x_values;
  x_values.reserve(points.size());
  for (const auto & point : points) {
    x_values.push_back(point.x);
  }

  return x_values;
}

std::vector<double> MotionPlanner::extractYPathValues(const std::vector<PathPoint> & points) const
{
  std::vector<double> y_values;
  y_values.reserve(points.size());
  for (const auto & point : points) {
    y_values.push_back(point.y);
  }

  return y_values;
}

std::vector<double> MotionPlanner::solveNaturalCubicSecondDerivatives(
  const std::vector<double> & parameter_values,
  const std::vector<double> & sample_values) const
{
  const std::size_t point_count = sample_values.size();
  std::vector<double> second_derivatives(point_count, 0.0);
  if (point_count < 3) {
    return second_derivatives;
  }

  const std::size_t interior_count = point_count - 2;
  std::vector<double> lower_diagonal(interior_count, 0.0);
  std::vector<double> main_diagonal(interior_count, 0.0);
  std::vector<double> upper_diagonal(interior_count, 0.0);
  std::vector<double> right_hand_side(interior_count, 0.0);

  for (std::size_t interior_index = 0; interior_index < interior_count; ++interior_index) {
    const std::size_t knot_index = interior_index + 1;
    const double previous_interval =
      parameter_values[knot_index] - parameter_values[knot_index - 1];
    const double next_interval =
      parameter_values[knot_index + 1] - parameter_values[knot_index];
    if (previous_interval <= 1e-9 || next_interval <= 1e-9) {
      return second_derivatives;
    }

    lower_diagonal[interior_index] = previous_interval;
    main_diagonal[interior_index] = 2.0 * (previous_interval + next_interval);
    upper_diagonal[interior_index] = next_interval;
    right_hand_side[interior_index] = 6.0 * (
      (sample_values[knot_index + 1] - sample_values[knot_index]) / next_interval -
      (sample_values[knot_index] - sample_values[knot_index - 1]) / previous_interval);
  }

  for (std::size_t interior_index = 1; interior_index < interior_count; ++interior_index) {
    const double elimination_factor = lower_diagonal[interior_index] /
      main_diagonal[interior_index - 1];
    main_diagonal[interior_index] -= elimination_factor * upper_diagonal[interior_index - 1];
    right_hand_side[interior_index] -= elimination_factor * right_hand_side[interior_index - 1];
  }

  second_derivatives[point_count - 2] = right_hand_side.back() / main_diagonal.back();
  for (std::size_t interior_index = interior_count - 1; interior_index > 0; --interior_index) {
    second_derivatives[interior_index] =
      (right_hand_side[interior_index - 1] -
      upper_diagonal[interior_index - 1] * second_derivatives[interior_index + 1]) /
      main_diagonal[interior_index - 1];
  }

  return second_derivatives;
}

double MotionPlanner::evaluateNaturalCubicSpline(
  const std::vector<double> & parameter_values,
  const std::vector<double> & sample_values,
  const std::vector<double> & second_derivatives,
  double query_value) const
{
  const auto upper_bound = std::upper_bound(
    parameter_values.begin(), parameter_values.end(), query_value);
  const std::size_t segment_index = static_cast<std::size_t>(
    std::max<std::ptrdiff_t>(0, std::distance(parameter_values.begin(), upper_bound) - 1));
  const std::size_t clamped_segment_index =
    std::min(segment_index, parameter_values.size() - 2);

  const double segment_start = parameter_values[clamped_segment_index];
  const double segment_end = parameter_values[clamped_segment_index + 1];
  const double interval = segment_end - segment_start;
  if (interval <= 1e-9) {
    return sample_values[clamped_segment_index];
  }

  const double left_distance = segment_end - query_value;
  const double right_distance = query_value - segment_start;
  return
    second_derivatives[clamped_segment_index] *
    left_distance * left_distance * left_distance / (6.0 * interval) +
    second_derivatives[clamped_segment_index + 1] *
    right_distance * right_distance * right_distance / (6.0 * interval) +
    (sample_values[clamped_segment_index] -
    second_derivatives[clamped_segment_index] * interval * interval / 6.0) *
    (left_distance / interval) +
    (sample_values[clamped_segment_index + 1] -
    second_derivatives[clamped_segment_index + 1] * interval * interval / 6.0) *
    (right_distance / interval);
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

void MotionPlanner::gridToWorld(int grid_x, int grid_y, double & world_x, double & world_y) const
{
  const double resolution = map_.info.resolution;
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;

  world_x = origin_x + (static_cast<double>(grid_x) + 0.5) * resolution;
  world_y = origin_y + (static_cast<double>(grid_y) + 0.5) * resolution;
}

bool MotionPlanner::isValidCell(int grid_x, int grid_y) const
{
  return grid_x >= 0 &&
         grid_y >= 0 &&
         grid_x < static_cast<int>(inflated_map_.info.width) &&
         grid_y < static_cast<int>(inflated_map_.info.height);
}

bool MotionPlanner::isCellFreeForPlanning(int grid_x, int grid_y) const
{
  const int8_t cell_value = inflated_map_.data[gridToIndex(grid_x, grid_y)];
  return cell_value >= 0 && cell_value < config_.occupied_threshold;
}

int MotionPlanner::gridToIndex(int grid_x, int grid_y) const
{
  return grid_y * static_cast<int>(map_.info.width) + grid_x;
}

double MotionPlanner::computeHeuristicCost(int x0, int y0, int x1, int y1) const
{
  const double delta_x = static_cast<double>(x1 - x0);
  const double delta_y = static_cast<double>(y1 - y0);
  return std::sqrt(delta_x * delta_x + delta_y * delta_y);
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

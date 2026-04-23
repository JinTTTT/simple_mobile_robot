#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct OpenSetEntry
{
  int index = -1;
  double f_score = 0.0;
};

struct CompareOpenSetEntry
{
  bool operator()(const OpenSetEntry & left, const OpenSetEntry & right) const
  {
    return left.f_score > right.f_score;
  }
};

struct PathPoint
{
  double x = 0.0;
  double y = 0.0;
};
}  // namespace

class MotionPlanningNode : public rclcpp::Node
{
public:
  MotionPlanningNode()
  : Node("motion_planning_node")
  {
    robot_radius_m_ = this->declare_parameter<double>("robot_radius_m", 0.35);
    occupied_threshold_ = this->declare_parameter<int>("occupied_threshold", 50);
    enable_path_smoothing_ = this->declare_parameter<bool>("enable_path_smoothing", true);
    enable_cubic_spline_smoothing_ =
      this->declare_parameter<bool>("enable_cubic_spline_smoothing", true);
    spline_sample_spacing_m_ =
      this->declare_parameter<double>("spline_sample_spacing_m", 0.05);

    rclcpp::QoS static_map_qos(1);
    static_map_qos.transient_local();
    static_map_qos.reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", static_map_qos,
      std::bind(&MotionPlanningNode::mapCallback, this, std::placeholders::_1));

    start_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/estimated_pose", 10,
      std::bind(&MotionPlanningNode::startPoseCallback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&MotionPlanningNode::goalPoseCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    smoothed_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/smoothed_planned_path", 10);
    inflated_map_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_map", static_map_qos);

    RCLCPP_INFO(
      this->get_logger(),
      "MotionPlanningNode started. Inputs: /map, /estimated_pose, /goal_pose. Outputs: /planned_path, /smoothed_planned_path.");
    RCLCPP_INFO(
      this->get_logger(),
      "Using conservative circular robot radius %.2f m based on simulation geometry.",
      robot_radius_m_);
    RCLCPP_INFO(
      this->get_logger(),
      "Occupied threshold: %d, line-of-sight smoothing: %s, cubic spline smoothing: %s.",
      occupied_threshold_,
      enable_path_smoothing_ ? "enabled" : "disabled",
      enable_cubic_spline_smoothing_ ? "enabled" : "disabled");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = *msg;
    has_map_ = true;
    rebuildInflatedMap();
    inflated_map_pub_->publish(inflated_map_);

    RCLCPP_INFO(
      this->get_logger(),
      "Map received: %u x %u cells, resolution %.3f m. Inflated by %.2f m (%d cells).",
      map_.info.width,
      map_.info.height,
      map_.info.resolution,
      robot_radius_m_,
      inflation_radius_cells_);

    tryPlanIfRequested();
  }

  void startPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    start_pose_ = *msg;
    has_start_pose_ = true;
    tryPlanIfRequested();
  }

  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_pose_ = true;
    planning_requested_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "New goal received at x=%.2f y=%.2f in frame %s.",
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y,
      goal_pose_.header.frame_id.c_str());

    tryPlanIfRequested();
  }

  void tryPlanIfRequested()
  {
    if (!planning_requested_) {
      return;
    }

    if (!has_map_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Waiting for /map before planning.");
      return;
    }

    if (!has_start_pose_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for /estimated_pose before planning.");
      return;
    }

    if (!has_goal_pose_) {
      return;
    }

    if (start_pose_.header.frame_id != "map" || goal_pose_.header.frame_id != "map") {
      RCLCPP_WARN(
        this->get_logger(),
        "Planner expects start and goal in map frame. Start frame: '%s', goal frame: '%s'.",
        start_pose_.header.frame_id.c_str(),
        goal_pose_.header.frame_id.c_str());
      return;
    }

    if (planPath()) {
      planning_requested_ = false;
    }
  }

  bool planPath()
  {
    int start_x = 0;
    int start_y = 0;
    int goal_x = 0;
    int goal_y = 0;

    if (!worldToGrid(start_pose_.pose.position.x, start_pose_.pose.position.y, start_x, start_y)) {
      RCLCPP_WARN(this->get_logger(), "Start pose is outside the map bounds.");
      return false;
    }

    if (!worldToGrid(goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_x, goal_y)) {
      RCLCPP_WARN(this->get_logger(), "Goal pose is outside the map bounds.");
      return false;
    }

    if (!isCellFreeForPlanning(start_x, start_y)) {
      RCLCPP_WARN(this->get_logger(), "Start cell is occupied, unknown, or inside inflated clearance.");
      return false;
    }

    if (!isCellFreeForPlanning(goal_x, goal_y)) {
      RCLCPP_WARN(this->get_logger(), "Goal cell is occupied, unknown, or inside inflated clearance.");
      return false;
    }

    const std::vector<int> path_indices = runAStar(start_x, start_y, goal_x, goal_y);
    if (path_indices.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "A* could not find a path from (%d, %d) to (%d, %d).",
        start_x, start_y, goal_x, goal_y);
      return false;
    }

    const std::vector<int> shortcut_path_indices =
      enable_path_smoothing_ ? smoothPath(path_indices) : path_indices;
    const nav_msgs::msg::Path shortcut_path = buildGridPathMessage(shortcut_path_indices);
    const nav_msgs::msg::Path smoothed_path =
      enable_cubic_spline_smoothing_ ? buildSplinePath(shortcut_path) : shortcut_path;

    path_pub_->publish(shortcut_path);
    smoothed_path_pub_->publish(smoothed_path);
    RCLCPP_INFO(
      this->get_logger(),
      "Published shortcut path with %zu poses and spline path with %zu poses (raw A* path had %zu poses) from (%d, %d) to (%d, %d).",
      shortcut_path.poses.size(), smoothed_path.poses.size(), path_indices.size(),
      start_x, start_y, goal_x, goal_y);
    return true;
  }

  std::vector<int> runAStar(int start_x, int start_y, int goal_x, int goal_y) const
  {
    const int width = static_cast<int>(inflated_map_.info.width);
    const int height = static_cast<int>(inflated_map_.info.height);
    const int total_cells = width * height;
    const int start_index = gridToIndex(start_x, start_y);
    const int goal_index = gridToIndex(goal_x, goal_y);

    std::vector<double> g_score(total_cells, std::numeric_limits<double>::infinity());
    std::vector<int> came_from(total_cells, -1);
    std::vector<bool> closed(total_cells, false);

    std::priority_queue<OpenSetEntry, std::vector<OpenSetEntry>, CompareOpenSetEntry> open_set;

    g_score[start_index] = 0.0;
    open_set.push({start_index, heuristic(start_x, start_y, goal_x, goal_y)});

    const std::vector<std::pair<int, int>> neighbor_offsets = {
      {-1, -1}, {0, -1}, {1, -1},
      {-1,  0},           {1,  0},
      {-1,  1}, {0,  1}, {1,  1}
    };

    while (!open_set.empty()) {
      const OpenSetEntry current_entry = open_set.top();
      open_set.pop();

      const int current_index = current_entry.index;
      if (closed[current_index]) {
        continue;
      }
      closed[current_index] = true;

      if (current_index == goal_index) {
        return reconstructPath(came_from, goal_index);
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
        if (closed[next_index]) {
          continue;
        }

        const bool diagonal_move = offset.first != 0 && offset.second != 0;
        const double move_cost = diagonal_move ? std::sqrt(2.0) : 1.0;
        const double tentative_g = g_score[current_index] + move_cost;

        if (tentative_g >= g_score[next_index]) {
          continue;
        }

        came_from[next_index] = current_index;
        g_score[next_index] = tentative_g;
        const double f_score = tentative_g + heuristic(next_x, next_y, goal_x, goal_y);
        open_set.push({next_index, f_score});
      }
    }

    return {};
  }

  std::vector<int> reconstructPath(const std::vector<int> & came_from, int goal_index) const
  {
    std::vector<int> path;
    int current = goal_index;

    while (current != -1) {
      path.push_back(current);
      current = came_from[current];
    }

    std::reverse(path.begin(), path.end());
    return path;
  }

  std::vector<int> smoothPath(const std::vector<int> & raw_path_indices) const
  {
    if (raw_path_indices.size() <= 2) {
      return raw_path_indices;
    }

    std::vector<int> smoothed_path;
    smoothed_path.reserve(raw_path_indices.size());
    smoothed_path.push_back(raw_path_indices.front());

    std::size_t anchor_index = 0;
    while (anchor_index < raw_path_indices.size() - 1) {
      std::size_t furthest_visible_index = anchor_index + 1;

      for (std::size_t candidate_index = anchor_index + 1;
        candidate_index < raw_path_indices.size();
        ++candidate_index)
      {
        if (!hasLineOfSight(raw_path_indices[anchor_index], raw_path_indices[candidate_index])) {
          break;
        }
        furthest_visible_index = candidate_index;
      }

      smoothed_path.push_back(raw_path_indices[furthest_visible_index]);
      anchor_index = furthest_visible_index;
    }

    return smoothed_path;
  }

  bool hasLineOfSight(int start_index, int end_index) const
  {
    const int width = static_cast<int>(inflated_map_.info.width);

    const int start_x = start_index % width;
    const int start_y = start_index / width;
    const int end_x = end_index % width;
    const int end_y = end_index / width;

    const auto cells_on_line = bresenhamLine(start_x, start_y, end_x, end_y);
    for (const auto & cell : cells_on_line) {
      if (!isValidCell(cell.first, cell.second) ||
        !isCellFreeForPlanning(cell.first, cell.second))
      {
        return false;
      }
    }

    return true;
  }

  std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) const
  {
    std::vector<std::pair<int, int>> cells;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int current_x = x0;
    int current_y = y0;

    while (true) {
      cells.emplace_back(current_x, current_y);
      if (current_x == x1 && current_y == y1) {
        break;
      }

      const int twice_err = 2 * err;
      if (twice_err > -dy) {
        err -= dy;
        current_x += sx;
      }
      if (twice_err < dx) {
        err += dx;
        current_y += sy;
      }
    }

    return cells;
  }

  nav_msgs::msg::Path buildGridPathMessage(const std::vector<int> & path_indices) const
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";
    path_msg.poses.reserve(path_indices.size());

    for (std::size_t i = 0; i < path_indices.size(); ++i) {
      const int index = path_indices[i];
      const int x = index % static_cast<int>(map_.info.width);
      const int y = index / static_cast<int>(map_.info.width);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      gridToWorld(x, y, pose.pose.position.x, pose.pose.position.y);
      pose.pose.position.z = 0.0;

      if (i == path_indices.size() - 1) {
        pose.pose.orientation = goal_pose_.pose.orientation;
      } else {
        const double yaw = computePoseYaw(path_indices, i);
        pose.pose.orientation.z = std::sin(yaw * 0.5);
        pose.pose.orientation.w = std::cos(yaw * 0.5);
      }
      path_msg.poses.push_back(pose);
    }
    return path_msg;
  }

  double computePoseYaw(const std::vector<int> & path_indices, std::size_t index_in_path) const
  {
    if (path_indices.size() < 2) {
      return 0.0;
    }

    const std::size_t next_index_in_path =
      std::min(index_in_path + 1, path_indices.size() - 1);
    const std::size_t prev_index_in_path =
      (index_in_path == 0) ? 0 : index_in_path - 1;

    double from_x = 0.0;
    double from_y = 0.0;
    double to_x = 0.0;
    double to_y = 0.0;

    const int from_grid_index = path_indices[prev_index_in_path];
    const int to_grid_index = path_indices[next_index_in_path];

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

  nav_msgs::msg::Path buildSplinePath(const nav_msgs::msg::Path & base_path) const
  {
    if (base_path.poses.size() < 3) {
      return base_path;
    }

    std::vector<PathPoint> control_points;
    control_points.reserve(base_path.poses.size());
    for (const auto & pose : base_path.poses) {
      control_points.push_back({pose.pose.position.x, pose.pose.position.y});
    }

    const std::vector<double> path_parameter = buildPathParameter(control_points);
    if (path_parameter.size() < 3 || path_parameter.back() <= 1e-6) {
      return base_path;
    }

    const std::vector<double> control_x = extractCoordinate(control_points, true);
    const std::vector<double> control_y = extractCoordinate(control_points, false);
    const std::vector<double> spline_x =
      buildNaturalCubicSecondDerivatives(path_parameter, control_x);
    const std::vector<double> spline_y =
      buildNaturalCubicSecondDerivatives(path_parameter, control_y);

    nav_msgs::msg::Path smoothed_path;
    smoothed_path.header = base_path.header;
    smoothed_path.poses.reserve(
      std::max<std::size_t>(
        base_path.poses.size(),
        static_cast<std::size_t>(std::ceil(path_parameter.back() / spline_sample_spacing_m_)) + 1U));

    const double total_length = path_parameter.back();
    const double sample_step = std::max(spline_sample_spacing_m_, map_.info.resolution * 0.5);

    std::vector<PathPoint> sampled_points;
    sampled_points.reserve(smoothed_path.poses.capacity());
    sampled_points.push_back(control_points.front());

    for (double t = sample_step; t < total_length; t += sample_step) {
      sampled_points.push_back({
        evaluateNaturalCubic(path_parameter, control_x, spline_x, t),
        evaluateNaturalCubic(path_parameter, control_y, spline_y, t)
      });
    }

    sampled_points.push_back(control_points.back());

    for (const auto & point : sampled_points) {
      int grid_x = 0;
      int grid_y = 0;
      if (!worldToGrid(point.x, point.y, grid_x, grid_y) || !isCellFreeForPlanning(grid_x, grid_y)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Spline-smoothed path entered an occupied or unknown cell. Falling back to non-spline path.");
        return base_path;
      }
    }

    for (std::size_t i = 0; i < sampled_points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = smoothed_path.header;
      pose.pose.position.x = sampled_points[i].x;
      pose.pose.position.y = sampled_points[i].y;
      pose.pose.position.z = 0.0;

      if (i == sampled_points.size() - 1) {
        pose.pose.orientation = goal_pose_.pose.orientation;
      } else {
        const std::size_t next_index = std::min(i + 1, sampled_points.size() - 1);
        const std::size_t prev_index = (i == 0) ? 0 : i - 1;
        const double yaw = std::atan2(
          sampled_points[next_index].y - sampled_points[prev_index].y,
          sampled_points[next_index].x - sampled_points[prev_index].x);
        pose.pose.orientation.z = std::sin(yaw * 0.5);
        pose.pose.orientation.w = std::cos(yaw * 0.5);
      }

      smoothed_path.poses.push_back(pose);
    }

    return smoothed_path;
  }

  std::vector<double> buildPathParameter(const std::vector<PathPoint> & points) const
  {
    std::vector<double> parameter(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
      const double dx = points[i].x - points[i - 1].x;
      const double dy = points[i].y - points[i - 1].y;
      parameter[i] = parameter[i - 1] + std::sqrt(dx * dx + dy * dy);
    }

    return parameter;
  }

  std::vector<double> extractCoordinate(
    const std::vector<PathPoint> & points,
    bool extract_x) const
  {
    std::vector<double> coordinate;
    coordinate.reserve(points.size());
    for (const auto & point : points) {
      coordinate.push_back(extract_x ? point.x : point.y);
    }

    return coordinate;
  }

  std::vector<double> buildNaturalCubicSecondDerivatives(
    const std::vector<double> & parameter,
    const std::vector<double> & values) const
  {
    const std::size_t point_count = values.size();
    std::vector<double> second_derivative(point_count, 0.0);
    if (point_count < 3) {
      return second_derivative;
    }

    const std::size_t interior_count = point_count - 2;
    std::vector<double> lower(interior_count, 0.0);
    std::vector<double> diagonal(interior_count, 0.0);
    std::vector<double> upper(interior_count, 0.0);
    std::vector<double> rhs(interior_count, 0.0);

    for (std::size_t i = 0; i < interior_count; ++i) {
      const std::size_t knot_index = i + 1;
      const double h_prev = parameter[knot_index] - parameter[knot_index - 1];
      const double h_next = parameter[knot_index + 1] - parameter[knot_index];
      if (h_prev <= 1e-9 || h_next <= 1e-9) {
        return second_derivative;
      }

      lower[i] = h_prev;
      diagonal[i] = 2.0 * (h_prev + h_next);
      upper[i] = h_next;
      rhs[i] = 6.0 * (
        (values[knot_index + 1] - values[knot_index]) / h_next -
        (values[knot_index] - values[knot_index - 1]) / h_prev);
    }

    for (std::size_t i = 1; i < interior_count; ++i) {
      const double factor = lower[i] / diagonal[i - 1];
      diagonal[i] -= factor * upper[i - 1];
      rhs[i] -= factor * rhs[i - 1];
    }

    second_derivative[point_count - 2] = rhs.back() / diagonal.back();
    for (std::size_t i = interior_count - 1; i > 0; --i) {
      second_derivative[i] =
        (rhs[i - 1] - upper[i - 1] * second_derivative[i + 1]) / diagonal[i - 1];
    }

    return second_derivative;
  }

  double evaluateNaturalCubic(
    const std::vector<double> & parameter,
    const std::vector<double> & values,
    const std::vector<double> & second_derivative,
    double query) const
  {
    const auto upper = std::upper_bound(parameter.begin(), parameter.end(), query);
    const std::size_t segment_index = static_cast<std::size_t>(
      std::max<std::ptrdiff_t>(0, std::distance(parameter.begin(), upper) - 1));
    const std::size_t clamped_segment_index =
      std::min(segment_index, parameter.size() - 2);

    const double t0 = parameter[clamped_segment_index];
    const double t1 = parameter[clamped_segment_index + 1];
    const double interval = t1 - t0;
    if (interval <= 1e-9) {
      return values[clamped_segment_index];
    }

    const double left = t1 - query;
    const double right = query - t0;
    return
      second_derivative[clamped_segment_index] * left * left * left / (6.0 * interval) +
      second_derivative[clamped_segment_index + 1] * right * right * right / (6.0 * interval) +
      (values[clamped_segment_index] -
      second_derivative[clamped_segment_index] * interval * interval / 6.0) * (left / interval) +
      (values[clamped_segment_index + 1] -
      second_derivative[clamped_segment_index + 1] * interval * interval / 6.0) *
      (right / interval);
  }

  bool worldToGrid(double world_x, double world_y, int & grid_x, int & grid_y) const
  {
    const double resolution = map_.info.resolution;
    const double origin_x = map_.info.origin.position.x;
    const double origin_y = map_.info.origin.position.y;

    grid_x = static_cast<int>(std::floor((world_x - origin_x) / resolution));
    grid_y = static_cast<int>(std::floor((world_y - origin_y) / resolution));
    return isValidCell(grid_x, grid_y);
  }

  void gridToWorld(int grid_x, int grid_y, double & world_x, double & world_y) const
  {
    const double resolution = map_.info.resolution;
    const double origin_x = map_.info.origin.position.x;
    const double origin_y = map_.info.origin.position.y;

    world_x = origin_x + (static_cast<double>(grid_x) + 0.5) * resolution;
    world_y = origin_y + (static_cast<double>(grid_y) + 0.5) * resolution;
  }

  bool isValidCell(int grid_x, int grid_y) const
  {
    return grid_x >= 0 &&
           grid_y >= 0 &&
           grid_x < static_cast<int>(inflated_map_.info.width) &&
           grid_y < static_cast<int>(inflated_map_.info.height);
  }

  bool isCellFreeForPlanning(int grid_x, int grid_y) const
  {
    const int8_t cell_value = inflated_map_.data[gridToIndex(grid_x, grid_y)];
    return cell_value >= 0 && cell_value < occupied_threshold_;
  }

  int gridToIndex(int grid_x, int grid_y) const
  {
    return grid_y * static_cast<int>(map_.info.width) + grid_x;
  }

  double heuristic(int x0, int y0, int x1, int y1) const
  {
    const double dx = static_cast<double>(x1 - x0);
    const double dy = static_cast<double>(y1 - y0);
    return std::sqrt(dx * dx + dy * dy);
  }

  void rebuildInflatedMap()
  {
    inflated_map_ = map_;

    const double resolution = map_.info.resolution;
    inflation_radius_cells_ = static_cast<int>(std::ceil(robot_radius_m_ / resolution));

    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const int total_cells = width * height;

    for (int index = 0; index < total_cells; ++index) {
      if (map_.data[index] >= occupied_threshold_) {
        inflated_map_.data[index] = 100;
        continue;
      }

      if (map_.data[index] < 0) {
        inflated_map_.data[index] = -1;
      } else {
        inflated_map_.data[index] = 0;
      }
    }

    for (int source_y = 0; source_y < height; ++source_y) {
      for (int source_x = 0; source_x < width; ++source_x) {
        const int source_index = gridToIndex(source_x, source_y);
        if (map_.data[source_index] < occupied_threshold_) {
          continue;
        }

        for (int dy = -inflation_radius_cells_; dy <= inflation_radius_cells_; ++dy) {
          for (int dx = -inflation_radius_cells_; dx <= inflation_radius_cells_; ++dx) {
            const int target_x = source_x + dx;
            const int target_y = source_y + dy;

            if (!isValidInflationTarget(target_x, target_y, width, height)) {
              continue;
            }

            const double distance_cells = std::sqrt(static_cast<double>(dx * dx + dy * dy));
            if (distance_cells > static_cast<double>(inflation_radius_cells_)) {
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

    inflated_map_.header.stamp = this->now();
    inflated_map_.header.frame_id = "map";
  }

  bool isValidInflationTarget(int grid_x, int grid_y, int width, int height) const
  {
    return grid_x >= 0 && grid_y >= 0 && grid_x < width && grid_y < height;
  }

  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::OccupancyGrid inflated_map_;
  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_pub_;

  bool has_map_ = false;
  bool has_start_pose_ = false;
  bool has_goal_pose_ = false;
  bool planning_requested_ = false;

  int occupied_threshold_ = 50;
  double robot_radius_m_ = 0.35;
  bool enable_path_smoothing_ = true;
  bool enable_cubic_spline_smoothing_ = true;
  double spline_sample_spacing_m_ = 0.05;
  int inflation_radius_cells_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionPlanningNode>());
  rclcpp::shutdown();
  return 0;
}

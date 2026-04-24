#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <cmath>

class OccupancyMapper : public rclcpp::Node
{
public:
  OccupancyMapper();

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publish_map_timer();

  void world_to_grid(double wx, double wy, int & gx, int & gy);
  bool is_valid_cell(int x, int y);
  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);
  std::vector<std::pair<int, int>> bresenham_line(int x0, int y0, int x1, int y1);

  double resolution_;
  int width_;
  int height_;
  double origin_x_;
  double origin_y_;

  std::vector<int> hits_;
  std::vector<int> passes_;

  geometry_msgs::msg::Pose current_pose_;
  bool pose_received_;

  nav_msgs::msg::OccupancyGrid map_msg_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

OccupancyMapper::OccupancyMapper()
: Node("occupancy_mapper_simple_count_method")
{
  pose_received_ = false;

  resolution_ = 0.05;
  width_ = 200;
  height_ = 200;
  origin_x_ = -width_ * resolution_ / 2.0;
  origin_y_ = -height_ * resolution_ / 2.0;

  const int total_cells = width_ * height_;
  hits_.resize(total_cells, 0);
  passes_.resize(total_cells, 0);

  map_msg_.header.frame_id = "odom";
  map_msg_.info.resolution = resolution_;
  map_msg_.info.width = width_;
  map_msg_.info.height = height_;
  map_msg_.info.origin.position.x = origin_x_;
  map_msg_.info.origin.position.y = origin_y_;
  map_msg_.info.origin.orientation.w = 1.0;
  map_msg_.data.resize(total_cells, -1);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&OccupancyMapper::odom_callback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&OccupancyMapper::scan_callback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&OccupancyMapper::publish_map_timer, this));
}

void OccupancyMapper::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  pose_received_ = true;
}

void OccupancyMapper::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!pose_received_) {
    return;
  }

  const double robot_x = current_pose_.position.x;
  const double robot_y = current_pose_.position.y;
  const double robot_theta = get_yaw_from_quaternion(current_pose_.orientation);

  int robot_grid_x = 0;
  int robot_grid_y = 0;
  world_to_grid(robot_x, robot_y, robot_grid_x, robot_grid_y);

  if (!is_valid_cell(robot_grid_x, robot_grid_y)) {
    return;
  }

  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    const float range = msg->ranges[i];
    if (range < msg->range_min || range > msg->range_max || std::isnan(range) ||
      std::isinf(range))
    {
      continue;
    }

    const double beam_angle = msg->angle_min + i * msg->angle_increment;
    const double world_angle = robot_theta + beam_angle;

    const double end_x = robot_x + range * std::cos(world_angle);
    const double end_y = robot_y + range * std::sin(world_angle);

    int end_grid_x = 0;
    int end_grid_y = 0;
    world_to_grid(end_x, end_y, end_grid_x, end_grid_y);

    if (!is_valid_cell(end_grid_x, end_grid_y)) {
      continue;
    }

    const auto cells = bresenham_line(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y);
    for (std::size_t j = 0; j + 1 < cells.size(); ++j) {
      const int index = cells[j].second * width_ + cells[j].first;
      passes_[index]++;
    }

    const int hit_index = cells.back().second * width_ + cells.back().first;
    hits_[hit_index]++;
  }
}

void OccupancyMapper::world_to_grid(double wx, double wy, int & gx, int & gy)
{
  gx = static_cast<int>((wx - origin_x_) / resolution_);
  gy = static_cast<int>((wy - origin_y_) / resolution_);
}

bool OccupancyMapper::is_valid_cell(int x, int y)
{
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

double OccupancyMapper::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void OccupancyMapper::publish_map_timer()
{
  map_msg_.header.stamp = this->now();

  for (int i = 0; i < width_ * height_; ++i) {
    const int total = hits_[i] + passes_[i];
    if (total == 0) {
      map_msg_.data[i] = -1;
      continue;
    }

    const double probability = static_cast<double>(hits_[i]) / static_cast<double>(total);
    if (probability > 0.65) {
      map_msg_.data[i] = 100;
    } else if (probability < 0.35) {
      map_msg_.data[i] = 0;
    } else {
      map_msg_.data[i] = -1;
    }
  }

  map_pub_->publish(map_msg_);
}

std::vector<std::pair<int, int>> OccupancyMapper::bresenham_line(int x0, int y0, int x1, int y1)
{
  std::vector<std::pair<int, int>> cells;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  const int sx = (x0 < x1) ? 1 : -1;
  const int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  int x = x0;
  int y = y0;

  while (true) {
    cells.emplace_back(x, y);
    if (x == x1 && y == y1) {
      break;
    }

    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }

  return cells;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyMapper>());
  rclcpp::shutdown();
  return 0;
}

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
    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publish_map_timer();

    // Helper functions
    void world_to_grid(double wx, double wy, int& gx, int& gy);
    bool is_valid_cell(int x, int y);
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q);
    std::vector<std::pair<int, int>> bresenham_line(int x0, int y0, int x1, int y1);

    // Map configuration
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;

    // Hit/pass counters
    std::vector<int> hits_;
    std::vector<int> passes_;

    geometry_msgs::msg::Pose current_pose_;
    bool pose_received_;
    
    // Map message and publisher
    nav_msgs::msg::OccupancyGrid map_msg_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

OccupancyMapper::OccupancyMapper() : Node("occupancy_mapper")
{
    pose_received_ = false;

    // Initialize map parameters
    resolution_ = 0.05;
    width_ = 200;
    height_ = 200;
    origin_x_ = -width_ * resolution_ / 2.0;
    origin_y_ = -height_ * resolution_ / 2.0;

    RCLCPP_INFO(this->get_logger(), "=== Occupancy Mapper Starting ===");
    RCLCPP_INFO(this->get_logger(), "Map size: %d x %d cells", width_, height_);
    RCLCPP_INFO(this->get_logger(), "Resolution: %.2f m/cell", resolution_);
    RCLCPP_INFO(this->get_logger(), "Coverage: %.1f x %.1f meters", width_ * resolution_, height_ * resolution_);

    // Initialize map data
    int total_cells = width_ * height_;
    hits_.resize(total_cells, 0);
    passes_.resize(total_cells, 0);

    RCLCPP_INFO(this->get_logger(), "Map data initialized with %d cells", total_cells);

    // Setup map message
    map_msg_.header.frame_id = "odom";
    map_msg_.info.resolution = resolution_;
    map_msg_.info.width = width_;
    map_msg_.info.height = height_;
    map_msg_.info.origin.position.x = origin_x_;
    map_msg_.info.origin.position.y = origin_y_;
    map_msg_.info.origin.position.z = 0.0;
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.data.resize(total_cells, -1);

    // Create subscriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&OccupancyMapper::odom_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&OccupancyMapper::scan_callback, this, std::placeholders::_1));

    // Create publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&OccupancyMapper::publish_map_timer, this));

    RCLCPP_INFO(this->get_logger(), "Subscribed to /odom and /scan");
    RCLCPP_INFO(this->get_logger(), "Publishing map on /map at 2 Hz");
    RCLCPP_INFO(this->get_logger(), "Waiting for data...");
}

void OccupancyMapper::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_ = msg->pose.pose;

    if(!pose_received_)
    {
        RCLCPP_INFO(this->get_logger(), "First pose received");
        pose_received_ = true;
    }
}

void OccupancyMapper::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!pose_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Scan received but no odometry yet - skipping");
        return;
    }

    double robot_x = current_pose_.position.x;
    double robot_y = current_pose_.position.y;
    double robot_theta = get_yaw_from_quaternion(current_pose_.orientation);

    int robot_grid_x, robot_grid_y;
    world_to_grid(robot_x, robot_y, robot_grid_x, robot_grid_y);

    if (!is_valid_cell(robot_grid_x, robot_grid_y)) {
        RCLCPP_WARN(this->get_logger(), "Robot outside map bounds! Skipping this scan");
        return;
    }

    int beams_processed = 0;
    for (size_t i = 0; i < msg->ranges.size(); i++) {
        float range = msg->ranges[i];

        if (range < msg->range_min || range > msg->range_max) {
            continue;
        }
        if (std::isnan(range) || std::isinf(range)) {
            continue;
        }

        double beam_angle = msg->angle_min + i * msg->angle_increment;
        double world_angle = robot_theta + beam_angle;

        double end_x = robot_x + range * std::cos(world_angle);
        double end_y = robot_y + range * std::sin(world_angle);

        int end_grid_x, end_grid_y;
        world_to_grid(end_x, end_y, end_grid_x, end_grid_y);

        if (!is_valid_cell(end_grid_x, end_grid_y)) {
            continue;
        }

        auto cells = bresenham_line(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y);

        for (size_t j = 0; j < cells.size() - 1; j++) {
            int cell_x = cells[j].first;
            int cell_y = cells[j].second;
            int index = cell_y * width_ + cell_x;
            passes_[index]++;
        }

        int hit_x = cells.back().first;
        int hit_y = cells.back().second;
        int hit_index = hit_y * width_ + hit_x;
        hits_[hit_index]++;

        beams_processed++;
    }

    if (beams_processed > 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Processed %d beams...", beams_processed);
    }

 
}


void OccupancyMapper::world_to_grid(double wx, double wy, int& gx, int& gy)
{
    gx = static_cast<int>((wx - origin_x_) / resolution_);
    gy = static_cast<int>((wy - origin_y_) / resolution_);
}

bool OccupancyMapper::is_valid_cell(int x, int y)
{
    return (x >= 0 && x < width_ && y >= 0 && y < height_);
}

double OccupancyMapper::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

void OccupancyMapper::publish_map_timer()
{
    map_msg_.header.stamp = this->now();

    for (int i = 0; i < width_ * height_; i++) {
        int total = hits_[i] + passes_[i];

        if(total == 0) {
            map_msg_.data[i] = -1;
        } else {
            double probability = static_cast<double>(hits_[i]) / total;
            // map_msg_.data[i] = static_cast<int8_t>(probability * 100);

            // threshold: > 0.65 wall. < 0.35 free, otherwise unknown
            if (probability > 0.65) {
                map_msg_.data[i] = 100;
            } else if (probability < 0.35) {
                map_msg_.data[i] = 0;
            } else {
                map_msg_.data[i] = -1;
            }
        }
    }

    map_pub_->publish(map_msg_);
}

std::vector<std::pair<int, int>> OccupancyMapper::bresenham_line(int x0, int y0, int x1, int y1)
{
    std::vector<std::pair<int, int>> cells;
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    while (true) {
        cells.push_back(std::make_pair(x, y));
        
        if (x == x1 && y == y1) {
            break;
        }
        
        int e2 = 2 * err;
        
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


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Compiler figures out the type of the node
    auto node = std::make_shared<OccupancyMapper>();

    RCLCPP_INFO(node->get_logger(), "Node created, starting spin...");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

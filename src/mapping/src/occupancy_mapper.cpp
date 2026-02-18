#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class OccupancyMapper : public rclcpp::Node
{
public:
    OccupancyMapper();

private:
    // Callbacks
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

    // parameters for calculating the occupancy probability
    // use log odds
    std::vector<double> map_log_odds_;

    // case 1:
    const double p_hit_occ = 0.90;  //  p_occ = P(sensor says 'hit' | cell occupied)
    const double p_pass_occ = 1.0 - p_hit_occ; // p_pass_occ = P(sensor says 'pass' | cell occupied)    
    // case 2:
    const double p_hit_free = 0.05;  // p_hit_free = P(sensor says 'hit' | cell free)
    const double p_pass_free = 1.0 - p_hit_free; // p_pass_free = P(sensor says 'pass' | cell free)
    
    double log_odds_hit;
    double log_odds_pass;

    const double log_odds_max = 1000.0;
    const double log_odds_min = -1000.0;

    // TF2 for transform lookup
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::Pose current_pose_;
    bool pose_received_;
    
    // Map message and publisher
    nav_msgs::msg::OccupancyGrid map_msg_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

OccupancyMapper::OccupancyMapper() : Node("occupancy_mapper")
{
    pose_received_ = false;

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

    // Initialize for log odds mapping update
    map_log_odds_.resize(width_ * height_, 0.0); // initialize prior as 0, means no prior knowledge
    
    log_odds_hit = std::log(p_hit_occ / p_hit_free);
    log_odds_pass = std::log(p_pass_occ / p_pass_free);

    RCLCPP_INFO(this->get_logger(), "Log odds hit: %.2f, Log odds pass: %.2f", log_odds_hit, log_odds_pass);

    // Setup map message
    map_msg_.header.frame_id = "odom";
    map_msg_.info.resolution = resolution_;
    map_msg_.info.width = width_;
    map_msg_.info.height = height_;
    map_msg_.info.origin.position.x = origin_x_;
    map_msg_.info.origin.position.y = origin_y_;
    map_msg_.info.origin.position.z = 0.0;
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.data.resize(width_ * height_, -1);

    // Create subscriptions
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&OccupancyMapper::scan_callback, this, std::placeholders::_1));

    // Create publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&OccupancyMapper::publish_map_timer, this));

    RCLCPP_INFO(this->get_logger(), "Subscribed to /scan");
    RCLCPP_INFO(this->get_logger(), "Listening to TF: odom -> base_link");
    RCLCPP_INFO(this->get_logger(), "Publishing map on /map at 2 Hz");
    RCLCPP_INFO(this->get_logger(), "Waiting for data...");
}

void OccupancyMapper::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Look up the transform from base_link to odom at the scan timestamp
    geometry_msgs::msg::TransformStamped transform_stamped;
    
    try {
        // Wait up to 100ms for the transform
        transform_stamped = tf_buffer_->lookupTransform(
            "odom", "base_link",
            msg->header.stamp,
            rclcpp::Duration::from_seconds(0.1));
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        return;
    }
    
    if(!pose_received_)
    {
        RCLCPP_INFO(this->get_logger(), "First transform received");
        pose_received_ = true;
    }

    // Extract pose from transform
    double robot_x = transform_stamped.transform.translation.x;
    double robot_y = transform_stamped.transform.translation.y;
    
    // Convert quaternion to yaw
    tf2::Quaternion q(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, robot_theta;
    m.getRPY(roll, pitch, robot_theta);

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
            
            // update log odds, reduce prob with log odds pass
            map_log_odds_[index] += log_odds_pass;
            if (map_log_odds_[index] < log_odds_min) {
                map_log_odds_[index] = log_odds_min;
            }
        }

        int hit_x = cells.back().first;
        int hit_y = cells.back().second;
        int hit_index = hit_y * width_ + hit_x;
        
        map_log_odds_[hit_index] += log_odds_hit;
        if (map_log_odds_[hit_index] > log_odds_max) {
            map_log_odds_[hit_index] = log_odds_max;
        }

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
        double log_odds = map_log_odds_[i];

        // convert log odds to probability: L = log(P / (1 - P)), so P = 1 / (1 + exp(-L))
        double prob = 1.0 / (1.0 + std::exp(-log_odds));

        // // add threshold with hysteresis to prevent flickering
        // if ( prob > 0.70) {
        //     map_msg_.data[i] = 100; //occupied
        // } else if (prob < 0.30) {
        //     map_msg_.data[i] = 0; // free
        // } else {
        //     map_msg_.data[i] = -1; // unknown
        // }

        // no threshold, just map the log odds to probability
        map_msg_.data[i] = static_cast<int>(prob * 100);
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

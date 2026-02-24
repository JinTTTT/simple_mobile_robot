#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LocalizationNode : public rclcpp::Node 
{
public:
    LocalizationNode();

private:

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // member variables
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_ = false;

};

LocalizationNode::LocalizationNode() : Node("localization_node")
{
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&LocalizationNode::map_callback, this, std::placeholders::_1));
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&LocalizationNode::odom_callback, this, std::placeholders::_1));
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&LocalizationNode::scan_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Localization node has been started, waiting for /map, /odom, and /scan topics...");

}

void LocalizationNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg; // copy the map message, store it
    map_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Map received: %d x %d cells, resolution %.3f m/cell", 
        msg->info.width, msg->info.height, msg->info.resolution);
}

void LocalizationNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr /*msg*/)
{
    // double x = msg->pose.pose.position.x;
    // double y = msg->pose.pose.position.y;
    // RCLCPP_INFO(this->get_logger(), "Odometry received: x=%.3f, y=%.3f", x, y);
}

void LocalizationNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/)
{
    //RCLCPP_INFO(this->get_logger(), "Scan received: %zu ranges, angle_min=%.2f, angle_max=%.2f",
    //    msg->ranges.size(), msg->angle_min, msg->angle_max);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationNode>();
    RCLCPP_INFO(node->get_logger(), "Localization node created, spinning...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

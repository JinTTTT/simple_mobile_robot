#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class TestNode : public rclcpp::Node
{
public:
    TestNode() : Node("test_node")
    {
        // Print hello message
        RCLCPP_INFO(this->get_logger(), "Hello World! Test node started.");
        
        // Subscribe to laser scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&TestNode::scan_callback, this, std::placeholders::_1)
        );
        
        // Timer that prints every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TestNode::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Waiting for laser scans on /scan...");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received scan with %zu points", msg->ranges.size());
    }
    
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Tick... (1 second passed)");
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}
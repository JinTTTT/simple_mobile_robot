#include "rclcpp/rclcpp.hpp"

class PathFollowControlNode : public rclcpp::Node
{
public:
  PathFollowControlNode()
  : Node("path_follow_control_node")
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Path follow control package initialized. Controller implementation will be added next.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowControlNode>());
  rclcpp::shutdown();
  return 0;
}

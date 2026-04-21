#include "rclcpp/rclcpp.hpp"

class MotionPlanningNode : public rclcpp::Node
{
public:
  MotionPlanningNode()
  : Node("motion_planning_node")
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Motion planning package initialized. Planner implementation will be added next.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionPlanningNode>());
  rclcpp::shutdown();
  return 0;
}

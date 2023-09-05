#include "trinamic_control_node.hpp"

using trinamic_control::TrinamicControlNode;

TrinamicControlNode::TrinamicControlNode() : Node("trinamic_control_node")
{
  RCLCPP_INFO(this->get_logger(), "hello world!");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrinamicControlNode>());
  rclcpp::shutdown();
  return 0;
}
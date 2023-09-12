#include "trinamic_control_node.hpp"

using trinamic_control::TrinamicControlNode;

TrinamicControlNode::TrinamicControlNode() : Node("trinamic_control_node")
{
  this->declare_parameter("config_file_path", "noch_nicht_da");
  RCLCPP_INFO(this->get_logger(), "hello trinamic control node!");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrinamicControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
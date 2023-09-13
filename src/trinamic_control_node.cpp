#include "trinamic_control_node.hpp"

#include "trinamic_control/trinamic_joint/trinamic_joint.hpp"

#include <yaml-cpp/yaml.h>

using trinamic_control::TrinamicControlNode;

TrinamicControlNode::TrinamicControlNode() : Node("trinamic_control_node")
{
  
  this->declare_parameter("config_file_path", "noch_nicht_da");

  std::string config_file_path = this->get_parameter("config_file_path").as_string();
  
  YAML::Node yaml = YAML::LoadFile(config_file_path);

  RCLCPP_INFO(this->get_logger(), "joints %ld", yaml["joints"].size());
/*
  RCLCPP_INFO(this->get_logger(), "hello trinamic control node!");

  RCLCPP_INFO(
      this->get_logger(),
      "Parameter blackboard node named '%s' ready, and serving '%zu' parameters already!",
      this->get_fully_qualified_name(), this->list_parameters(
        {}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE).names.size());

  std::vector<std::string> prefix;
  prefix.emplace_back(this->get_fully_qualified_name());
  auto parameters = this->list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
  
  for (auto parameter : parameters.names)
  {
    RCLCPP_INFO(this->get_logger(), "%s", parameter.c_str());
  }
*/
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<TrinamicControlNode>());
  rclcpp::shutdown();
  return 0;
}
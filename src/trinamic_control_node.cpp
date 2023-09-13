#include "trinamic_control_node.hpp"

//#include "trinamic_control/trinamic_joint/trinamic_joint.hpp"

#include <yaml-cpp/yaml.h>

using trinamic_control::TrinamicControlNode;

TrinamicControlNode::TrinamicControlNode() : Node("trinamic_control_node")
{
  this->declare_parameter("config_file_path", "noch_nicht_da");
  std::string config_file_path = this->get_parameter("config_file_path").as_string();
  
  YAML::Node yaml2 = YAML::Load("{address: 0, identifier: tsp_i, description: tsp_d}");
  TrinamicParameter tp(yaml2);

  YAML::Node yaml = YAML::LoadFile(config_file_path);
  for (auto joint_yaml : yaml["joints"])
  {
    this->joints.emplace_back(TrinamicJoint(joint_yaml));
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrinamicControlNode>());
  rclcpp::shutdown();
  return 0;
}
#include <rclcpp/rclcpp.hpp>

#include <trinamic_control/trinamic_joint/trinamic_joint.hpp>

namespace trinamic_control
{
class TrinamicControlNode : public rclcpp::Node
{
public:
  TrinamicControlNode();
private:
  std::vector<TrinamicJoint> joints;
};
}
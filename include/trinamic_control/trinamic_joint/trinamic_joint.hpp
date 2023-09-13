// Copyright 2023 Eugen Kaltenegger

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

namespace trinamic_control
{
class TrinamicJoint
{
public:
  TrinamicJoint(YAML::Node yaml);
private:
// name

// connection(port, baud)

// config

// position_resolution
// velocity_resolution
// effort_resolution

// current_position
// current_velocity
// current_effort

// target_position
// target_velocity
// target_effort
};
}
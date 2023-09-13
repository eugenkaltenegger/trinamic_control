// Copyright 2023 Eugen Kaltenegger

#include <yaml-cpp/yaml.h>

#include <trinamic_control/trinamic_config/trinamic_config.hpp>
#include <trinamic_control/trinamic_connection/trinamic_connection.hpp>
#include <trinamic_control/trinamic_parameter/trinamic_parameter.hpp>

namespace trinamic_control
{
class TrinamicJoint
{
public:
  TrinamicJoint(YAML::Node yaml);
  void writePosition(double target);
  void writeVelocity(double target);
  void writeEffort(double target);
  void read();
private:
  std::shared_ptr<std::string> name_;

  std::shared_ptr<TrinamicConnection> connection_;
  std::shared_ptr<TrinamicConfig> config_;

  double diameter_ {0.0};

  double position_resolution_ {0.0};
  double velocity_resolution_ {0.0};
  double effort_resolution_ {0.0};

  std::unique_ptr<TrinamicParameter> current_position_parameter_;
  std::unique_ptr<TrinamicParameter> current_velocity_parameter_;
  std::unique_ptr<TrinamicParameter> current_effort_parameter_;

  double current_position_ {0.0};
  double current_velocity_ {0.0};
  double current_effort_ {0.0};

  std::unique_ptr<TrinamicParameter> target_position_parameter_;
  std::unique_ptr<TrinamicParameter> target_velocity_parameter_;
  std::unique_ptr<TrinamicParameter> target_effort_parameter_;

  double target_position_ {0.0};
  double target_velocity_ {0.0};
  double target_effort_ {0.0};
};
}
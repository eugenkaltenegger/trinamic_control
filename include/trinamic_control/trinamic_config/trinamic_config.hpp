// Copyright 2023 Eugen Kaltenegger

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

#include <trinamic_control/trinamic_parameter/trinamic_parameter.hpp>

namespace trinamic_control
{
class TrinamicConfig
{
public:
  TrinamicConfig(YAML::Node yaml);
  std::shared_ptr<std::list<std::pair<TrinamicParameter, int>>> getConfigList();
private:
  std::shared_ptr<std::list<std::pair<TrinamicParameter, int>>> config_list_;
};
}  // namespace trinamic_control
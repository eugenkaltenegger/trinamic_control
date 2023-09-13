// Copyright 2023 Eugen Kaltenegger

#include <trinamic_control/trinamic_config/trinamic_config.hpp>

using trinamic_control::TrinamicConfig;
using trinamic_control::TrinamicParameter;


TrinamicConfig::TrinamicConfig(YAML::Node yaml)
{
  // TODO
  return;
}

std::shared_ptr<std::list<std::pair<TrinamicParameter, int>>> TrinamicConfig::getConfigList()
{
  // TODO
  return nullptr;
}
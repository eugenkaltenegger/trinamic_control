// Copyright 2023 Eugen Kaltenegger

#ifndef TUW_HARDWARE_INTERFACE_TEMPLATE_GENERIC_HARDWARE_PARAMETER_H
#define TUW_HARDWARE_INTERFACE_TEMPLATE_GENERIC_HARDWARE_PARAMETER_H

#include <memory>
#include <string>

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

namespace trinamic_control
{
class TrinamicParameter
{
public:
  TrinamicParameter(YAML::Node yaml);
  int getAddress();
  std::string getIdentifier();
  std::string getDescription();
protected:
  std::unique_ptr<int> address_ {nullptr};
  std::unique_ptr<std::string> identifier_ {nullptr};
  std::unique_ptr<std::string> description_ {nullptr};
};
}  // namespace trinamic_control

#endif  // TUW_HARDWARE_INTERFACE_TEMPLATE_GENERIC_HARDWARE_PARAMETER_H
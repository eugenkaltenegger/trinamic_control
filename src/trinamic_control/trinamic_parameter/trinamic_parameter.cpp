// Copyright 202 Eugen Kaltenegger

#include "trinamic_control/trinamic_parameter/trinamic_parameter.hpp"

using trinamic_control::TrinamicParameter;

TrinamicParameter::TrinamicParameter(YAML::Node yaml)
{
  if (yaml["address"].IsDefined() && !yaml["address"].IsNull()) {
    this->address_ = std::make_unique<int>(yaml["address"].as<int>());
  }

  if (yaml["identifier"].IsDefined() && !yaml["identifier"].IsNull()) {
    this->identifier_ = std::make_unique<std::string>(yaml["identifier"].as<std::string>());
  }

  if (yaml["description"].IsDefined() && !yaml["description"].IsNull()) {
    this->description_ = std::make_unique<std::string>(yaml["description"].as<std::string>());
  }
}

int TrinamicParameter::getAddress()
{
  return *this->address_;
}

std::string TrinamicParameter::getIdentifier()
{
  return *this->identifier_;
}

std::string TrinamicParameter::getDescription()
{
  return *this->description_;
}
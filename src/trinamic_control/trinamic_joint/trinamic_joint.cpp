// Copyright 2023 Eugen Kaltenegger

#include "trinamic_control/trinamic_joint/trinamic_joint.hpp"

#include <yaml-cpp/yaml.h>

using trinamic_control::TrinamicJoint;

TrinamicJoint::TrinamicJoint(YAML::Node yaml)
{
  this->name_ = std::make_shared<std::string>(yaml["name"].as<std::string>());

  this->diameter_ = yaml["diameter"].as<double>();

  this->position_resolution_ = yaml["resolution"]["position"].as<double>();
  this->velocity_resolution_ = yaml["resolution"]["velocity"].as<double>();
  this->effort_resolution_  = yaml["resolution"]["effort"].as<double>();
}
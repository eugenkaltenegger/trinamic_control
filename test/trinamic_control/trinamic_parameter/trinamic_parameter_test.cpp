// Copyright 2023 Eugen Kaltenegger

/// Note this may have a problem: https://github.com/google/googletest/issues/930

#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <trinamic_control/trinamic_parameter/trinamic_parameter.hpp>

using trinamic_control::TrinamicParameter;

TEST(TrinamicParameterTest, target_state_parameter_test)
{
  YAML::Node yaml = YAML::Load("{address: 0, identifier: tsp_i, description: tsp_d}");
  TrinamicParameter tp(yaml);

  ASSERT_EQ(tp.getAddress(), 0);
  ASSERT_EQ(tp.getIdentifier(), "tsp_i");
  ASSERT_EQ(tp.getDescription(), "tsp_d");
}
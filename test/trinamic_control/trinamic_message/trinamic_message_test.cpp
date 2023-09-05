// Copyright 2023 Eugen Kaltenegger

/// Note this may have a problem: https://github.com/google/googletest/issues/930

#include <gtest/gtest.h>

#include <trinamic_control/trinamic_message/trinamic_message.hpp>

using trinamic_control::TrinamicMessage;

TEST(TrinamicMessageTest, verifyChecksumSetVelocity)
{
  TrinamicMessage message(1, 5, 2, 0, 0);
  ASSERT_EQ(message.calculateChecksum(), 8);
}
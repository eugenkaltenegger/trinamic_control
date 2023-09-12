// Copyright 2023 Eugen Kaltenegger

/// Note this may have a problem: https://github.com/google/googletest/issues/930

#include <gtest/gtest.h>

#include <trinamic_control/trinamic_message/implementation/trinamic_response.hpp>

using trinamic_control::TrinamicMessageResponse;

TEST(TrinamicTest, getVelocity0)
{
  TrinamicMessageResponse reply(2, 1, 100, 5, 0);
  ASSERT_EQ(reply.toString(), "02 01 64 05 00 00 00 00 6C");
}

TEST(TrinamicTest, setVelocity10)
{
  TrinamicMessageResponse reply(2, 1, 100, 5, 10);
  ASSERT_EQ(reply.toString(), "02 01 64 05 00 00 00 0A 76");
}
TEST(TrinamicTest, setVelocity100)
{
  TrinamicMessageResponse reply(2, 1, 100, 5, 100);
  ASSERT_EQ(reply.toString(), "02 01 64 05 00 00 00 64 D0");
}

TEST(TrinamicTest, setVelocity1000)
{
  TrinamicMessageResponse reply(2, 1, 100, 5, 1000);
  ASSERT_EQ(reply.toString(), "02 01 64 05 00 00 03 E8 57");
}

TEST(TrinamicTest, getVelocity)
{
  TrinamicMessageResponse reply(2, 1, 100, 6, 0);
  ASSERT_EQ(reply.toString(), "02 01 64 06 00 00 00 00 6D");
}
#include <gtest/gtest.h>
#include "fast_lio/common/TimeUtils.hpp"

using namespace fast_lio;

TEST(TimeUtilsTest, GetTimeSec)
{
    builtin_interfaces::msg::Time msg_time;
    msg_time.sec = 100;
    msg_time.nanosec = 500000000; // 0.5 seconds

    double time_sec = time_utils::getTimeSec(msg_time);
    EXPECT_DOUBLE_EQ(time_sec, 100.5);
}

TEST(TimeUtilsTest, GetRosTime)
{
    double timestamp = 100.5;
    auto ros_time = time_utils::getRosTime(timestamp);

    EXPECT_EQ(ros_time.seconds(), timestamp);
}

TEST(TimeUtilsTest, RoundTrip)
{
    // Test that conversion is reversible
    double original = 123.456789;
    auto ros_time = time_utils::getRosTime(original);

    builtin_interfaces::msg::Time msg_time;
    msg_time.sec = ros_time.seconds();
    msg_time.nanosec = ros_time.nanoseconds() % 1000000000;

    double recovered = time_utils::getTimeSec(msg_time);

    EXPECT_NEAR(original, recovered, 1e-9);
}
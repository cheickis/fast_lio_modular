#pragma once

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace fast_lio
{
    namespace time_utils
    {

        inline double getTimeSec(const builtin_interfaces::msg::Time &time)
        {
            return rclcpp::Time(time).seconds();
        }

        inline rclcpp::Time getRosTime(double timestamp)
        {
            int32_t sec = std::floor(timestamp);
            uint32_t nanosec = static_cast<uint32_t>((timestamp - sec) * 1e9);
            return rclcpp::Time(sec, nanosec);
        }
    }
} // namespace fast_lio
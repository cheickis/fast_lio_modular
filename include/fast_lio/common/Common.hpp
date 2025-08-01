#pragma once

// Include all common headers
#include "fast_lio/common/Constants.hpp"
#include "fast_lio/common/Types.hpp"
#include "fast_lio/common/DataStructures.hpp"
#include "fast_lio/common/MathUtils.hpp"
#include "fast_lio/common/GeometryUtils.hpp"
#include "fast_lio/common/TimeUtils.hpp"

// Backward compatibility aliases
namespace fast_lio
{
    // Import into global namespace for easier migration
    using namespace types;
    using namespace constants;

    // Utility functions
    using geometry::calcDist;
    using math_utils::deg2rad;
    using math_utils::rad2deg;
    using time_utils::getRosTime;
    using time_utils::getTimeSec;

    // Backward compatibility function names
    inline float calc_dist(const PointType &p1, const PointType &p2)
    {
        return geometry::calcDist(p1, p2);
    }

    inline double get_time_sec(const builtin_interfaces::msg::Time &time)
    {
        return time_utils::getTimeSec(time);
    }

    inline rclcpp::Time get_ros_time(double timestamp)
    {
        return time_utils::getRosTime(timestamp);
    }

    template <typename T>
    auto set_pose6d(double t, const Eigen::Matrix<T, 3, 1> &a,
                    const Eigen::Matrix<T, 3, 1> &g,
                    const Eigen::Matrix<T, 3, 1> &v,
                    const Eigen::Matrix<T, 3, 1> &p,
                    const Eigen::Matrix<T, 3, 3> &R)
    {
        return geometry::setPose6D(t, a, g, v, p, R);
    }
}
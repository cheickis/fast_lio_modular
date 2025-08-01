#pragma once

#include "Constants.hpp"
#include "Types.hpp"

namespace fast_lio
{
    namespace math_utils
    {

        template <typename T>
        inline T rad2deg(T radians)
        {
            return radians * 180.0 / constants::PI;
        }

        template <typename T>
        inline T deg2rad(T degrees)
        {
            return degrees * constants::PI / 180.0;
        }

        template <typename T>
        inline T constrain(T v, T min, T max)
        {
            return (v > min) ? ((v < max) ? v : max) : min;
        }

        // Validation
        template <typename T>
        inline bool isValid(T a)
        {
            return std::abs(a) <= 1e8;
        }

// Convenience macros (for backward compatibility)
#define CONSTRAIN(v, min, max) fast_lio::math_utils::constrain(v, min, max)
#define IS_VALID(a) fast_lio::math_utils::isValid(a)
    }
} // namespace fast_lio
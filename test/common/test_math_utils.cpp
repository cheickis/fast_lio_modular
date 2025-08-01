#include <gtest/gtest.h>
#include "fast_lio/common/MathUtils.hpp"

using namespace fast_lio;

TEST(MathUtilsTest, RadToDeg)
{
    EXPECT_NEAR(math_utils::rad2deg(constants::PI), 180.0, 1e-10);
    EXPECT_NEAR(math_utils::rad2deg(constants::PI / 2), 90.0, 1e-10);
    EXPECT_NEAR(math_utils::rad2deg(0.0), 0.0, 1e-10);
}

TEST(MathUtilsTest, DegToRad)
{
    EXPECT_NEAR(math_utils::deg2rad(180.0), constants::PI, 1e-10);
    EXPECT_NEAR(math_utils::deg2rad(90.0), constants::PI / 2, 1e-10);
    EXPECT_NEAR(math_utils::deg2rad(0.0), 0.0, 1e-10);
}

TEST(MathUtilsTest, Constrain)
{
    EXPECT_EQ(math_utils::constrain(5.0, 0.0, 10.0), 5.0);
    EXPECT_EQ(math_utils::constrain(-5.0, 0.0, 10.0), 0.0);
    EXPECT_EQ(math_utils::constrain(15.0, 0.0, 10.0), 10.0);

    // Test with integers
    EXPECT_EQ(math_utils::constrain(5, 0, 10), 5);
    EXPECT_EQ(math_utils::constrain(-5, 0, 10), 0);
    EXPECT_EQ(math_utils::constrain(15, 0, 10), 10);
}

TEST(MathUtilsTest, IsValid)
{
    EXPECT_TRUE(math_utils::isValid(0.0));
    EXPECT_TRUE(math_utils::isValid(1e8));
    EXPECT_TRUE(math_utils::isValid(-1e8));
    EXPECT_FALSE(math_utils::isValid(1e9));
    EXPECT_FALSE(math_utils::isValid(-1e9));
}
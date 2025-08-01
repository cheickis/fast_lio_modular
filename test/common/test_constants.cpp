#include <gtest/gtest.h>
#include "fast_lio/common/Constants.hpp"

using namespace fast_lio::constants;

TEST(ConstantsTest, MathConstants)
{
    EXPECT_NEAR(PI, 3.14159265358, 1e-10);
    EXPECT_EQ(PI_M, PI);
}

TEST(ConstantsTest, PhysicalConstants)
{
    EXPECT_NEAR(G_m_s2, 9.81, 1e-10);
}

TEST(ConstantsTest, SystemDimensions)
{
    EXPECT_EQ(DIM_STATE, 18);
    EXPECT_EQ(DIM_PROC_N, 12);
}

TEST(ConstantsTest, AlgorithmParameters)
{
    EXPECT_EQ(CUBE_LEN, 6.0);
    EXPECT_EQ(LIDAR_SP_LEN, 2);
    EXPECT_EQ(INIT_COV, 1.0);
    EXPECT_EQ(NUM_MATCH_POINTS, 5);
    EXPECT_EQ(MAX_MEAS_DIM, 10000);
    EXPECT_EQ(MAXN, 720000);
}
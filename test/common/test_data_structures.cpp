#include <gtest/gtest.h>
#include "fast_lio/common/DataStructures.hpp"

using namespace fast_lio;

TEST(MeasureGroupTest, DefaultConstruction)
{
    MeasureGroup meas;

    EXPECT_DOUBLE_EQ(meas.lidar_beg_time, 0.0);
    EXPECT_DOUBLE_EQ(meas.lidar_end_time, 0.0);
    EXPECT_NE(meas.lidar, nullptr);
    EXPECT_TRUE(meas.imu.empty());
}

TEST(StatesGroupTest, DefaultConstruction)
{
    StatesGroup states;

    EXPECT_TRUE(states.rot_end.isApprox(types::M3D::Identity()));
    EXPECT_TRUE(states.pos_end.isZero());
    EXPECT_TRUE(states.vel_end.isZero());
    EXPECT_TRUE(states.bias_g.isZero());
    EXPECT_TRUE(states.bias_a.isZero());
    EXPECT_TRUE(states.gravity.isZero());
}

TEST(StatesGroupTest, Addition)
{
    StatesGroup s1;
    s1.pos_end = types::V3D(1, 2, 3);
    s1.vel_end = types::V3D(4, 5, 6);

    Eigen::Matrix<double, constants::DIM_STATE, 1> delta;
    delta.setZero();
    delta.block<3, 1>(3, 0) = types::V3D(1, 1, 1); // position delta
    delta.block<3, 1>(6, 0) = types::V3D(2, 2, 2); // velocity delta

    StatesGroup s2 = s1 + delta;

    EXPECT_TRUE(s2.pos_end.isApprox(types::V3D(2, 3, 4)));
    EXPECT_TRUE(s2.vel_end.isApprox(types::V3D(6, 7, 8)));
}

TEST(StatesGroupTest, ResetPose)
{
    StatesGroup states;
    states.pos_end = types::V3D(1, 2, 3);
    states.vel_end = types::V3D(4, 5, 6);
    states.rot_end = types::M3D::Random();

    states.resetpose();

    EXPECT_TRUE(states.rot_end.isApprox(types::M3D::Identity()));
    EXPECT_TRUE(states.pos_end.isZero());
    EXPECT_TRUE(states.vel_end.isZero());
    // bias and gravity should remain unchanged
}
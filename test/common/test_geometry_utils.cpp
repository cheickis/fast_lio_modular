#include <gtest/gtest.h>
#include "fast_lio/common/GeometryUtils.hpp"

using namespace fast_lio;

class GeometryUtilsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create test points
        p1.x = 1.0;
        p1.y = 2.0;
        p1.z = 3.0;
        p2.x = 4.0;
        p2.y = 5.0;
        p2.z = 6.0;

        // Create points for plane fitting
        plane_points.resize(5);
        // Points on plane z = 1
        for (int i = 0; i < 5; i++)
        {
            plane_points[i].x = i;
            plane_points[i].y = i * 2;
            plane_points[i].z = 1.0;
        }
    }

    types::PointType p1, p2;
    types::PointVector plane_points;
};

TEST_F(GeometryUtilsTest, CalcDist)
{
    float dist = geometry::calcDist(p1, p2);
    float expected = 3 * 3 + 3 * 3 + 3 * 3; // 27
    EXPECT_FLOAT_EQ(dist, expected);
}

TEST_F(GeometryUtilsTest, SetPose6D)
{
    types::V3D acc(1, 2, 3);
    types::V3D gyr(4, 5, 6);
    types::V3D vel(7, 8, 9);
    types::V3D pos(10, 11, 12);
    types::M3D rot = types::M3D::Identity();

    auto pose = geometry::setPose6D(1.5, acc, gyr, vel, pos, rot);

    EXPECT_DOUBLE_EQ(pose.offset_time, 1.5);
    EXPECT_FLOAT_EQ(pose.acc[0], 1);
    EXPECT_FLOAT_EQ(pose.acc[1], 2);
    EXPECT_FLOAT_EQ(pose.acc[2], 3);
    EXPECT_FLOAT_EQ(pose.gyr[0], 4);
    EXPECT_FLOAT_EQ(pose.vel[0], 7);
    EXPECT_FLOAT_EQ(pose.pos[0], 10);
}

TEST_F(GeometryUtilsTest, EstiPlane)
{
    VF(4)
    pca_result;
    float threshold = 0.1f;

    bool success = geometry::estiPlane(pca_result, plane_points, threshold);

    EXPECT_TRUE(success);
    // For plane z = 1, normal should be approximately (0, 0, 1)
    EXPECT_NEAR(pca_result(0), 0.0f, 0.1f);
    EXPECT_NEAR(pca_result(1), 0.0f, 0.1f);
    EXPECT_NEAR(abs(pca_result(2)), 1.0f, 0.1f);
}
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "fast_lio/config/Config.hpp"

using namespace fast_lio;

class ConfigTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Initialize ROS for parameter testing
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
    }

    void TearDown() override
    {
        // Cleanup if needed
    }
};

TEST_F(ConfigTest, DefaultValues)
{
    Config config;

    // Test algorithm defaults
    EXPECT_DOUBLE_EQ(config.algorithm.init_time, 0.1);
    EXPECT_DOUBLE_EQ(config.algorithm.laser_point_cov, 0.001);
    EXPECT_EQ(config.algorithm.pubframe_period, 20);
    EXPECT_EQ(config.algorithm.max_iterations, 4);
    EXPECT_FLOAT_EQ(config.algorithm.det_range, 300.0f);
    EXPECT_FLOAT_EQ(config.algorithm.mov_threshold, 1.5f);

    // Test lidar defaults
    EXPECT_EQ(config.lidar.type, 1);
    EXPECT_EQ(config.lidar.topic, "/livox/lidar");
    EXPECT_DOUBLE_EQ(config.lidar.fov_degree, 180.0);

    // Test IMU defaults
    EXPECT_EQ(config.imu.topic, "/livox/imu");
    EXPECT_DOUBLE_EQ(config.imu.gyr_cov, 0.1);
    EXPECT_DOUBLE_EQ(config.imu.acc_cov, 0.1);

    // Test publishing defaults
    EXPECT_TRUE(config.publish.path_enable);
    EXPECT_FALSE(config.publish.effect_map_enable);
    EXPECT_TRUE(config.publish.scan_publish_enable);
}

TEST_F(ConfigTest, ComputeDerivedValues)
{
    Config config;
    config.lidar.fov_degree = 70.0;
    config.computeDerivedValues();

    // Check FOV expansion
    EXPECT_DOUBLE_EQ(config.lidar.fov_deg_expanded, 80.0);

    // Check half FOV cosine
    double expected_cos = cos(80.0 * 0.5 * M_PI / 180.0);
    EXPECT_NEAR(config.lidar.half_fov_cos, expected_cos, 1e-10);

    // Test FOV clamping
    config.lidar.fov_degree = 180.0;
    config.computeDerivedValues();
    EXPECT_DOUBLE_EQ(config.lidar.fov_deg_expanded, 179.9);
}

TEST_F(ConfigTest, Validation)
{
    Config config;

    // Valid config should pass
    EXPECT_NO_THROW(config.validate());

    // Invalid detection range
    config.algorithm.det_range = -1.0;
    EXPECT_THROW(config.validate(), std::invalid_argument);
    config.algorithm.det_range = 300.0;

    // Invalid filter size
    config.algorithm.filter_size_map = 0.0;
    EXPECT_THROW(config.validate(), std::invalid_argument);
    config.algorithm.filter_size_map = 0.5;

    // Invalid FOV
    config.lidar.fov_degree = 400.0;
    EXPECT_THROW(config.validate(), std::invalid_argument);
    config.lidar.fov_degree = 180.0;
}

TEST_F(ConfigTest, LoadFromROSParams)
{
    // Create a test node with parameters
    auto node = std::make_shared<rclcpp::Node>("test_config_node");

    // Set some test parameters
    node->declare_parameter("max_iteration", 6);
    node->declare_parameter("mapping.det_range", 500.0f);
    node->declare_parameter("common.lid_topic", "/test/lidar");
    node->declare_parameter("mapping.gyr_cov", 0.2);
    node->declare_parameter("publish.path_en", false);

    // Load config
    Config config = Config::fromROSParams(node.get());

    // Verify loaded values
    EXPECT_EQ(config.algorithm.max_iterations, 6);
    EXPECT_FLOAT_EQ(config.algorithm.det_range, 500.0f);
    EXPECT_EQ(config.lidar.topic, "/test/lidar");
    EXPECT_DOUBLE_EQ(config.imu.gyr_cov, 0.2);
    EXPECT_FALSE(config.publish.path_enable);
}

TEST_F(ConfigTest, ExtrinsicParameters)
{
    Config config;

    // Check default extrinsic translation
    EXPECT_EQ(config.imu.extrinsic_T.size(), 3);
    EXPECT_DOUBLE_EQ(config.imu.extrinsic_T[0], 0.0);
    EXPECT_DOUBLE_EQ(config.imu.extrinsic_T[1], 0.0);
    EXPECT_DOUBLE_EQ(config.imu.extrinsic_T[2], 0.0);

    // Check default extrinsic rotation (identity)
    EXPECT_EQ(config.imu.extrinsic_R.size(), 9);
    EXPECT_DOUBLE_EQ(config.imu.extrinsic_R[0], 1.0);
    EXPECT_DOUBLE_EQ(config.imu.extrinsic_R[4], 1.0);
    EXPECT_DOUBLE_EQ(config.imu.extrinsic_R[8], 1.0);
}

TEST_F(ConfigTest, MemoryConstants)
{
    // Test that memory constants are accessible
    EXPECT_EQ(Config::MemoryParams::MAXN, 720000);
    EXPECT_EQ(Config::MemoryParams::MAX_POINTS, 100000);
}

// Integration test with migration bridge
TEST_F(ConfigTest, MigrationBridgeCompatibility)
{
    // This test would verify that the macros work correctly
    // It would be in a separate file that includes the migration bridge

    // For now, just test that we can create and use config
    Config config;
    config.algorithm.det_range = 250.0f;

    // In actual use, the macro would expand to g_config.algorithm.det_range
    EXPECT_FLOAT_EQ(config.algorithm.det_range, 250.0f);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
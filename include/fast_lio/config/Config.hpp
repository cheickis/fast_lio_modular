#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

namespace fast_lio
{

    /**
     * @brief Configuration management for FAST-LIO
     *
     * This class centralizes all configuration parameters, replacing the
     * numerous global variables in the original implementation.
     */
    class Config
    {
    public:
        // ============== Core Algorithm Parameters ==============
        struct AlgorithmParams
        {
            // Timing
            double init_time = 0.1;         // INIT_TIME - Initialization period
            double laser_point_cov = 0.001; // LASER_POINT_COV - Point covariance
            int pubframe_period = 20;       // PUBFRAME_PERIOD - Publishing period

            // Iteration control
            int max_iterations = 4;   // NUM_MAX_ITERATIONS
            int num_match_points = 5; // NUM_MATCH_POINTS (from common_lib.h)

            // Map parameters
            float det_range = 300.0f;        // DET_RANGE - Detection range
            float mov_threshold = 1.5f;      // MOV_THRESHOLD - Movement threshold
            double cube_side_length = 200.0; // cube_len - Map cube size

            // Filter sizes
            double filter_size_corner = 0.5; // filter_size_corner_min
            double filter_size_surf = 0.5;   // filter_size_surf_min
            double filter_size_map = 0.5;    // filter_size_map_min
        } algorithm;

        // ============== Sensor Configuration ==============
        struct LidarParams
        {
            // Type and topics
            int type = 1;                       // AVIA=1, VELODYNE=2, etc. (from preprocess.h)
            std::string topic = "/livox/lidar"; // lid_topic

            // FOV parameters
            double fov_degree = 180.0;  // fov_deg
            double blind_radius = 0.01; // From preprocess

            // Scan parameters
            int scan_lines = 16;      // N_SCANS
            int scan_rate = 10;       // SCAN_RATE
            int point_filter_num = 2; // point_filter_num
            int timestamp_unit = 2;   // US=2, MS=3 (from preprocess.h)

            // Feature extraction
            bool feature_enabled = false; // feature_extract_enable

            // Computed values (set in constructor)
            double fov_deg_expanded = 0.0; // FOV_DEG = (fov_deg + 10.0)
            double half_fov_cos = 0.0;     // HALF_FOV_COS
        } lidar;

        struct IMUParams
        {
            std::string topic = "/livox/imu"; // imu_topic

            // Covariances
            double gyr_cov = 0.1;      // gyr_cov
            double acc_cov = 0.1;      // acc_cov
            double b_gyr_cov = 0.0001; // b_gyr_cov
            double b_acc_cov = 0.0001; // b_acc_cov

            // Extrinsic calibration
            std::vector<double> extrinsic_T{0.0, 0.0, 0.0}; // extrinT
            std::vector<double> extrinsic_R{9, 0.0};        // extrinR (identity by default)

            // Initialize R as identity
            IMUParams()
            {
                extrinsic_R = {1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
            }
        } imu;

        // ============== Time Synchronization ==============
        struct TimeSyncParams
        {
            bool enable = false;                   // time_sync_en
            double time_offset_lidar_to_imu = 0.0; // time_diff_lidar_to_imu
        } time_sync;

        // ============== Publishing Configuration ==============
        struct PublishParams
        {
            // Path and visualization
            bool path_enable = true;        // path_en
            bool effect_map_enable = false; // effect_pub_en
            bool map_enable = false;        // map_pub_en

            // Point cloud publishing
            bool scan_publish_enable = true;    // scan_pub_en
            bool dense_publish_enable = true;   // dense_pub_en
            bool scan_bodyframe_enable = false; // scan_body_pub_en
        } publish;

        // ============== PCD Save Configuration ==============
        struct PCDSaveParams
        {
            bool enable = false;        // pcd_save_en
            std::string save_path = ""; // map_file_path
            int interval = -1;          // pcd_save_interval
        } pcd_save;

        // ============== Runtime Options ==============
        struct RuntimeParams
        {
            bool position_log_enable = false;        // runtime_pos_log
            bool extrinsic_estimation_enable = true; // extrinsic_est_en
            std::string root_dir = ".";              // root_dir (from ROOT_DIR macro)
        } runtime;

        // ============== Memory Pre-allocation ==============
        struct MemoryParams
        {
            static constexpr int MAXN = 720000;       // MAXN - Max array size
            static constexpr int MAX_POINTS = 100000; // For point arrays
        } memory;

        // ============== Methods ==============

        /**
         * @brief Load configuration from ROS parameters
         */
        static Config fromROSParams(rclcpp::Node *node)
        {
            Config config;

            // Algorithm parameters
            node->get_parameter_or("max_iteration", config.algorithm.max_iterations, 4);
            node->get_parameter_or("mapping.det_range", config.algorithm.det_range, 300.0f);
            node->get_parameter_or("cube_side_length", config.algorithm.cube_side_length, 200.0);
            node->get_parameter_or("filter_size_corner", config.algorithm.filter_size_corner, 0.5);
            node->get_parameter_or("filter_size_surf", config.algorithm.filter_size_surf, 0.5);
            node->get_parameter_or("filter_size_map", config.algorithm.filter_size_map, 0.5);

            // Lidar parameters
            node->get_parameter_or("common.lid_topic", config.lidar.topic, std::string("/livox/lidar"));
            node->get_parameter_or("preprocess.lidar_type", config.lidar.type, 1);
            node->get_parameter_or("mapping.fov_degree", config.lidar.fov_degree, 180.0);
            node->get_parameter_or("preprocess.blind", config.lidar.blind_radius, 0.01);
            node->get_parameter_or("preprocess.scan_line", config.lidar.scan_lines, 16);
            node->get_parameter_or("preprocess.scan_rate", config.lidar.scan_rate, 10);
            node->get_parameter_or("preprocess.timestamp_unit", config.lidar.timestamp_unit, 2);
            node->get_parameter_or("point_filter_num", config.lidar.point_filter_num, 2);
            node->get_parameter_or("feature_extract_enable", config.lidar.feature_enabled, false);

            // IMU parameters
            node->get_parameter_or("common.imu_topic", config.imu.topic, std::string("/livox/imu"));
            node->get_parameter_or("mapping.gyr_cov", config.imu.gyr_cov, 0.1);
            node->get_parameter_or("mapping.acc_cov", config.imu.acc_cov, 0.1);
            node->get_parameter_or("mapping.b_gyr_cov", config.imu.b_gyr_cov, 0.0001);
            node->get_parameter_or("mapping.b_acc_cov", config.imu.b_acc_cov, 0.0001);
            node->get_parameter_or("mapping.extrinsic_T", config.imu.extrinsic_T, std::vector<double>{0.0, 0.0, 0.0});
            node->get_parameter_or("mapping.extrinsic_R", config.imu.extrinsic_R,
                                   std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

            // Time sync
            node->get_parameter_or("common.time_sync_en", config.time_sync.enable, false);
            node->get_parameter_or("common.time_offset_lidar_to_imu", config.time_sync.time_offset_lidar_to_imu, 0.0);

            // Publishing
            node->get_parameter_or("publish.path_en", config.publish.path_enable, true);
            node->get_parameter_or("publish.effect_map_en", config.publish.effect_map_enable, false);
            node->get_parameter_or("publish.map_en", config.publish.map_enable, false);
            node->get_parameter_or("publish.scan_publish_en", config.publish.scan_publish_enable, true);
            node->get_parameter_or("publish.dense_publish_en", config.publish.dense_publish_enable, true);
            node->get_parameter_or("publish.scan_bodyframe_pub_en", config.publish.scan_bodyframe_enable, true);

            // PCD save
            node->get_parameter_or("pcd_save.pcd_save_en", config.pcd_save.enable, false);
            node->get_parameter_or("map_file_path", config.pcd_save.save_path, std::string(""));
            node->get_parameter_or("pcd_save.interval", config.pcd_save.interval, -1);
            // Runtime
            node->get_parameter_or("runtime_pos_log_enable", config.runtime.position_log_enable, false);
            node->get_parameter_or("mapping.extrinsic_est_en", config.runtime.extrinsic_estimation_enable, true);

            // Compute derived values
            config.computeDerivedValues();

            return config;
        }

        /**
         * @brief Compute derived values after loading parameters
         */
        void computeDerivedValues()
        {
            // FOV calculations
            lidar.fov_deg_expanded = (lidar.fov_degree + 10.0) > 179.9 ? 179.9 : (lidar.fov_degree + 10.0);
            lidar.half_fov_cos = cos((lidar.fov_deg_expanded) * 0.5 * M_PI / 180.0);
        }

        /**
         * @brief Validate configuration
         */
        void validate() const
        {
            if (algorithm.det_range <= 0)
            {
                throw std::invalid_argument("Detection range must be positive");
            }
            if (algorithm.filter_size_map <= 0)
            {
                throw std::invalid_argument("Map filter size must be positive");
            }
            if (lidar.fov_degree <= 0 || lidar.fov_degree > 360)
            {
                throw std::invalid_argument("FOV must be between 0 and 360 degrees");
            }
            // Add more validation as needed
        }

        /**
         * @brief Print configuration for debugging
         */
        void print(rclcpp::Logger logger) const
        {
            RCLCPP_INFO(logger, "=== FAST-LIO Configuration ===");
            RCLCPP_INFO(logger, "Algorithm:");
            RCLCPP_INFO(logger, "  Detection range: %.1f m", algorithm.det_range);
            RCLCPP_INFO(logger, "  Map cube size: %.1f m", algorithm.cube_side_length);
            RCLCPP_INFO(logger, "  Max iterations: %d", algorithm.max_iterations);

            RCLCPP_INFO(logger, "Lidar:");
            RCLCPP_INFO(logger, "  Type: %d", lidar.type);
            RCLCPP_INFO(logger, "  Topic: %s", lidar.topic.c_str());
            RCLCPP_INFO(logger, "  FOV: %.1f degrees", lidar.fov_degree);

            RCLCPP_INFO(logger, "IMU:");
            RCLCPP_INFO(logger, "  Topic: %s", imu.topic.c_str());
            RCLCPP_INFO(logger, "  Gyro cov: %.3f", imu.gyr_cov);

            RCLCPP_INFO(logger, "Publishing:");
            RCLCPP_INFO(logger, "  Path: %s", publish.path_enable ? "enabled" : "disabled");
            RCLCPP_INFO(logger, "  Scan: %s", publish.scan_publish_enable ? "enabled" : "disabled");

            if (pcd_save.enable)
            {
                RCLCPP_INFO(logger, "PCD Save: %s", pcd_save.save_path.c_str());
            }
        }
    };

} // namespace fast_lio
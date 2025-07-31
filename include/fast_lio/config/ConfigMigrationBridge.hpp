#pragma once

#include "Config.hpp"

namespace fast_lio
{

    /**
     * @brief Migration bridge to help transition from globals to Config class
     *
     * This provides macro definitions that map old global variable names to
     * the new config structure. This allows incremental refactoring without
     * breaking the entire codebase at once.
     *
     * USAGE:
     * 1. Include this header after Config.hpp
     * 2. Create a global Config instance
     * 3. Gradually replace macro usage with direct config access
     * 4. Eventually remove this bridge file
     */

    // Assuming we have a global config instance (will be created in laserMapping.cpp)
    extern Config g_config;

// ============== Algorithm Parameters ==============
#define INIT_TIME (g_config.algorithm.init_time)
#define LASER_POINT_COV (g_config.algorithm.laser_point_cov)
#define PUBFRAME_PERIOD (g_config.algorithm.pubframe_period)
#define NUM_MAX_ITERATIONS (g_config.algorithm.max_iterations)
#define DET_RANGE (g_config.algorithm.det_range)
#define MOV_THRESHOLD (g_config.algorithm.mov_threshold)
#define cube_len (g_config.algorithm.cube_side_length)
#define filter_size_corner_min (g_config.algorithm.filter_size_corner)
#define filter_size_surf_min (g_config.algorithm.filter_size_surf)
#define filter_size_map_min (g_config.algorithm.filter_size_map)

// ============== Lidar Parameters ==============
#define fov_deg (g_config.lidar.fov_degree)
#define FOV_DEG (g_config.lidar.fov_deg_expanded)
#define HALF_FOV_COS (g_config.lidar.half_fov_cos)
#define lid_topic (g_config.lidar.topic)

// ============== IMU Parameters ==============
#define imu_topic (g_config.imu.topic)
/*#define gyr_cov (g_config.imu.gyr_cov)
#define acc_cov (g_config.imu.acc_cov)
#define b_gyr_cov (g_config.imu.b_gyr_cov)
#define b_acc_cov (g_config.imu.b_acc_cov)*/
#define extrinT (g_config.imu.extrinsic_T)
#define extrinR (g_config.imu.extrinsic_R)

// ============== Time Sync ==============
#define time_sync_en (g_config.time_sync.enable)
#define time_diff_lidar_to_imu (g_config.time_sync.time_offset_lidar_to_imu)

// ============== Publishing Flags ==============
#define path_en (g_config.publish.path_enable)
#define scan_pub_en (g_config.publish.scan_publish_enable)
#define dense_pub_en (g_config.publish.dense_publish_enable)
#define scan_body_pub_en (g_config.publish.scan_bodyframe_enable)
    // #define effect_pub_en (g_config.publish.effect_map_enable)
// #define map_pub_en (g_config.publish.map_enable)

// ============== PCD Save ==============
#define pcd_save_en (g_config.pcd_save.enable)
#define map_file_path (g_config.pcd_save.save_path)
#define pcd_save_interval (g_config.pcd_save.interval)

// ============== Runtime ==============
#define runtime_pos_log (g_config.runtime.position_log_enable)
#define extrinsic_est_en (g_config.runtime.extrinsic_estimation_enable)
// #define root_dir (g_config.runtime.root_dir)

// ============== Memory Constants ==============
#define MAXN (fast_lio::Config::MemoryParams::MAXN)

    // ============== Helper Macros ==============
    // These will need manual replacement as they access p_pre
    // #define p_pre->lidar_type  (g_config.lidar.type)
    // #define p_pre->blind       (g_config.lidar.blind_radius)
    // etc.

    /**
     * @brief Initialize preprocessor with config values
     * Call this after creating config to sync preprocessor settings
     */
    inline void syncPreprocessorConfig(shared_ptr<Preprocess> &p_pre)
    {
        p_pre->lidar_type = g_config.lidar.type;
        p_pre->blind = g_config.lidar.blind_radius;
        p_pre->N_SCANS = g_config.lidar.scan_lines;
        p_pre->SCAN_RATE = g_config.lidar.scan_rate;
        p_pre->time_unit = g_config.lidar.timestamp_unit;
        p_pre->point_filter_num = g_config.lidar.point_filter_num;
        p_pre->feature_enabled = g_config.lidar.feature_enabled;
    }

} // namespace fast_lio
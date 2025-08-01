#pragma once

#include "Types.hpp"
#include "Constants.hpp"
#include <deque>
#include <sensor_msgs/msg/imu.hpp>

namespace fast_lio
{

    struct MeasureGroup
    {
        double lidar_beg_time = 0.0;
        double lidar_end_time = 0.0;
        types::PointCloudXYZI::Ptr lidar;
        std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;

        MeasureGroup()
        {
            lidar.reset(new types::PointCloudXYZI());
        }
    };

    struct StatesGroup
    {
        types::M3D rot_end;
        types::V3D pos_end;
        types::V3D vel_end;
        types::V3D bias_g;
        types::V3D bias_a;
        types::V3D gravity;
        Eigen::Matrix<double, constants::DIM_STATE, constants::DIM_STATE> cov;

        StatesGroup()
        {
            rot_end = types::M3D::Identity();
            pos_end = types::V3D::Zero();
            vel_end = types::V3D::Zero();
            bias_g = types::V3D::Zero();
            bias_a = types::V3D::Zero();
            gravity = types::V3D::Zero();
            cov = decltype(cov)::Identity() * constants::INIT_COV;
            cov.block<9, 9>(9, 9) = Eigen::Matrix<double, 9, 9>::Identity() * 0.00001;
        }

        StatesGroup(const StatesGroup &b) = default;
        StatesGroup &operator=(const StatesGroup &b) = default;

        StatesGroup operator+(const Eigen::Matrix<double, constants::DIM_STATE, 1> &state_add) const;
        StatesGroup &operator+=(const Eigen::Matrix<double, constants::DIM_STATE, 1> &state_add);
        Eigen::Matrix<double, constants::DIM_STATE, 1> operator-(const StatesGroup &b) const;

        void resetpose()
        {
            rot_end = types::M3D::Identity();
            pos_end = types::V3D::Zero();
            vel_end = types::V3D::Zero();
        }
    };

} // namespace fast_lio
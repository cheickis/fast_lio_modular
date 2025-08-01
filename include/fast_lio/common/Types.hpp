#pragma once

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fast_lio/msg/pose6_d.hpp> // ROS2 converts Pose6D to pose6_d

namespace fast_lio
{
    namespace types
    {
        // ROS message types
        using Pose6D = fast_lio::msg::Pose6D;

        // Point cloud types
        using PointType = pcl::PointXYZINormal;
        using PointCloudXYZI = pcl::PointCloud<PointType>;
        using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

        // Eigen types
        using V3D = Eigen::Vector3d;
        using M3D = Eigen::Matrix3d;
        using V3F = Eigen::Vector3f;
        using M3F = Eigen::Matrix3f;

        // Dynamic matrix types
        template <int rows, int cols>
        using MatrixD = Eigen::Matrix<double, rows, cols>;

        template <int rows>
        using VectorD = Eigen::Matrix<double, rows, 1>;

        template <int rows, int cols>
        using MatrixF = Eigen::Matrix<float, rows, cols>;

        template <int rows>
        using VectorF = Eigen::Matrix<float, rows, 1>;

// Convenience aliases
#define MD(a, b) fast_lio::types::MatrixD<(a), (b)>
#define VD(a) fast_lio::types::VectorD<(a)>
#define MF(a, b) fast_lio::types::MatrixF<(a), (b)>
#define VF(a) fast_lio::types::VectorF<(a)>

        // Global constants (for backward compatibility)
        inline const M3D Eye3d = M3D::Identity();
        inline const M3F Eye3f = M3F::Identity();
        inline const V3D Zero3d = V3D::Zero();
        inline const V3F Zero3f = V3F::Zero();
    }
} // namespace fast_lio
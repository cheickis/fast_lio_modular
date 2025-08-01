#pragma once

#include "fast_lio/common/Types.hpp"
#include "fast_lio/common/Constants.hpp"
#include "so3_math.h"
#include <Eigen/Dense>

namespace fast_lio
{
    namespace geometry
    {

        inline float calcDist(const types::PointType &p1, const types::PointType &p2)
        {
            float dx = p1.x - p2.x;
            float dy = p1.y - p2.y;
            float dz = p1.z - p2.z;
            return dx * dx + dy * dy + dz * dz;
        }

        template <typename T>
        types::Pose6D setPose6D(double t, const Eigen::Matrix<T, 3, 1> &a,
                                const Eigen::Matrix<T, 3, 1> &g,
                                const Eigen::Matrix<T, 3, 1> &v,
                                const Eigen::Matrix<T, 3, 1> &p,
                                const Eigen::Matrix<T, 3, 3> &R)
        {
            types::Pose6D rot_kp;
            rot_kp.offset_time = t;
            for (int i = 0; i < 3; i++)
            {
                rot_kp.acc[i] = a(i);
                rot_kp.gyr[i] = g(i);
                rot_kp.vel[i] = v(i);
                rot_kp.pos[i] = p(i);
                for (int j = 0; j < 3; j++)
                    rot_kp.rot[i * 3 + j] = R(i, j);
            }
            return rot_kp;
        }

        template <typename T>
        bool estiPlane(Eigen::Matrix<T, 4, 1> &pca_result,
                       const types::PointVector &points,
                       const T &threshold);

        template <typename T>
        bool estiNormVector(Eigen::Matrix<T, 3, 1> &normvec,
                            const types::PointVector &points,
                            const T &threshold,
                            int point_num);
    }
} // namespace fast_lio
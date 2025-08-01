#include "fast_lio/common/GeometryUtils.hpp"
#include "fast_lio/common/Constants.hpp"
#include <cmath>

namespace fast_lio
{
    namespace geometry
    {

        // Template specializations to avoid the constants issue
        namespace detail
        {
            constexpr int NUM_MATCH_POINTS = 5;
        }

        template <typename T>
        bool estiPlane(Eigen::Matrix<T, 4, 1> &pca_result,
                       const types::PointVector &points,
                       const T &threshold)
        {
            if (points.size() < detail::NUM_MATCH_POINTS)
            {
                return false;
            }

            // Use dynamic size matrices to avoid template issues
            Eigen::MatrixXd A(detail::NUM_MATCH_POINTS, 3);
            Eigen::VectorXd b(detail::NUM_MATCH_POINTS);
            A.setZero();
            b.setOnes();
            b *= -1.0;

            for (int j = 0; j < detail::NUM_MATCH_POINTS; j++)
            {
                A(j, 0) = points[j].x;
                A(j, 1) = points[j].y;
                A(j, 2) = points[j].z;
            }

            Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);

            double n = normvec.norm();
            pca_result(0) = T(normvec(0) / n);
            pca_result(1) = T(normvec(1) / n);
            pca_result(2) = T(normvec(2) / n);
            pca_result(3) = T(1.0 / n);

            for (int j = 0; j < detail::NUM_MATCH_POINTS; j++)
            {
                if (std::fabs(pca_result(0) * points[j].x +
                              pca_result(1) * points[j].y +
                              pca_result(2) * points[j].z +
                              pca_result(3)) > threshold)
                {
                    return false;
                }
            }
            return true;
        }

        template <typename T>
        bool estiNormVector(Eigen::Matrix<T, 3, 1> &normvec,
                            const types::PointVector &points,
                            const T &threshold,
                            int point_num)
        {
            if (points.size() < static_cast<size_t>(point_num))
            {
                return false;
            }

            // Use double precision for computation to avoid type issues
            Eigen::MatrixXd A(point_num, 3);
            Eigen::VectorXd b(point_num);
            b.setOnes();
            b *= -1.0;

            for (int j = 0; j < point_num; j++)
            {
                A(j, 0) = points[j].x;
                A(j, 1) = points[j].y;
                A(j, 2) = points[j].z;
            }

            Eigen::Vector3d normvec_d = A.colPivHouseholderQr().solve(b);

            // Convert back to template type
            for (int i = 0; i < 3; i++)
            {
                normvec(i) = T(normvec_d(i));
            }

            for (int j = 0; j < point_num; j++)
            {
                if (std::fabs(normvec(0) * points[j].x +
                              normvec(1) * points[j].y +
                              normvec(2) * points[j].z + T(1.0)) > threshold)
                {
                    return false;
                }
            }

            normvec.normalize();
            return true;
        }

        // Explicit instantiations
        template bool estiPlane<float>(Eigen::Matrix<float, 4, 1> &, const types::PointVector &, const float &);
        template bool estiPlane<double>(Eigen::Matrix<double, 4, 1> &, const types::PointVector &, const double &);
        template bool estiNormVector<float>(Eigen::Matrix<float, 3, 1> &, const types::PointVector &, const float &, int);
        template bool estiNormVector<double>(Eigen::Matrix<double, 3, 1> &, const types::PointVector &, const double &, int);

    } // namespace geometry
} // namespace fast_lio
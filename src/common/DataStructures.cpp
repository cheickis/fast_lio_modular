#include "fast_lio/common/DataStructures.hpp"
#include "so3_math.h"

namespace fast_lio
{

    StatesGroup StatesGroup::operator+(const Eigen::Matrix<double, constants::DIM_STATE, 1> &state_add) const
    {
        StatesGroup a;
        a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
        a.vel_end = this->vel_end + state_add.block<3, 1>(6, 0);
        a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
        a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
        a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
        a.cov = this->cov;
        return a;
    }

    StatesGroup &StatesGroup::operator+=(const Eigen::Matrix<double, constants::DIM_STATE, 1> &state_add)
    {
        this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
        this->pos_end += state_add.block<3, 1>(3, 0);
        this->vel_end += state_add.block<3, 1>(6, 0);
        this->bias_g += state_add.block<3, 1>(9, 0);
        this->bias_a += state_add.block<3, 1>(12, 0);
        this->gravity += state_add.block<3, 1>(15, 0);
        return *this;
    }

    Eigen::Matrix<double, constants::DIM_STATE, 1> StatesGroup::operator-(const StatesGroup &b) const
    {
        Eigen::Matrix<double, constants::DIM_STATE, 1> a;
        types::M3D rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3, 1>(0, 0) = Log(rotd);
        a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
        a.block<3, 1>(6, 0) = this->vel_end - b.vel_end;
        a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
        a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
        a.block<3, 1>(15, 0) = this->gravity - b.gravity;
        return a;
    }

} // namespace fast_lio